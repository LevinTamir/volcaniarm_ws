"""Core orchestration for calibration runs.

Decouples test execution from any GUI. The runner spins on a passed-in
``rclpy.Node`` (supplied by the rqt host) or builds its own when
launched headless. It emits status callbacks instead of Qt signals so
the module has no Qt import; the rqt widget bridges callbacks to
``QtCore.Signal``.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Tuple

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RclpyDuration
from rclpy.time import Time as RclpyTime

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from volcaniarm_msgs.srv import ComputeIK, ComputeFK

import tf2_ros

from .data_writer import RunWriter
from .tests import BaseTest


@dataclass
class RunRequest:
    """User-supplied parameters for a single calibration run."""
    test: BaseTest
    output_root: Path
    # Initial pose: where the arm parks before/between goal visits.
    # Goals: ordered list of (y, z) metres in workspace that the arm
    # visits. Single-pose tests pass [(y, z)]; the workspace-coverage
    # test passes the full envelope. Each iteration of the run does a
    # round-robin sweep across all goals, with a fresh-stamp gated
    # capture at every visit. IK is resolved on every entry once up
    # front; an unreachable goal aborts before any motion.
    initial_pose: tuple = (0.0, 0.5)
    goals: tuple = ((0.0, 0.5),)
    joint_names: tuple = (
        'volcaniarm_right_elbow_joint', 'volcaniarm_left_elbow_joint',
    )
    trajectory_duration: float = 2.0
    # Tag frames published by the apriltag detector. Ground truth is
    # T_base_to_ee = lookup_transform(base_tag_frame, ee_tag_frame),
    # which TF resolves through the camera.
    base_tag_frame: str = 'apriltag_marker_base'
    ee_tag_frame: str = 'apriltag_marker_ee'
    # URDF-side counterparts of the apriltag frames. Published by
    # robot_state_publisher from the static URDF mounts; serve as the
    # ground-truth segment-length expectation against which detection
    # is compared (Y-Z origin distance in the world frame).
    base_urdf_frame: str = 'apriltag_base_link'
    ee_urdf_frame: str = 'apriltag_ee_link'
    # Common world-aligned frame that both detected and URDF marker
    # origins are looked up in before the Y-Z difference is computed.
    # Comparing Y-Z components of T(parent, child) directly is wrong
    # because apriltag_marker_* and apriltag_*_link have different
    # parent orientations. Both must be expressed in a single
    # consistently-oriented frame for the Y-Z projection to be the
    # same axes for detection and URDF.
    world_frame: str = 'world'
    # Wait budget for a single TF lookup. 1 s comfortably covers normal
    # apriltag latency at 30 Hz; a tag that is undetectable (e.g.
    # edge-on at the current pose) fails fast instead of stalling.
    detection_timeout_s: float = 1.0
    # Maximum age of the TF stamp accepted as a fresh detection. Guards
    # against the TF buffer returning a stale transform from when the
    # tag was last seen seconds ago.
    detection_max_age_s: float = 0.5
    # Home-confirm gate (used by tests with verify_home_with_tag=True,
    # currently the repeatability test). After every return-to-home
    # trajectory the runner waits for the detected vs URDF Y-Z segment
    # length to agree within home_tol_m, held over home_hold_frames
    # consecutive *fresh* detections, bounded by home_timeout_s.
    home_tol_m: float = 0.02
    home_hold_frames: int = 5
    home_timeout_s: float = 10.0


# Callback signatures used by the runner. The widget wires Qt signals.
StatusCb = Callable[[str], None]
ProgressCb = Callable[[int, int], None]
FinishedCb = Callable[[Path, str], None]
# Fired once per iteration just before the operator-gated wait. Lets
# the dashboard switch into "waiting for continue" UI state.
AwaitingContinueCb = Callable[[int, int], None]
# Fired ~10 Hz while the runner is waiting at a goal pose. The widget
# uses this to enable/disable the Continue button and update the
# freshness label. ``age_s`` is meaningful only when ``is_fresh`` is
# True; otherwise pass 0.0.
DetectionStateCb = Callable[[bool, float], None]


class CalibrationRunner:
    """Drives a calibration test through its lifecycle."""

    def __init__(self, node: Node):
        self.node = node
        self._cb_group = ReentrantCallbackGroup()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        self._ik_client = node.create_client(
            ComputeIK, 'compute_ik', callback_group=self._cb_group)
        self._fk_client = node.create_client(
            ComputeFK, 'compute_fk', callback_group=self._cb_group)
        self._action_client = ActionClient(
            node, FollowJointTrajectory,
            '/volcaniarm_controller/follow_joint_trajectory',
            callback_group=self._cb_group)

        self._run_thread: Optional[threading.Thread] = None
        self._goto_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._continue_event = threading.Event()
        # Set by internal auto-abort paths (e.g. detection lost during
        # sampling). Distinguishes 'failed' from operator-driven
        # 'canceled' at finalize time. Cleared at the start of each run.
        self._failure_reason: Optional[str] = None
        self._writer: Optional[RunWriter] = None
        # Default joint names used by goto / reset_to when no run is
        # active. Mirrors RunRequest's default.
        self._default_joint_names = (
            'volcaniarm_right_elbow_joint', 'volcaniarm_left_elbow_joint',
        )
        self._default_trajectory_duration = 2.0

        self.status_cb: Optional[StatusCb] = None
        self.progress_cb: Optional[ProgressCb] = None
        self.finished_cb: Optional[FinishedCb] = None
        self.awaiting_continue_cb: Optional[AwaitingContinueCb] = None
        self.detection_state_cb: Optional[DetectionStateCb] = None

    # -- public control surface ------------------------------------

    def request_run(self, request: RunRequest) -> bool:
        if self._run_thread is not None and self._run_thread.is_alive():
            # The operator may have just clicked Cancel/Reset and is
            # immediately starting another test; the previous worker
            # typically exits within a few hundred ms once the stop
            # event is set. Give it a short window to die before
            # refusing, so a quick "cancel then start" sequence works
            # without an apparent UI lockout.
            self._run_thread.join(timeout=2.0)
            if self._run_thread.is_alive():
                self._emit_status('busy: another run is in progress')
                return False
        self._stop_event.clear()
        self._continue_event.clear()
        self._failure_reason = None
        self._run_thread = threading.Thread(
            target=self._run, args=(request,), daemon=True)
        self._run_thread.start()
        return True

    def proceed(self):
        """Operator clicked Continue: capture and move on."""
        self._continue_event.set()

    def cancel(self):
        """Operator clicked Cancel: abort the run as soon as possible.

        Stops in place. Any in-flight trajectory goal is cancelled at
        the action server too, so the arm coasts to a halt rather than
        completing the move. Use ``reset_to`` if you want the arm
        returned somewhere safe after cancelling.
        """
        self._stop_event.set()
        self._continue_event.set()

    def goto(self, y: float, z: float,
             joint_names: Optional[tuple] = None,
             trajectory_duration: Optional[float] = None) -> bool:
        """Send the arm to (y, z) workspace pose. Refuses while a run
        or another goto is in flight.

        Used by the dashboard's "Move to initial" affordance and by the
        Reset flow (via reset_to) to park the arm at a known pose
        outside of a test run.
        """
        if self._run_thread is not None and self._run_thread.is_alive():
            self._emit_status('busy: cannot move while a run is in progress')
            return False
        if self._goto_thread is not None and self._goto_thread.is_alive():
            self._emit_status('busy: another move is in progress')
            return False
        # Clear stop_event in case it was set by a prior cancel; the
        # goto worker honours it the same way the run loop does.
        self._stop_event.clear()
        self._goto_thread = threading.Thread(
            target=self._goto_worker,
            args=(y, z,
                  joint_names or self._default_joint_names,
                  trajectory_duration or self._default_trajectory_duration),
            daemon=True)
        self._goto_thread.start()
        return True

    def reset_to(self, y: float, z: float,
                 joint_names: Optional[tuple] = None,
                 trajectory_duration: Optional[float] = None):
        """Cancel any in-flight run and return the arm to (y, z).

        Spawns a worker that waits for the run thread to die before
        sending the move, so the caller (the GUI thread) returns
        immediately and stays responsive.
        """
        self.cancel()
        threading.Thread(
            target=self._reset_worker,
            args=(y, z,
                  joint_names or self._default_joint_names,
                  trajectory_duration or self._default_trajectory_duration),
            daemon=True).start()

    def shutdown(self):
        self.cancel()
        if self._run_thread is not None:
            self._run_thread.join(timeout=5.0)
        if self._goto_thread is not None:
            self._goto_thread.join(timeout=5.0)

    # -- emit helpers ----------------------------------------------

    def _emit_status(self, msg: str):
        self.node.get_logger().info(msg)
        if self.status_cb:
            self.status_cb(msg)

    def _emit_progress(self, current: int, total: int):
        if self.progress_cb:
            self.progress_cb(current, total)

    def _emit_finished(self, path: Path, status: str):
        if self.finished_cb:
            self.finished_cb(path, status)

    def _emit_awaiting_continue(self, iteration: int, total: int):
        if self.awaiting_continue_cb:
            self.awaiting_continue_cb(iteration, total)

    def _emit_detection_state(self, is_fresh: bool, age_s: float):
        if self.detection_state_cb:
            self.detection_state_cb(is_fresh, age_s)

    # -- main run loop --------------------------------------------

    def _run(self, request: RunRequest):
        if not self._wait_for_clients():
            self._emit_status('aborting: required services or actions unavailable')
            self._emit_finished(Path(), 'failed')
            return

        config = {
            'test_name': request.test.name,
            'num_cycles': request.test.num_cycles,
            'settle_time': request.test.settle_time,
            'verify_home_with_tag': request.test.verify_home_with_tag,
            'initial_pose': list(request.initial_pose),
            'goals': [list(g) for g in request.goals],
            'joint_names': list(request.joint_names),
            'trajectory_duration': request.trajectory_duration,
            'base_tag_frame': request.base_tag_frame,
            'ee_tag_frame': request.ee_tag_frame,
            'base_urdf_frame': request.base_urdf_frame,
            'ee_urdf_frame': request.ee_urdf_frame,
            'world_frame': request.world_frame,
            'detection_timeout_s': request.detection_timeout_s,
            'detection_max_age_s': request.detection_max_age_s,
            'home_tol_m': request.home_tol_m,
            'home_hold_frames': request.home_hold_frames,
            'home_timeout_s': request.home_timeout_s,
        }

        with RunWriter(request.output_root, request.test.name, config) as writer:
            self._writer = writer
            try:
                ok = self._execute(request, writer)
                if self._failure_reason is not None:
                    # Auto-aborts (detection loss, etc.) set both
                    # _stop_event and _failure_reason. Surface as
                    # 'failed' with the reason, not as 'canceled' --
                    # the operator didn't choose to stop.
                    writer.set_failure_reason(self._failure_reason)
                    writer.finalize('failed')
                elif self._stop_event.is_set():
                    writer.finalize('canceled')
                elif ok:
                    writer.finalize('completed')
                else:
                    # _execute returned early due to a precondition
                    # failure (unreachable IK, motion failure, missing
                    # services, etc.) -- the run did NOT complete
                    # successfully. Without this the dashboard
                    # silently reports 'completed' on every aborted
                    # run, hiding the actual problem.
                    writer.finalize('failed')
            except Exception as exc:
                self.node.get_logger().error(f'run failed: {exc}')
                writer.set_failure_reason(f'exception: {exc}')
                writer.finalize('failed')
            self._emit_finished(writer.run_dir, writer.status)
        self._writer = None

    def _execute(self, request: RunRequest, writer: RunWriter) -> bool:
        """Run the test end to end. Returns True only on full success;
        False on any early exit (IK failure, motion failure, cancel).
        ``_run`` distinguishes 'canceled' vs 'failed' via _stop_event,
        so cancellation paths can return False here without confusing
        the status reporting.
        """
        # Resolve every pose up front so an unreachable goal aborts
        # before any motion. IK is cheap and the operator gets a clear
        # error rather than watching the arm move halfway then stop.
        initial_ik = self._call_ik(*request.initial_pose)
        if initial_ik is None:
            self._emit_status(
                f'IK failed for initial pose y={request.initial_pose[0]:.3f} '
                f'z={request.initial_pose[1]:.3f}; aborting')
            return False
        initial_theta_r, initial_theta_l = initial_ik

        goals = list(request.goals)
        if not goals:
            self._emit_status('no goals supplied; aborting')
            return False
        goal_iks: list = []
        goal_fks: list = []
        for idx, (gy, gz) in enumerate(goals, start=1):
            ik = self._call_ik(gy, gz)
            if ik is None:
                self._emit_status(
                    f'IK failed for goal {idx} y={gy:.3f} z={gz:.3f}; aborting')
                return False
            goal_iks.append(ik)
            # FK is best-effort; if it fails we still record the
            # commanded (y, z) so the analysis notebook has something.
            goal_fks.append(self._call_fk(*ik) or (0.0, gy, gz))

        # Move to initial pose, settle, snapshot the home reference.
        self._emit_status(
            f'moving to initial pose y={request.initial_pose[0]:.3f} '
            f'z={request.initial_pose[1]:.3f}')
        if not self._send_and_wait(
                request.joint_names, initial_theta_r, initial_theta_l,
                request.trajectory_duration):
            self._emit_status('initial-pose move failed; aborting run')
            return False
        if not self._settle(request.test.settle_time, 'initial'):
            return False
        # When the test gates on tag-confirmed home, run the gate now
        # too so the very first iteration starts from a verified state.
        if request.test.verify_home_with_tag:
            if not self._wait_for_home_confirmed(request, label='initial'):
                return False
        self._capture_observations(
            request, writer,
            phase='home', cycle=0, target_idx=None,
            theta_right=initial_theta_r, theta_left=initial_theta_l)

        # Round-robin: each iteration sweeps across every goal in
        # order. Total visits = num_cycles * len(goals). Single-goal
        # tests (accuracy / repeatability) use len(goals) == 1 so the
        # loop is a straight N-iteration capture; the workspace test
        # uses K > 1 to walk the envelope each cycle.
        total_iterations = request.test.num_cycles
        total_visits = total_iterations * len(goals)
        visit_count = 0
        for iteration in range(1, total_iterations + 1):
            if self._stop_event.is_set():
                return False
            for goal_idx, (gy, gz) in enumerate(goals, start=1):
                if self._stop_event.is_set():
                    return False
                theta_r, theta_l = goal_iks[goal_idx - 1]
                fk_xyz = goal_fks[goal_idx - 1]

                self._emit_status(
                    f'cycle {iteration}/{total_iterations} '
                    f'goal {goal_idx}/{len(goals)}: moving to '
                    f'y={gy:.3f} z={gz:.3f}')
                if not self._send_and_wait(
                        request.joint_names, theta_r, theta_l,
                        request.trajectory_duration):
                    self._emit_status('goal trajectory failed; aborting run')
                    return False
                if not self._settle(request.test.settle_time, 'goal'):
                    return False

                # Repeatability / accuracy tests are hands-off (the gate
                # is just to wait for a fresh detection); workspace-
                # coverage runs without an operator gate too. The
                # awaiting-continue + auto-continue plumbing in the
                # dashboard remains for backwards compatibility but no
                # longer blocks: capture is gated only on a freshly
                # progressed TF stamp post-settle.
                self._emit_awaiting_continue(iteration, total_iterations)
                if not self._wait_for_continue(request):
                    # Either cancelled or the runner was shut down.
                    return False

                self._capture_observations(
                    request, writer,
                    phase='target', cycle=iteration, target_idx=goal_idx,
                    theta_right=theta_r, theta_left=theta_l)
                writer.add_fk({
                    'cycle': iteration,
                    'target_idx': goal_idx,
                    'theta_right': theta_r,
                    'theta_left': theta_l,
                    'fk_x': fk_xyz[0],
                    'fk_y': fk_xyz[1],
                    'fk_z': fk_xyz[2],
                })
                visit_count += 1
                self._emit_progress(visit_count, total_visits)

                # Single-pose tests return to the initial pose between
                # visits so each visit starts from a known state and
                # the run ends with the arm parked at the operator-
                # chosen initial. The workspace_coverage sweep skips
                # this so it can walk the envelope without doubling
                # back on every goal.
                if request.test.return_to_initial_between_visits:
                    self._emit_status(
                        f'cycle {iteration}/{total_iterations} '
                        f'goal {goal_idx}/{len(goals)}: returning to initial')
                    if not self._send_and_wait(
                            request.joint_names,
                            initial_theta_r, initial_theta_l,
                            request.trajectory_duration):
                        self._emit_status(
                            'return-to-initial move failed; aborting run')
                        return False
                    if request.test.verify_home_with_tag:
                        if not self._wait_for_home_confirmed(
                                request, label=f'cycle {iteration}'):
                            return False
        # End-of-run park. Single-pose tests already returned to
        # initial after their last visit, so this is a no-op for them
        # and we skip it. The workspace sweep ends at the last goal,
        # so park back at initial here so the arm always ends a
        # successful run at a known location.
        if not request.test.return_to_initial_between_visits:
            self._emit_status('end of run: returning to initial')
            if not self._send_and_wait(
                    request.joint_names,
                    initial_theta_r, initial_theta_l,
                    request.trajectory_duration):
                self._emit_status('end-of-run park failed; data is saved')
                # Data was captured; the cleanup move missing is a
                # UX issue, not a data corruption -- still report
                # success.
        return True

    def _goto_worker(self, y: float, z: float,
                     joint_names: tuple, duration: float):
        """Background worker for ``goto``. Runs IK then sends a single
        trajectory. Mirrors the wait/abort semantics of the run loop."""
        if not self._wait_for_clients():
            self._emit_status('cannot move: required services unavailable')
            return
        ik = self._call_ik(y, z)
        if ik is None:
            self._emit_status(f'IK failed for ({y:.3f}, {z:.3f}); not moving')
            return
        theta_r, theta_l = ik
        self._emit_status(f'moving to y={y:.3f} z={z:.3f}')
        if self._send_and_wait(joint_names, theta_r, theta_l, duration):
            self._emit_status(f'arrived at y={y:.3f} z={z:.3f}')
        else:
            self._emit_status(f'move to y={y:.3f} z={z:.3f} failed')

    def _reset_worker(self, y: float, z: float,
                      joint_names: tuple, duration: float):
        """Background worker for ``reset_to``: wait for the active run
        (if any) to wind down, then drive the arm to (y, z)."""
        if self._run_thread is not None:
            self._run_thread.join(timeout=10.0)
        if self._goto_thread is not None and self._goto_thread.is_alive():
            self._goto_thread.join(timeout=10.0)
        # The cancel that preceded us left _stop_event set; clear it so
        # the goto worker doesn't bail out before sending the move.
        self._stop_event.clear()
        self._goto_thread = threading.Thread(
            target=self._goto_worker,
            args=(y, z, joint_names, duration),
            daemon=True)
        self._goto_thread.start()

    # -- ROS plumbing ---------------------------------------------

    def _wait_for_clients(self, timeout_s: float = 5.0) -> bool:
        ok = (self._ik_client.wait_for_service(timeout_sec=timeout_s)
              and self._fk_client.wait_for_service(timeout_sec=timeout_s)
              and self._action_client.wait_for_server(timeout_sec=timeout_s))
        return ok

    def _call_ik(self, y: float, z: float):
        req = ComputeIK.Request()
        req.x = 0.0
        req.y = y
        req.z = z
        future = self._ik_client.call_async(req)
        # Wait synchronously: the runner is on its own thread; the
        # node's executor (run on a separate thread by the host)
        # services the future.
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            if self._stop_event.is_set():
                return None
            time.sleep(0.02)
        if not future.done():
            return None
        resp = future.result()
        if resp is None or not resp.success:
            return None
        return float(resp.theta1), float(resp.theta2)

    def _call_fk(self, theta_right: float, theta_left: float):
        req = ComputeFK.Request()
        req.theta1 = theta_right
        req.theta2 = theta_left
        future = self._fk_client.call_async(req)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            if self._stop_event.is_set():
                return None
            time.sleep(0.02)
        if not future.done():
            return None
        resp = future.result()
        if resp is None or not resp.success:
            return None
        return float(resp.x), float(resp.y), float(resp.z)

    def _send_and_wait(self, joint_names, theta_right: float,
                       theta_left: float, duration: float) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [theta_right, theta_left]
        point.velocities = [0.0, 0.0]
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal.trajectory.points.append(point)

        send_future = self._action_client.send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not send_future.done() and time.monotonic() < deadline:
            if self._stop_event.is_set():
                return False
            time.sleep(0.02)
        if not send_future.done():
            return False
        handle = send_future.result()
        if not handle or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        wait_total = duration + 5.0
        deadline = time.monotonic() + wait_total
        while not result_future.done() and time.monotonic() < deadline:
            if self._stop_event.is_set():
                handle.cancel_goal_async()
                return False
            time.sleep(0.05)
        if not result_future.done():
            return False
        return result_future.result().result.error_code == 0

    def _wait_for_continue(self, request: RunRequest) -> bool:
        """Block until the operator clicks Continue or Cancel.

        The capture step is now gated on a freshly progressed TF stamp
        after settle, so for hands-off tests this gate is effectively
        a pass-through (the dashboard's auto-continue trips immediately
        on the next fresh detection). It remains for the operator's
        Cancel handle and for any future test that needs human framing.

        Freshness is tracked by stamp progression, not by comparing the
        stamp against any node clock. Each poll, if the TF stamp differs
        from the previous poll's stamp, a new detection has arrived; we
        record the monotonic time of change. The detection is reported
        fresh while the most recent change was within
        ``detection_max_age_s`` monotonic seconds. Works identically in
        sim (where stamps are sim-time and the rqt clock may be wall-
        time) and on real hardware.
        """
        self._continue_event.clear()
        # Reset the detection indicator so the dashboard starts in the
        # "no detection yet" state on each gate entry.
        self._emit_detection_state(False, 0.0)
        last_stamp_ns: Optional[int] = None
        last_change_mono: Optional[float] = None
        fresh_window_s = max(request.detection_max_age_s, 0.1)
        while not self._continue_event.wait(0.1):
            if self._stop_event.is_set():
                return False
            tf, _ = self._lookup_base_to_ee(request, 0.0)
            now_mono = time.monotonic()
            if tf is None:
                self._emit_detection_state(False, 0.0)
                continue
            stamp_ns = (tf.header.stamp.sec * 1_000_000_000
                        + tf.header.stamp.nanosec)
            if last_stamp_ns is None or stamp_ns != last_stamp_ns:
                last_stamp_ns = stamp_ns
                last_change_mono = now_mono
            age_since_new_stamp = now_mono - (last_change_mono or now_mono)
            self._emit_detection_state(
                age_since_new_stamp < fresh_window_s, age_since_new_stamp)
        # Either the operator clicked Continue (proceed) or Cancel
        # (also signals the event but sets _stop_event). Re-check stop
        # so we don't capture/save during a cancelled run.
        if self._stop_event.is_set():
            return False
        return True

    def _settle(self, settle_time: float, label: str) -> bool:
        """Wait `settle_time` for motors and camera latency before sampling.

        Returns False if the run was cancelled while we were waiting.
        """
        if settle_time > 0:
            self._emit_status(f'settling at {label} for {settle_time:.1f}s')
            if self._stop_event.wait(settle_time):
                return False
        return True

    def _lookup_base_to_ee(self, request: RunRequest, wait_s: float):
        """Return (tf, reason) where tf is T(base_tag -> ee_tag) or None.

        Pure TF lookup, no clock-based freshness check. Freshness, when
        wanted, is tracked by the caller via stamp progression: poll
        repeatedly and treat a stamp that has just changed as a fresh
        detection. That approach is independent of the node's
        ``use_sim_time`` setting, which previously made every
        comparison against ``self.node.get_clock().now()`` register the
        stamp as ~1.78e9 seconds stale when the dashboard ran with
        wall-clock time and the apriltag detections inherited sim-time
        stamps from the Gazebo camera.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                request.base_tag_frame, request.ee_tag_frame,
                RclpyTime(),
                timeout=RclpyDuration(seconds=wait_s))
        except Exception as exc:
            return None, f'lookup failed: {exc}'
        return tf, ''

    def _lookup_origin_in_world(self, request: RunRequest,
                                child_frame: str, wait_s: float):
        """Return (tf, reason) where tf is T(world_frame -> child_frame).

        Used to express each marker's origin in a single world-aligned
        frame. Comparing Y-Z components of `T(apriltag_marker_base,
        apriltag_marker_ee).translation` against `T(apriltag_base_link,
        apriltag_ee_link).translation` is wrong: those parent frames
        have different orientations, so 'Y' and 'Z' don't refer to the
        same axes. Looking each origin up in `world` before taking the
        Y-Z difference fixes the comparison.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                request.world_frame, child_frame,
                RclpyTime(),
                timeout=RclpyDuration(seconds=wait_s))
        except Exception as exc:
            return None, f'lookup failed: {exc}'
        return tf, ''

    def _yz_segment_world(self, request: RunRequest,
                          base_frame: str, ee_frame: str,
                          wait_s: float
                          ) -> Tuple[Optional[float], Optional[Tuple[float, float, float, float]], str]:
        """Compute |EE_origin - base_origin|_yz in the world frame.

        Returns ``(distance_m, (base_y, base_z, ee_y, ee_z), reason)``.
        ``distance_m`` is None if either lookup failed; the per-origin
        tuple is provided for logging so the analysis can plot the
        cluster directly in world Y-Z without re-deriving from base.
        """
        base_tf, base_reason = self._lookup_origin_in_world(
            request, base_frame, wait_s)
        if base_tf is None:
            return None, None, base_reason
        ee_tf, ee_reason = self._lookup_origin_in_world(
            request, ee_frame, 0.0)
        if ee_tf is None:
            return None, None, ee_reason
        b = base_tf.transform.translation
        e = ee_tf.transform.translation
        dy = e.y - b.y
        dz = e.z - b.z
        return math.hypot(dy, dz), (b.y, b.z, e.y, e.z), ''

    def _wait_for_fresh_detection(
            self, request: RunRequest, timeout_s: float
            ) -> Optional['object']:
        """Block until a detected base->ee TF arrives with a stamp that
        has progressed since this call started. Returns the TF on
        success, or None on timeout / cancel.

        The first lookup snapshots the current stamp as the baseline;
        subsequent lookups (~10 Hz) pass once the stamp differs. This
        guarantees the captured sample reflects a detection that
        arrived *after* the arm settled, not a stale buffered TF from
        before the move.
        """
        tf0, _ = self._lookup_base_to_ee(request, timeout_s)
        if tf0 is None:
            return None
        baseline_stamp_ns = (tf0.header.stamp.sec * 1_000_000_000
                             + tf0.header.stamp.nanosec)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return None
            tf, _ = self._lookup_base_to_ee(request, 0.0)
            if tf is not None:
                stamp_ns = (tf.header.stamp.sec * 1_000_000_000
                            + tf.header.stamp.nanosec)
                if stamp_ns != baseline_stamp_ns:
                    return tf
            self._stop_event.wait(0.05)
        return None

    def _wait_for_fresh_frame(
            self, frame: str, timeout_s: float,
            parent: str = 'world') -> Optional['object']:
        """Stamp-gated single-frame variant of _wait_for_fresh_detection.

        Looks up T(parent, frame); returns the TF once its header.stamp
        has progressed since this call started. Used by the camera-
        position calibration runner to wait for a fresh apriltag_marker_ee
        detection between EE-sweep poses, without needing the test
        runner's base/ee tag pair semantics.
        """
        try:
            tf0 = self._tf_buffer.lookup_transform(
                parent, frame, RclpyTime(),
                timeout=RclpyDuration(seconds=timeout_s))
        except Exception:
            return None
        baseline_stamp_ns = (tf0.header.stamp.sec * 1_000_000_000
                             + tf0.header.stamp.nanosec)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return None
            try:
                tf = self._tf_buffer.lookup_transform(
                    parent, frame, RclpyTime(),
                    timeout=RclpyDuration(seconds=0.0))
            except Exception:
                tf = None
            if tf is not None:
                stamp_ns = (tf.header.stamp.sec * 1_000_000_000
                            + tf.header.stamp.nanosec)
                if stamp_ns != baseline_stamp_ns:
                    return tf
            self._stop_event.wait(0.05)
        return None

    def _wait_for_home_confirmed(self, request: RunRequest,
                                 label: str) -> bool:
        """Wait for the EE marker to settle near the URDF home pose.

        Polls detected base->ee and URDF base->ee at ~20 Hz and
        compares their Y-Z segment lengths. The detection counts only
        when its TF stamp has progressed since the last poll (so we
        ignore a stale TF buffered from before the home move). The
        gate passes when ``|d_detected - d_urdf| < home_tol_m`` for
        ``home_hold_frames`` consecutive fresh detections.

        On timeout, sets ``_failure_reason`` and signals stop so the
        run finalises as 'failed' with a clear reason rather than
        silently continuing with an unverified home.
        """
        timeout_s = request.home_timeout_s
        tol_m = request.home_tol_m
        hold = max(1, int(request.home_hold_frames))
        deadline = time.monotonic() + timeout_s
        last_stamp_ns: Optional[int] = None
        consecutive_in_tol = 0
        last_err: Optional[float] = None
        self._emit_status(
            f'home-confirm ({label}): waiting for detected vs URDF '
            f'world Y-Z segment match (tol={tol_m * 1000:.0f} mm, '
            f'hold={hold} frames, timeout={timeout_s:.1f} s)')
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return False
            # Use base_tag -> ee_tag stamp progression as the freshness
            # signal: it ticks every time apriltag_ros publishes a new
            # detection. The actual segment magnitude is then computed
            # in world-frame Y-Z so detection and URDF use the same axes.
            det_tf, _ = self._lookup_base_to_ee(request, 0.0)
            if det_tf is None:
                consecutive_in_tol = 0
                self._stop_event.wait(0.05)
                continue
            stamp_ns = (det_tf.header.stamp.sec * 1_000_000_000
                        + det_tf.header.stamp.nanosec)
            if stamp_ns == last_stamp_ns:
                # Same stamp; no new detection yet.
                self._stop_event.wait(0.05)
                continue
            last_stamp_ns = stamp_ns
            d_det, _, det_reason = self._yz_segment_world(
                request, request.base_tag_frame, request.ee_tag_frame, 0.0)
            if d_det is None:
                consecutive_in_tol = 0
                continue
            d_urdf, _, urdf_reason = self._yz_segment_world(
                request, request.base_urdf_frame, request.ee_urdf_frame, 0.0)
            if d_urdf is None:
                self._failure_reason = (
                    f'home-confirm ({label}): URDF tag chain '
                    f'unavailable in world frame ({urdf_reason})')
                self._emit_status(f'aborting: {self._failure_reason}')
                self._stop_event.set()
                return False
            err = abs(d_det - d_urdf)
            last_err = err
            if err < tol_m:
                consecutive_in_tol += 1
                if consecutive_in_tol >= hold:
                    self._emit_status(
                        f'home-confirm ({label}): confirmed '
                        f'(err={err * 1000:.1f} mm over '
                        f'{consecutive_in_tol} frames)')
                    return True
            else:
                consecutive_in_tol = 0
        elapsed = time.monotonic() - (deadline - timeout_s)
        last_err_mm = ('n/a' if last_err is None
                       else f'{last_err * 1000:.1f} mm')
        self._failure_reason = (
            f'home-confirm ({label}): timeout after {elapsed:.1f} s '
            f'(last err {last_err_mm}, tol {tol_m * 1000:.0f} mm)')
        self._emit_status(f'aborting: {self._failure_reason}')
        self._stop_event.set()
        return False

    def _capture_observations(self, request: RunRequest, writer: RunWriter,
                              phase: str, cycle: int,
                              target_idx: Optional[int],
                              theta_right: float, theta_left: float):
        """Capture exactly one base->ee detection at the current pose.

        Multi-sample averaging was dropped: the arm is stationary by
        the time we read, so repeated lookups recorded the detector
        pixel noise floor without measuring anything that varied. The
        single capture is gated on a TF stamp progression (a detection
        whose stamp differs from the one buffered before settle ended)
        so the row reflects a measurement made *after* the arm settled,
        not a stale buffered transform.

        Each row stores the raw apriltag base->ee transform (for tag-
        frame diagnostics), the world-frame Y-Z origins of both the
        detected and URDF base/EE markers (so the analysis can plot
        clusters in a single consistent frame), and the headline
        scalars ``d_detected``, ``d_urdf`` and the signed difference
        ``d_error``. The d_* scalars are computed in the world frame
        so the Y and Z axes refer to the same physical directions for
        detection and URDF, regardless of how each parent tag frame
        is oriented.
        """
        if self._stop_event.is_set():
            return
        det = self._wait_for_fresh_detection(
            request, request.detection_timeout_s)
        if det is None:
            reason = (f'no fresh detection after settle '
                      f'(timeout={request.detection_timeout_s:.1f} s)')
            self._emit_status(
                f'{phase} capture: {reason} (cycle={cycle}, '
                f'target_idx={target_idx})')
            # 'target' phase: the run cannot recover -- we needed a
            # measurement at this pose. 'home' baseline missing is
            # logged but not fatal (the analysis still has the goal
            # rows).
            if phase == 'target':
                self._failure_reason = (
                    f'detection lost during sampling at cycle={cycle}, '
                    f'target_idx={target_idx}: {reason}')
                self._emit_status(f'aborting: {self._failure_reason}')
                self._stop_event.set()
            return
        d_detected, det_origins, det_reason = self._yz_segment_world(
            request, request.base_tag_frame, request.ee_tag_frame, 0.1)
        d_urdf, urdf_origins, urdf_reason = self._yz_segment_world(
            request, request.base_urdf_frame, request.ee_urdf_frame, 0.1)
        if d_detected is None:
            self._emit_status(
                f'{phase} capture: detected segment unavailable in '
                f'world frame ({det_reason}); aborting')
            if phase == 'target':
                self._failure_reason = (
                    f'world-frame detection lookup failed at cycle={cycle}, '
                    f'target_idx={target_idx}: {det_reason}')
                self._stop_event.set()
            return
        if d_urdf is None:
            d_urdf_val = float('nan')
            d_error = float('nan')
            urdf_origins = (float('nan'),) * 4
            self._emit_status(
                f'{phase} capture: URDF segment unavailable in world '
                f'frame ({urdf_reason}); logging detection only')
        else:
            d_urdf_val = d_urdf
            d_error = d_detected - d_urdf
        det_base_y, det_base_z, det_ee_y, det_ee_z = det_origins
        urdf_base_y, urdf_base_z, urdf_ee_y, urdf_ee_z = urdf_origins
        t = det.transform.translation
        r = det.transform.rotation
        stamp = RclpyTime.from_msg(det.header.stamp)
        writer.add_tag_observation({
            'phase': phase,
            'cycle': cycle,
            'target_idx': '' if target_idx is None else target_idx,
            'sample_idx': 1,
            't_ros_ns': stamp.nanoseconds,
            'theta_right': theta_right,
            'theta_left': theta_left,
            'x': t.x, 'y': t.y, 'z': t.z,
            'qx': r.x, 'qy': r.y, 'qz': r.z, 'qw': r.w,
            'det_base_y': det_base_y, 'det_base_z': det_base_z,
            'det_ee_y': det_ee_y, 'det_ee_z': det_ee_z,
            'urdf_base_y': urdf_base_y, 'urdf_base_z': urdf_base_z,
            'urdf_ee_y': urdf_ee_y, 'urdf_ee_z': urdf_ee_z,
            'd_detected': d_detected,
            'd_urdf': d_urdf_val,
            'd_error': d_error,
        })
        if math.isnan(d_error):
            self._emit_status(
                f'{phase} capture (cycle={cycle}, target_idx={target_idx}): '
                f'd_detected={d_detected * 1000:.1f} mm  d_urdf=n/a')
        else:
            self._emit_status(
                f'{phase} capture (cycle={cycle}, target_idx={target_idx}): '
                f'd_detected={d_detected * 1000:.1f} mm  '
                f'd_urdf={d_urdf_val * 1000:.1f} mm  '
                f'd_error={d_error * 1000:+.1f} mm')
