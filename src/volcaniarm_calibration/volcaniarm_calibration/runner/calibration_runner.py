"""Core orchestration for calibration runs.

Decouples test execution from any GUI. The runner spins on a passed-in
``rclpy.Node`` (supplied by the rqt host) or builds its own when
launched headless. It emits status callbacks instead of Qt signals so
the module has no Qt import; the rqt widget bridges callbacks to
``QtCore.Signal``.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

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
    # round-robin sweep across all goals, with the operator-gated
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
    # which TF resolves through the camera. Compared in post-processing
    # against FK after applying the known link->tag offsets.
    base_tag_frame: str = 'apriltag_marker_base'
    ee_tag_frame: str = 'apriltag_marker_ee'
    # Wait budget for a single TF lookup. 1 s comfortably covers normal
    # apriltag latency at 30 Hz; a tag that is undetectable (e.g.
    # edge-on at the current pose) fails fast instead of stalling.
    detection_timeout_s: float = 1.0
    # Maximum age of the TF stamp accepted as a fresh detection. Guards
    # against the TF buffer returning a stale transform from when the
    # tag was last seen seconds ago.
    detection_max_age_s: float = 0.5


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
            self._emit_status('busy: another run is in progress')
            return False
        self._stop_event.clear()
        self._continue_event.clear()
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
            'samples_per_visit': request.test.samples_per_visit,
            'settle_time': request.test.settle_time,
            'sample_interval': request.test.sample_interval,
            'initial_pose': list(request.initial_pose),
            'goals': [list(g) for g in request.goals],
            'joint_names': list(request.joint_names),
            'trajectory_duration': request.trajectory_duration,
            'base_tag_frame': request.base_tag_frame,
            'ee_tag_frame': request.ee_tag_frame,
            'detection_timeout_s': request.detection_timeout_s,
            'detection_max_age_s': request.detection_max_age_s,
        }

        with RunWriter(request.output_root, request.test.name, config) as writer:
            self._writer = writer
            try:
                self._execute(request, writer)
                if self._stop_event.is_set():
                    writer.finalize('canceled')
                else:
                    writer.finalize('completed')
            except Exception as exc:
                self.node.get_logger().error(f'run failed: {exc}')
                writer.finalize('failed')
            self._emit_finished(writer.run_dir, writer.status)
        self._writer = None

    def _execute(self, request: RunRequest, writer: RunWriter):
        # Resolve every pose up front so an unreachable goal aborts
        # before any motion. IK is cheap and the operator gets a clear
        # error rather than watching the arm move halfway then stop.
        initial_ik = self._call_ik(*request.initial_pose)
        if initial_ik is None:
            self._emit_status(
                f'IK failed for initial pose y={request.initial_pose[0]:.3f} '
                f'z={request.initial_pose[1]:.3f}; aborting')
            return
        initial_theta_r, initial_theta_l = initial_ik

        goals = list(request.goals)
        if not goals:
            self._emit_status('no goals supplied; aborting')
            return
        goal_iks: list = []
        goal_fks: list = []
        for idx, (gy, gz) in enumerate(goals, start=1):
            ik = self._call_ik(gy, gz)
            if ik is None:
                self._emit_status(
                    f'IK failed for goal {idx} y={gy:.3f} z={gz:.3f}; aborting')
                return
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
            return
        if not self._settle(request.test.settle_time, 'initial'):
            return
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
                return
            for goal_idx, (gy, gz) in enumerate(goals, start=1):
                if self._stop_event.is_set():
                    return
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
                    return
                if not self._settle(request.test.settle_time, 'goal'):
                    return

                # Operator-gated wait: detection_state_cb keeps the
                # dashboard in sync; proceed() / cancel() unblock.
                self._emit_awaiting_continue(iteration, total_iterations)
                if not self._wait_for_continue(request):
                    # Either cancelled or the runner was shut down.
                    return

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

                # Always return to the initial pose between visits so
                # each visit starts from a known state and the run
                # ends with the arm parked at the operator-chosen
                # initial.
                self._emit_status(
                    f'cycle {iteration}/{total_iterations} '
                    f'goal {goal_idx}/{len(goals)}: returning to initial')
                if not self._send_and_wait(
                        request.joint_names,
                        initial_theta_r, initial_theta_l,
                        request.trajectory_duration):
                    self._emit_status(
                        'return-to-initial move failed; aborting run')
                    return

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

        While blocked, the runner polls the apriltag TF at ~10 Hz and
        emits ``detection_state_cb(is_fresh, age_s)`` so the dashboard
        can gate the Continue button. There's no hard timeout: the
        operator may need an arbitrary amount of time to physically
        reposition the camera until both markers appear.

        Freshness is tracked by stamp *progression*, not by comparing
        the stamp against any node clock. Each poll, if the TF stamp
        differs from the previous poll's stamp, a new detection has
        arrived and we record the monotonic time of change. The
        detection is reported fresh while the most recent change was
        within ``detection_max_age_s`` monotonic seconds. Works
        identically in sim (where stamps are sim-time and the rqt
        clock may be wall-time) and on real hardware.
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

    def _capture_observations(self, request: RunRequest, writer: RunWriter,
                              phase: str, cycle: int,
                              target_idx: Optional[int],
                              theta_right: float, theta_left: float):
        """Take ``samples_per_visit`` snapshots of base->ee at the current pose.

        Each sample is stored as one row in tag_observations.csv tagged
        with the phase (home|target). Strategy: try the first lookup
        with the full ``detection_timeout_s`` budget; if it fails (e.g.
        EE tag edge-on at home), abort the rest of the burst with one
        log line so the run doesn't stall N * timeout seconds. Once the
        first sample succeeds, subsequent lookups use a short wait
        since detections are streaming at camera rate.
        """
        n = max(1, request.test.samples_per_visit)
        interval = max(0.0, request.test.sample_interval)
        captured = 0
        last_reason = ''
        for i in range(n):
            if self._stop_event.is_set():
                return
            wait_s = request.detection_timeout_s if i == 0 else 0.1
            tf, reason = self._lookup_base_to_ee(request, wait_s)
            if tf is not None:
                t = tf.transform.translation
                r = tf.transform.rotation
                stamp = RclpyTime.from_msg(tf.header.stamp)
                writer.add_tag_observation({
                    'phase': phase,
                    'cycle': cycle,
                    'target_idx': '' if target_idx is None else target_idx,
                    'sample_idx': i + 1,
                    't_ros_ns': stamp.nanoseconds,
                    'theta_right': theta_right,
                    'theta_left': theta_left,
                    'x': t.x, 'y': t.y, 'z': t.z,
                    'qx': r.x, 'qy': r.y, 'qz': r.z, 'qw': r.w,
                })
                captured += 1
            else:
                last_reason = reason
                if i == 0:
                    # Tag is not visible at this pose; don't burn the
                    # rest of the burst trying.
                    break
            if i < n - 1 and interval > 0:
                if self._stop_event.wait(interval):
                    return
        if captured == 0:
            self._emit_status(
                f'{phase} capture: 0 samples (cycle={cycle}, '
                f'target_idx={target_idx}); {last_reason}')
        else:
            self._emit_status(
                f'{phase} capture: {captured}/{n} samples '
                f'(cycle={cycle}, target_idx={target_idx})')
