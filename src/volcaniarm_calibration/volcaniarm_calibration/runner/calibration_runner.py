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
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
import volcaniarm_kinematics_py as vk

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
    # Wait budget for a fresh detection after settle (and for a single
    # TF lookup). 5 s rides out real-world detector gaps (2 s proved
    # too tight on hardware) while still failing on a tag that is
    # genuinely undetectable (e.g. edge-on at the current pose).
    # Exposed as the "detection timeout" spinbox in the dashboard;
    # capped there so a very long timeout can't mask a marginal
    # detection setup.
    detection_timeout_s: float = 5.0
    # Resume support: when resume_dir points at an existing (failed)
    # run directory, its CSVs are appended to instead of a new run dir
    # being created, and the cycle loop starts at start_cycle so the
    # already-captured cycles are kept. The dashboard's Resume button
    # fills both from the failed run's config + data.
    resume_dir: Optional[Path] = None
    start_cycle: int = 1
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
    # -- Exp0 extensions -------------------------------------------
    # Samples captured per visit. 1 (default) keeps the historical
    # single-sample behaviour. The noise_gate test uses ~500 to map
    # the detector noise floor; settle_probe uses a burst sized to
    # cover ~4 s so t_ros_ns traces pose vs time after arrival.
    # Every sample is individually stamp-gated, so N samples are N
    # distinct detections, not N reads of one buffered TF.
    samples_per_capture: int = 1
    # Minimum wall-clock spacing between samples (0 = as fast as
    # fresh detections arrive).
    sample_min_period_s: float = 0.0
    # Sweep-pass metadata, recorded in config.yaml. Pass 2 of the
    # Exp0 serpentine runs on a different day / after power cycle;
    # the analysis joins passes on (pass_id, label).
    pass_id: int = 1
    session_note: str = ''
    # Visit-level resume for sweeps: skip the first N *captured*
    # visits (and any motion-only pre-points that precede them). To
    # resume an interrupted sweep in a NEW run directory, count the
    # target rows in the dead run's fk_poses.csv and pass that count
    # here; cycle/target_idx numbering stays aligned with the
    # original visit order because bookkeeping is computed over the
    # full visit list. Orthogonal to the dashboard's cycle-level
    # resume (resume_dir/start_cycle), which appends to the old dir.
    skip_visits: int = 0


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
# (pair_is_fresh, pair_age_s, base_age_s, ee_age_s) - per-tag ages are
# monotonic seconds since that marker's TF last advanced while the
# runner was polling, or -1.0 when the marker was never seen.
DetectionStateCb = Callable[[bool, float, float, float], None]


def _visit_bookkeeping(visits) -> Tuple[list, list, int]:
    """Per-visit (cycle, target_idx) derived from captured visits' labels.

    Matches the historical round-robin numbering exactly for the
    pre-existing tests: target_idx is the order of the label's first
    captured appearance, cycle is the 1-based count of captured visits
    to that label so far. Motion-only visits (capture=False, e.g.
    backlash pre-points carrying distinct '... pre' labels) get cycle
    for status text only and target_idx 0, and never shift the
    numbering. Returns (cycle_of, tidx_of, total_captures).
    """
    label_first: dict = {}
    label_count: dict = {}
    cycle_of: list = []
    tidx_of: list = []
    for v in visits:
        if v.capture:
            if v.label not in label_first:
                label_first[v.label] = len(label_first) + 1
            label_count[v.label] = label_count.get(v.label, 0) + 1
            cycle_of.append(label_count[v.label])
            tidx_of.append(label_first[v.label])
        else:
            cycle_of.append(label_count.get(v.label, 0) + 1)
            tidx_of.append(0)
    total_captures = sum(1 for v in visits if v.capture)
    return cycle_of, tidx_of, total_captures


def resume_start_index(visits, skip_captured: int) -> int:
    """Index of the first visit to execute after skipping the first
    ``skip_captured`` captured visits (motion-only pre-points that
    precede them are skipped too). Returns len(visits) when nothing is
    left to do, -1 when skip_captured exceeds the captured total."""
    if skip_captured <= 0:
        return 0
    seen = 0
    for i, v in enumerate(visits):
        if v.capture:
            seen += 1
            if seen == skip_captured:
                return i + 1
    return -1


class CalibrationRunner:
    """Drives a calibration test through its lifecycle."""

    def __init__(self, node: Node):
        self.node = node
        self._cb_group = ReentrantCallbackGroup()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        # Kinematics in-process via the C++ library (single source of truth);
        # no compute_ik/compute_fk service. Defaults mirror the URDF exactly.
        self._kin = vk.Params()
        self._action_client = ActionClient(
            node, FollowJointTrajectory,
            '/volcaniarm_controller/follow_joint_trajectory',
            callback_group=self._cb_group)
        # Limit-switch homing service advertised by the hardware
        # interface. The GUI's Start tab calls this to re-zero the arm
        # when it was booted with auto_home:=false.
        self._home_client = self.node.create_client(
            Trigger, '/volcaniarm_hardware_interface/home',
            callback_group=self._cb_group)

        # Per-marker stamp-progression tracker for _tag_age:
        # frame -> (last_stamp_ns, monotonic_time_of_last_change).
        self._tag_seen: dict = {}

        self._run_thread: Optional[threading.Thread] = None
        self._goto_thread: Optional[threading.Thread] = None
        self._home_thread: Optional[threading.Thread] = None
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
        # Homing completion: (ok: bool, message: str).
        self.home_finished_cb: Optional[Callable[[bool, str], None]] = None

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

    def home(self) -> bool:
        """Trigger limit-switch homing via the volcaniarm_hardware service.

        Re-zeros the arm on its limit switches (the arm moves; the seek
        takes up to ~30 s). Used by the dashboard's Start tab when the
        robot was booted with auto_home:=false. Refuses while a run,
        move, or another home is in flight. Reports via home_finished_cb.
        """
        if self._run_thread is not None and self._run_thread.is_alive():
            self._emit_status('busy: cannot home while a run is in progress')
            return False
        if self._goto_thread is not None and self._goto_thread.is_alive():
            self._emit_status('busy: cannot home while a move is in progress')
            return False
        if self._home_thread is not None and self._home_thread.is_alive():
            self._emit_status('busy: homing already in progress')
            return False
        # Clear stop_event in case a prior cancel set it; the home worker
        # honours it so Cancel aborts the wait.
        self._stop_event.clear()
        self._home_thread = threading.Thread(
            target=self._home_worker, daemon=True)
        self._home_thread.start()
        return True

    def shutdown(self):
        self.cancel()
        if self._run_thread is not None:
            self._run_thread.join(timeout=5.0)
        if self._goto_thread is not None:
            self._goto_thread.join(timeout=5.0)
        if self._home_thread is not None:
            self._home_thread.join(timeout=5.0)

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

    def _emit_detection_state(self, is_fresh: bool, age_s: float,
                              base_age_s: float = -1.0,
                              ee_age_s: float = -1.0):
        if self.detection_state_cb:
            self.detection_state_cb(is_fresh, age_s, base_age_s, ee_age_s)

    # -- per-tag freshness tracking -------------------------------

    def _tag_age(self, request: RunRequest, frame: str) -> float:
        """Monotonic seconds since `frame`'s TF stamp last advanced
        while we were polling, or -1.0 when the marker has never been
        resolved. Stamp-progression based (sim-time safe): looked up
        against the world frame so the static URDF part of the chain
        never limits the stamp."""
        try:
            tf = self._tf_buffer.lookup_transform(
                request.world_frame, frame, RclpyTime(),
                timeout=RclpyDuration(seconds=0.0))
        except Exception:
            return -1.0
        stamp_ns = (tf.header.stamp.sec * 1_000_000_000
                    + tf.header.stamp.nanosec)
        now = time.monotonic()
        prev = self._tag_seen.get(frame)
        if prev is None or prev[0] != stamp_ns:
            self._tag_seen[frame] = (stamp_ns, now)
            return 0.0
        return now - prev[1]

    def _tag_ages(self, request: RunRequest):
        """(base_age_s, ee_age_s) via _tag_age."""
        return (self._tag_age(request, request.base_tag_frame),
                self._tag_age(request, request.ee_tag_frame))

    @staticmethod
    def _age_text(age_s: float) -> str:
        return 'never seen' if age_s < 0 else f'seen {age_s:.1f}s ago'

    def _tag_ages_text(self, request: RunRequest) -> str:
        base_age, ee_age = self._tag_ages(request)
        return (f'base tag {self._age_text(base_age)}, '
                f'ee tag {self._age_text(ee_age)}')

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
            'samples_per_capture': request.samples_per_capture,
            'sample_min_period_s': request.sample_min_period_s,
            'pass_id': request.pass_id,
            'session_note': request.session_note,
            'skip_visits': request.skip_visits,
        }
        # Record which URDF apriltag mount values this run was taken
        # with. The analysis groups runs by these so runs recorded
        # before/after a mount recalibration are never averaged
        # together, and the loader no longer needs hand-mirrored xacro
        # constants for new runs. Best effort: None if TF is not up yet.
        mounts = self._lookup_urdf_mounts(request)
        if mounts is not None:
            config['urdf_mounts'] = mounts

        with RunWriter(request.output_root, request.test.name, config,
                       resume_dir=request.resume_dir) as writer:
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

        # The visit list comes from the test's iter_visits(), which
        # owns the pattern (order, repetitions, motion-only
        # pre-points). The runner is pattern-agnostic: it moves,
        # settles, and captures wherever Target.capture is True.
        visits = list(request.test.iter_visits())
        if not visits:
            self._emit_status('no visits supplied; aborting')
            return False
        cycle_of, tidx_of, total_captures = _visit_bookkeeping(visits)

        # Two resume mechanisms map onto one captured-visit skip:
        # 1. skip_visits (Exp0 sweep resume, NEW run dir): skip the
        #    first N captured visits verbatim.
        # 2. start_cycle (dashboard resume, appends to the old dir):
        #    skip every captured visit belonging to a cycle below
        #    start_cycle -- identical to the historical cycle-granular
        #    restart for the round-robin tests.
        # Bookkeeping was computed over the FULL list, so cycle /
        # target_idx numbering stays aligned with the original order.
        skip_captured = 0
        if request.skip_visits > 0:
            skip_captured = int(request.skip_visits)
        elif request.start_cycle > 1:
            skip_captured = sum(
                1 for i, v in enumerate(visits)
                if v.capture and cycle_of[i] < request.start_cycle)
        start_idx = resume_start_index(visits, skip_captured)
        if start_idx < 0 or start_idx >= len(visits):
            self._emit_status(
                f'resume leaves no work (skipping {skip_captured} of '
                f'{total_captures} captured visits); aborting')
            return False
        if skip_captured > 0:
            self._emit_status(
                f'resuming: skipping first {skip_captured} captured '
                f'visits ({start_idx} visits total)')
            self._emit_progress(skip_captured, total_captures)

        # Resolve IK for every remaining visit up front, seeded with
        # the previous solution for branch continuity along the sweep;
        # an unreachable visit aborts before any motion. FK is
        # best-effort; if it fails we still record the commanded
        # (y, z) so the analysis notebook has something.
        visit_iks: list = []
        visit_fks: list = []
        seed_r, seed_l = initial_ik
        for i in range(start_idx, len(visits)):
            v = visits[i]
            ik = self._call_ik(v.y, v.z, seed_left=seed_l, seed_right=seed_r)
            if ik is None:
                self._emit_status(
                    f'IK failed for visit {i + 1} ({v.label or "unlabeled"}) '
                    f'y={v.y:.3f} z={v.z:.3f}; aborting')
                return False
            visit_iks.append(ik)
            seed_r, seed_l = ik
            visit_fks.append(self._call_fk(seed_r, seed_l) or (0.0, v.y, v.z))

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
        # The home baseline row exists from the original attempt when
        # resuming; capture it only on a fresh run.
        if request.start_cycle <= 1:
            self._capture_observations(
                request, writer,
                phase='home', cycle=0, target_idx=None,
                theta_right=initial_theta_r, theta_left=initial_theta_l)

        # Visit-driven loop. Motion-only visits (Target.capture=False,
        # e.g. backlash approach pre-points) move and settle but skip
        # the capture, the FK row, and any return-to-initial.
        total_iterations = request.test.num_cycles
        captured = skip_captured
        for i in range(start_idx, len(visits)):
            if self._stop_event.is_set():
                return False
            v = visits[i]
            theta_r, theta_l = visit_iks[i - start_idx]
            fk_xyz = visit_fks[i - start_idx]
            kind = 'goal' if v.capture else 'via'
            self._emit_status(
                f'visit {i + 1}/{len(visits)} ({kind} {v.label}): '
                f'moving to y={v.y:.3f} z={v.z:.3f}')
            if not self._send_and_wait(
                    request.joint_names, theta_r, theta_l,
                    request.trajectory_duration):
                self._emit_status('visit trajectory failed; aborting run')
                return False
            if not self._settle(request.test.settle_time, kind):
                return False
            if not v.capture:
                continue

            # Repeatability / accuracy tests are hands-off (the gate
            # is just to wait for a fresh detection); workspace-
            # coverage runs without an operator gate too. The
            # awaiting-continue + auto-continue plumbing in the
            # dashboard remains for backwards compatibility but no
            # longer blocks: capture is gated only on a freshly
            # progressed TF stamp post-settle.
            # Clear the continue event BEFORE emitting: a headless
            # awaiting_continue_cb calls proceed() synchronously from
            # this same thread, and clearing inside the wait would eat
            # that proceed and deadlock the run.
            self._continue_event.clear()
            self._emit_awaiting_continue(cycle_of[i], total_iterations)
            if not self._wait_for_continue(request):
                # Either cancelled or the runner was shut down.
                return False

            self._capture_observations(
                request, writer,
                phase='target', cycle=cycle_of[i], target_idx=tidx_of[i],
                theta_right=theta_r, theta_left=theta_l,
                label=v.label, approach=v.approach)
            writer.add_fk({
                'cycle': cycle_of[i],
                'target_idx': tidx_of[i],
                'theta_right': theta_r,
                'theta_left': theta_l,
                'fk_x': fk_xyz[0],
                'fk_y': fk_xyz[1],
                'fk_z': fk_xyz[2],
                'label': v.label,
                'approach': v.approach,
            })
            captured += 1
            self._emit_progress(captured, total_captures)

            # Single-pose tests return to the initial pose between
            # visits so each visit starts from a known state and
            # the run ends with the arm parked at the operator-
            # chosen initial. Sweep-style tests skip this so they can
            # walk the envelope without doubling back on every goal.
            if request.test.return_to_initial_between_visits:
                self._emit_status(
                    f'visit {i + 1}/{len(visits)}: returning to initial')
                if not self._send_and_wait(
                        request.joint_names,
                        initial_theta_r, initial_theta_l,
                        request.trajectory_duration):
                    self._emit_status(
                        'return-to-initial move failed; aborting run')
                    return False
                if request.test.verify_home_with_tag:
                    if not self._wait_for_home_confirmed(
                            request, label=f'cycle {cycle_of[i]}'):
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

    def _emit_home_finished(self, ok: bool, message: str):
        if self.home_finished_cb:
            self.home_finished_cb(ok, message)

    def _home_worker(self):
        """Background worker for ``home``: call the Trigger service and
        poll its future, honouring _stop_event so Cancel aborts the wait.
        The hardware seek blocks up to ~30 s, so allow a 35 s deadline."""
        if not self._home_client.wait_for_service(timeout_sec=5.0):
            self._emit_status(
                'cannot home: /volcaniarm_hardware_interface/home unavailable')
            self._emit_home_finished(False, 'home service unavailable')
            return
        self._emit_status('homing: seeking limit switches (up to ~30 s)...')
        future = self._home_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 35.0
        while not future.done() and time.monotonic() < deadline:
            if self._stop_event.is_set():
                self._emit_status('homing canceled')
                self._emit_home_finished(False, 'canceled')
                return
            time.sleep(0.05)
        if not future.done():
            self._emit_status('homing timed out')
            self._emit_home_finished(False, 'timed out')
            return
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self._emit_status(f'homing failed: {exc}')
            self._emit_home_finished(False, str(exc))
            return
        if resp.success:
            self._emit_status('homing completed')
        else:
            self._emit_status(f'homing failed: {resp.message}')
        self._emit_home_finished(bool(resp.success), resp.message or '')

    def check_reachable(self, y: float, z: float) -> Tuple[bool, str]:
        """Cheap reachability probe for the dashboard's goal guard.

        Runs the same in-process IK the run loop resolves goals with;
        no motion, no ROS round-trip, safe to call from the Qt thread
        on every spinbox change. Returns (ok, reason).
        """
        ik = self._call_ik(y, z)
        if ik is None:
            return False, f'unreachable: no IK solution for ({y:.3f}, {z:.3f})'
        # URDF elbow limits are +/-3.14 rad; IK closure is the real
        # constraint, but bound-check anyway so a wrapped branch can
        # never slip through to the controller.
        theta_r, theta_l = ik
        if abs(theta_r) > 3.14 or abs(theta_l) > 3.14:
            return False, (f'IK solution outside joint limits '
                           f'(right={theta_r:.2f}, left={theta_l:.2f} rad)')
        return True, ''

    # -- ROS plumbing ---------------------------------------------

    def _wait_for_clients(self, timeout_s: float = 5.0) -> bool:
        # IK/FK are in-process now; only the trajectory action needs waiting.
        return self._action_client.wait_for_server(timeout_sec=timeout_s)

    def _call_ik(self, y: float, z: float,
                 seed_left: float = 0.0, seed_right: float = 0.0):
        # Returns (theta1=right, theta2=left) to match the prior service API.
        # Seeds keep the IK branch continuous along a sweep (each call
        # seeded with the previous solution); (0, 0) reproduces the
        # historical behaviour for one-shot calls.
        ik = vk.inverse_ee(self._kin, y, z, seed_left, seed_right)
        if not ik.valid:
            return None
        return float(ik.theta_right), float(ik.theta_left)

    def _call_fk(self, theta_right: float, theta_left: float):
        ee = vk.forward_ee(self._kin, theta_left, theta_right)
        if not ee.valid:
            return None
        return float(ee.x), float(ee.y), float(ee.z)

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

        The caller clears ``_continue_event`` before emitting
        awaiting_continue (NOT here): a headless callback proceeds
        synchronously from the worker thread, so clearing after the
        emit would discard that proceed and block forever.
        """
        # Reset the detection indicator so the dashboard starts in the
        # "no detection yet" state on each gate entry.
        self._emit_detection_state(False, 0.0)
        last_stamp_ns: Optional[int] = None
        last_change_mono: Optional[float] = None
        fresh_window_s = max(request.detection_max_age_s, 0.1)
        while not self._continue_event.wait(0.1):
            if self._stop_event.is_set():
                return False
            base_age, ee_age = self._tag_ages(request)
            tf, _ = self._lookup_base_to_ee(request, 0.0)
            now_mono = time.monotonic()
            if tf is None:
                self._emit_detection_state(False, 0.0, base_age, ee_age)
                continue
            stamp_ns = (tf.header.stamp.sec * 1_000_000_000
                        + tf.header.stamp.nanosec)
            if last_stamp_ns is None or stamp_ns != last_stamp_ns:
                last_stamp_ns = stamp_ns
                last_change_mono = now_mono
            age_since_new_stamp = now_mono - (last_change_mono or now_mono)
            self._emit_detection_state(
                age_since_new_stamp < fresh_window_s, age_since_new_stamp,
                base_age, ee_age)
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

    def _lookup_urdf_mounts(self, request: RunRequest) -> Optional[dict]:
        """Snapshot the static URDF apriltag mount transforms for
        config.yaml. All three are static (robot_state_publisher), so a
        short timeout suffices; returns None when any lookup fails so a
        half-recorded marker never masquerades as a full one."""
        def _tf(parent, child):
            tf = self._tf_buffer.lookup_transform(
                parent, child, RclpyTime(),
                timeout=RclpyDuration(seconds=2.0))
            t = tf.transform.translation
            r = tf.transform.rotation
            return ([float(t.x), float(t.y), float(t.z)],
                    [float(r.x), float(r.y), float(r.z), float(r.w)])
        try:
            base_xyz, base_quat = _tf(
                'volcaniarm_base_link', request.base_urdf_frame)
            ee_xyz, ee_quat = _tf(
                'right_arm_tip_link', request.ee_urdf_frame)
            _, world_quat = _tf(
                request.world_frame, 'volcaniarm_base_link')
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(
                f'urdf mount snapshot unavailable: {exc}')
            return None
        return {
            'base_xyz': base_xyz, 'base_quat': base_quat,
            'ee_xyz': ee_xyz, 'ee_quat': ee_quat,
            'base_link_world_quat': world_quat,
        }

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
            # Keep the per-tag trackers warm so a timeout can report
            # which marker went stale.
            self._tag_ages(request)
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
                              theta_right: float, theta_left: float,
                              label: str = '', approach: str = ''):
        """Capture ``request.samples_per_capture`` detections here.

        Default is exactly one, preserving the historical behaviour:
        multi-sample averaging was dropped for the accuracy tests
        because the arm is stationary by the time we read, so repeated
        lookups recorded the detector pixel noise floor without
        measuring anything that varied. The noise_gate and
        settle_probe tests re-enable bursts deliberately -- the former
        to *characterize* that noise floor, the latter because its
        timestamped samples trace pose vs time after arrival.

        Every sample is individually gated on TF stamp progression (a
        detection whose stamp differs from the previous one), so N
        samples are N distinct detections, never N reads of one
        buffered transform. A missing FIRST sample at a target pose
        aborts the run (we needed a measurement here); losing
        detection mid-burst logs how many samples landed and moves on.
        """
        if self._stop_event.is_set():
            return
        n = max(1, int(request.samples_per_capture))
        got = 0
        for sample_idx in range(1, n + 1):
            if self._stop_event.is_set():
                return
            ok = self._capture_one_sample(
                request, writer, phase, cycle, target_idx,
                theta_right, theta_left, label, approach,
                sample_idx, verbose=(sample_idx == 1 or sample_idx % 100 == 0))
            if not ok:
                if sample_idx > 1:
                    self._emit_status(
                        f'{phase} capture: burst ended early at sample '
                        f'{sample_idx}/{n} ({got} recorded)')
                return
            got += 1
            if request.sample_min_period_s > 0 and sample_idx < n:
                if self._stop_event.wait(request.sample_min_period_s):
                    return
        if n > 1:
            self._emit_status(
                f'{phase} capture (cycle={cycle}, target_idx={target_idx}): '
                f'{got}/{n} samples recorded')

    def _capture_one_sample(self, request: RunRequest, writer: RunWriter,
                            phase: str, cycle: int,
                            target_idx: Optional[int],
                            theta_right: float, theta_left: float,
                            label: str, approach: str,
                            sample_idx: int, verbose: bool) -> bool:
        """One stamp-gated detection -> one CSV row.

        Returns False when no row was written. First-sample failures
        at a 'target' phase set the failure reason and stop the run
        (matching the historical single-sample semantics); later
        samples fail soft so a burst that loses the tag partway keeps
        what it has.

        Each row stores the raw apriltag base->ee transform (for tag-
        frame diagnostics), the world-frame Y-Z origins of both the
        detected and URDF base/EE markers (so the analysis can plot
        clusters in a single consistent frame), and the headline
        scalars ``d_detected``, ``d_urdf`` and the signed difference
        ``d_error``, computed in the world frame so Y and Z refer to
        the same physical axes for detection and URDF.
        """
        det = self._wait_for_fresh_detection(
            request, request.detection_timeout_s)
        if det is None:
            # Name which marker went stale: one tag old + one fresh is a
            # visibility problem at this pose; both old together is a
            # stream (USB / DDS buffer) problem.
            reason = (f'no fresh detection '
                      f'(timeout={request.detection_timeout_s:.1f} s; '
                      f'{self._tag_ages_text(request)})')
            self._emit_status(
                f'{phase} capture: {reason} (cycle={cycle}, '
                f'target_idx={target_idx}, sample={sample_idx})')
            # First sample at a 'target' phase: the run cannot recover
            # -- we needed a measurement at this pose. 'home' baseline
            # missing is logged but not fatal (the analysis still has
            # the goal rows).
            if phase == 'target' and sample_idx == 1:
                self._failure_reason = (
                    f'detection lost during sampling at cycle={cycle}, '
                    f'target_idx={target_idx}: {reason}')
                self._emit_status(f'aborting: {self._failure_reason}')
                self._stop_event.set()
            return False
        d_detected, det_origins, det_reason = self._yz_segment_world(
            request, request.base_tag_frame, request.ee_tag_frame, 0.1)
        d_urdf, urdf_origins, urdf_reason = self._yz_segment_world(
            request, request.base_urdf_frame, request.ee_urdf_frame, 0.1)
        if d_detected is None:
            self._emit_status(
                f'{phase} capture: detected segment unavailable in '
                f'world frame ({det_reason})')
            if phase == 'target' and sample_idx == 1:
                self._failure_reason = (
                    f'world-frame detection lookup failed at cycle={cycle}, '
                    f'target_idx={target_idx}: {det_reason}')
                self._stop_event.set()
            return False
        if d_urdf is None:
            d_urdf_val = float('nan')
            d_error = float('nan')
            urdf_origins = (float('nan'),) * 4
            if verbose:
                self._emit_status(
                    f'{phase} capture: URDF segment unavailable in world '
                    f'frame ({urdf_reason}); logging detection only')
        else:
            d_urdf_val = d_urdf
            d_error = d_detected - d_urdf
        det_base_y, det_base_z, det_ee_y, det_ee_z = det_origins
        urdf_base_y, urdf_base_z, urdf_ee_y, urdf_ee_z = urdf_origins
        # Tip orientation at capture (world->right_arm_tip_link),
        # recorded as per-row provenance for diagnostics. Best effort;
        # NaN keeps the row usable for everything else.
        tip_q = (float('nan'),) * 4
        tip_tf, _ = self._lookup_origin_in_world(
            request, 'right_arm_tip_link', 0.1)
        if tip_tf is not None:
            tq = tip_tf.transform.rotation
            tip_q = (tq.x, tq.y, tq.z, tq.w)
        t = det.transform.translation
        r = det.transform.rotation
        stamp = RclpyTime.from_msg(det.header.stamp)
        writer.add_tag_observation({
            'phase': phase,
            'cycle': cycle,
            'target_idx': '' if target_idx is None else target_idx,
            'sample_idx': sample_idx,
            'label': label,
            'approach': approach,
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
            'tip_qx': tip_q[0], 'tip_qy': tip_q[1],
            'tip_qz': tip_q[2], 'tip_qw': tip_q[3],
        })
        if verbose:
            if math.isnan(d_error):
                self._emit_status(
                    f'{phase} capture (cycle={cycle}, '
                    f'target_idx={target_idx}, sample={sample_idx}): '
                    f'd_detected={d_detected * 1000:.1f} mm  d_urdf=n/a')
            else:
                self._emit_status(
                    f'{phase} capture (cycle={cycle}, '
                    f'target_idx={target_idx}, sample={sample_idx}): '
                    f'd_detected={d_detected * 1000:.1f} mm  '
                    f'd_urdf={d_urdf_val * 1000:.1f} mm  '
                    f'd_error={d_error * 1000:+.1f} mm')
        return True
