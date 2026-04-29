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
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

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
from .tests import BaseTest, Target


@dataclass
class RunRequest:
    """User-supplied parameters for a single calibration run."""
    test: BaseTest
    output_root: Path
    auto_approve: bool = False
    approval_timeout_s: float = 60.0
    joint_names: tuple = (
        'volcaniarm_right_elbow_joint', 'volcaniarm_left_elbow_joint',
    )
    home_position: tuple = (0.0, 0.0)
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


@dataclass
class PreviewPayload:
    """Snapshot of the upcoming visit, surfaced to the dashboard.

    The dashboard renders the marker preview and waits for the user to
    approve before the runner sends the trajectory.
    """
    cycle: int
    cycle_total: int
    target_idx: int
    target_total: int
    target: Target
    theta_right: float
    theta_left: float
    fk_xyz: tuple = field(default=(0.0, 0.0, 0.0))
    joint_path_xyz: list = field(default_factory=list)


# Callback signatures used by the runner. The widget wires Qt signals.
StatusCb = Callable[[str], None]
PreviewCb = Callable[[PreviewPayload], None]
ProgressCb = Callable[[int, int], None]
FinishedCb = Callable[[Path, str], None]


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
        self._stop_event = threading.Event()
        self._approval_event = threading.Event()
        self._approval_decision: bool = False
        self._writer: Optional[RunWriter] = None

        self.status_cb: Optional[StatusCb] = None
        self.preview_cb: Optional[PreviewCb] = None
        self.progress_cb: Optional[ProgressCb] = None
        self.finished_cb: Optional[FinishedCb] = None

    # -- public control surface ------------------------------------

    def request_run(self, request: RunRequest) -> bool:
        if self._run_thread is not None and self._run_thread.is_alive():
            self._emit_status('busy: another run is in progress')
            return False
        self._stop_event.clear()
        self._approval_event.clear()
        self._run_thread = threading.Thread(
            target=self._run, args=(request,), daemon=True)
        self._run_thread.start()
        return True

    def approve(self):
        self._approval_decision = True
        self._approval_event.set()

    def reject(self):
        self._approval_decision = False
        self._approval_event.set()

    def cancel(self):
        self._stop_event.set()
        self._approval_event.set()

    def shutdown(self):
        self.cancel()
        if self._run_thread is not None:
            self._run_thread.join(timeout=5.0)

    # -- emit helpers ----------------------------------------------

    def _emit_status(self, msg: str):
        self.node.get_logger().info(msg)
        if self.status_cb:
            self.status_cb(msg)

    def _emit_preview(self, payload: PreviewPayload):
        if self.preview_cb:
            self.preview_cb(payload)

    def _emit_progress(self, current: int, total: int):
        if self.progress_cb:
            self.progress_cb(current, total)

    def _emit_finished(self, path: Path, status: str):
        if self.finished_cb:
            self.finished_cb(path, status)

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
            'targets': request.test.targets,
            'auto_approve': request.auto_approve,
            'joint_names': list(request.joint_names),
            'home_position': list(request.home_position),
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
        total = request.test.total_visits()
        visits = list(request.test.iter_visits())
        per_cycle = len(request.test.targets)

        # Always start at the home pose before any test movement.
        self._emit_status(
            f'homing to theta=({request.home_position[0]:.3f}, '
            f'{request.home_position[1]:.3f})')
        if not self._send_and_wait(
            request,
            request.home_position[0], request.home_position[1],
            request.trajectory_duration):
            self._emit_status('initial home move failed; aborting run')
            return

        # Settle, then snapshot the base->ee apriltag transform as the
        # home reference (cycle 0 / no target_idx).
        if not self._settle(request.test.settle_time, 'home'):
            return
        self._capture_observations(
            request, writer,
            phase='home', cycle=0, target_idx=None,
            theta_right=request.home_position[0],
            theta_left=request.home_position[1])

        for visit_idx, target in enumerate(visits):
            if self._stop_event.is_set():
                return
            cycle = visit_idx // per_cycle
            in_cycle_idx = visit_idx % per_cycle

            self._emit_status(
                f'planning cycle {cycle + 1}/{request.test.num_cycles}, '
                f'target {in_cycle_idx + 1}/{per_cycle}: '
                f'y={target.y:.3f} z={target.z:.3f}')

            ik = self._call_ik(target.y, target.z)
            if ik is None:
                self._emit_status(f'IK failed at target {target.label}, skipping')
                continue
            theta_r, theta_l = ik

            fk_xyz = self._call_fk(theta_r, theta_l) or (0.0, 0.0, 0.0)
            joint_path = self._sweep_fk_path(
                request.home_position[0], request.home_position[1],
                theta_r, theta_l, samples=10)
            payload = PreviewPayload(
                cycle=cycle + 1,
                cycle_total=request.test.num_cycles,
                target_idx=in_cycle_idx + 1,
                target_total=per_cycle,
                target=target,
                theta_right=theta_r,
                theta_left=theta_l,
                fk_xyz=fk_xyz,
                joint_path_xyz=joint_path,
            )
            self._emit_preview(payload)

            if not self._wait_for_approval(request):
                self._emit_status('move not approved, skipping')
                continue

            if not self._send_and_wait(
                request, theta_r, theta_l, request.trajectory_duration):
                self._emit_status('trajectory failed, skipping target')
                continue

            if not self._settle(request.test.settle_time, 'target'):
                return

            self._capture_observations(
                request, writer,
                phase='target', cycle=cycle + 1,
                target_idx=in_cycle_idx + 1,
                theta_right=theta_r, theta_left=theta_l)
            writer.add_fk({
                'cycle': cycle + 1,
                'target_idx': in_cycle_idx + 1,
                'theta_right': theta_r,
                'theta_left': theta_l,
                'fk_x': fk_xyz[0], 'fk_y': fk_xyz[1], 'fk_z': fk_xyz[2],
            })
            self._emit_progress(visit_idx + 1, total)

            # Always return home after every visit (including the last)
            # so each cycle is home -> target -> measure -> home and the
            # run ends with the arm at the home pose.
            self._emit_status('returning home')
            if not self._send_and_wait(
                request,
                request.home_position[0], request.home_position[1],
                request.trajectory_duration):
                self._emit_status('home move failed; aborting run')
                return

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

    def _sweep_fk_path(self, theta_r0: float, theta_l0: float,
                       theta_r1: float, theta_l1: float,
                       samples: int = 10) -> list:
        """Linearly interpolate joint-space and return the cartesian path
        from each sampled FK call. Falls back to start+end if FK fails."""
        path: list = []
        for i in range(samples + 1):
            alpha = i / samples
            tr = theta_r0 + alpha * (theta_r1 - theta_r0)
            tl = theta_l0 + alpha * (theta_l1 - theta_l0)
            xyz = self._call_fk(tr, tl)
            if xyz is not None:
                path.append(xyz)
        return path

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

    def _send_and_wait(self, request: RunRequest,
                       theta_right: float, theta_left: float,
                       duration: float) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(request.joint_names)
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

    def _wait_for_approval(self, request: RunRequest) -> bool:
        if request.auto_approve:
            return True
        self._approval_event.clear()
        self._emit_status('waiting for approval (preview shown)')
        approved = self._approval_event.wait(timeout=request.approval_timeout_s)
        if self._stop_event.is_set():
            return False
        if not approved:
            self._emit_status('approval timed out')
            return False
        return self._approval_decision

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

        TF resolves the lookup through the camera (the apriltag detector
        publishes camera->base_tag and camera->ee_tag). Rejects buffer
        entries older than ``detection_max_age_s`` so a tag that briefly
        went undetected does not silently surface a stale pose. The
        caller controls the per-call wait so we don't burn the full
        ``detection_timeout_s`` on every retry inside a sample loop.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                request.base_tag_frame, request.ee_tag_frame,
                RclpyTime(),
                timeout=RclpyDuration(seconds=wait_s))
        except Exception as exc:
            return None, f'lookup failed: {exc}'
        stamp = RclpyTime.from_msg(tf.header.stamp)
        now = self.node.get_clock().now()
        age_s = (now - stamp).nanoseconds * 1e-9
        if age_s > request.detection_max_age_s:
            return None, (f'stale ({age_s:.2f}s > '
                          f'{request.detection_max_age_s:.2f}s)')
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
