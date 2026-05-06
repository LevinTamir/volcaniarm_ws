"""Automated camera-calibration sweep using easy_handeye2.

Drives the arm through a curated list of safe workspace poses (loaded
from a YAML config), programmatically samples easy_handeye2's server,
computes + saves the eye-on-base calibration, and parks the arm at
home. Operator stays in control via the dashboard's Cancel button,
which sets the same `_stop_event` used by the test runner.

Composition over inheritance: this class holds a ``CalibrationRunner``
reference and reuses its IK / trajectory / TF primitives rather than
duplicating motion code. The two cannot run concurrently -- the
busy-check guards both threads.
"""

from __future__ import annotations

import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import yaml

from .calibration_runner import CalibrationRunner


# Conservative envelope. Tighter than the kinematics service's hard
# limits (y in [-0.4, 0.4], z in [0.1, 0.9]) to keep margin from
# singularities. Operator edits the YAML; the runner enforces this box
# regardless, so a typo can't drive the arm into the workspace edge.
SAFE_Y_MIN = -0.30
SAFE_Y_MAX = 0.30
SAFE_Z_MIN = 0.35
SAFE_Z_MAX = 0.65

# easy_handeye2 storage location. Mirrored from
# easy_handeye2/__init__.py:CALIBRATIONS_DIRECTORY.
HANDEYE_CALIBRATIONS_DIR = Path('~/.ros2/easy_handeye2/calibrations').expanduser()
HANDEYE_NAME = 'volcaniarm_calibration'

# Frame configuration matching volcaniarm_calibration/launch/calibration.launch.py.
HANDEYE_CALIBRATION_TYPE = 'eye_on_base'
HANDEYE_ROBOT_BASE_FRAME = 'volcaniarm_base_link'
HANDEYE_ROBOT_EFFECTOR_FRAME = 'right_arm_tip_link'
HANDEYE_TRACKING_BASE_FRAME = 'camera_color_optical_frame'
HANDEYE_TRACKING_MARKER_FRAME = 'apriltag_marker_ee'

# Time the operator (or auto-publish logic) needs the publisher down
# before re-launching it. The handeye_server we spawn for sampling
# would otherwise fight a stale publisher subprocess for the same TF.
SERVER_STARTUP_GRACE_S = 3.0


StatusCb = Callable[[str], None]
ProgressCb = Callable[[int, int], None]
# Fired when the camera calibration finishes. status is one of
# 'completed', 'canceled', 'failed'. calib_path is the saved .calib
# file (None on failure / cancel). reason is the failure_reason if
# any, otherwise empty string.
FinishedCb = Callable[[str, Optional[Path], str], None]


@dataclass
class _PoseEntry:
    y: float
    z: float
    theta_right: float
    theta_left: float
    comment: str


class CameraCalibrationRunner:
    """Automated eye-on-base hand-eye calibration sweep."""

    def __init__(self, runner: CalibrationRunner):
        self._runner = runner
        self._node = runner.node
        self._thread: Optional[threading.Thread] = None
        self._server_proc: Optional[subprocess.Popen] = None
        self._client = None  # easy_handeye2 HandeyeClient, set in _run

        self.status_cb: Optional[StatusCb] = None
        self.progress_cb: Optional[ProgressCb] = None
        self.finished_cb: Optional[FinishedCb] = None

    # -- public control surface --------------------------------------

    def is_busy(self) -> bool:
        if self._thread is not None and self._thread.is_alive():
            return True
        # Tests and calibration share the runner's _stop_event and
        # motion primitives; refuse to start while a test is running.
        rt = self._runner._run_thread  # noqa: SLF001
        if rt is not None and rt.is_alive():
            return True
        return False

    def request(self, yaml_path: Path) -> bool:
        """Kick off a calibration sweep. Returns False if busy."""
        if self.is_busy():
            self._emit_status('busy: another run is in progress')
            return False
        self._runner._stop_event.clear()  # noqa: SLF001
        self._runner._failure_reason = None  # noqa: SLF001
        self._thread = threading.Thread(
            target=self._run, args=(Path(yaml_path),), daemon=True)
        self._thread.start()
        return True

    def cancel(self):
        """Stop the active sweep at the next checkpoint.

        Sets the runner's `_stop_event`, which is polled inside motion
        primitives and at our own loop boundaries. Any in-flight
        trajectory goal is also cancelled at the action server.
        """
        self._runner._stop_event.set()  # noqa: SLF001

    def shutdown(self):
        self.cancel()
        if self._thread is not None:
            self._thread.join(timeout=10.0)
        self._stop_server()

    # -- emit helpers ------------------------------------------------

    def _emit_status(self, msg: str):
        self._node.get_logger().info(msg)
        if self.status_cb:
            self.status_cb(msg)

    def _emit_progress(self, current: int, total: int):
        if self.progress_cb:
            self.progress_cb(current, total)

    def _emit_finished(self, status: str, calib_path: Optional[Path], reason: str):
        if self.finished_cb:
            self.finished_cb(status, calib_path, reason)

    # -- main worker --------------------------------------------------

    def _run(self, yaml_path: Path):
        try:
            cfg = self._load_yaml(yaml_path)
        except Exception as exc:
            self._emit_status(f'failed to load {yaml_path}: {exc}')
            self._emit_finished('failed', None, f'yaml load: {exc}')
            return

        poses_raw = cfg.get('poses') or []
        trajectory_duration = float(cfg.get('trajectory_duration', 2.5))
        settle_time = float(cfg.get('settle_time', 1.5))
        detection_timeout_s = float(cfg.get('detection_timeout_s', 2.0))
        detection_max_age_s = float(cfg.get('detection_max_age_s', 0.5))
        min_successful_samples = int(cfg.get('min_successful_samples', 7))

        # Validate envelope + pre-resolve IK for every pose. Failures
        # are dropped (with warn) rather than aborting; the run only
        # aborts if too few survive.
        poses = self._validate_and_resolve(poses_raw)
        if len(poses) < min_successful_samples:
            reason = (f'only {len(poses)} pose(s) reachable; need at '
                      f'least {min_successful_samples}')
            self._emit_status(f'aborting: {reason}')
            self._emit_finished('failed', None, reason)
            return

        # Spawn the handeye_server. We don't include the rqt calibrator
        # (the operator drives via our dashboard, not its UI).
        if not self._start_server():
            self._emit_finished('failed', None, 'handeye_server failed to spawn')
            return

        try:
            # Build the client. Constructor blocks on wait_for_service
            # for every server-side service, so this returns only once
            # the server is fully up. The dashboard host's executor
            # services the calls because `self._node` is the same node
            # the rqt plugin spins.
            from easy_handeye2.handeye_client import HandeyeClient
            from easy_handeye2.handeye_calibration import HandeyeCalibrationParameters

            self._emit_status('connecting to handeye_server...')
            params = HandeyeCalibrationParameters(
                name=HANDEYE_NAME,
                calibration_type=HANDEYE_CALIBRATION_TYPE,
                tracking_base_frame=HANDEYE_TRACKING_BASE_FRAME,
                tracking_marker_frame=HANDEYE_TRACKING_MARKER_FRAME,
                robot_base_frame=HANDEYE_ROBOT_BASE_FRAME,
                robot_effector_frame=HANDEYE_ROBOT_EFFECTOR_FRAME,
                freehand_robot_movement=True,
            )
            self._client = HandeyeClient(self._node, params)
            self._emit_status('handeye_server ready')

            successful = self._sweep_poses(
                poses, trajectory_duration, settle_time,
                detection_timeout_s, detection_max_age_s)

            if self._runner._stop_event.is_set():  # noqa: SLF001
                self._emit_status('calibration canceled by operator')
                self._emit_finished('canceled', None, '')
                return

            if successful < min_successful_samples:
                reason = (f'only {successful} pose(s) sampled; need at '
                          f'least {min_successful_samples}. Tag visibility?')
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return

            self._emit_status(f'computing calibration from {successful} samples')
            compute_resp = self._client.compute_calibration()
            if not compute_resp.valid:
                reason = 'compute_calibration returned invalid'
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return

            self._emit_status('saving calibration')
            save_resp = self._client.save()
            if not save_resp.success:
                reason = 'save_calibration returned failure'
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return

            calib_path = HANDEYE_CALIBRATIONS_DIR / f'{HANDEYE_NAME}.calib'
            if not calib_path.exists():
                reason = f'expected .calib at {calib_path}, not found'
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return

            self._emit_status(f'saved: {calib_path}')

            # Park the arm at home (theta=0, 0). This uses the same
            # _send_and_wait the test runner already uses; if it
            # fails, we still report 'completed' since the calibration
            # data is good -- parking is UX.
            self._emit_status('parking arm at home (theta=0, 0)')
            ok = self._runner._send_and_wait(  # noqa: SLF001
                self._runner._default_joint_names,  # noqa: SLF001
                0.0, 0.0,
                self._runner._default_trajectory_duration)  # noqa: SLF001
            if not ok:
                self._emit_status('park-at-home move did not complete cleanly')

            self._emit_finished('completed', calib_path, '')

        except Exception as exc:
            self._node.get_logger().error(f'camera calibration crashed: {exc}')
            self._emit_status(f'failed: {exc}')
            self._emit_finished('failed', None, f'exception: {exc}')
        finally:
            self._stop_server()
            self._client = None

    # -- pose validation + sweep -------------------------------------

    def _validate_and_resolve(self, poses_raw: list) -> list:
        out: list = []
        for i, p in enumerate(poses_raw, start=1):
            try:
                y = float(p['y'])
                z = float(p['z'])
            except (KeyError, TypeError, ValueError):
                self._emit_status(
                    f'pose {i}: malformed entry {p!r}; skipping')
                continue
            comment = str(p.get('comment', ''))
            if not (SAFE_Y_MIN <= y <= SAFE_Y_MAX
                    and SAFE_Z_MIN <= z <= SAFE_Z_MAX):
                self._emit_status(
                    f'pose {i} ({y:.3f}, {z:.3f}): out of safe envelope '
                    f'[{SAFE_Y_MIN}, {SAFE_Y_MAX}] x '
                    f'[{SAFE_Z_MIN}, {SAFE_Z_MAX}]; skipping')
                continue
            ik = self._runner._call_ik(y, z)  # noqa: SLF001
            if ik is None:
                self._emit_status(
                    f'pose {i} ({y:.3f}, {z:.3f}): IK failed; skipping')
                continue
            out.append(_PoseEntry(y=y, z=z,
                                  theta_right=ik[0], theta_left=ik[1],
                                  comment=comment))
        self._emit_status(
            f'{len(out)}/{len(poses_raw)} poses validated and IK-resolved')
        return out

    def _sweep_poses(self, poses: list, trajectory_duration: float,
                     settle_time: float, detection_timeout_s: float,
                     detection_max_age_s: float) -> int:
        """Drive the arm through each pose; sample at each. Returns
        the count of successfully captured samples."""
        successful = 0
        total = len(poses)
        for idx, p in enumerate(poses, start=1):
            if self._runner._stop_event.is_set():  # noqa: SLF001
                return successful
            self._emit_status(
                f'calibrating: pose {idx}/{total} '
                f'({p.y:.3f}, {p.z:.3f}) {p.comment}')
            ok = self._runner._send_and_wait(  # noqa: SLF001
                self._runner._default_joint_names,  # noqa: SLF001
                p.theta_right, p.theta_left, trajectory_duration)
            if not ok:
                if self._runner._stop_event.is_set():  # noqa: SLF001
                    return successful
                self._runner._failure_reason = (  # noqa: SLF001
                    f'motion failed at pose {idx} '
                    f'({p.y:.3f}, {p.z:.3f})')
                self._emit_status(
                    f'aborting: {self._runner._failure_reason}')  # noqa: SLF001
                self._runner._stop_event.set()  # noqa: SLF001
                return successful
            # Settle in pieces so Cancel responds quickly.
            settled = self._sleep_with_cancel(settle_time)
            if not settled:
                return successful
            # Wait for a fresh tag detection before sampling. easy_handeye2
            # itself reads the same TF buffer when we call take_sample(),
            # but we explicitly gate on freshness here so we can warn-
            # and-skip a pose where the tag isn't visible (rather than
            # have easy_handeye2 silently sample a stale TF).
            if not self._wait_fresh_detection(
                    detection_timeout_s, detection_max_age_s):
                self._emit_status(
                    f'pose {idx}: tag not visible; skipping sample')
                self._emit_progress(idx, total)
                continue
            sample_count_before = self._safe_sample_count()
            try:
                self._client.take_sample()
            except Exception as exc:
                self._emit_status(
                    f'pose {idx}: take_sample raised {exc}; skipping')
                self._emit_progress(idx, total)
                continue
            sample_count_after = self._safe_sample_count()
            if sample_count_after <= sample_count_before:
                self._emit_status(
                    f'pose {idx}: sample not registered '
                    f'(count {sample_count_before} -> {sample_count_after}); '
                    f'skipping')
                self._emit_progress(idx, total)
                continue
            successful += 1
            self._emit_progress(idx, total)
        return successful

    def _safe_sample_count(self) -> int:
        try:
            samples = self._client.get_sample_list()
            return len(samples.samples)
        except Exception:
            return -1

    def _wait_fresh_detection(self, timeout_s: float,
                              max_age_s: float) -> bool:
        """Block up to ``timeout_s`` for the apriltag base->ee TF to
        become fresh. Mirrors the freshness-by-stamp-progression check
        the runner uses in ``_wait_for_continue``: the detection is
        fresh while the most recent stamp change happened within
        ``max_age_s`` monotonic seconds, regardless of clock topology.

        We use the apriltag tag-to-tag TF (apriltag_marker_base ->
        apriltag_marker_ee) as the freshness probe because that's the
        signal that says "we're seeing both tags right now". easy_handeye2
        will subsequently sample its own TF chain when ``take_sample()``
        is called.
        """
        deadline = time.monotonic() + timeout_s
        last_stamp_ns: Optional[int] = None
        last_change_mono: Optional[float] = None
        while time.monotonic() < deadline:
            if self._runner._stop_event.is_set():  # noqa: SLF001
                return False
            tf = self._lookup_apriltag_tf()
            now_mono = time.monotonic()
            if tf is None:
                time.sleep(0.05)
                continue
            stamp_ns = (tf.header.stamp.sec * 1_000_000_000
                        + tf.header.stamp.nanosec)
            if last_stamp_ns is None or stamp_ns != last_stamp_ns:
                last_stamp_ns = stamp_ns
                last_change_mono = now_mono
            age = now_mono - (last_change_mono or now_mono)
            if age < max_age_s:
                return True
            time.sleep(0.05)
        return False

    def _lookup_apriltag_tf(self):
        """Look up apriltag_marker_base -> apriltag_marker_ee, no wait.

        Uses the runner's TF buffer directly so we don't need to fake a
        RunRequest. Returns the TransformStamped or None.
        """
        from rclpy.time import Time as RclpyTime
        from rclpy.duration import Duration as RclpyDuration
        try:
            return self._runner._tf_buffer.lookup_transform(  # noqa: SLF001
                'apriltag_marker_base', 'apriltag_marker_ee',
                RclpyTime(),
                timeout=RclpyDuration(seconds=0.0))
        except Exception:
            return None

    def _sleep_with_cancel(self, seconds: float) -> bool:
        if seconds <= 0:
            return True
        return not self._runner._stop_event.wait(seconds)  # noqa: SLF001

    # -- subprocess + YAML -------------------------------------------

    def _load_yaml(self, path: Path) -> dict:
        with path.open() as f:
            return yaml.safe_load(f) or {}

    def _start_server(self) -> bool:
        if self._server_proc is not None and self._server_proc.poll() is None:
            return True
        cmd = [
            'ros2', 'run', 'easy_handeye2', 'handeye_server',
            '--ros-args',
            '-p', f'name:={HANDEYE_NAME}',
            '-p', f'calibration_type:={HANDEYE_CALIBRATION_TYPE}',
            '-p', f'tracking_base_frame:={HANDEYE_TRACKING_BASE_FRAME}',
            '-p', f'tracking_marker_frame:={HANDEYE_TRACKING_MARKER_FRAME}',
            '-p', f'robot_base_frame:={HANDEYE_ROBOT_BASE_FRAME}',
            '-p', f'robot_effector_frame:={HANDEYE_ROBOT_EFFECTOR_FRAME}',
            '-p', 'freehand_robot_movement:=true',
        ]
        try:
            self._emit_status('spawning easy_handeye2 server')
            self._server_proc = subprocess.Popen(cmd)
        except OSError as exc:
            self._emit_status(f'spawn failed: {exc}')
            return False
        # Give the server a head start so HandeyeClient's wait_for_service
        # doesn't busy-wait against a not-yet-listening socket.
        time.sleep(SERVER_STARTUP_GRACE_S)
        if self._server_proc.poll() is not None:
            self._emit_status('handeye_server exited during startup')
            self._server_proc = None
            return False
        return True

    def _stop_server(self):
        if self._server_proc is None:
            return
        try:
            self._server_proc.terminate()
            try:
                self._server_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._server_proc.kill()
                self._server_proc.wait(timeout=2.0)
        except OSError:
            pass
        finally:
            self._server_proc = None
