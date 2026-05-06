"""Single-shot camera localization from the base AprilTag.

Computes the actual pose of ``camera_color_optical_frame`` in
``volcaniarm_base_link`` frame from one good apriltag_ros detection
of the base tag (ID 5). Compared to easy_handeye2's hand-eye sweep
this is non-invasive: no arm motion, no algorithm sensitive to
rotation diversity, no TF publish that would conflict with the URDF
chain. The result is logged and saved to a YAML record so the
operator can compare measured vs URDF-expected and decide whether to
update the URDF.

Public API matches the previous ``CameraCalibrationRunner`` so the
dashboard wiring is unchanged: ``request``, ``cancel``, ``is_busy``,
``status_cb``, ``progress_cb``, ``finished_cb``, ``shutdown``.
"""

from __future__ import annotations

import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import yaml

from rclpy.duration import Duration as RclpyDuration
from rclpy.time import Time as RclpyTime

from .calibration_runner import CalibrationRunner


# Frames we look up. The URDF chain links the arm root to the rigidly
# mounted tag, and apriltag_ros publishes the camera-to-detected-tag
# pose every frame. The two should reference the same physical point,
# so composing them yields camera-in-arm-base.
ROBOT_BASE_FRAME = 'volcaniarm_base_link'
URDF_TAG_FRAME = 'apriltag_base_link'
DETECTED_TAG_FRAME = 'apriltag_marker_base'
CAMERA_FRAME = 'camera_color_optical_frame'

DEFAULT_DATA_ROOT = (
    Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/data')
    .expanduser())

# Knobs (fixed in code; no GUI exposure per plan).
DEFAULT_NUM_SAMPLES = 10
DEFAULT_SAMPLE_INTERVAL_S = 0.1
DEFAULT_DETECTION_TIMEOUT_S = 5.0
DEFAULT_DETECTION_MAX_AGE_S = 0.5
MIN_SUCCESSFUL_SAMPLES = 6


StatusCb = Callable[[str], None]
ProgressCb = Callable[[int, int], None]
# (status, result_path_or_empty, reason)
FinishedCb = Callable[[str, Optional[Path], str], None]


class CameraCalibrationRunner:
    """Measure camera-in-base from the base AprilTag.

    Composes with ``CalibrationRunner`` to share the TF buffer and the
    operator's ``_stop_event`` so Cancel works mid-measurement.
    """

    def __init__(self, runner: CalibrationRunner):
        self._runner = runner
        self._node = runner.node
        self._thread: Optional[threading.Thread] = None

        self.status_cb: Optional[StatusCb] = None
        self.progress_cb: Optional[ProgressCb] = None
        self.finished_cb: Optional[FinishedCb] = None

    # -- public control surface --------------------------------------

    def is_busy(self) -> bool:
        if self._thread is not None and self._thread.is_alive():
            return True
        rt = self._runner._run_thread  # noqa: SLF001
        if rt is not None and rt.is_alive():
            return True
        return False

    def request(self, _yaml_path: Optional[Path] = None) -> bool:
        """Kick off a localization. The argument is ignored; kept for
        backwards compatibility with the old YAML-driven API so the
        dashboard's call site doesn't change."""
        if self.is_busy():
            self._emit_status('busy: another run is in progress')
            return False
        self._runner._stop_event.clear()  # noqa: SLF001
        self._runner._failure_reason = None  # noqa: SLF001
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def cancel(self):
        self._runner._stop_event.set()  # noqa: SLF001

    def shutdown(self):
        self.cancel()
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    # -- emit helpers ------------------------------------------------

    def _emit_status(self, msg: str):
        self._node.get_logger().info(msg)
        if self.status_cb:
            self.status_cb(msg)

    def _emit_progress(self, current: int, total: int):
        if self.progress_cb:
            self.progress_cb(current, total)

    def _emit_finished(self, status: str, path: Optional[Path], reason: str):
        if self.finished_cb:
            self.finished_cb(status, path, reason)

    # -- main worker -------------------------------------------------

    def _run(self):
        try:
            urdf_tag = self._lookup_static(ROBOT_BASE_FRAME, URDF_TAG_FRAME)
            if urdf_tag is None:
                reason = (f'URDF chain {ROBOT_BASE_FRAME} -> {URDF_TAG_FRAME} '
                          f'is not in the TF tree')
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return

            self._emit_status(
                f'waiting for fresh detection of base tag '
                f'({DETECTED_TAG_FRAME})')
            if not self._wait_fresh_detection(
                    DEFAULT_DETECTION_TIMEOUT_S,
                    DEFAULT_DETECTION_MAX_AGE_S):
                if self._runner._stop_event.is_set():  # noqa: SLF001
                    self._emit_finished('canceled', None, '')
                    return
                reason = (f'no fresh detection of {DETECTED_TAG_FRAME} '
                          f'within {DEFAULT_DETECTION_TIMEOUT_S}s')
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return

            self._emit_status(f'sampling {DEFAULT_NUM_SAMPLES} detections')
            samples = self._collect_samples(
                DEFAULT_NUM_SAMPLES, DEFAULT_SAMPLE_INTERVAL_S)

            if self._runner._stop_event.is_set():  # noqa: SLF001
                self._emit_finished('canceled', None, '')
                return

            if len(samples) < MIN_SUCCESSFUL_SAMPLES:
                reason = (f'only {len(samples)}/{DEFAULT_NUM_SAMPLES} samples '
                          f'succeeded; need at least {MIN_SUCCESSFUL_SAMPLES}')
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return

            measured_xyz, measured_quat = _average_samples(samples)
            stddev_xyz = _xyz_stddev(samples)
            stddev_rpy = _rpy_stddev(samples)

            urdf_xyz, urdf_quat = _xyz_quat_from_transform(urdf_tag.transform)
            # T(robot_base -> camera) = T(robot_base -> tag) @ inv(T(camera -> tag))
            # Each detection sample is already T(robot_base -> camera) (precomposed
            # in _collect_samples), so measured_* IS what we want.
            urdf_camera_xyz, urdf_camera_quat = self._urdf_camera_pose()

            result = {
                'measured': _pose_dict(measured_xyz, measured_quat),
                'stddev': {
                    'xyz': [float(v) for v in stddev_xyz],
                    'rpy_deg': [float(v) for v in stddev_rpy],
                    'samples': len(samples),
                },
                'urdf_expected': (
                    _pose_dict(urdf_camera_xyz, urdf_camera_quat)
                    if urdf_camera_xyz is not None else None),
                'delta': (
                    _delta_dict(measured_xyz, measured_quat,
                                urdf_camera_xyz, urdf_camera_quat)
                    if urdf_camera_xyz is not None else None),
                'frames': {
                    'robot_base': ROBOT_BASE_FRAME,
                    'urdf_tag': URDF_TAG_FRAME,
                    'detected_tag': DETECTED_TAG_FRAME,
                    'camera': CAMERA_FRAME,
                },
                'timestamp': datetime.now().isoformat(),
            }

            self._log_result_block(result)
            result_path = self._save_yaml(result)
            self._emit_status(f'saved: {result_path}')
            self._emit_finished('completed', result_path, '')
        except Exception as exc:
            self._node.get_logger().error(
                f'camera localization crashed: {exc}')
            self._emit_status(f'failed: {exc}')
            self._emit_finished('failed', None, f'exception: {exc}')

    # -- TF + sampling ----------------------------------------------

    def _lookup_static(self, parent: str, child: str):
        """One-shot TF lookup, no wait. Returns the TransformStamped or None."""
        try:
            return self._runner._tf_buffer.lookup_transform(  # noqa: SLF001
                parent, child, RclpyTime(),
                timeout=RclpyDuration(seconds=0.5))
        except Exception:
            return None

    def _wait_fresh_detection(self, timeout_s: float,
                              max_age_s: float) -> bool:
        """Block up to ``timeout_s`` for the camera->base-tag TF to
        become fresh. Freshness is tracked by stamp progression so the
        check is independent of the node's clock topology.
        """
        deadline = time.monotonic() + timeout_s
        last_stamp_ns: Optional[int] = None
        last_change_mono: Optional[float] = None
        while time.monotonic() < deadline:
            if self._runner._stop_event.is_set():  # noqa: SLF001
                return False
            tf = self._lookup_static(CAMERA_FRAME, DETECTED_TAG_FRAME)
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

    def _collect_samples(self, n: int, interval_s: float) -> list:
        """Take up to ``n`` samples of T(robot_base -> camera).

        Each sample composes URDF (robot_base -> urdf_tag) with
        inv(apriltag_ros (camera -> detected_tag)) so the produced
        list is directly the per-sample camera-in-base pose. Skipped
        samples (no fresh detection at that tick) are not added; the
        caller checks the count against MIN_SUCCESSFUL_SAMPLES.
        """
        samples: list = []
        for i in range(n):
            if self._runner._stop_event.is_set():  # noqa: SLF001
                break
            tf_urdf = self._lookup_static(ROBOT_BASE_FRAME, URDF_TAG_FRAME)
            tf_cam = self._lookup_static(CAMERA_FRAME, DETECTED_TAG_FRAME)
            self._emit_progress(i + 1, n)
            if tf_urdf is None or tf_cam is None:
                self._emit_status(
                    f'sample {i + 1}: missing TF, skipping')
            else:
                samples.append(_compose_sample(tf_urdf, tf_cam))
            if i < n - 1:
                if self._runner._stop_event.wait(interval_s):  # noqa: SLF001
                    break
        return samples

    def _urdf_camera_pose(self):
        """T(robot_base -> camera_optical_frame) via the URDF chain.

        Goes through whatever static joints connect the arm root to
        the camera frame in the URDF (typically robot_base -> base_link
        -> camera_link -> camera_color_frame -> camera_color_optical_frame).
        Returns (xyz, quat) or (None, None) if the chain is broken.
        """
        tf = self._lookup_static(ROBOT_BASE_FRAME, CAMERA_FRAME)
        if tf is None:
            return None, None
        return _xyz_quat_from_transform(tf.transform)

    # -- output ------------------------------------------------------

    def _log_result_block(self, result: dict):
        m = result['measured']
        s = result['stddev']
        lines = [
            'measured: xyz=({:.4f}, {:.4f}, {:.4f}) rpy_deg=({:.2f}, {:.2f}, {:.2f}) quat=({:.4f}, {:.4f}, {:.4f}, {:.4f})'.format(
                *m['xyz'], *m['rpy_deg'], *m['quat']),
            'stddev:   xyz=({:.4f}, {:.4f}, {:.4f}) rpy_deg=({:.2f}, {:.2f}, {:.2f}) [over {} samples]'.format(
                *s['xyz'], *s['rpy_deg'], s['samples']),
        ]
        if result['urdf_expected'] is not None:
            u = result['urdf_expected']
            d = result['delta']
            lines.append(
                'urdf:     xyz=({:.4f}, {:.4f}, {:.4f}) rpy_deg=({:.2f}, {:.2f}, {:.2f})'.format(
                    *u['xyz'], *u['rpy_deg']))
            lines.append(
                'delta:    xyz=({:.4f}, {:.4f}, {:.4f}) rpy_deg=({:.2f}, {:.2f}, {:.2f})'.format(
                    *d['xyz'], *d['rpy_deg']))
        else:
            lines.append('urdf:     not available (chain broken)')
        for line in lines:
            self._emit_status(line)

    def _save_yaml(self, result: dict) -> Path:
        now = datetime.now()
        run_dir = (DEFAULT_DATA_ROOT / 'camera_localization'
                   / now.strftime('%Y-%m-%d')
                   / now.strftime('%H-%M-%S'))
        run_dir.mkdir(parents=True, exist_ok=True)
        path = run_dir / 'result.yaml'
        with path.open('w') as f:
            yaml.safe_dump(result, f, sort_keys=False)
        return path


# -- math helpers ----------------------------------------------------

def _xyz_quat_from_transform(t):
    return (
        np.array([t.translation.x, t.translation.y, t.translation.z]),
        np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]),
    )


def _quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def _matrix_to_quat(R):
    # Standard conversion. R is 3x3 rotation.
    t = R.trace()
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        return np.array([
            (R[2, 1] - R[1, 2]) * s,
            (R[0, 2] - R[2, 0]) * s,
            (R[1, 0] - R[0, 1]) * s,
            0.25 / s,
        ])
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[2, 1] - R[1, 2]) / s,
        ])
    if R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return np.array([
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
            (R[0, 2] - R[2, 0]) / s,
        ])
    s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    return np.array([
        (R[0, 2] + R[2, 0]) / s,
        (R[1, 2] + R[2, 1]) / s,
        0.25 * s,
        (R[1, 0] - R[0, 1]) / s,
    ])


def _quat_to_rpy(q):
    """ZYX intrinsic Euler (the ROS convention)."""
    x, y, z, w = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw])


def _se3(xyz, quat):
    M = np.eye(4)
    M[:3, :3] = _quat_to_matrix(quat)
    M[:3, 3] = xyz
    return M


def _se3_inv(M):
    R = M[:3, :3]
    t = M[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def _se3_to_xyz_quat(M):
    return M[:3, 3].copy(), _matrix_to_quat(M[:3, :3])


def _compose_sample(tf_urdf, tf_cam):
    """T(robot_base -> camera) = T(robot_base -> urdf_tag) @ inv(T(camera -> detected_tag)).

    The urdf_tag and detected_tag frames are assumed to coincide
    physically (both anchored at the printed tag's centre)."""
    urdf_xyz, urdf_quat = _xyz_quat_from_transform(tf_urdf.transform)
    cam_xyz, cam_quat = _xyz_quat_from_transform(tf_cam.transform)
    M_urdf = _se3(urdf_xyz, urdf_quat)
    M_cam_to_tag = _se3(cam_xyz, cam_quat)
    M_base_to_cam = M_urdf @ _se3_inv(M_cam_to_tag)
    xyz, quat = _se3_to_xyz_quat(M_base_to_cam)
    return xyz, quat


def _average_samples(samples: list):
    xyzs = np.array([s[0] for s in samples])
    quats = np.array([s[1] for s in samples])
    # Sign-align quaternions onto the same hemisphere as the first one
    # before averaging (q and -q represent the same rotation).
    pivot = quats[0]
    for i in range(1, len(quats)):
        if np.dot(pivot, quats[i]) < 0:
            quats[i] = -quats[i]
    mean_xyz = xyzs.mean(axis=0)
    mean_quat = quats.mean(axis=0)
    mean_quat = mean_quat / np.linalg.norm(mean_quat)
    return mean_xyz, mean_quat


def _xyz_stddev(samples: list):
    return np.array([s[0] for s in samples]).std(axis=0)


def _rpy_stddev(samples: list):
    rpys = np.array([_quat_to_rpy(s[1]) for s in samples])
    return np.degrees(rpys.std(axis=0))


def _pose_dict(xyz, quat):
    rpy = _quat_to_rpy(quat)
    return {
        'xyz': [float(v) for v in xyz],
        'quat': [float(v) for v in quat],
        'rpy_deg': [float(v) for v in np.degrees(rpy)],
    }


def _delta_dict(measured_xyz, measured_quat, urdf_xyz, urdf_quat):
    dxyz = measured_xyz - urdf_xyz
    # Rotation difference: R_delta = R_measured @ R_urdf^T → as RPY.
    M_meas = _quat_to_matrix(measured_quat)
    M_urdf = _quat_to_matrix(urdf_quat)
    R_delta = M_meas @ M_urdf.T
    quat_delta = _matrix_to_quat(R_delta)
    rpy_delta = _quat_to_rpy(quat_delta)
    return {
        'xyz': [float(v) for v in dxyz],
        'rpy_deg': [float(v) for v in np.degrees(rpy_delta)],
    }
