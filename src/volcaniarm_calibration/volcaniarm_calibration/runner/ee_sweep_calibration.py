"""EE-sweep camera-pose calibration with marker-orientation prior.

Drives the arm through the EE poses in calibration_poses.yaml, captures
(detected R + t of the EE tag in optical frame, URDF-truth tag origin
in camera's parent frame) per visit, and solves for the full 6-DoF
camera_link pose. Mode is auto-detected from the URDF parent of
`camera_link`:

  * `world` parent (camera-on-stand, URDF arg `mode:=tests`)
  * `camera_mount_rev_link` parent (camera-on-robot, URDF arg `mode:=work`)

cv2.calibrateHandEye is underconstrained on the volcaniarm's 2-DOF
planar mechanism (insufficient EE rotation diversity), so we exploit
a known constant world-orientation of the EE marker as a prior:

    R(world, optical) = R(world, marker)_known @ R(optical, marker)_detected^-1

A *single* detection plus the prior gives camera orientation directly;
we average across poses for noise. Translation is solved from the
existing per-axis median + MAD-trim path, using the recovered
orientation per pose. See ``solve_with_marker_orientation_prior`` for
the pure-function math.

Output is the persistent `config/camera_pose.yaml` (consumed by
`real_bringup.launch.py` at startup) plus a per-run audit
`data/camera_localization/<date>/<time>/result.yaml`.

Frame discipline: every input lookup is decoupled from the unknown
camera_link translation. Truth-side TF traverses
`<parent> -> ... -> apriltag_ee_link` (no camera traversal). Detection
is published directly by apriltag_ros as
`camera_color_optical_frame -> apriltag_marker_ee`. The prior is
expressed in `apriltag_marker_ee` (image axes, what apriltag_ros
publishes), so the URDF body-axis frame difference doesn't enter the
solve.
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, List, Optional, Tuple

import numpy as np
import yaml

from rclpy.duration import Duration as RclpyDuration
from rclpy.time import Time as RclpyTime

from .calibration_runner import CalibrationRunner


SAFE_Y_MIN = -0.30
SAFE_Y_MAX = 0.30
SAFE_Z_MIN = 0.35
SAFE_Z_MAX = 0.65

DEFAULT_DATA_ROOT = (
    Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/data')
    .expanduser())
DEFAULT_CAMERA_POSE_CONFIG = (
    Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/'
         'config/camera_pose.yaml').expanduser())
DEFAULT_POSES_YAML = (
    Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/'
         'config/calibration_poses.yaml').expanduser())

# Fixed frames we always work with.
WORLD_FRAME = 'world'
CAMERA_LINK_FRAME = 'camera_link'
OPTICAL_FRAME = 'camera_color_optical_frame'
EE_URDF_FRAME = 'apriltag_ee_link'
EE_DETECTED_FRAME = 'apriltag_marker_ee'
ROBOT_EE_FRAME = 'right_arm_tip_link'
CAMERA_MOUNT_FRAME = 'camera_mount_rev_link'

MODE_STAND = 'calibration_stand'   # camera parented to world
MODE_ON_ROBOT = 'on_robot_mount'   # camera parented to camera_mount_rev_link


def write_camera_pose_config(mode: str, parent_frame: str,
                             xyz: np.ndarray, quat: np.ndarray,
                             source: str = 'ee_sweep') -> Path:
    """Persist the solved camera pose to DEFAULT_CAMERA_POSE_CONFIG.

    Single source of the camera_pose.yaml schema consumed by
    real_bringup's validator. Both the EE-sweep solver and the
    base-tag locator write through here so the two stay compatible.
    `source` records which tool produced the pose.
    """
    rpy = _quat_to_rpy(quat)
    DEFAULT_CAMERA_POSE_CONFIG.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        'mode': mode,
        'parent_frame': parent_frame,
        'child_frame': CAMERA_LINK_FRAME,
        'xyz': [float(v) for v in xyz],
        'rpy': [float(v) for v in rpy],
        'quat': [float(v) for v in quat],
        'rpy_deg': [float(math.degrees(v)) for v in rpy],
        'last_updated': datetime.now().isoformat(timespec='seconds'),
        'source': source,
    }
    with DEFAULT_CAMERA_POSE_CONFIG.open('w') as f:
        yaml.safe_dump(payload, f, sort_keys=False)
    return DEFAULT_CAMERA_POSE_CONFIG


StatusCb = Callable[[str], None]
ProgressCb = Callable[[int, int], None]
FinishedCb = Callable[[str, Optional[Path], str], None]


@dataclass
class _PoseEntry:
    y: float
    z: float
    theta_right: float
    theta_left: float
    comment: str


@dataclass
class _PoseSample:
    pose_idx: int
    comment: str
    # Origin of apriltag_ee_link expressed in the camera's URDF parent
    # frame (world for stand, camera_mount_rev_link for on-robot). The
    # parent traversal does NOT go through the unknown camera_link xyz,
    # so this is decoupled from what we're solving for.
    p_parent_truth: np.ndarray
    # Detected pose of apriltag_marker_ee in camera_color_optical_frame
    # (image axes per apriltag_ros). Both rotation and translation are
    # used by the orientation-prior solver.
    R_optical_marker: np.ndarray
    t_optical_marker: np.ndarray


# ------------------------------------------------------------------ #
# Pure math helpers (no rclpy / TF). Easier to unit-test in isolation.
# ------------------------------------------------------------------ #


def _xyz_quat_from_transform(t):
    """Extract (xyz, quat) ndarrays from a geometry_msgs Transform."""
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
    """ROS extrinsic XYZ Euler angles from a quaternion."""
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


def _rpy_to_quat(roll, pitch, yaw):
    """Inverse of _quat_to_rpy: ROS extrinsic XYZ Euler -> quaternion."""
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy_ = math.cos(yaw * 0.5); sy_ = math.sin(yaw * 0.5)
    return np.array([
        sr * cp * cy_ - cr * sp * sy_,
        cr * sp * cy_ + sr * cp * sy_,
        cr * cp * sy_ - sr * sp * cy_,
        cr * cp * cy_ + sr * sp * sy_,
    ])


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


def _se3_from_rt(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = np.asarray(t).reshape(3)
    return M


def _quat_mean(quats: np.ndarray) -> np.ndarray:
    """Average a list of quaternions in the same hemisphere as the first.

    Sign-flips each quaternion to be on the same hemisphere as q[0]
    (q and -q represent the same rotation; without alignment they
    cancel in the arithmetic mean). Returns the L2-normalised mean.
    """
    qs = np.asarray(quats, dtype=float).reshape(-1, 4)
    pivot = qs[0]
    aligned = np.where((qs @ pivot)[:, None] < 0, -qs, qs)
    mean = aligned.mean(axis=0)
    return mean / np.linalg.norm(mean)


def _angle_between_quats_rad(q1: np.ndarray, q2: np.ndarray) -> float:
    """Smallest rotation angle between two unit quaternions (radians)."""
    d = abs(float(np.dot(q1, q2)))
    d = min(1.0, max(-1.0, d))
    return 2.0 * math.acos(d)


def solve_with_marker_orientation_prior(
    R_optical_marker_list: List[np.ndarray],
    t_optical_marker_list: List[np.ndarray],
    p_parent_truth_list: List[np.ndarray],
    R_world_marker_image: np.ndarray,
    R_parent_world: np.ndarray,
    R_camera_link_optical: np.ndarray,
    p_camera_link_optical: np.ndarray,
    *,
    mad_threshold: float = 3.0,
    max_rotation_variance_deg: float = 5.0,
) -> dict:
    """6-DoF camera pose recovery using a known marker world-orientation.

    Replaces the underconstrained cv2.calibrateHandEye approach with a
    closed-form solve that exploits the prior: on a 2-DOF planar arm
    the EE marker has a known constant orientation in the world frame
    (the user's case: parallel to ground, +X-facing). With that prior:

        R_world_optical_i = R_world_marker_image @ R_optical_marker_i^-1

    is exact for every pose, so we don't need rotation diversity.
    Variance across poses is a noise / consistency check.

    All rotations input/output are in the *image-axis* convention used
    by apriltag_ros (`apriltag_marker_ee`, not `apriltag_ee_link`).
    `R_world_marker_image` is the rotation that the operator (or URDF)
    asserts: it gets composed only with the detector's
    `R_optical_marker`, so the URDF body-axis convention is not needed.

    Inputs (per pose, lists of length N):
        R_optical_marker_list[i]  : (3,3) detected, image axes
        t_optical_marker_list[i]  : (3,)  detected
        p_parent_truth_list[i]    : (3,)  from URDF + FK -- origin of
                                          apriltag_ee_link in the
                                          camera's parent frame
                                          (world for stand, mount for on-robot)

    Constants:
        R_world_marker_image      : (3,3) the prior
        R_parent_world            : (3,3) static URDF (for stand mode this
                                          is identity; for on-robot mode it
                                          is the orientation of
                                          camera_mount_rev_link in world)
        R_camera_link_optical     : (3,3) static URDF
        p_camera_link_optical     : (3,)  static URDF

    Returns:
        R_parent_camera_link, t_parent_camera_link  -- the calibrated camera
                                                       pose, ready for
                                                       camera_pose.yaml
        per_pose_R_world_optical, rotation_variance_deg  -- diagnostics
        position residual stats (rms / std / max / kept / rejected)

    Raises ValueError if rotation variance across poses exceeds
    `max_rotation_variance_deg` -- the prior is wrong, OR the marker
    isn't actually at a constant world orientation, OR the FK / detection
    chain is corrupted.
    """
    n = len(R_optical_marker_list)
    if n < 1:
        raise ValueError('need at least one pose')
    if not (n == len(t_optical_marker_list) == len(p_parent_truth_list)):
        raise ValueError('input list lengths mismatch')

    R_world_marker_image = np.asarray(R_world_marker_image, dtype=float).reshape(3, 3)
    R_parent_world = np.asarray(R_parent_world, dtype=float).reshape(3, 3)

    # Per-pose orientation recovery.
    R_world_optical_list = []
    quats_world_optical = []
    for R_om in R_optical_marker_list:
        R_om = np.asarray(R_om, dtype=float).reshape(3, 3)
        R_wo = R_world_marker_image @ R_om.T
        R_world_optical_list.append(R_wo)
        quats_world_optical.append(_matrix_to_quat(R_wo))
    quats_world_optical = np.array(quats_world_optical)

    # Quaternion mean + variance check.
    q_mean = _quat_mean(quats_world_optical)
    angles = np.array([_angle_between_quats_rad(q, q_mean)
                       for q in quats_world_optical])
    max_dev_deg = float(math.degrees(angles.max())) if n else 0.0
    if max_dev_deg > max_rotation_variance_deg:
        raise ValueError(
            f'rotation variance across poses {max_dev_deg:.2f} deg > '
            f'{max_rotation_variance_deg:.2f} deg threshold; the marker '
            f'orientation prior is likely wrong, or the detector / FK '
            f'chain is too noisy. Per-pose deviations (deg): '
            f'{[round(math.degrees(a), 2) for a in angles]}')

    R_world_optical = _quat_to_matrix(q_mean)

    # Per-pose translation estimate of optical-frame origin in parent frame.
    # Use the per-pose R_world_optical (not the mean) to be self-consistent
    # with that pose's detection, then aggregate by per-axis median + MAD.
    p_parent_optical_per_pose = []
    for R_wo, t_om, p_pt in zip(R_world_optical_list,
                                 t_optical_marker_list,
                                 p_parent_truth_list):
        t_om = np.asarray(t_om, dtype=float).reshape(3)
        p_pt = np.asarray(p_pt, dtype=float).reshape(3)
        R_po = R_parent_world @ R_wo
        p_po = p_pt - R_po @ t_om
        p_parent_optical_per_pose.append(p_po)
    estimates = np.array(p_parent_optical_per_pose)

    med = np.median(estimates, axis=0)
    mad = np.median(np.abs(estimates - med), axis=0)
    mad_safe = np.where(mad > 1e-9, mad, np.inf)
    deviation = np.abs(estimates - med) / mad_safe
    keep_mask = np.all(deviation <= mad_threshold, axis=1)
    if not keep_mask.any():
        keep_mask[:] = True
    kept = estimates[keep_mask]
    p_parent_optical = np.median(kept, axis=0)

    # Compose T_parent_camera_link = T_parent_optical @ inv(T_camera_link_optical).
    R_parent_optical = R_parent_world @ R_world_optical
    R_camera_link_optical = np.asarray(R_camera_link_optical, dtype=float).reshape(3, 3)
    p_camera_link_optical = np.asarray(p_camera_link_optical, dtype=float).reshape(3)
    R_optical_camera_link = R_camera_link_optical.T
    p_optical_camera_link = -R_optical_camera_link @ p_camera_link_optical
    R_parent_camera_link = R_parent_optical @ R_optical_camera_link
    p_parent_camera_link = p_parent_optical + R_parent_optical @ p_optical_camera_link

    residuals = kept - p_parent_optical
    norms = np.linalg.norm(residuals, axis=1)
    return {
        'R_parent_camera_link': R_parent_camera_link,
        't_parent_camera_link': p_parent_camera_link,
        'R_parent_optical': R_parent_optical,
        't_parent_optical': p_parent_optical,
        'per_pose_R_world_optical': R_world_optical_list,
        'per_pose_t_parent_optical': estimates,
        'rotation_variance_deg': max_dev_deg,
        'rms_residual_m': float(np.sqrt(np.mean(norms ** 2))) if len(norms) else 0.0,
        'std_residual_m': float(np.std(norms)) if len(norms) else 0.0,
        'max_residual_m': float(np.max(norms)) if len(norms) else 0.0,
        'kept_indices': [int(i) for i in np.where(keep_mask)[0]],
        'rejected_indices': [int(i) for i in np.where(~keep_mask)[0]],
    }


# ------------------------------------------------------------------ #
# Runner class
# ------------------------------------------------------------------ #


class EESweepCameraCalibrationRunner:
    """Drive an EE-sweep, solve for camera pose, persist to YAML.

    Composes with ``CalibrationRunner`` for TF buffer, IK/FK/action
    clients, and the shared ``_stop_event`` that backs Cancel.
    """

    def __init__(self, runner: CalibrationRunner,
                 marker_world_rpy: Optional[Tuple[float, float, float]] = None):
        """``marker_world_rpy`` is the rpy (radians) of `apriltag_marker_ee`
        (image axes) in the world frame. The orientation-prior solver uses
        this to recover camera orientation without needing rotation
        diversity in the EE motion (which the 2-DOF planar arm doesn't
        have). When None (default), the runner reads R(world,
        apriltag_marker_ee) via TF lookup at run start, which uses the
        URDF placeholder camera orientation as a starting point. Override
        when the URDF placeholder is biased.
        """
        self._runner = runner
        self._node = runner.node
        self._thread: Optional[threading.Thread] = None
        self._marker_world_rpy = marker_world_rpy
        self.status_cb: Optional[StatusCb] = None
        self.progress_cb: Optional[ProgressCb] = None
        self.finished_cb: Optional[FinishedCb] = None

    # -- public control surface ---------------------------------------

    def is_busy(self) -> bool:
        if self._thread is not None and self._thread.is_alive():
            return True
        rt = self._runner._run_thread  # noqa: SLF001
        if rt is not None and rt.is_alive():
            return True
        return False

    def request(self, _yaml_path: Optional[Path] = None) -> bool:
        """Kick off a sweep. Argument ignored (kept for back-compat with
        the old single-shot CameraCalibrationRunner.request signature)."""
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
            self._thread.join(timeout=10.0)

    def detect_mode(self) -> Optional[str]:
        """Return MODE_STAND / MODE_ON_ROBOT / None based on URDF.

        Queries the TF buffer's frame graph for the *direct* parent of
        `camera_link`. A bare `lookup_transform(parent, camera_link)` is
        not sufficient: TF resolves the chain through whichever
        intermediate frames exist, so both `world` and
        `camera_mount_rev_link` lookups succeed in both modes -- the
        difference between them is which static joint is published
        directly above `camera_link`, not whether a TF path exists.
        """
        try:
            frames_yaml = self._runner._tf_buffer.all_frames_as_yaml()  # noqa: SLF001
        except Exception:
            return None
        try:
            frames = yaml.safe_load(frames_yaml) or {}
        except Exception:
            return None
        entry = frames.get(CAMERA_LINK_FRAME)
        if not isinstance(entry, dict):
            return None
        parent = entry.get('parent')
        if parent == WORLD_FRAME:
            return MODE_STAND
        if parent == CAMERA_MOUNT_FRAME:
            return MODE_ON_ROBOT
        return None

    # -- emit helpers -------------------------------------------------

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

    # -- main worker --------------------------------------------------

    def _run(self):
        try:
            mode = self.detect_mode()
            if mode is None:
                reason = (f'URDF is not in calibration-capable mode: '
                          f'{CAMERA_LINK_FRAME} parent must be '
                          f'{WORLD_FRAME} (mode=tests) or '
                          f'{CAMERA_MOUNT_FRAME} (mode=work)')
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return

            self._emit_status(f'detected mode: {mode}')

            cfg = self._load_yaml(DEFAULT_POSES_YAML)
            poses_raw = cfg.get('poses') or []
            trajectory_duration = float(cfg.get('trajectory_duration', 2.5))
            settle_time = float(cfg.get('settle_time', 1.5))
            detection_timeout_s = float(cfg.get('detection_timeout_s', 2.0))
            min_successful_samples = int(cfg.get('min_successful_samples', 7))

            poses = self._validate_and_resolve(poses_raw)
            if len(poses) < min_successful_samples:
                reason = (f'only {len(poses)} pose(s) reachable; need at '
                          f'least {min_successful_samples}')
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return

            # The camera's URDF parent frame: where camera_link is anchored
            # in the URDF, and the frame the calibrated XYZ ends up
            # expressed in.
            urdf_parent = WORLD_FRAME if mode == MODE_STAND else CAMERA_MOUNT_FRAME

            # Static URDF chains gathered up front. We use them as the
            # *initial* `R_parent_optical` for the freshness-drift guard
            # and as the camera_link->optical static offset (always
            # rigid). The actual orientation we solve for is parent->
            # camera_link.
            R_parent_optical_init = self._lookup_rotation(
                urdf_parent, OPTICAL_FRAME)
            tf_link_to_optical = self._lookup_static(
                CAMERA_LINK_FRAME, OPTICAL_FRAME)
            tf_parent_world = self._lookup_static(urdf_parent, WORLD_FRAME)
            if (R_parent_optical_init is None or tf_link_to_optical is None
                    or tf_parent_world is None):
                reason = (f'URDF chain incomplete: cannot read '
                          f'{urdf_parent}->{OPTICAL_FRAME}, '
                          f'{CAMERA_LINK_FRAME}->{OPTICAL_FRAME}, or '
                          f'{urdf_parent}->{WORLD_FRAME}')
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return
            link_to_optical_xyz, link_to_optical_quat = _xyz_quat_from_transform(
                tf_link_to_optical.transform)
            R_camera_link_optical = _quat_to_matrix(link_to_optical_quat)
            _, parent_world_quat = _xyz_quat_from_transform(
                tf_parent_world.transform)
            R_parent_world = _quat_to_matrix(parent_world_quat)

            # Resolve the marker world-orientation prior. Either an
            # operator-supplied rpy (image-axis convention of
            # apriltag_marker_ee in world), or a TF lookup at run start
            # of `world -> apriltag_marker_ee` which uses the URDF
            # placeholder camera orientation as a starting point.
            if self._marker_world_rpy is not None:
                r, p_, y_ = self._marker_world_rpy
                R_world_marker_image = _quat_to_matrix(_rpy_to_quat(r, p_, y_))
                self._emit_status(
                    f'marker world rpy (override): '
                    f'{[round(math.degrees(v), 2) for v in (r, p_, y_)]} deg')
            else:
                R_world_marker_image = self._lookup_rotation(
                    WORLD_FRAME, EE_DETECTED_FRAME)
                if R_world_marker_image is None:
                    reason = (f'cannot determine marker world-orientation '
                              f'prior: TF {WORLD_FRAME}->{EE_DETECTED_FRAME} '
                              f'unavailable and no `marker_world_rpy` '
                              f'override was provided')
                    self._emit_status(f'aborting: {reason}')
                    self._emit_finished('failed', None, reason)
                    return
                self._emit_status(
                    f'marker world rpy (URDF lookup): '
                    f'{np.degrees(_quat_to_rpy(_matrix_to_quat(R_world_marker_image))).round(2).tolist()} deg')

            # Sweep + sample.
            samples, drift = self._sweep_poses(
                poses, urdf_parent, R_parent_optical_init,
                trajectory_duration, settle_time, detection_timeout_s)

            if self._runner._stop_event.is_set():  # noqa: SLF001
                self._emit_status('camera calibration canceled')
                self._emit_finished('canceled', None, '')
                return

            if len(samples) < min_successful_samples:
                reason = (f'only {len(samples)}/{len(poses)} poses produced '
                          f'a fresh EE-tag detection; need at least '
                          f'{min_successful_samples}')
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return

            # 6-DoF orientation-prior solve. Both modes use the same math
            # (mode A passes R_parent_world=identity; mode B passes the
            # mount->world rotation from URDF + FK). Returns both the
            # camera_link xyz AND rpy in the parent frame; orientation is
            # recovered from the marker prior, not held at URDF defaults.
            try:
                solved = solve_with_marker_orientation_prior(
                    [s.R_optical_marker for s in samples],
                    [s.t_optical_marker for s in samples],
                    [s.p_parent_truth for s in samples],
                    R_world_marker_image=R_world_marker_image,
                    R_parent_world=R_parent_world,
                    R_camera_link_optical=R_camera_link_optical,
                    p_camera_link_optical=link_to_optical_xyz,
                )
            except ValueError as exc:
                reason = f'orientation-prior solve failed: {exc}'
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return
            solved_xyz = solved['t_parent_camera_link']
            solved_quat = _matrix_to_quat(solved['R_parent_camera_link'])
            residual_stats = {
                'rms_m': solved['rms_residual_m'],
                'std_m': solved['std_residual_m'],
                'max_m': solved['max_residual_m'],
                'kept_indices': solved['kept_indices'],
                'rejected_indices': solved['rejected_indices'],
                'rotation_variance_deg': solved['rotation_variance_deg'],
                'per_pose_t_parent_optical': solved['per_pose_t_parent_optical'].tolist(),
            }
            self._emit_status(
                f'rotation variance across poses: '
                f'{solved["rotation_variance_deg"]:.3f} deg')

            # Park at home so the arm ends in a known state.
            self._emit_status('parking arm at home (theta=0, 0)')
            self._runner._send_and_wait(  # noqa: SLF001
                self._runner._default_joint_names,  # noqa: SLF001
                0.0, 0.0,
                self._runner._default_trajectory_duration)  # noqa: SLF001

            # Defensive validation. With both modes now using the
            # position-only median solve this should be unreachable in
            # practice (no cv2 NaN risk), but the URDF parser is still
            # downstream of this YAML and a bad write would brick the
            # next launch -- so we keep the guard.
            non_finite = (
                not np.all(np.isfinite(solved_xyz))
                or not np.all(np.isfinite(solved_quat)))
            if non_finite:
                reason = (f'solver returned non-finite values '
                          f'(xyz={solved_xyz.tolist()}, '
                          f'quat={solved_quat.tolist()}); '
                          f'camera_pose.yaml NOT updated.')
                self._emit_status(f'aborting: {reason}')
                result = self._build_result(
                    mode, samples, solved_xyz, solved_quat, residual_stats,
                    drift, urdf_parent)
                result['solver_status'] = 'failed_non_finite'
                try:
                    result_path = self._save_yaml(result)
                    self._emit_status(f'saved per-run audit: {result_path}')
                except Exception:
                    result_path = None
                self._emit_finished('failed', result_path, reason)
                return

            # Write artefacts.
            result = self._build_result(
                mode, samples, solved_xyz, solved_quat, residual_stats,
                drift, urdf_parent)
            result_path = self._save_yaml(result)
            self._emit_status(f'saved per-run audit: {result_path}')
            config_path = self._save_camera_pose_config(
                mode, urdf_parent, solved_xyz, solved_quat)
            self._emit_status(
                f'updated default camera pose config: {config_path}')
            self._log_summary(mode, solved_xyz, solved_quat, residual_stats)
            self._emit_finished('completed', result_path, '')
        except Exception as exc:
            self._node.get_logger().error(
                f'camera calibration crashed: {exc}')
            self._emit_status(f'failed: {exc}')
            self._emit_finished('failed', None, f'exception: {exc}')

    # -- pose validation ---------------------------------------------

    def _validate_and_resolve(self, poses_raw: list) -> List[_PoseEntry]:
        out: List[_PoseEntry] = []
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

    # -- sweep + sample ---------------------------------------------

    def _sweep_poses(self, poses: List[_PoseEntry], urdf_parent: str,
                     R_parent_optical_init: np.ndarray,
                     trajectory_duration: float, settle_time: float,
                     detection_timeout_s: float
                     ) -> Tuple[List[_PoseSample], float]:
        """Drive arm through each pose, sample (URDF-truth EE, detected EE).

        ``urdf_parent`` is whatever frame the camera is parented to in
        the URDF -- `world` for stand mode, `camera_mount_rev_link` for
        on-robot. Truth is always `lookup(urdf_parent, apriltag_ee_link)`,
        which traverses only static URDF + FK and is decoupled from the
        unknown camera_link translation.

        Returns (samples, max_R_parent_optical_drift_F). The drift scalar
        is the max Frobenius norm difference between R_parent_optical at
        each pose vs the run-start snapshot; should be ~0 since the
        chain is fully static.
        """
        samples: List[_PoseSample] = []
        total = len(poses)
        max_drift = 0.0
        for idx, p in enumerate(poses, start=1):
            if self._runner._stop_event.is_set():  # noqa: SLF001
                return samples, max_drift
            self._emit_status(
                f'pose {idx}/{total} ({p.y:.3f}, {p.z:.3f}) {p.comment}')
            ok = self._runner._send_and_wait(  # noqa: SLF001
                self._runner._default_joint_names,  # noqa: SLF001
                p.theta_right, p.theta_left, trajectory_duration)
            if not ok:
                if self._runner._stop_event.is_set():  # noqa: SLF001
                    return samples, max_drift
                self._emit_status(
                    f'pose {idx}: motion failed; skipping sample')
                self._emit_progress(idx, total)
                continue

            if not self._sleep_with_cancel(settle_time):
                return samples, max_drift

            # Wait for a fresh EE-tag detection.
            tf_fresh = self._runner._wait_for_fresh_frame(  # noqa: SLF001
                EE_DETECTED_FRAME, detection_timeout_s,
                parent=OPTICAL_FRAME)
            if tf_fresh is None:
                self._emit_status(
                    f'pose {idx}: EE tag not visible; skipping sample')
                self._emit_progress(idx, total)
                continue

            # Truth side: parent -> apriltag_ee_link (origin only).
            tf_truth = self._lookup_static(urdf_parent, EE_URDF_FRAME)
            if tf_truth is None:
                self._emit_status(
                    f'pose {idx}: URDF chain {urdf_parent}->{EE_URDF_FRAME} '
                    f'unavailable; skipping sample')
                self._emit_progress(idx, total)
                continue
            p_parent_truth, _ = _xyz_quat_from_transform(tf_truth.transform)
            t_optical_marker, q_optical_marker = _xyz_quat_from_transform(
                tf_fresh.transform)
            R_optical_marker = _quat_to_matrix(q_optical_marker)

            # Static-rotation guard: if the chain rotation drifts mid-run
            # the orientation we hold constant is wrong.
            R_now = self._lookup_rotation(urdf_parent, OPTICAL_FRAME)
            if R_now is not None:
                drift = float(np.linalg.norm(R_now - R_parent_optical_init))
                if drift > max_drift:
                    max_drift = drift

            samples.append(_PoseSample(
                pose_idx=idx,
                comment=p.comment,
                p_parent_truth=p_parent_truth,
                R_optical_marker=R_optical_marker,
                t_optical_marker=t_optical_marker,
            ))
            self._emit_progress(idx, total)

        return samples, max_drift

    def _lookup_static(self, parent: str, child: str):
        try:
            return self._runner._tf_buffer.lookup_transform(  # noqa: SLF001
                parent, child, RclpyTime(),
                timeout=RclpyDuration(seconds=0.5))
        except Exception:
            return None

    def _lookup_rotation(self, parent: str, child: str) -> Optional[np.ndarray]:
        tf = self._lookup_static(parent, child)
        if tf is None:
            return None
        _, q = _xyz_quat_from_transform(tf.transform)
        return _quat_to_matrix(q)

    def _sleep_with_cancel(self, seconds: float) -> bool:
        if seconds <= 0:
            return True
        return not self._runner._stop_event.wait(seconds)  # noqa: SLF001

    # -- output ------------------------------------------------------

    def _build_result(self, mode, samples, solved_xyz, solved_quat,
                      residual_stats, drift, parent_frame
                      ) -> dict:
        rpy = _quat_to_rpy(solved_quat)
        return {
            'mode': mode,
            'method': 'ee_sweep_orientation_prior',
            'timestamp': datetime.now().isoformat(),
            'samples_used': len(samples),
            'solved': {
                'parent_frame': parent_frame,
                'child_frame': CAMERA_LINK_FRAME,
                'xyz': [float(v) for v in solved_xyz],
                'rpy': [float(v) for v in rpy],
                'quat': [float(v) for v in solved_quat],
            },
            'residuals': {
                'rms_m': float(residual_stats.get('rms_m', 0.0)),
                'std_m': float(residual_stats.get('std_m', 0.0)),
                'max_m': float(residual_stats.get('max_m', 0.0)),
                'kept_indices': [int(i) for i in residual_stats.get('kept_indices', [])],
                'rejected_indices': [int(i) for i in residual_stats.get('rejected_indices', [])],
                'rotation_variance_deg': float(residual_stats.get('rotation_variance_deg', 0.0)),
            },
            'pose_visits': [
                {'pose_idx': s.pose_idx, 'comment': s.comment}
                for s in samples
            ],
            'frames': {
                'truth_chain': f'{parent_frame} -> {EE_URDF_FRAME}',
                'detection': f'{OPTICAL_FRAME} -> {EE_DETECTED_FRAME}',
                'marker_orientation_prior': f'{WORLD_FRAME} -> {EE_DETECTED_FRAME}',
                'output': f'{parent_frame} -> {CAMERA_LINK_FRAME}',
            },
            'r_parent_optical_drift_during_run': float(drift),
        }

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

    def _save_camera_pose_config(self, mode: str, parent_frame: str,
                                 xyz: np.ndarray, quat: np.ndarray) -> Path:
        return write_camera_pose_config(
            mode, parent_frame, xyz, quat, source='ee_sweep')

    def _log_summary(self, mode: str, xyz: np.ndarray, quat: np.ndarray,
                     residual_stats: dict):
        rpy_deg = np.degrees(_quat_to_rpy(quat))
        self._emit_status(
            f'solved {mode}: xyz=({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}) '
            f'rpy_deg=({rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f})')
        self._emit_status(
            f'residual rms={residual_stats.get("rms_m", 0.0) * 1000:.2f} mm '
            f'max={residual_stats.get("max_m", 0.0) * 1000:.2f} mm '
            f'rotvar={residual_stats.get("rotation_variance_deg", 0.0):.2f} deg')
        if residual_stats.get('rms_m', 0.0) > 0.020:
            self._emit_status(
                'WARNING: rms residual > 20 mm; URDF apriltag mount may '
                'be biased -- remeasure apriltag_ee_mount_joint origin')

    # -- yaml load ---------------------------------------------------

    def _load_yaml(self, path: Path) -> dict:
        with path.open() as f:
            return yaml.safe_load(f) or {}
