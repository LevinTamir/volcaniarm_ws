"""EE-sweep camera-pose calibration -- position-only, two parent frames.

Drives the arm through the EE poses in calibration_poses.yaml, captures
(URDF-truth EE-tag origin, detected EE-tag origin) per visit, and
solves for the camera position. Mode is auto-detected from the URDF
parent of `camera_link`:

  * `world` parent (camera-on-stand, URDF arg `mode:=tests`)
  * `camera_mount_rev_link` parent (camera-on-robot, URDF arg `mode:=work`)

The math is the same in both modes: per-axis median of
``p_parent_truth_i - R_parent_optical @ p_optical_detected_i`` with
MAD-based outlier rejection. Camera orientation is *not* recovered --
the volcaniarm is a 2-DOF planar arm and EE rotation diversity isn't
sufficient for cv2.calibrateHandEye-style 6-DoF solves. Orientation is
held at the URDF default of the parent->camera_link joint, which
encodes the rigid mount geometry (the part that doesn't change between
remountings).

Output is the persistent `config/camera_pose.yaml` (consumed by
`real_bringup.launch.py` at startup) plus a per-run audit
`data/camera_localization/<date>/<time>/result.yaml`.

Frame discipline (mirrors `_yz_segment_world` in calibration_runner):
the truth-side and detection-side TF lookups are decoupled from the
unknown camera_link xyz. Truth traverses
`<parent> -> ... -> apriltag_ee_link` (no camera traversal). Detection
is published directly by apriltag_ros as
`camera_color_optical_frame -> apriltag_marker_ee`. Origins of those
two frames refer to the same physical point (the tag centre) so we
only ever compare translations, never orientations.
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
    # Origin of apriltag_marker_ee in optical frame (apriltag_ros).
    p_optical_detected: np.ndarray


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


def solve_position_only(
    p_world_truths: np.ndarray,
    p_optical_detecteds: np.ndarray,
    R_world_optical: np.ndarray,
    p_camera_link_to_optical: np.ndarray,
    *,
    mad_threshold: float = 3.0,
) -> dict:
    """Position-only camera localization (mode A: camera on stand).

    For each pose i:
        p_world_optical_i = p_world_truth_i - R_world_optical @ p_optical_detected_i

    The result is robust-aggregated by per-axis median, MAD-trim, and
    re-median, then converted to camera_link via the static URDF offset:
        p_world_camera_link = p_world_optical - R_world_optical @ p_camera_link_to_optical

    Pure function -- no TF, no rclpy. Synthetic inputs are easy to test.
    """
    truths = np.asarray(p_world_truths, dtype=float).reshape(-1, 3)
    detected = np.asarray(p_optical_detecteds, dtype=float).reshape(-1, 3)
    if truths.shape != detected.shape or truths.shape[0] < 1:
        raise ValueError('p_world_truths and p_optical_detecteds must be (N,3) and matching')

    R = np.asarray(R_world_optical, dtype=float).reshape(3, 3)
    estimates = truths - (R @ detected.T).T  # shape (N, 3)

    # Per-axis median + MAD trim. Single pass; for N <= 11 this is enough.
    med = np.median(estimates, axis=0)
    mad = np.median(np.abs(estimates - med), axis=0)
    # Avoid zero MAD (would reject everything); fall back to no trim.
    mad_safe = np.where(mad > 1e-9, mad, np.inf)
    deviation = np.abs(estimates - med) / mad_safe
    keep_mask = np.all(deviation <= mad_threshold, axis=1)
    if not keep_mask.any():
        keep_mask[:] = True
    kept = estimates[keep_mask]
    p_world_optical = np.median(kept, axis=0)

    p_world_camera_link = p_world_optical - R @ np.asarray(
        p_camera_link_to_optical, dtype=float).reshape(3)

    residuals = kept - p_world_optical
    norms = np.linalg.norm(residuals, axis=1)
    return {
        'p_world_optical': p_world_optical,
        'p_world_camera_link': p_world_camera_link,
        'per_pose_estimates': estimates,
        'per_pose_residuals': estimates - p_world_optical,
        'rms_residual_m': float(np.sqrt(np.mean(norms ** 2))) if len(norms) else 0.0,
        'std_residual_m': float(np.std(norms)) if len(norms) else 0.0,
        'max_residual_m': float(np.max(norms)) if len(norms) else 0.0,
        'kept_indices': [int(i) for i in np.where(keep_mask)[0]],
        'rejected_indices': [int(i) for i in np.where(~keep_mask)[0]],
    }


def solve_handeye_eye_to_hand(
    R_mount_ee_list: List[np.ndarray],
    t_mount_ee_list: List[np.ndarray],
    R_optical_marker_list: List[np.ndarray],
    t_optical_marker_list: List[np.ndarray],
) -> dict:
    """Eye-to-hand 6-DoF camera-on-base calibration (mode B).

    Camera is rigidly fixed to camera_mount_rev_link; marker is on EE.
    Per pose i we have:
        gripper2base = T(camera_mount_rev_link, right_arm_tip_link)_i (FK)
        target2cam   = T(camera_color_optical_frame, apriltag_marker_ee)_i (detector)

    cv2.calibrateHandEye is parameterised for eye-in-hand by default
    (camera moves with the gripper). For eye-to-hand we invert the
    gripper2base inputs, which yields T(camera_color_optical_frame,
    camera_mount_rev_link) as the result.

    The constant offset between `apriltag_marker_ee` (image axes) and
    `apriltag_ee_link` (URDF body axes) is absorbed by the algorithm
    since it only operates on relative pose changes between pose pairs.
    """
    import cv2  # local import: keep top-of-file imports cheap

    if len(R_mount_ee_list) != len(t_mount_ee_list) \
            or len(R_optical_marker_list) != len(t_optical_marker_list) \
            or len(R_mount_ee_list) != len(R_optical_marker_list):
        raise ValueError('input list lengths mismatch')
    n = len(R_mount_ee_list)
    if n < 3:
        raise ValueError(f'need >= 3 poses, got {n}')

    # Eye-to-hand convention: pass `inv(T(mount, ee))` (= T(ee, mount))
    # as the "gripper2base" input and `T(optical, marker)` as the
    # "target2cam" input; cv2.calibrateHandEye returns T(mount, optical)
    # directly. Empirically verified against synthetic data: when the
    # gripper2base inputs are NOT inverted the returned R/t do not
    # correspond to either T(mount, optical) or its inverse.
    R_ee_mount = []
    t_ee_mount = []
    for R, t in zip(R_mount_ee_list, t_mount_ee_list):
        M = _se3_from_rt(R, t)
        Mi = _se3_inv(M)
        R_ee_mount.append(Mi[:3, :3])
        t_ee_mount.append(Mi[:3, 3])

    R_target2cam_arr = np.array(R_optical_marker_list)
    t_target2cam_arr = np.array(t_optical_marker_list).reshape(-1, 3, 1)
    R_grip2base_arr = np.array(R_ee_mount)
    t_grip2base_arr = np.array(t_ee_mount).reshape(-1, 3, 1)

    R_mount_optical, t_mount_optical = cv2.calibrateHandEye(
        R_grip2base_arr, t_grip2base_arr,
        R_target2cam_arr, t_target2cam_arr,
        method=cv2.CALIB_HAND_EYE_PARK)

    R_mount_optical = np.asarray(R_mount_optical, dtype=float)
    t_mount_optical = np.asarray(t_mount_optical, dtype=float).reshape(3)
    M_mount_to_optical = _se3_from_rt(R_mount_optical, t_mount_optical)
    return {
        'M_mount_to_optical': M_mount_to_optical,
        'R_mount_to_optical': R_mount_optical,
        't_mount_to_optical': t_mount_optical,
    }


# ------------------------------------------------------------------ #
# Runner class
# ------------------------------------------------------------------ #


class EESweepCameraCalibrationRunner:
    """Drive an EE-sweep, solve for camera pose, persist to YAML.

    Composes with ``CalibrationRunner`` for TF buffer, IK/FK/action
    clients, and the shared ``_stop_event`` that backs Cancel.
    """

    def __init__(self, runner: CalibrationRunner):
        self._runner = runner
        self._node = runner.node
        self._thread: Optional[threading.Thread] = None
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

            # Static URDF transforms gathered up front. R_parent_optical's
            # rotation is determined by static joints only -- the unknown
            # translation in the camera_joint / calibration_camera_joint
            # doesn't affect rotation, so this lookup is correct even
            # before any calibration.
            R_parent_optical = self._lookup_rotation(urdf_parent, OPTICAL_FRAME)
            tf_link_to_optical = self._lookup_static(
                CAMERA_LINK_FRAME, OPTICAL_FRAME)
            if R_parent_optical is None or tf_link_to_optical is None:
                reason = (f'URDF chain incomplete: cannot read '
                          f'{urdf_parent}->{OPTICAL_FRAME} or '
                          f'{CAMERA_LINK_FRAME}->{OPTICAL_FRAME}')
                self._emit_status(f'aborting: {reason}')
                self._emit_finished('failed', None, reason)
                return
            link_to_optical_xyz, link_to_optical_quat = _xyz_quat_from_transform(
                tf_link_to_optical.transform)
            self._emit_status(
                f'{urdf_parent}->{OPTICAL_FRAME} '
                f'rpy_deg={np.degrees(_quat_to_rpy(_matrix_to_quat(R_parent_optical))).round(2).tolist()} '
                f'(static URDF; held constant for the run)')

            # Read the URDF orientation of camera_link in its parent. We
            # solve for translation only and copy this orientation through
            # to camera_pose.yaml unchanged -- the rigid mount geometry
            # (URDF defaults of camera_joint / calibration_camera_joint
            # rpy) is what fixes the camera orientation in physical space.
            tf_parent_camlink = self._lookup_static(urdf_parent, CAMERA_LINK_FRAME)
            urdf_camlink_xyz, urdf_camlink_quat = (
                _xyz_quat_from_transform(tf_parent_camlink.transform)
                if tf_parent_camlink is not None else (None, None))

            # Sweep + sample.
            samples, drift = self._sweep_poses(
                poses, urdf_parent, R_parent_optical,
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

            # Same position-only solve in both modes; only the parent frame
            # differs. Orientation is held at the URDF default of the
            # parent->camera_link joint.
            solved_xyz, solved_quat, residual_stats = self._solve(
                samples, R_parent_optical,
                link_to_optical_xyz, urdf_camlink_quat)

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
                    R_parent_optical, drift, urdf_parent)
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
                R_parent_optical, drift, urdf_parent)
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
            p_optical_detected, _ = _xyz_quat_from_transform(
                tf_fresh.transform)

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
                p_optical_detected=p_optical_detected,
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

    # -- the math ----------------------------------------------------

    def _solve(self, samples: List[_PoseSample],
               R_parent_optical: np.ndarray,
               link_to_optical_xyz: np.ndarray,
               urdf_camlink_quat: Optional[np.ndarray]):
        """Position-only solve, parent-frame agnostic.

        For each pose i, the per-axis estimate of the optical-frame
        origin in the camera's parent frame is
        ``p_parent_optical_i = p_parent_truth_i - R_parent_optical
        @ p_optical_detected_i``. Aggregated by per-axis median +
        MAD-trim, then converted to camera_link via the static
        ``camera_link -> optical`` offset.

        Orientation is held at the URDF default of the parent->camera_link
        joint -- that's the rigid mount geometry, which is what determines
        camera orientation in physical space (no observable rotation
        diversity to recover orientation from on this 2-DOF arm anyway).
        """
        truths = np.array([s.p_parent_truth for s in samples])
        detecteds = np.array([s.p_optical_detected for s in samples])
        out = solve_position_only(
            truths, detecteds,
            R_parent_optical, link_to_optical_xyz)
        if urdf_camlink_quat is not None:
            quat = urdf_camlink_quat
        else:
            quat = np.array([0.0, 0.0, 0.0, 1.0])
        residual_stats = {
            'rms_m': out['rms_residual_m'],
            'std_m': out['std_residual_m'],
            'max_m': out['max_residual_m'],
            'kept_indices': out['kept_indices'],
            'rejected_indices': out['rejected_indices'],
            'per_pose_estimates': out['per_pose_estimates'].tolist(),
            'per_pose_residuals': out['per_pose_residuals'].tolist(),
        }
        return out['p_world_camera_link'], quat, residual_stats

    # -- output ------------------------------------------------------

    def _build_result(self, mode, samples, solved_xyz, solved_quat,
                      residual_stats, R_world_optical, drift, parent_frame
                      ) -> dict:
        rpy = _quat_to_rpy(solved_quat)
        return {
            'mode': mode,
            'method': 'ee_sweep_position_only',
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
            },
            'pose_visits': [
                {'pose_idx': s.pose_idx, 'comment': s.comment}
                for s in samples
            ],
            'frames': {
                'truth_chain': f'{parent_frame} -> {EE_URDF_FRAME}',
                'detection': f'{OPTICAL_FRAME} -> {EE_DETECTED_FRAME}',
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
            'source': 'ee_sweep',
        }
        with DEFAULT_CAMERA_POSE_CONFIG.open('w') as f:
            yaml.safe_dump(payload, f, sort_keys=False)
        return DEFAULT_CAMERA_POSE_CONFIG

    def _log_summary(self, mode: str, xyz: np.ndarray, quat: np.ndarray,
                     residual_stats: dict):
        rpy_deg = np.degrees(_quat_to_rpy(quat))
        self._emit_status(
            f'solved {mode}: xyz=({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}) '
            f'rpy_deg=({rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f})')
        self._emit_status(
            f'residual rms={residual_stats.get("rms_m", 0.0) * 1000:.2f} mm '
            f'max={residual_stats.get("max_m", 0.0) * 1000:.2f} mm')
        if residual_stats.get('rms_m', 0.0) > 0.020:
            self._emit_status(
                'WARNING: rms residual > 20 mm; URDF apriltag mount may '
                'be biased -- remeasure apriltag_ee_mount_joint origin')

    # -- yaml load ---------------------------------------------------

    def _load_yaml(self, path: Path) -> dict:
        with path.open() as f:
            return yaml.safe_load(f) or {}
