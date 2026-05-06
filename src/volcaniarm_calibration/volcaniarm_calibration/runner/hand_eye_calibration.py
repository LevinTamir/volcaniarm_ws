"""Hand-eye camera calibration via OpenCV (cart-mounted scenario).

Drives the arm through a curated list of safe poses, captures the
FK pose of the EE tip and the apriltag detection of the EE tag at
each, and solves for T(base -> camera) using cv2.calibrateHandEye
with the Daniilidis dual-quaternion algorithm (handles limited
rotation diversity better than Tsai-Lenz, which matters for the
2-DOF planar arm).

Public API matches CameraCalibrationRunner so the dashboard can
wire it the same way: request, cancel, is_busy, status_cb,
progress_cb, finished_cb, shutdown.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np
import yaml

from rclpy.duration import Duration as RclpyDuration
from rclpy.time import Time as RclpyTime

from .calibration_runner import CalibrationRunner
from . import camera_calibration as cc  # SE3 helpers, frame constants


# Conservative safe envelope (matches the single-shot localizer).
# The runner refuses poses outside this box even if the YAML lists
# them, so a typo can't drive the arm into a singularity.
SAFE_Y_MIN = -0.30
SAFE_Y_MAX = 0.30
SAFE_Z_MIN = 0.35
SAFE_Z_MAX = 0.65

DEFAULT_DATA_ROOT = (
    Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/data')
    .expanduser())
DEFAULT_CART_CONFIG_PATH = (
    Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/'
         'config/camera_pose_cart.yaml').expanduser())

# Frames. The hand-eye solve uses the EE tag (apriltag_marker_ee) as
# the moving target -- the base tag is not assumed to be present.
ROBOT_BASE_FRAME = 'volcaniarm_base_link'
ROBOT_EE_FRAME = 'right_arm_tip_link'
CAMERA_FRAME = 'camera_color_optical_frame'
DETECTED_EE_TAG_FRAME = 'apriltag_marker_ee'
CAMERA_MOUNT_FRAME = 'camera_mount_rev_link'
CAMERA_LINK_FRAME = 'camera_link'


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


class HandEyeCameraCalibrationRunner:
    """EE-sweep hand-eye calibration using cv2.calibrateHandEye.

    Composes with CalibrationRunner so the same _stop_event drives
    Cancel for whichever flow is active (single-shot or sweep),
    and motion / IK / TF lookups all reuse the existing primitives.
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

    def request(self, yaml_path: Path) -> bool:
        """Kick off a hand-eye sweep. Returns False if busy."""
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
        self._runner._stop_event.set()  # noqa: SLF001

    def shutdown(self):
        self.cancel()
        if self._thread is not None:
            self._thread.join(timeout=10.0)

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

        poses = self._validate_and_resolve(poses_raw)
        if len(poses) < min_successful_samples:
            reason = (f'only {len(poses)} pose(s) reachable; need at '
                      f'least {min_successful_samples}')
            self._emit_status(f'aborting: {reason}')
            self._emit_finished('failed', None, reason)
            return

        # Drive the arm + collect (FK pose, EE tag detection) per pose.
        samples = self._sweep_poses(
            poses, trajectory_duration, settle_time,
            detection_timeout_s, detection_max_age_s)

        if self._runner._stop_event.is_set():  # noqa: SLF001
            self._emit_status('hand-eye calibration canceled by operator')
            self._emit_finished('canceled', None, '')
            return

        if len(samples) < min_successful_samples:
            reason = (f'only {len(samples)}/{len(poses)} poses produced '
                      f'a fresh EE-tag detection; need at least '
                      f'{min_successful_samples}')
            self._emit_status(f'aborting: {reason}')
            self._emit_finished('failed', None, reason)
            return

        # Solve hand-eye via cv2.
        self._emit_status(
            f'solving hand-eye from {len(samples)} samples '
            f'(method=Daniilidis)')
        try:
            base_to_camera_xyz, base_to_camera_quat = \
                self._solve_handeye(samples)
        except Exception as exc:
            reason = f'cv2.calibrateHandEye raised: {exc}'
            self._emit_status(f'aborting: {reason}')
            self._emit_finished('failed', None, reason)
            return

        # Compose with the URDF chain to get the camera_joint override
        # (camera_mount_rev_link -> camera_link), same conversion as
        # the single-shot localizer.
        override = self._compute_urdf_override(
            base_to_camera_xyz, base_to_camera_quat)
        if override is None:
            reason = ('URDF chain incomplete: cannot compose '
                      'camera_mount_rev_link -> camera_link')
            self._emit_status(f'aborting: {reason}')
            self._emit_finished('failed', None, reason)
            return

        result = self._build_result(
            samples, base_to_camera_xyz, base_to_camera_quat, override)

        self._log_result_block(result)
        result_path = self._save_yaml(result)
        self._emit_status(f'saved per-run audit: {result_path}')
        config_path = self._save_camera_pose_config(override)
        self._emit_status(
            f'updated default camera pose config: {config_path}')

        # Park at home so the arm ends in a known state.
        self._emit_status('parking arm at home (theta=0, 0)')
        ok = self._runner._send_and_wait(  # noqa: SLF001
            self._runner._default_joint_names,  # noqa: SLF001
            0.0, 0.0,
            self._runner._default_trajectory_duration)  # noqa: SLF001
        if not ok:
            self._emit_status('park-at-home move did not complete cleanly')

        self._emit_finished('completed', result_path, '')

    # -- pose validation ---------------------------------------------

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

    # -- sweep + sample ---------------------------------------------

    def _sweep_poses(self, poses: list, trajectory_duration: float,
                     settle_time: float, detection_timeout_s: float,
                     detection_max_age_s: float) -> list:
        """Drive arm through each pose, sample (T_base_ee, T_cam_marker)."""
        samples = []
        total = len(poses)
        for idx, p in enumerate(poses, start=1):
            if self._runner._stop_event.is_set():  # noqa: SLF001
                return samples
            self._emit_status(
                f'pose {idx}/{total} ({p.y:.3f}, {p.z:.3f}) {p.comment}')
            ok = self._runner._send_and_wait(  # noqa: SLF001
                self._runner._default_joint_names,  # noqa: SLF001
                p.theta_right, p.theta_left, trajectory_duration)
            if not ok:
                if self._runner._stop_event.is_set():  # noqa: SLF001
                    return samples
                self._runner._failure_reason = (  # noqa: SLF001
                    f'motion failed at pose {idx} '
                    f'({p.y:.3f}, {p.z:.3f})')
                self._emit_status(
                    f'aborting: {self._runner._failure_reason}')  # noqa: SLF001
                self._runner._stop_event.set()  # noqa: SLF001
                return samples

            settled = self._sleep_with_cancel(settle_time)
            if not settled:
                return samples

            # Wait for a fresh EE-tag detection. If the tag isn't visible
            # at this pose, skip with warning rather than aborting.
            if not self._wait_fresh_detection(
                    detection_timeout_s, detection_max_age_s):
                self._emit_status(
                    f'pose {idx}: EE tag not visible; skipping sample')
                self._emit_progress(idx, total)
                continue

            # Look up the two transforms we need for hand-eye.
            tf_base_ee = self._lookup_static(ROBOT_BASE_FRAME, ROBOT_EE_FRAME)
            tf_cam_marker = self._lookup_static(
                CAMERA_FRAME, DETECTED_EE_TAG_FRAME)
            if tf_base_ee is None or tf_cam_marker is None:
                self._emit_status(
                    f'pose {idx}: TF lookup failed; skipping sample')
                self._emit_progress(idx, total)
                continue

            samples.append((tf_base_ee, tf_cam_marker, p))
            self._emit_progress(idx, total)
        return samples

    def _wait_fresh_detection(self, timeout_s: float,
                              max_age_s: float) -> bool:
        """Block up to timeout_s for a fresh EE-tag detection, by
        stamp-progression freshness check (matches the single-shot
        localizer's pattern)."""
        deadline = time.monotonic() + timeout_s
        last_stamp_ns: Optional[int] = None
        last_change_mono: Optional[float] = None
        while time.monotonic() < deadline:
            if self._runner._stop_event.is_set():  # noqa: SLF001
                return False
            tf = self._lookup_static(CAMERA_FRAME, DETECTED_EE_TAG_FRAME)
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

    def _lookup_static(self, parent: str, child: str):
        try:
            return self._runner._tf_buffer.lookup_transform(  # noqa: SLF001
                parent, child, RclpyTime(),
                timeout=RclpyDuration(seconds=0.5))
        except Exception:
            return None

    def _sleep_with_cancel(self, seconds: float) -> bool:
        if seconds <= 0:
            return True
        return not self._runner._stop_event.wait(seconds)  # noqa: SLF001

    # -- the math ----------------------------------------------------

    def _solve_handeye(self, samples: list):
        """Run cv2.calibrateHandEye on the collected samples.

        For eye-on-base (camera fixed in environment, marker on robot
        EE), we pass the FK transforms (base -> EE) and the apriltag
        detections (camera -> marker). cv2.calibrateHandEye expects
        the eye-in-hand convention internally, so we pass the eye-on-
        base inputs in the slot that corresponds to the same axis of
        rigidity: see opencv docs / Daniilidis 1999. The returned
        rotation/translation is interpreted as base -> camera.

        Returns (xyz, quat) for T(volcaniarm_base_link -> camera).
        """
        R_base_ee = []
        t_base_ee = []
        R_cam_marker = []
        t_cam_marker = []
        for tf_base_ee, tf_cam_marker, _p in samples:
            be_xyz, be_quat = cc._xyz_quat_from_transform(tf_base_ee.transform)
            cm_xyz, cm_quat = cc._xyz_quat_from_transform(tf_cam_marker.transform)
            R_base_ee.append(cc._quat_to_matrix(be_quat))
            t_base_ee.append(be_xyz)
            R_cam_marker.append(cc._quat_to_matrix(cm_quat))
            t_cam_marker.append(cm_xyz)

        R_arr = np.array(R_base_ee)
        t_arr = np.array(t_base_ee).reshape(-1, 3, 1)
        Rcm_arr = np.array(R_cam_marker)
        tcm_arr = np.array(t_cam_marker).reshape(-1, 3, 1)

        # method=DANIILIDIS: dual-quaternion solve, performs better than
        # Tsai-Lenz when rotation diversity is limited (our 2-DOF case).
        R_base_to_cam, t_base_to_cam = cv2.calibrateHandEye(
            R_arr, t_arr, Rcm_arr, tcm_arr,
            method=cv2.CALIB_HAND_EYE_DANIILIDIS)
        xyz = np.asarray(t_base_to_cam, dtype=float).reshape(3)
        quat = cc._matrix_to_quat(np.asarray(R_base_to_cam, dtype=float))
        return xyz, quat

    def _compute_urdf_override(self, base_to_cam_xyz, base_to_cam_quat):
        """Same composition the single-shot localizer does:
        T(camera_mount_rev_link -> camera_link) =
            inv(T(robot_base -> mount))           [URDF static]
            · T(robot_base -> camera_optical, solved)
            · inv(T(camera_link -> camera_optical))   [URDF static]
        Returns None if either URDF lookup fails.
        """
        robot_to_mount = self._lookup_static(
            ROBOT_BASE_FRAME, CAMERA_MOUNT_FRAME)
        link_to_optical = self._lookup_static(
            CAMERA_LINK_FRAME, CAMERA_FRAME)
        if robot_to_mount is None or link_to_optical is None:
            return None
        rm_xyz, rm_quat = cc._xyz_quat_from_transform(robot_to_mount.transform)
        link_xyz, link_quat = cc._xyz_quat_from_transform(
            link_to_optical.transform)
        M_robot_to_mount = cc._se3(rm_xyz, rm_quat)
        M_robot_to_optical = cc._se3(base_to_cam_xyz, base_to_cam_quat)
        M_link_to_optical = cc._se3(link_xyz, link_quat)
        M = (cc._se3_inv(M_robot_to_mount)
             @ M_robot_to_optical
             @ cc._se3_inv(M_link_to_optical))
        xyz, quat = cc._se3_to_xyz_quat(M)
        rpy = cc._quat_to_rpy(quat)
        return {
            'parent': CAMERA_MOUNT_FRAME,
            'child': CAMERA_LINK_FRAME,
            'xyz': [float(v) for v in xyz],
            'quat': [float(v) for v in quat],
            'rpy': [float(v) for v in rpy],
            'rpy_deg': [float(math.degrees(v)) for v in rpy],
        }

    # -- output -----------------------------------------------------

    def _build_result(self, samples, base_to_cam_xyz, base_to_cam_quat,
                      override) -> dict:
        rpy = cc._quat_to_rpy(base_to_cam_quat)
        return {
            'method': 'cv2.calibrateHandEye DANIILIDIS',
            'samples_used': len(samples),
            'base_to_camera_optical': {
                'xyz': [float(v) for v in base_to_cam_xyz],
                'quat': [float(v) for v in base_to_cam_quat],
                'rpy_deg': [float(math.degrees(v)) for v in rpy],
            },
            'urdf_camera_joint_override': override,
            'frames': {
                'robot_base': ROBOT_BASE_FRAME,
                'robot_ee': ROBOT_EE_FRAME,
                'detected_ee_tag': DETECTED_EE_TAG_FRAME,
                'camera': CAMERA_FRAME,
                'camera_mount': CAMERA_MOUNT_FRAME,
                'camera_link': CAMERA_LINK_FRAME,
            },
            'pose_visits': [
                {'y': float(p.y), 'z': float(p.z),
                 'comment': p.comment}
                for _tfb, _tfc, p in samples
            ],
            'timestamp': datetime.now().isoformat(),
        }

    def _log_result_block(self, result: dict):
        b2c = result['base_to_camera_optical']
        ovr = result['urdf_camera_joint_override']
        self._emit_status(
            f'solved T(base->camera_optical): xyz=({b2c["xyz"][0]:.4f}, '
            f'{b2c["xyz"][1]:.4f}, {b2c["xyz"][2]:.4f}) '
            f'rpy_deg=({b2c["rpy_deg"][0]:.2f}, {b2c["rpy_deg"][1]:.2f}, '
            f'{b2c["rpy_deg"][2]:.2f})')
        self._emit_status(
            f'URDF camera_joint origin: xyz=({ovr["xyz"][0]:.6f}, '
            f'{ovr["xyz"][1]:.6f}, {ovr["xyz"][2]:.6f}) '
            f'rpy=({ovr["rpy"][0]:.6f}, {ovr["rpy"][1]:.6f}, '
            f'{ovr["rpy"][2]:.6f})')

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

    def _save_camera_pose_config(self, override: dict) -> Path:
        DEFAULT_CART_CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            'parent_frame': override['parent'],
            'child_frame': override['child'],
            'xyz': override['xyz'],
            'quat': override['quat'],
            'rpy': override['rpy'],
            'rpy_deg': override['rpy_deg'],
            'last_updated': datetime.now().isoformat(timespec='seconds'),
            'source': 'hand_eye_calibration',
        }
        with DEFAULT_CART_CONFIG_PATH.open('w') as f:
            yaml.safe_dump(payload, f, sort_keys=False)
        return DEFAULT_CART_CONFIG_PATH

    def _load_yaml(self, path: Path) -> dict:
        with path.open() as f:
            return yaml.safe_load(f) or {}
