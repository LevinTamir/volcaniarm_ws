"""Least-squares calibration of the AprilTag mount offsets.

The URDF apriltag mounts (``volcaniarm_apriltag.xacro``) carry
placeholder translations, which show up as a constant systematic bias
in every accuracy run (``d_error`` far from zero). This module solves
for corrections to both mount translations from multi-pose run data so
the xacro can be updated and absolute accuracy becomes meaningful.

Model (planar arm, all in-plane rotations are about world X). For
observation k, taken with the tip at in-plane rotation ``R_wt_k``
(world <- right_arm_tip_link):

    r_k = v_det_k - v_urdf_k          # world Y-Z, 2-vector
    v_det_k  = det_ee_k  - det_base_k
    v_urdf_k = urdf_ee_k - urdf_base_k

    r_k = A_k @ d - b

where ``d = (dy, dz)`` is the correction to ADD to the EE tag mount
origin in the tip frame, ``b = (by, bz)`` the correction to ADD to the
base tag mount in world Y-Z, and ``A_k`` is the world-Y-Z image of the
tip frame's (y, z) axes. Stacking all observations gives a linear
system solved by ``numpy.linalg.lstsq``.

Why this works despite an uncalibrated camera pose: both detections
traverse the same world <- camera transform, so a camera TRANSLATION
error cancels exactly in ``det_ee - det_base``. Camera rotation error
does not cancel; ``estimate_camera_rotation=True`` adds a nuisance
unknown for the in-plane component as a cross-check.

Observability: each observation contributes 2 equations in 4 unknowns.
Two poses with distinct tip angles phi1 != phi2 already give rank 4
(det(R(phi1) - R(phi2)) = 2 - 2 cos(phi1 - phi2)), and the workspace
spans roughly 70 degrees of tip rotation, so a workspace sweep is
well-conditioned. A single-goal run is the degenerate case: only the
combination ``R(phi) d - b`` is observable, which is why static
accuracy data at one goal can never separate the two mounts.

The tip rotation comes from the ``tip_q*`` CSV columns when present
(recorded by the runner from TF). Legacy runs lack them, so a pure
Python mirror of the five-bar forward kinematics reconstructs the tip
angle from the recorded joint angles. Constants mirror
``volcaniarm_kinematics/include/volcaniarm_kinematics/params.hpp``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pandas as pd


# ---- five-bar mirror (volcaniarm_kinematics params.hpp defaults) ----

_L1 = 0.41621
_L2 = 0.6225
_L0 = 0.215
_BASE_Z = 0.0632
_ARM_LATERAL = 0.02
_LEFT_ELBOW_RPY = 0.7854
_RIGHT_ELBOW_RPY = -0.7854

# world <- volcaniarm_base_link rotation: Rz(pi) (world->base_link)
# followed by Rx(pi) (base_link->volcaniarm_base_link) = diag(-1, 1, -1).
R_WORLD_BASELINK_DEFAULT = np.diag([-1.0, 1.0, -1.0])


def _elbow_tip(elbow_rpy: float, theta: float,
               shoulder_y: float, lateral: float) -> tuple:
    """Mirror of five_bar.cpp elbowTip(): arm-joint origin in the
    volcaniarm_base_link Y-Z plane."""
    alpha = elbow_rpy + theta
    y = shoulder_y + lateral * math.cos(alpha) - _L1 * math.sin(alpha)
    z = _BASE_Z + lateral * math.sin(alpha) + _L1 * math.cos(alpha)
    return y, z


def tip_angle_from_thetas(theta_left: float, theta_right: float) -> float:
    """In-plane rotation (about volcaniarm_base_link X, radians) of
    right_arm_tip_link, reconstructed from the commanded joint angles.

    Mirrors elbowTip + circleIntersect + the cumulative right-side
    angle from five_bar.cpp passiveAngles(). The tip joint itself has
    no rpy offset in the URDF, so the cumulative right-arm angle IS the
    tip angle. Returns NaN when the linkage cannot close.
    """
    yl, zl = _elbow_tip(_LEFT_ELBOW_RPY, theta_left, -_L0, _ARM_LATERAL)
    yr, zr = _elbow_tip(_RIGHT_ELBOW_RPY, theta_right, _L0, -_ARM_LATERAL)
    dy = yl - yr
    dz = zl - zr
    d = math.hypot(dy, dz)
    if d > 2.0 * _L2 or d < 1e-9:
        return float('nan')
    a = d / 2.0
    h = math.sqrt(max(0.0, _L2 * _L2 - a * a))
    my = yr + a * dy / d
    mz = zr + a * dz / d
    ee_y = my + h * dz / d
    ee_z = mz - h * dy / d
    return math.atan2(-(ee_y - yr), ee_z - zr)


def _rx(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]])


def _rot_from_quat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
         2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
         2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
         1 - 2 * (qx * qx + qy * qy)],
    ])


@dataclass
class MountSolution:
    """Result of a mount solve. All lengths in metres."""
    d_tip_yz: np.ndarray          # (2,) EE mount correction, tip frame
    b_world_yz: np.ndarray        # (2,) base mount correction, world frame
    b_parent_yz: np.ndarray       # (2,) same, in volcaniarm_base_link frame
    omega_rad: Optional[float]    # camera in-plane rotation nuisance, if fit
    rms_before_mm: float
    rms_after_mm: float
    cond: float                   # condition number of the design matrix
    n_obs: int
    phi_range_deg: tuple          # (min, max) tip angle seen in the data
    per_row_residual_mm: np.ndarray  # per-observation |residual| after fit

    def __str__(self) -> str:
        omega = (f'  camera rotation={math.degrees(self.omega_rad):+.3f} deg'
                 if self.omega_rad is not None else '')
        return (
            f'n={self.n_obs}  cond={self.cond:.1f}  '
            f'phi span {self.phi_range_deg[0]:.1f}..'
            f'{self.phi_range_deg[1]:.1f} deg\n'
            f'EE mount d (tip frame):   '
            f'dy={self.d_tip_yz[0] * 1000:+.2f} mm  '
            f'dz={self.d_tip_yz[1] * 1000:+.2f} mm\n'
            f'base mount b (world):     '
            f'by={self.b_world_yz[0] * 1000:+.2f} mm  '
            f'bz={self.b_world_yz[1] * 1000:+.2f} mm\n'
            f'rms {self.rms_before_mm:.2f} -> {self.rms_after_mm:.2f} mm'
            f'{omega}')


def solve_mounts(df: pd.DataFrame,
                 R_world_baselink: Optional[np.ndarray] = None,
                 estimate_camera_rotation: bool = False) -> MountSolution:
    """Solve mount corrections from stacked target-phase observations.

    ``df`` needs the world Y-Z origin columns (``det_base_y/z``,
    ``det_ee_y/z``, ``urdf_base_y/z``, ``urdf_ee_y/z``) plus either the
    ``tip_q*`` quaternion columns or ``theta_left`` / ``theta_right``
    for the kinematic fallback. Rows with NaN in any needed column are
    dropped. Use data spanning several goals; a single goal is
    degenerate (see module docstring), flagged by a huge ``cond``.
    """
    if R_world_baselink is None:
        R_world_baselink = R_WORLD_BASELINK_DEFAULT

    needed = ['det_base_y', 'det_base_z', 'det_ee_y', 'det_ee_z',
              'urdf_base_y', 'urdf_base_z', 'urdf_ee_y', 'urdf_ee_z',
              'theta_left', 'theta_right']
    data = df.dropna(subset=[c for c in needed if c in df.columns]).copy()
    if len(data) == 0:
        raise ValueError('no usable observations (all rows have NaN in '
                         'detection or URDF columns)')

    residuals = []
    A_blocks = []
    phis = []
    J = np.array([[0.0, -1.0], [1.0, 0.0]])  # d/domega of Rx(omega) on YZ
    v_urdf_rows = []
    have_quats = all(c in data.columns
                     for c in ('tip_qx', 'tip_qy', 'tip_qz', 'tip_qw'))
    for _, row in data.iterrows():
        v_det = np.array([row['det_ee_y'] - row['det_base_y'],
                          row['det_ee_z'] - row['det_base_z']])
        v_urdf = np.array([row['urdf_ee_y'] - row['urdf_base_y'],
                           row['urdf_ee_z'] - row['urdf_base_z']])
        phi = tip_angle_from_thetas(row['theta_left'], row['theta_right'])
        if have_quats and np.isfinite(row['tip_qw']):
            R_wt = _rot_from_quat(row['tip_qx'], row['tip_qy'],
                                  row['tip_qz'], row['tip_qw'])
        else:
            if math.isnan(phi):
                continue
            R_wt = R_world_baselink @ _rx(phi)
        A = R_wt[1:3, 1:3]
        residuals.append(v_det - v_urdf)
        A_blocks.append(A)
        v_urdf_rows.append(v_urdf)
        phis.append(phi)

    n = len(residuals)
    if n < 2:
        raise ValueError(f'only {n} usable observation(s); need at least '
                         'two poses with distinct tip angles')

    r = np.concatenate(residuals)
    n_unknowns = 5 if estimate_camera_rotation else 4
    M = np.zeros((2 * n, n_unknowns))
    for k, (A, v_urdf) in enumerate(zip(A_blocks, v_urdf_rows)):
        M[2 * k:2 * k + 2, 0:2] = A
        M[2 * k:2 * k + 2, 2:4] = -np.eye(2)
        if estimate_camera_rotation:
            M[2 * k:2 * k + 2, 4] = J @ v_urdf

    x, _, _, _ = np.linalg.lstsq(M, r, rcond=None)
    fit_residual = r - M @ x
    per_row = np.linalg.norm(fit_residual.reshape(-1, 2), axis=1)

    d_tip = x[0:2]
    b_world = x[2:4]
    omega = float(x[4]) if estimate_camera_rotation else None
    # Base mount parent is volcaniarm_base_link; map the world-frame
    # correction into it (YZ block of R_baselink_world).
    B = R_world_baselink[1:3, 1:3].T
    b_parent = B @ b_world

    phis_deg = np.degrees([p for p in phis if not math.isnan(p)])
    return MountSolution(
        d_tip_yz=d_tip,
        b_world_yz=b_world,
        b_parent_yz=b_parent,
        omega_rad=omega,
        rms_before_mm=float(np.sqrt(np.mean(
            np.linalg.norm(np.array(residuals), axis=1) ** 2)) * 1000.0),
        rms_after_mm=float(np.sqrt(np.mean(per_row ** 2)) * 1000.0),
        cond=float(np.linalg.cond(M)),
        n_obs=n,
        phi_range_deg=(float(phis_deg.min()), float(phis_deg.max()))
        if len(phis_deg) else (float('nan'), float('nan')),
        per_row_residual_mm=per_row * 1000.0,
    )


def suggest_xacro(sol: MountSolution,
                  current_base_xyz=(0.65, 0.0, -0.03),
                  current_ee_xyz=(0.02, 0.0, 0.0)) -> str:
    """Ready-to-paste origin lines for volcaniarm_apriltag.xacro.

    The corrections are ADDED to the current mount translations:
    the EE correction in the right_arm_tip_link (mount joint parent)
    frame, the base correction in the volcaniarm_base_link frame. X is
    out of the arm's working plane and unobservable from Y-Z data, so
    it is left at the current value.
    """
    ee = (current_ee_xyz[0],
          current_ee_xyz[1] + sol.d_tip_yz[0],
          current_ee_xyz[2] + sol.d_tip_yz[1])
    base = (current_base_xyz[0],
            current_base_xyz[1] + sol.b_parent_yz[0],
            current_base_xyz[2] + sol.b_parent_yz[1])
    return (
        'Suggested mount origins for volcaniarm_apriltag.xacro\n'
        '(X unchanged: out-of-plane, not observable from Y-Z data)\n\n'
        'apriltag_ee_mount_joint:\n'
        f'  <origin xyz="{ee[0]:.4f} {ee[1]:.4f} {ee[2]:.4f}" '
        'rpy="${-pi/3.4} 0 0" />\n\n'
        'apriltag_base_mount_joint:\n'
        f'  <origin xyz="{base[0]:.4f} {base[1]:.4f} {base[2]:.4f}" '
        'rpy="0 0 0" />\n')
