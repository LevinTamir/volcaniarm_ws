"""Loaders for calibration run output.

A run lives at ``data/<test_name>/<YYYY-MM-DD>/<HH-MM-SS>/`` with three
files written by ``RunWriter``: ``config.yaml``, ``tag_observations.csv``
(ground-truth base->ee tag transform per sample) and ``fk_poses.csv``
(analytic FK at each commanded joint state). The schema is fixed in
``data_writer.py`` and the loaders here expect the same column names.

Helpers are intentionally tiny -- the notebooks do the analysis. These
just standardise path resolution and the join between tag and FK rows.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import yaml


DEFAULT_DATA_ROOT = (
    Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/data')
    .expanduser())


# ---- URDF apriltag mount offsets ---------------------------------
# Mirrored from src/volcaniarm_description/urdf/volcaniarm_apriltag.xacro.
# UPDATE THESE IF THE XACRO CHANGES -- they're the bridge that turns
# tag observations into thesis-quotable residuals in volcaniarm_base_link
# frame. Keeping them in sync with the URDF is a manual step until we
# wire up automated extraction from /tf_static at run time.
#
# Chain (volcaniarm_base_link -> apriltag_base_link):
#   apriltag_base_mount_joint: xyz=(0.65, 0.0, -0.03), rpy=(0, 0, 0)
#   apriltag_base_joint:       xyz=(0, 0, 0),          rpy=(0, pi/2, 0)
#
# Chain (right_arm_tip_link -> apriltag_ee_link):
#   apriltag_ee_mount_joint:   xyz=(0.02, 0, 0),       rpy=(-pi/3.4, 0, 0)
#   apriltag_ee_joint:         xyz=(0.02, 0, 0),       rpy=(0, pi/2, 0)


def _se3(xyz, rpy):
    """4x4 SE(3) from URDF-style xyz + rpy (roll-pitch-yaw, intrinsic
    XYZ aka R = Rz @ Ry @ Rx)."""
    cx, sx = math.cos(rpy[0]), math.sin(rpy[0])
    cy, sy = math.cos(rpy[1]), math.sin(rpy[1])
    cz, sz = math.cos(rpy[2]), math.sin(rpy[2])
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


T_VOLCANIARM_TO_APRILTAG_BASE = (
    _se3([0.65, 0.0, -0.03], [0.0, 0.0, 0.0]) @
    _se3([0.0, 0.0, 0.0], [0.0, math.pi / 2, 0.0])
)
T_APRILTAG_BASE_TO_VOLCANIARM = np.linalg.inv(T_VOLCANIARM_TO_APRILTAG_BASE)

T_TIP_TO_APRILTAG_EE = (
    _se3([0.02, 0.0, 0.0], [-math.pi / 3.4, 0.0, 0.0]) @
    _se3([0.02, 0.0, 0.0], [0.0, math.pi / 2, 0.0])
)


def list_runs(test_name: str,
              data_root: Optional[Path] = None) -> list:
    """Return all run dirs for ``test_name``, oldest-first.

    Skips entries that don't look like a complete run (must contain at
    least ``config.yaml``). Returns an empty list if the test has no
    runs yet.
    """
    root = (data_root or DEFAULT_DATA_ROOT) / test_name
    if not root.exists():
        return []
    runs = []
    for day_dir in sorted(root.iterdir()):
        if not day_dir.is_dir():
            continue
        for run_dir in sorted(day_dir.iterdir()):
            if not run_dir.is_dir():
                continue
            if (run_dir / 'config.yaml').exists():
                runs.append(run_dir)
    return runs


def latest_run(test_name: str,
               data_root: Optional[Path] = None,
               status: Optional[str] = None) -> Path:
    """Return the most recent run dir for ``test_name``.

    If ``status`` is given (e.g. 'completed'), filter to runs whose
    ``config.yaml`` records that status. Raises ``FileNotFoundError``
    if there are no matching runs.
    """
    runs = list_runs(test_name, data_root)
    if status is not None:
        runs = [r for r in runs
                if _safe_status(r) == status]
    if not runs:
        raise FileNotFoundError(
            f'no runs found for test={test_name!r} '
            f'(status filter={status!r})')
    return runs[-1]


def _safe_status(run_dir: Path) -> Optional[str]:
    try:
        with (run_dir / 'config.yaml').open() as f:
            cfg = yaml.safe_load(f) or {}
        return cfg.get('status')
    except OSError:
        return None


def load_run(run_dir: Path) -> dict:
    """Load a single run dir into a dict of {config, tag, fk}.

    config: parsed config.yaml as a Python dict.
    tag: tag_observations.csv as a pandas DataFrame.
    fk: fk_poses.csv as a pandas DataFrame.
    """
    run_dir = Path(run_dir)
    with (run_dir / 'config.yaml').open() as f:
        config = yaml.safe_load(f) or {}
    tag = pd.read_csv(run_dir / 'tag_observations.csv')
    fk = pd.read_csv(run_dir / 'fk_poses.csv')
    return {'config': config, 'tag': tag, 'fk': fk, 'run_dir': run_dir}


def tag_in_base_frame(tag: pd.DataFrame) -> pd.DataFrame:
    """Transform tag observations from `apriltag_marker_base` frame
    into `volcaniarm_base_link` frame using the URDF static apriltag
    base offset. Adds columns ``x_base, y_base, z_base`` (in metres).

    The tag observation `(x, y, z)` is the EE-tag position as seen
    from the base tag; that frame is geometrically `apriltag_base_link`
    (URDF). Bringing it into `volcaniarm_base_link` lets us compare
    directly against FK / commanded values in the same frame.
    """
    out = tag.copy()
    n = len(out)
    if n == 0:
        for c in ('x_base', 'y_base', 'z_base'):
            out[c] = pd.Series(dtype=float)
        return out
    pts = np.column_stack([out['x'].to_numpy(),
                           out['y'].to_numpy(),
                           out['z'].to_numpy(),
                           np.ones(n)])
    pts_base = (T_VOLCANIARM_TO_APRILTAG_BASE @ pts.T).T
    out['x_base'] = pts_base[:, 0]
    out['y_base'] = pts_base[:, 1]
    out['z_base'] = pts_base[:, 2]
    return out


def fk_apriltag_position(fk: pd.DataFrame) -> pd.DataFrame:
    """Where the URDF says the EE apriltag should be at each FK pose,
    in `volcaniarm_base_link` frame. Adds ``fk_apriltag_x/y/z`` (m).

    NOTE: assumes the tip's orientation in `volcaniarm_base_link` is
    identity (only translation from FK). True for a 2-DOF planar arm
    *if* `right_arm_tip_link` happens to be parallel to the base, which
    is approximately the case here. For exact thesis-grade numbers,
    the runner would need to record the tip orientation per FK; that's
    a future enhancement. Translation-only application of the EE-tag
    offset still gives a usable approximation for residual analysis.
    """
    out = fk.copy()
    n = len(out)
    if n == 0:
        for c in ('fk_apriltag_x', 'fk_apriltag_y', 'fk_apriltag_z'):
            out[c] = pd.Series(dtype=float)
        return out
    tip_xyz = np.column_stack([out['fk_x'].to_numpy(),
                               out['fk_y'].to_numpy(),
                               out['fk_z'].to_numpy()])
    apriltag_offset = T_TIP_TO_APRILTAG_EE[:3, 3]
    apriltag_xyz = tip_xyz + apriltag_offset[None, :]
    out['fk_apriltag_x'] = apriltag_xyz[:, 0]
    out['fk_apriltag_y'] = apriltag_xyz[:, 1]
    out['fk_apriltag_z'] = apriltag_xyz[:, 2]
    return out


def align_fk_to_tag(tag: pd.DataFrame, fk: pd.DataFrame) -> pd.DataFrame:
    """Join target-phase tag samples to their commanded FK pose and
    add scalar distance metrics that are robust to frame-convention
    differences between apriltag_ros and the URDF.

    Returns columns:
      - All tag observation columns (x, y, z in apriltag_marker_base
        frame; quaternion).
      - ``fk_x, fk_y, fk_z`` from FK (volcaniarm_base_link frame).
      - ``d_tag`` = ``|tag_observation|`` (m): distance from base
        AprilTag center to EE AprilTag center, scalar. **Rotation-
        invariant** so any URDF-vs-apriltag_ros orientation mismatch
        does not affect this number.
      - ``d_fk`` = ``|FK position|`` (m): distance from
        volcaniarm_base_link to right_arm_tip_link.
      - ``d_diff = d_tag - d_fk`` (m): the difference between the
        two distances. Carries only the URDF mount-offset translation
        bias (rotation cancels in scalar distances).

    Vector residual columns (``dx, dy, dz``) are intentionally NOT
    computed: they require a frame bridge between apriltag_marker_base
    (apriltag_ros image-derived axes) and volcaniarm_base_link (URDF
    body axes). The two are not the same orientation in general, and
    the static rotation between them is not encoded anywhere we can
    consult without live TF. Use ``d_diff`` for accuracy and the
    cluster scatter helpers in ``metrics.py`` for repeatability.
    """
    target_tag = tag[tag['phase'] == 'target'].copy()
    target_tag['target_idx'] = target_tag['target_idx'].astype(int)
    fk = fk.copy()
    fk['target_idx'] = fk['target_idx'].astype(int)
    merged = target_tag.merge(
        fk[['cycle', 'target_idx', 'fk_x', 'fk_y', 'fk_z']],
        on=['cycle', 'target_idx'], how='left')
    merged['d_tag'] = np.sqrt(
        merged['x'] ** 2 + merged['y'] ** 2 + merged['z'] ** 2)
    merged['d_fk'] = np.sqrt(
        merged['fk_x'] ** 2 + merged['fk_y'] ** 2 + merged['fk_z'] ** 2)
    merged['d_diff'] = merged['d_tag'] - merged['d_fk']
    return merged
