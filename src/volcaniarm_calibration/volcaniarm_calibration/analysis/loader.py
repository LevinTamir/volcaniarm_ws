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

from pathlib import Path
from typing import Optional

import pandas as pd
import yaml


DEFAULT_DATA_ROOT = (
    Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/data')
    .expanduser())


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


def align_fk_to_tag(tag: pd.DataFrame, fk: pd.DataFrame) -> pd.DataFrame:
    """Join target-phase tag samples to their commanded FK pose.

    Returns a DataFrame with one row per tag sample (target phase only)
    augmented with the corresponding ``fk_x``, ``fk_y``, ``fk_z`` and
    per-axis residuals ``dx``, ``dy``, ``dz`` plus a magnitude ``dr``.
    Home-phase rows (no FK twin) are dropped.
    """
    target_tag = tag[tag['phase'] == 'target'].copy()
    # target_idx may carry empty strings from home-phase rows in the
    # CSV; coerce both sides to int for the join.
    target_tag['target_idx'] = target_tag['target_idx'].astype(int)
    fk = fk.copy()
    fk['target_idx'] = fk['target_idx'].astype(int)
    merged = target_tag.merge(
        fk[['cycle', 'target_idx', 'fk_x', 'fk_y', 'fk_z']],
        on=['cycle', 'target_idx'], how='left')
    merged['dx'] = merged['x'] - merged['fk_x']
    merged['dy'] = merged['y'] - merged['fk_y']
    merged['dz'] = merged['z'] - merged['fk_z']
    merged['dr'] = (merged['dx'] ** 2
                    + merged['dy'] ** 2
                    + merged['dz'] ** 2) ** 0.5
    return merged
