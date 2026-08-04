"""Persists calibration runs to a thesis-friendly directory layout.

Layout, grouped by test type then day then start time so weeks of
runs across multiple test types stay legible:

    data/<test_name>/<YYYY-MM-DD>/<HH-MM-SS>/
      config.yaml           test config + git SHA + status
      tag_observations.csv  base->ee apriltag transform per sample,
                            tagged with phase (home|target). Ground truth.
      fk_poses.csv          analytic FK at each commanded joint state.
      run.log               logger output (written by RunWriter.attach_log)

ISO date and dash-separated time make the directory listing sort
chronologically and avoid colon-in-path issues on tooling.
Residuals, per-target aggregates, and repeatability stats are computed
in the analysis notebooks from these two CSVs.
"""

import csv
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Optional

import yaml


TAG_OBS_FIELDS = [
    'run_id', 'phase', 'cycle', 'target_idx', 'sample_idx', 't_ros_ns',
    'theta_right', 'theta_left',
    # Raw apriltag-frame transform (apriltag_marker_base -> apriltag_marker_ee).
    # Kept for tag-frame diagnostics; not used by the headline metrics.
    'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
    # World-frame Y-Z origins at capture instant. Both detection and
    # URDF expressed in the same world-aligned frame so the Y and Z
    # axes refer to the same physical directions.
    'det_base_y', 'det_base_z', 'det_ee_y', 'det_ee_z',
    'urdf_base_y', 'urdf_base_z', 'urdf_ee_y', 'urdf_ee_z',
    # Headline scalars (metres):
    # d_detected = ||(det_ee - det_base)_yz||  (world-frame Y-Z)
    # d_urdf     = ||(urdf_ee - urdf_base)_yz|| (world-frame Y-Z)
    # d_error    = d_detected - d_urdf  (signed accuracy residual)
    # NaN when the URDF chain wasn't reachable at capture time.
    'd_detected', 'd_urdf', 'd_error',
    # world -> right_arm_tip_link rotation at capture, recorded as
    # tip-orientation provenance for diagnostics; NaN when the lookup
    # failed. Appended last so older CSVs stay column-compatible.
    'tip_qx', 'tip_qy', 'tip_qz', 'tip_qw',
    # Visit label (the target's '(y,z)' string) and approach direction
    # tag ('+y'/'-y' for the backlash test, '' otherwise). Appended
    # after tip_q* to keep older CSVs column-compatible.
    'label', 'approach',
]

FK_FIELDS = [
    'run_id', 'cycle', 'target_idx',
    'theta_right', 'theta_left',
    'fk_x', 'fk_y', 'fk_z',
    'label', 'approach',
]


def load_resume_state(run_dir: Path, num_goals: int,
                      num_cycles: int) -> dict:
    """Inspect a run directory and report where a resume would pick up.

    Returns ``{'next_cycle': int, 'done_visits': int, 'resumable': bool}``.
    ``next_cycle`` is the first cycle without a full set of target
    captures (one per goal); a run whose every cycle is complete is not
    resumable. A cycle that was interrupted partway through a multi-goal
    sweep is repeated in full on resume, so those goals gain an extra
    sample; harmless for the aggregated metrics, noted in the docs.
    """
    per_cycle: dict = {}
    csv_path = Path(run_dir) / 'tag_observations.csv'
    if csv_path.exists():
        with csv_path.open(newline='') as f:
            for row in csv.DictReader(f):
                if row.get('phase') != 'target':
                    continue
                try:
                    c = int(float(row['cycle']))
                except (TypeError, ValueError):
                    continue
                per_cycle[c] = per_cycle.get(c, 0) + 1
    next_cycle = num_cycles + 1
    for c in range(1, num_cycles + 1):
        if per_cycle.get(c, 0) < num_goals:
            next_cycle = c
            break
    done_visits = (next_cycle - 1) * num_goals
    return {'next_cycle': next_cycle,
            'done_visits': done_visits,
            'resumable': next_cycle <= num_cycles}


class RunWriter:
    """Owns the per-run output directory and CSV writers.

    Use as a context manager:

        with RunWriter(base_dir, test_name, config) as rw:
            rw.add_tag_observation(...)
            rw.add_fk(...)
            rw.finalize(status='completed')

    Pass ``resume_dir`` to reopen an existing (failed) run instead of
    creating a new directory: CSV rows are appended, the original
    config.yaml (run_id, goals, mounts, git sha) is preserved, the
    status flips back to in_progress and a resume note is recorded.
    """

    def __init__(self, base_dir: Path, test_name: str, config: dict,
                 resume_dir: Optional[Path] = None):
        self.test_name = test_name
        self.config = dict(config)
        self.resume_dir = Path(resume_dir) if resume_dir else None
        now = datetime.now()
        self.day = now.strftime('%Y-%m-%d')
        self.clock = now.strftime('%H-%M-%S')
        # Kept for back-compat with anything reading config.yaml's
        # `timestamp` field. Schema-stable across the layout change.
        self.timestamp = now.strftime('%Y%m%d_%H%M%S')
        if self.resume_dir is not None:
            self.run_dir = self.resume_dir
            parts = self.run_dir.parts
            self.run_id = '/'.join(parts[-3:])
        else:
            self.run_id = f'{test_name}/{self.day}/{self.clock}'
            self.run_dir = (Path(base_dir).expanduser()
                            / test_name / self.day / self.clock)
        self.status: str = 'in_progress'
        self.failure_reason: Optional[str] = None
        self._tag_file = None
        self._fk_file = None
        self._tag_writer = None
        self._fk_writer = None

    def __enter__(self):
        resuming = self.resume_dir is not None
        self.run_dir.mkdir(parents=True, exist_ok=True)
        mode = 'a' if resuming else 'w'
        self._tag_file = (self.run_dir / 'tag_observations.csv').open(
            mode, newline='')
        self._tag_writer = csv.DictWriter(self._tag_file,
                                          fieldnames=TAG_OBS_FIELDS)
        self._fk_file = (self.run_dir / 'fk_poses.csv').open(mode, newline='')
        self._fk_writer = csv.DictWriter(self._fk_file, fieldnames=FK_FIELDS)
        if resuming:
            self._reopen_config()
        else:
            self._tag_writer.writeheader()
            self._fk_writer.writeheader()
            self._write_config()
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc is not None and self.status == 'in_progress':
            self.status = 'failed'
        for f in (self._tag_file, self._fk_file):
            if f is not None:
                f.close()
        self._update_config_status()

    def add_tag_observation(self, row: dict):
        row.setdefault('run_id', self.run_id)
        self._tag_writer.writerow(row)
        self._tag_file.flush()

    def add_fk(self, row: dict):
        row.setdefault('run_id', self.run_id)
        self._fk_writer.writerow(row)
        self._fk_file.flush()

    def finalize(self, status: str):
        self.status = status

    def set_failure_reason(self, reason: str):
        self.failure_reason = reason

    def _git_sha(self) -> Optional[str]:
        try:
            out = subprocess.check_output(
                ['git', 'rev-parse', 'HEAD'],
                cwd=str(Path(__file__).resolve().parent),
                stderr=subprocess.DEVNULL, text=True).strip()
            return out
        except (subprocess.CalledProcessError, FileNotFoundError):
            return None

    def _write_config(self):
        cfg = dict(self.config)
        cfg['run_id'] = self.run_id
        cfg['test_name'] = self.test_name
        cfg['timestamp'] = self.timestamp
        cfg['git_sha'] = self._git_sha()
        cfg['status'] = self.status
        with (self.run_dir / 'config.yaml').open('w') as f:
            yaml.safe_dump(cfg, f, sort_keys=False)

    def _reopen_config(self):
        """Resume mode: keep the original config (run_id, goals, git
        sha, mount snapshot), flip the status back to in_progress and
        record when the resume happened."""
        path = self.run_dir / 'config.yaml'
        with path.open() as f:
            cfg = yaml.safe_load(f) or {}
        self.config = cfg
        cfg['status'] = self.status
        cfg.setdefault('resumes', []).append(self.timestamp)
        with path.open('w') as f:
            yaml.safe_dump(cfg, f, sort_keys=False)

    def _update_config_status(self):
        path = self.run_dir / 'config.yaml'
        if not path.exists():
            return
        with path.open() as f:
            cfg = yaml.safe_load(f) or {}
        cfg['status'] = self.status
        if self.failure_reason is not None:
            cfg['failure_reason'] = self.failure_reason
        else:
            # A stale reason from a failed attempt that was later
            # resumed to completion must not survive in the config.
            cfg.pop('failure_reason', None)
        with path.open('w') as f:
            yaml.safe_dump(cfg, f, sort_keys=False)
