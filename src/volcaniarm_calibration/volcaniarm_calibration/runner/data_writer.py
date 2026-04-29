"""Persists calibration runs to a thesis-friendly directory layout.

Layout per run:

    data/<YYYYMMDD_HHMMSS>_<test_name>/
      config.yaml           test config + git SHA + status
      tag_observations.csv  base->ee apriltag transform per sample,
                            tagged with phase (home|target). Ground truth.
      fk_poses.csv          analytic FK at each commanded joint state.
      run.log               logger output (written by RunWriter.attach_log)

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
    'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
]

FK_FIELDS = [
    'run_id', 'cycle', 'target_idx',
    'theta_right', 'theta_left',
    'fk_x', 'fk_y', 'fk_z',
]


class RunWriter:
    """Owns the per-run output directory and CSV writers.

    Use as a context manager:

        with RunWriter(base_dir, test_name, config) as rw:
            rw.add_tag_observation(...)
            rw.add_fk(...)
            rw.finalize(status='completed')
    """

    def __init__(self, base_dir: Path, test_name: str, config: dict):
        self.test_name = test_name
        self.config = dict(config)
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_id = f'{self.timestamp}_{test_name}'
        self.run_dir = Path(base_dir).expanduser() / self.run_id
        self.status: str = 'in_progress'
        self._tag_file = None
        self._fk_file = None
        self._tag_writer = None
        self._fk_writer = None

    def __enter__(self):
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._tag_file = (self.run_dir / 'tag_observations.csv').open('w', newline='')
        self._tag_writer = csv.DictWriter(self._tag_file, fieldnames=TAG_OBS_FIELDS)
        self._tag_writer.writeheader()
        self._fk_file = (self.run_dir / 'fk_poses.csv').open('w', newline='')
        self._fk_writer = csv.DictWriter(self._fk_file, fieldnames=FK_FIELDS)
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

    def _update_config_status(self):
        path = self.run_dir / 'config.yaml'
        if not path.exists():
            return
        with path.open() as f:
            cfg = yaml.safe_load(f) or {}
        cfg['status'] = self.status
        with path.open('w') as f:
            yaml.safe_dump(cfg, f, sort_keys=False)
