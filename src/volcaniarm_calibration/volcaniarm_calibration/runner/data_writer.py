"""Persists calibration runs to a thesis-friendly directory layout.

Layout per run:

    data/<YYYYMMDD_HHMMSS>_<test_name>/
      config.yaml         test config + git SHA + status
      samples_raw.csv     one row per AprilTag sample
      fk_poses.csv        analytic FK at each commanded joint state
      residuals.csv       measured tag pose minus FK-predicted tag pose
      summary.csv         per-target aggregates
      run.log             logger output (written by RunWriter.attach_log)
"""

import csv
import math
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import yaml


SAMPLES_FIELDS = [
    'run_id', 'cycle', 'target_idx', 'target_y', 'target_z',
    'sample_idx', 't_ros_ns',
    'theta_right', 'theta_left',
    'measured_x', 'measured_y', 'measured_z',
    'measured_qx', 'measured_qy', 'measured_qz', 'measured_qw',
]

FK_FIELDS = [
    'run_id', 'cycle', 'target_idx',
    'theta_right', 'theta_left',
    'fk_x', 'fk_y', 'fk_z',
]

RESIDUAL_FIELDS = [
    'run_id', 'cycle', 'target_idx',
    'dx', 'dy', 'dz', 'd_pos_norm',
]


class RunWriter:
    """Owns the per-run output directory and CSV writers.

    Use as a context manager:

        with RunWriter(base_dir, test_name, config) as rw:
            rw.add_sample(...)
            rw.add_fk(...)
            rw.add_residual(...)
            rw.finalize(status='completed')
    """

    def __init__(self, base_dir: Path, test_name: str, config: dict):
        self.test_name = test_name
        self.config = dict(config)
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_id = f'{self.timestamp}_{test_name}'
        self.run_dir = Path(base_dir).expanduser() / self.run_id
        self.status: str = 'in_progress'
        self._samples_file = None
        self._fk_file = None
        self._res_file = None
        self._samples_writer = None
        self._fk_writer = None
        self._res_writer = None
        self._all_samples: list = []
        self._all_fk: list = []

    def __enter__(self):
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._samples_file = (self.run_dir / 'samples_raw.csv').open('w', newline='')
        self._samples_writer = csv.DictWriter(self._samples_file, fieldnames=SAMPLES_FIELDS)
        self._samples_writer.writeheader()
        self._fk_file = (self.run_dir / 'fk_poses.csv').open('w', newline='')
        self._fk_writer = csv.DictWriter(self._fk_file, fieldnames=FK_FIELDS)
        self._fk_writer.writeheader()
        self._res_file = (self.run_dir / 'residuals.csv').open('w', newline='')
        self._res_writer = csv.DictWriter(self._res_file, fieldnames=RESIDUAL_FIELDS)
        self._res_writer.writeheader()
        self._write_config()
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc is not None and self.status == 'in_progress':
            self.status = 'failed'
        for f in (self._samples_file, self._fk_file, self._res_file):
            if f is not None:
                f.close()
        self._update_config_status()
        self._write_summary()

    def add_sample(self, row: dict):
        row.setdefault('run_id', self.run_id)
        self._samples_writer.writerow(row)
        self._samples_file.flush()
        self._all_samples.append(row)

    def add_fk(self, row: dict):
        row.setdefault('run_id', self.run_id)
        self._fk_writer.writerow(row)
        self._fk_file.flush()
        self._all_fk.append(row)

    def add_residual(self, row: dict):
        row.setdefault('run_id', self.run_id)
        self._res_writer.writerow(row)
        self._res_file.flush()

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

    def _write_summary(self):
        if not self._all_samples:
            return
        per_target: dict = {}
        for s in self._all_samples:
            key = s['target_idx']
            d = per_target.setdefault(key, {
                'target_y': s['target_y'],
                'target_z': s['target_z'],
                'xs': [], 'ys': [], 'zs': [],
            })
            d['xs'].append(float(s['measured_x']))
            d['ys'].append(float(s['measured_y']))
            d['zs'].append(float(s['measured_z']))
        fields = [
            'target_idx', 'target_y', 'target_z', 'total_samples',
            'mean_x', 'mean_y', 'mean_z',
            'std_x', 'std_y', 'std_z',
            'repeatability_xyz', 'status',
        ]
        with (self.run_dir / 'summary.csv').open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for idx in sorted(per_target.keys()):
                d = per_target[idx]
                xs, ys, zs = np.array(d['xs']), np.array(d['ys']), np.array(d['zs'])
                std_x, std_y, std_z = np.std(xs), np.std(ys), np.std(zs)
                w.writerow({
                    'target_idx': idx,
                    'target_y': d['target_y'],
                    'target_z': d['target_z'],
                    'total_samples': len(xs),
                    'mean_x': f'{np.mean(xs):.6f}',
                    'mean_y': f'{np.mean(ys):.6f}',
                    'mean_z': f'{np.mean(zs):.6f}',
                    'std_x': f'{std_x:.6f}',
                    'std_y': f'{std_y:.6f}',
                    'std_z': f'{std_z:.6f}',
                    'repeatability_xyz': f'{math.sqrt(std_x**2 + std_y**2 + std_z**2):.6f}',
                    'status': self.status,
                })
