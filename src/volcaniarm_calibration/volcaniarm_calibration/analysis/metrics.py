"""ISO 9283-flavoured metrics for the calibration test notebooks.

Volcaniarm is a 2-DOF planar arm; metrics are computed in the Y-Z
plane unless explicitly stated. Matches conventions from ISO 9283
(industrial robot performance) so thesis numbers are comparable to
the literature, with caveats noted where our setup deviates (URDF
apriltag mount placeholders bias absolute accuracy; repeatability
is bias-free).

Application-relevant thresholds for weeding (1-5 cm weed size,
~1 cm targeting tolerance desired):
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence

import numpy as np
import pandas as pd
from scipy import stats as _scipy_stats


# Weeding-application thresholds (in mm). Tune to match the weed-detector
# precision and the harvester / sprayer head tolerance the arm drives.
WEEDING_ACCEPTABLE_MM = 10.0   # arm hits within 1 cm of target -> usable
WEEDING_MARGINAL_MM = 30.0     # 1-3 cm -> marginal, depends on weed size
# Above marginal -> failing for weeding.


@dataclass
class Stats:
    """Summary stats reportable in a thesis. All values in the input
    units (use ``in_mm()`` for a mm-formatted view)."""
    n: int
    mean: float
    std: float
    worst: float          # max absolute value (signed magnitude)
    ci95: float           # half-width of the t-based 95% CI on the mean
    median: float

    def in_mm(self) -> 'Stats':
        return Stats(
            n=self.n,
            mean=self.mean * 1000.0,
            std=self.std * 1000.0,
            worst=self.worst * 1000.0,
            ci95=self.ci95 * 1000.0,
            median=self.median * 1000.0,
        )

    def __str__(self) -> str:
        # Renders the stored values as-is; call ``in_mm()`` first for a
        # mm view (the notebooks do). Scaling here too would double-convert.
        return (
            f'n={self.n}  mean={self.mean:+.3f} mm  '
            f'std={self.std:.3f} mm  '
            f'worst={self.worst:.3f} mm  '
            f'95% CI ±{self.ci95:.3f} mm  '
            f'median={self.median:+.3f} mm')


def summary(values: Sequence[float]) -> Stats:
    """Mean / std / worst / 95% CI from a 1-D sequence."""
    arr = np.asarray(list(values), dtype=float)
    arr = arr[~np.isnan(arr)]
    n = arr.size
    if n == 0:
        return Stats(0, np.nan, np.nan, np.nan, np.nan, np.nan)
    mean = float(np.mean(arr))
    # ddof=1 = sample std, the convention for repeatability metrics.
    std = float(np.std(arr, ddof=1)) if n > 1 else 0.0
    # Worst = maximum absolute deviation (the worst-case the arm hit).
    # Useful for agricultural reporting -- "worst miss" matters more
    # than mean when a single bad pose can damage a plant.
    worst = float(np.max(np.abs(arr)))
    # t-based 95% CI half-width on the mean. Matters for small n:
    # t(df=2) = 4.30 vs the normal 1.96; at n=30 the difference is
    # negligible (t(29) = 2.045).
    if n > 1:
        t_crit = float(_scipy_stats.t.ppf(0.975, n - 1))
        ci95 = t_crit * std / np.sqrt(n)
    else:
        ci95 = 0.0
    median = float(np.median(arr))
    return Stats(n=n, mean=mean, std=std, worst=worst, ci95=ci95, median=median)


@dataclass
class CrossRunStats:
    """Two-level summary across repeated runs of the same test.

    ``mean`` and ``ci95`` describe the across-run average (mean of
    per-run means, t-based CI with df = n_runs - 1). This is the
    headline: it captures session-to-session variation (re-homing,
    camera relock) that within-run stats cannot see. ``pooled`` stacks
    every sample from every run for histograms and worst-case figures.
    All values in the input units (metres for d_error columns).
    """
    n_runs: int
    per_run: list = field(default_factory=list)   # list[Stats], run order
    mean: float = np.nan          # mean of per-run means
    std_between: float = np.nan   # ddof=1 std across per-run means
    ci95: float = np.nan          # t-based half-width, df = n_runs - 1
    pooled: Optional[Stats] = None

    def __str__(self) -> str:
        return (
            f'runs={self.n_runs}  mean-of-means={self.mean:+.3f}  '
            f'between-run std={self.std_between:.3f}  '
            f'95% CI ±{self.ci95:.3f}  '
            f'pooled n={self.pooled.n if self.pooled else 0}')


def cross_run_summary(per_run_values: Sequence[Sequence[float]]) -> CrossRunStats:
    """Aggregate the same metric measured over several independent runs.

    ``per_run_values`` is one sequence of samples per run (e.g. each
    run's target-phase ``d_error`` values). Runs whose samples are all
    NaN are dropped. With a single run the across-run CI is undefined
    (NaN); quote the pooled stats instead and say so in the text.
    """
    per_run = [summary(v) for v in per_run_values]
    per_run = [s for s in per_run if s.n > 0]
    n_runs = len(per_run)
    all_samples = [x for v in per_run_values
                   for x in np.asarray(list(v), dtype=float)
                   if not np.isnan(x)]
    pooled = summary(all_samples)
    if n_runs == 0:
        return CrossRunStats(n_runs=0, per_run=[], pooled=pooled)
    means = np.array([s.mean for s in per_run])
    mean = float(means.mean())
    if n_runs > 1:
        std_between = float(means.std(ddof=1))
        t_crit = float(_scipy_stats.t.ppf(0.975, n_runs - 1))
        ci95 = t_crit * std_between / np.sqrt(n_runs)
    else:
        std_between = np.nan
        ci95 = np.nan
    return CrossRunStats(n_runs=n_runs, per_run=per_run, mean=mean,
                         std_between=std_between, ci95=ci95, pooled=pooled)


def repeatability_iso9283(positions: pd.DataFrame,
                          dims: Sequence[str] = ('y', 'z')) -> dict:
    """ISO 9283 pose repeatability `RP` for a cluster of measurements.

    `RP = mean_distance_to_centroid + 3 * stddev_distance_to_centroid`

    Sub-stats reported:
        n            : number of points in the cluster
        centroid     : mean of each dim
        per_dim_std  : per-axis stddev (Y, Z separately) -- diagnostic
        d_to_centroid: distances from each point to centroid (Series)
        RP           : the headline ISO 9283 number
        worst        : max distance to centroid (worst-case scatter)

    Frame-independent: rotates with the data, so the URDF mount
    placeholder bias does NOT contaminate this number. This is the
    cleanest metric you can report.
    """
    pos = positions[list(dims)].to_numpy(dtype=float)
    pos = pos[~np.any(np.isnan(pos), axis=1)]
    n = pos.shape[0]
    if n < 2:
        return {
            'n': n,
            'centroid': pos[0].tolist() if n == 1 else [np.nan] * len(dims),
            'per_dim_std': [np.nan] * len(dims),
            'd_to_centroid': pd.Series([], dtype=float),
            'RP_m': np.nan,
            'worst_m': np.nan,
        }
    centroid = pos.mean(axis=0)
    d = np.linalg.norm(pos - centroid, axis=1)
    per_dim_std = pos.std(axis=0, ddof=1).tolist()
    return {
        'n': n,
        'centroid': centroid.tolist(),
        'per_dim_std': per_dim_std,
        'd_to_centroid': pd.Series(d),
        'RP_m': float(d.mean() + 3.0 * d.std(ddof=1)),
        'worst_m': float(d.max()),
    }


def accuracy_segment_length(observations: pd.DataFrame) -> Stats:
    """Y-Z segment-length accuracy metric for static-accuracy runs.

    Reads the runner-logged ``d_error`` column (signed, metres) where
    ``d_error = ||detected_base->ee||_yz - ||urdf_base->ee||_yz``,
    filtered to ``phase == 'target'`` rows, and returns mean / std /
    worst / 95% CI across them.

    Frame-independent: the comparison is between two scalars, so any
    static rotation between apriltag_marker_base and apriltag_base_link
    cancels. The number is what the thesis quotes for static accuracy.
    """
    target = observations[observations['phase'] == 'target']
    return summary(target['d_error'].dropna().tolist())


def accuracy_iso9283(positions: pd.DataFrame,
                     commanded: Sequence[float],
                     dims: Sequence[str] = ('y', 'z')) -> dict:
    """ISO 9283 pose accuracy `AP` (with caveat).

    `AP = |centroid - commanded|`

    The commanded value should be in the same frame as `positions[dims]`.
    For Volcaniarm, that means EITHER:
      - both in volcaniarm_base_link frame (FK side), OR
      - both in apriltag_marker_base frame (tag side).

    Mixing frames produces a biased number; we tolerate the bias when
    the URDF apriltag mount values are accurate to mm-scale (your
    measured values from commits 33e5fc1 / ec91605).

    Sub-stats:
        n         : sample count
        centroid  : mean position
        offset    : centroid - commanded (signed per-dim)
        AP_m      : |offset|
    """
    pos = positions[list(dims)].to_numpy(dtype=float)
    pos = pos[~np.any(np.isnan(pos), axis=1)]
    n = pos.shape[0]
    if n == 0:
        return {
            'n': 0,
            'centroid': [np.nan] * len(dims),
            'offset': [np.nan] * len(dims),
            'AP_m': np.nan,
        }
    centroid = pos.mean(axis=0)
    offset = centroid - np.asarray(commanded, dtype=float)
    return {
        'n': n,
        'centroid': centroid.tolist(),
        'offset': offset.tolist(),
        'AP_m': float(np.linalg.norm(offset)),
    }


def per_point_accuracy(df: pd.DataFrame) -> pd.DataFrame:
    """Per-grid-point accuracy over a stacked multi-run DataFrame.

    ``df`` comes from ``loader.concat_runs`` and must carry ``goal_y``,
    ``goal_z``, ``run_id`` and ``d_error`` columns. Groups by commanded
    goal (mm-rounded), returns one row per point with n_runs, n_samples,
    mean_mm, std_mm, ci95_mm, worst_mm and the weeding ``zone`` of the
    mean. The CI is t-based on per-run means when the point was seen in
    two or more runs, else on the pooled samples.
    """
    rows = []
    for (gy, gz), grp in df.groupby(['goal_y', 'goal_z'], sort=True):
        per_run_vals = [g['d_error'].dropna().tolist()
                        for _, g in grp.groupby('run_id')]
        cr = cross_run_summary(per_run_vals)
        if cr.n_runs >= 2:
            mean_m, ci_m = cr.mean, cr.ci95
        else:
            mean_m, ci_m = cr.pooled.mean, cr.pooled.ci95
        rows.append({
            'goal_y': gy,
            'goal_z': gz,
            'n_runs': cr.n_runs,
            'n_samples': cr.pooled.n,
            'mean_mm': mean_m * 1000.0,
            'std_mm': cr.pooled.std * 1000.0,
            'ci95_mm': ci_m * 1000.0,
            'worst_mm': cr.pooled.worst * 1000.0,
            'zone': threshold_zone(abs(mean_m) * 1000.0)
            if not np.isnan(mean_m) else 'n/a',
        })
    return pd.DataFrame(rows)


def per_point_repeatability(df: pd.DataFrame,
                            min_samples: int = 2) -> pd.DataFrame:
    """Per-grid-point repeatability over a stacked multi-run DataFrame.

    For each commanded goal, two numbers:
      - ``rp_pooled_mm``: ISO 9283 RP over ALL detected EE positions at
        that point stacked across runs. Includes cross-session scatter
        (camera relock between sessions), so it is pessimistic.
      - ``rp_within_run_mean_mm``: mean of per-run RPs over runs that
        contributed at least ``min_samples`` samples at that point.
        This is the ISO-comparable figure.

    Positions are the world-frame ``det_ee_y`` / ``det_ee_z`` columns.
    Points with fewer than ``min_samples`` total samples get NaN RPs.
    """
    rows = []
    for (gy, gz), grp in df.groupby(['goal_y', 'goal_z'], sort=True):
        cluster = grp.dropna(subset=['det_ee_y', 'det_ee_z'])
        n_samples = len(cluster)
        if n_samples >= min_samples:
            pooled = repeatability_iso9283(
                cluster, dims=('det_ee_y', 'det_ee_z'))
            rp_pooled_mm = pooled['RP_m'] * 1000.0
            worst_mm = pooled['worst_m'] * 1000.0
        else:
            rp_pooled_mm = np.nan
            worst_mm = np.nan
        within = []
        for _, g in cluster.groupby('run_id'):
            if len(g) >= min_samples:
                rp = repeatability_iso9283(
                    g, dims=('det_ee_y', 'det_ee_z'))['RP_m']
                if not np.isnan(rp):
                    within.append(rp * 1000.0)
        rows.append({
            'goal_y': gy,
            'goal_z': gz,
            'n_runs': cluster['run_id'].nunique(),
            'n_samples': n_samples,
            'rp_pooled_mm': rp_pooled_mm,
            'rp_within_run_mean_mm':
                float(np.mean(within)) if within else np.nan,
            'worst_mm': worst_mm,
        })
    return pd.DataFrame(rows)


def threshold_zone(value_mm: float) -> str:
    """Map a residual magnitude to a weeding-application zone label."""
    if value_mm <= WEEDING_ACCEPTABLE_MM:
        return 'acceptable'
    if value_mm <= WEEDING_MARGINAL_MM:
        return 'marginal'
    return 'failing'


def threshold_color(value_mm: float) -> str:
    """matplotlib-friendly colour for a residual magnitude."""
    return {
        'acceptable': '#2e9c4a',
        'marginal':   '#c79a3a',
        'failing':    '#d04b4b',
    }[threshold_zone(value_mm)]
