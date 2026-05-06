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

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import numpy as np
import pandas as pd


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
    ci95: float           # half-width of the normal-approx 95% CI
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
        return (
            f'n={self.n}  mean={self.mean*1000:+.3f} mm  '
            f'std={self.std*1000:.3f} mm  '
            f'worst={self.worst*1000:.3f} mm  '
            f'95% CI ±{self.ci95*1000:.3f} mm  '
            f'median={self.median*1000:+.3f} mm')


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
    # Normal-approx 95% CI half-width on the mean. Use t-distribution
    # for small n in a thesis-grade report; this is a quick estimate.
    ci95 = 1.96 * std / np.sqrt(n) if n > 1 else 0.0
    median = float(np.median(arr))
    return Stats(n=n, mean=mean, std=std, worst=worst, ci95=ci95, median=median)


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
