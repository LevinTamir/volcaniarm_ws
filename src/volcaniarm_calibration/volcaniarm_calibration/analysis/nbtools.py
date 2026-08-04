"""Shared plotting helpers for the calibration analysis notebooks.

Keeps the six notebooks free of duplicated boilerplate and makes every
figure thesis-ready by construction: one print-tuned style, a fixed
categorical colour order for runs, and ``save_fig`` which exports each
figure as a 300 dpi PNG (for pasting into the document) plus a vector
PDF (for LaTeX \\includegraphics) under ``notebooks/figures/``.

Pure matplotlib / pandas; no ROS imports, so notebooks run without a
sourced environment.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

PRIMARY = '#2b6cb0'
ACCENT = '#2e9c4a'
# Fixed categorical order for runs, assigned by run order and kept
# stable across every figure so a run keeps its colour everywhere.
RUN_COLORS = ['#2b6cb0', '#2e9c4a', '#c79a3a', '#8b5cb0',
              '#d04b4b', '#3a9ea6', '#b0642b', '#666666']

# Standard figure widths (inches). FULL matches a 12 pt thesis page
# \textwidth; SQUARE is for equal-aspect Y-Z plots.
FIG_FULL = (6.3, 3.6)
FIG_TALL = (6.3, 4.4)
FIG_SQUARE = (5.2, 5.0)
FIG_WIDE = (6.3, 3.2)

# Figures land next to the (legacy per-test) notebooks regardless of
# the kernel cwd. Both moved to the workspace-level experiments/ tree
# with the Exp0 integration; parents[4] is the workspace root
# (…/src/volcaniarm_calibration/volcaniarm_calibration/analysis/nbtools.py).
FIGURES_DIR = (Path(__file__).resolve().parents[4]
               / 'experiments' / 'notebooks' / 'legacy' / 'figures')


def apply_style():
    """Print-tuned matplotlib defaults shared by all notebooks."""
    plt.rcParams.update({
        'figure.facecolor': 'white', 'axes.facecolor': 'white',
        'figure.dpi': 110, 'savefig.dpi': 300,
        'font.size': 11, 'axes.titlesize': 12, 'axes.titleweight': 'bold',
        'axes.labelsize': 11, 'axes.edgecolor': '#999999', 'axes.grid': True,
        'grid.color': '#e8e8e8', 'grid.linewidth': 0.8,
        'xtick.labelsize': 10, 'ytick.labelsize': 10, 'legend.fontsize': 9,
        'legend.framealpha': 0.9,
    })


def run_short(run_id) -> str:
    """'static_accuracy/2026-07-12/13-28-53' -> '07-12 13:28'."""
    parts = str(run_id).split('/')
    if len(parts) == 3:
        return parts[1][5:] + ' ' + parts[2][:5].replace('-', ':')
    return str(run_id)


def per_axis_residuals_mm(frame):
    """Camera-independent per-axis residuals in world Y-Z (mm).

    Uses the base-relative vector so a camera translation error
    cancels: r = (det_ee - det_base) - (urdf_ee - urdf_base).
    """
    ry = ((frame['det_ee_y'] - frame['det_base_y'])
          - (frame['urdf_ee_y'] - frame['urdf_base_y'])) * 1000.0
    rz = ((frame['det_ee_z'] - frame['det_base_z'])
          - (frame['urdf_ee_z'] - frame['urdf_base_z'])) * 1000.0
    return ry.to_numpy(), rz.to_numpy()


def save_fig(fig, name: str):
    """Export a figure for the thesis and return it for inline display.

    Writes ``notebooks/figures/<name>.png`` (300 dpi) and
    ``<name>.pdf`` (vector). ``name`` should be stable across re-runs
    (e.g. 'static_accuracy/residual_vs_cycle') so document links never
    break; re-running a notebook simply refreshes the files.
    """
    fig.tight_layout()
    out = FIGURES_DIR / name
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out.with_suffix('.png'), dpi=300, bbox_inches='tight')
    fig.savefig(out.with_suffix('.pdf'), bbox_inches='tight')
    return fig
