"""Run discovery + loading helpers.

Only the loader lives in the package now: the dashboard uses it for run
counters, and it has no plotting dependencies. All plotting/statistics
moved to the experiments repo (<ws>/experiments/notebooks/) with the
Exp0 pipeline -- analysis on run data happens there, outside the
hardware-facing package.
"""

from .loader import (
    load_run, load_runs, latest_run, list_runs, align_fk_to_tag,
    tag_in_base_frame, fk_apriltag_position,
    goal_key, mount_key, concat_runs, group_runs, select_comparable_runs,
    filter_runs_by_goals, filter_runs_by_cycles,
)

__all__ = [
    'load_run', 'load_runs', 'latest_run', 'list_runs', 'align_fk_to_tag',
    'tag_in_base_frame', 'fk_apriltag_position',
    'goal_key', 'mount_key', 'concat_runs', 'group_runs',
    'select_comparable_runs', 'filter_runs_by_goals',
    'filter_runs_by_cycles',
]
