"""Workspace coverage test: walk a list of poses (typically the
operating envelope) once each, without returning to the initial pose
between visits.

The arm parks at the user-chosen initial pose, captures a baseline,
then sweeps each goal in order: move -> settle -> operator-gated
Continue -> capture -> next goal. The run ends at the last goal.

Use ``static_accuracy`` or ``repeatability`` when you want multiple
samples at a single pose with returns to a known starting state. This
test is for spatial coverage, not statistical repetition.
"""

from .base import BaseTest, Target


class WorkspaceCoverageTest(BaseTest):
    name = 'workspace_coverage'

    def __init__(self, *args, **kwargs):
        # Force the two semantics that distinguish this test from
        # accuracy / repeatability, regardless of what the dashboard
        # or headless node passes. Putting them here keeps the runner
        # generic and avoids per-test branching in _execute.
        kwargs['num_cycles'] = 1
        kwargs['return_to_initial_between_visits'] = False
        super().__init__(*args, **kwargs)

    def iter_visits(self):
        for _ in range(self.num_cycles):
            for t in self.targets:
                yield Target(y=t[0], z=t[1], label=f'({t[0]:.3f},{t[1]:.3f})')
