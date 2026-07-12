"""Workspace coverage test: sweep a list of poses (typically the
operating envelope) without returning to the initial pose between
visits.

The arm parks at the user-chosen initial pose, captures a baseline,
then sweeps each goal in order: move -> settle -> operator-gated
Continue -> capture -> next goal. Each cycle is one full sweep of the
goal list; running several cycles gives a small cluster per grid point
so the notebooks can map both accuracy and repeatability over the
Y-Z plane.

Use ``static_accuracy`` or ``repeatability`` when you want a deep
sample at a single pose with returns to a known starting state.
"""

from .base import BaseTest, Target


class WorkspaceCoverageTest(BaseTest):
    name = 'workspace_coverage'

    def __init__(self, *args, **kwargs):
        # No doubling back within a sweep; this is what distinguishes
        # the test from accuracy / repeatability regardless of what the
        # dashboard passes. Keeping it here keeps the runner generic.
        kwargs['return_to_initial_between_visits'] = False
        super().__init__(*args, **kwargs)

    def iter_visits(self):
        for _ in range(self.num_cycles):
            for t in self.targets:
                yield Target(y=t[0], z=t[1], label=f'({t[0]:.3f},{t[1]:.3f})')
