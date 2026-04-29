"""Workspace coverage test: visit a list of poses (typically the
operating envelope) N times each.

Used for ISO 9283-style multi-pose characterisation: each goal in the
list is visited ``num_cycles`` times in a round-robin pattern (full
sweep across goals counts as one cycle), with the runner gating at
every visit so the operator can adjust the camera if needed. The
data writer tags each row with the cycle index and the goal index so
the analysis notebook can group per-goal accuracy and repeatability
metrics from a single run.
"""

from .base import BaseTest, Target


class WorkspaceCoverageTest(BaseTest):
    name = 'workspace_coverage'

    def iter_visits(self):
        for _ in range(self.num_cycles):
            for t in self.targets:
                yield Target(y=t[0], z=t[1], label=f'({t[0]:.3f},{t[1]:.3f})')
