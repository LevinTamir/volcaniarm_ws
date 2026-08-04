"""Backlash test: approach-direction hysteresis at frozen points.

Each cycle visits every target twice: once arriving from -Y (via a
capture-free pre-point at y - approach_offset) and once from +Y (via
y + approach_offset). The difference between the per-direction mean
measured poses at a target estimates the mechanical backlash there.

Rows are tagged with ``approach`` ('+y' when arriving from -Y toward
+Y, '-y' for the opposite) so the analysis can group by direction
without reconstructing the visit order. Pre-points are motion-only
(Target.capture=False): the runner moves and settles there but writes
no observation.

Protocol: the 9 Exp0 repeatability points x num_cycles=5 gives 5 reps
per direction per point. Pre-points must themselves be reachable; the
runner's IK-before-motion precheck aborts with a clear message if the
offset pushes one outside the workspace, in which case shrink
approach_offset or move the target inward.
"""

from .base import BaseTest, Target


class BacklashTest(BaseTest):
    name = 'backlash'

    def __init__(self, targets, approach_offset_m: float = 0.05, **kwargs):
        if approach_offset_m <= 0.0:
            raise ValueError(
                f'approach_offset_m must be positive; got {approach_offset_m}')
        self.approach_offset_m = float(approach_offset_m)
        # Direction pairing replaces return-to-initial as the state
        # reset: what matters is the last move's direction, which the
        # pre-point pins down regardless of where the arm came from.
        kwargs['return_to_initial_between_visits'] = False
        super().__init__(targets, **kwargs)

    def iter_visits(self):
        off = self.approach_offset_m
        for _ in range(self.num_cycles):
            for t in self.targets:
                y, z = t[0], t[1]
                label = f'({y:.3f},{z:.3f})'
                yield Target(y=y - off, z=z, label=label + ' pre',
                             capture=False, approach='+y')
                yield Target(y=y, z=z, label=label, approach='+y')
                yield Target(y=y + off, z=z, label=label + ' pre',
                             capture=False, approach='-y')
                yield Target(y=y, z=z, label=label, approach='-y')
