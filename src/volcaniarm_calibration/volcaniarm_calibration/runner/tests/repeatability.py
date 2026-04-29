"""Repeatability test: visit each target once per cycle, returning home between visits."""

from .base import BaseTest, Target


class RepeatabilityTest(BaseTest):
    name = 'repeatability'

    def __init__(self, *args, **kwargs):
        kwargs['return_home_between_targets'] = True
        super().__init__(*args, **kwargs)

    def iter_visits(self):
        for _ in range(self.num_cycles):
            for t in self.targets:
                yield Target(y=t[0], z=t[1], label=f'({t[0]:.3f},{t[1]:.3f})')
