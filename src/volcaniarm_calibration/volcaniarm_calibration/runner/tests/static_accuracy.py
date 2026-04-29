"""Static accuracy test: visit each target once per cycle without homing between."""

from .base import BaseTest, Target


class StaticAccuracyTest(BaseTest):
    name = 'static_accuracy'

    def iter_visits(self):
        for _ in range(self.num_cycles):
            for t in self.targets:
                yield Target(y=t[0], z=t[1], label=f'({t[0]:.3f},{t[1]:.3f})')
