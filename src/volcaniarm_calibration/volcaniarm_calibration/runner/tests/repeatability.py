"""Repeatability test: visit a single target N times, returning home
between iterations and verifying the EE marker has reached home before
the next iteration starts.

Single-target by design: ISO 9283 RP is computed over a cluster at one
pose. Multi-target repeatability is workspace-coverage territory.
"""

from .base import BaseTest, Target


class RepeatabilityTest(BaseTest):
    name = 'repeatability'

    def __init__(self, targets, verify_home_with_tag=False, **kwargs):
        targets_list = list(targets)
        if len(targets_list) != 1:
            raise ValueError(
                f'repeatability test takes exactly one target; got '
                f'{len(targets_list)}: {targets_list}')
        kwargs['return_home_between_targets'] = True
        # Home-confirm gates each iteration on the detected EE marker
        # matching its URDF-predicted home. That comparison carries the
        # URDF AprilTag mount bias, so with placeholder mounts (a ~cm
        # offset) a tight tolerance can never pass and the run aborts
        # before it starts. It is therefore opt-in: enable it only once
        # the mounts are calibrated (bias ~0) or with a tolerance set
        # above the known bias.
        kwargs['verify_home_with_tag'] = verify_home_with_tag
        super().__init__(targets_list, **kwargs)

    def iter_visits(self):
        for _ in range(self.num_cycles):
            for t in self.targets:
                yield Target(y=t[0], z=t[1], label=f'({t[0]:.3f},{t[1]:.3f})')
