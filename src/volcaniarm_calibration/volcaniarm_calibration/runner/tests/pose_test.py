"""Pose test: visit a single target N times, returning home between
iterations. One run yields BOTH ISO 9283 pose statistics from the same
30-visit cluster:

  AP (pose accuracy):      distance from the cluster mean to the
                           commanded point (systematic offset);
  RP (pose repeatability): the cluster's spread around its own mean.

Single-target by design: both statistics are defined over a cluster at
one pose. Multi-target sweeps are workspace-coverage territory.

Optionally each return-to-home is gated on the EE marker being
confirmed near its URDF-predicted home (``verify_home_with_tag``), so
every iteration provably starts from the same physical state.
"""

from .base import BaseTest, Target


class PoseTest(BaseTest):
    name = 'pose_test'

    def __init__(self, targets, verify_home_with_tag=False, **kwargs):
        targets_list = list(targets)
        if len(targets_list) != 1:
            raise ValueError(
                f'pose test takes exactly one target; got '
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
