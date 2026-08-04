"""Noise-gate test: measurement-noise floor at static poses.

Visits a handful of poses spanning the workspace once each and, at
every pose, captures a long burst of detections while the arm is
stationary (set ``samples_per_capture`` in the RunRequest / node
params to ~500). The analysis computes per-axis sigma and bias of the
world-frame Y-Z origins, plus the effective noise after an M-frame
rolling median, and compares against the protocol gate (<= 1-2 mm per
axis).

This is Exp0's blocking validation step: no sweep runs until the gate
passes. It also settles the single-sample-vs-median question
empirically -- if the rolling median does not measurably beat the raw
sigma, per-visit multi-sampling stays off for the sweeps.
"""

from .base import BaseTest, Target


class NoiseGateTest(BaseTest):
    name = 'noise_gate'

    def __init__(self, *args, **kwargs):
        # One visit per pose, no doubling back between poses: the point
        # is the burst at each station, not statistical repetition of
        # the moves.
        kwargs['num_cycles'] = 1
        kwargs['return_to_initial_between_visits'] = False
        super().__init__(*args, **kwargs)

    def iter_visits(self):
        for t in self.targets:
            yield Target(y=t[0], z=t[1], label=f'({t[0]:.3f},{t[1]:.3f})')
