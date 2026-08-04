"""Settle-probe test: measure how long the arm takes to settle.

Moves to each pose from the initial pose and starts capturing
immediately (settle_time forced to 0), recording a timestamped burst
(set ``samples_per_capture`` to cover ~4 s at the detection rate,
e.g. 120 samples at 30 Hz). Each row's ``t_ros_ns`` gives pose vs
time since arrival; the analysis finds the first time where
inter-sample drift falls below the noise-gate floor. That measured
value replaces the guessed settle_time in every subsequent run and is
reported in the thesis protocol.

Returns to the initial pose between visits so every probe starts with
a comparable move.
"""

from .base import BaseTest, Target


class SettleProbeTest(BaseTest):
    name = 'settle_probe'

    def __init__(self, *args, **kwargs):
        kwargs['settle_time'] = 0.0
        kwargs['return_to_initial_between_visits'] = True
        super().__init__(*args, **kwargs)

    def iter_visits(self):
        for _ in range(self.num_cycles):
            for t in self.targets:
                yield Target(y=t[0], z=t[1], label=f'({t[0]:.3f},{t[1]:.3f})')
