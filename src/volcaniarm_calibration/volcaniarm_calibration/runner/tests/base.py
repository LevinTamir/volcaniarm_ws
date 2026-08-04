"""Base class for calibration tests."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Target:
    """A single EE position the runner should visit.

    ``capture`` False marks a motion-only visit (e.g. a backlash
    approach pre-point): the runner moves and settles there but takes
    no measurement. ``approach`` is a free-form tag ('+y' / '-y' for
    backlash) copied into the CSV rows so the analysis can group
    samples by approach direction.
    """
    y: float
    z: float
    label: str = ''
    capture: bool = True
    approach: str = ''


class BaseTest(ABC):
    """Abstract test definition consumed by CalibrationRunner.

    A test owns the visit pattern (which targets, in what order) and any
    per-test summary logic. The runner handles ROS plumbing.

    Sampling defaults to one detection per visit, gated on a freshly
    progressed TF stamp post-settle. Multi-sample averaging was dropped
    for the accuracy tests because the arm is stationary by the time we
    read; repeated lookups added latency confounds without reducing the
    fundamental detector pixel noise floor. The Exp0 characterization
    tests (noise_gate, settle_probe) re-enable per-visit bursts
    deliberately via ``RunRequest.samples_per_capture`` -- the former to
    measure that noise floor, the latter to trace pose vs time.
    """

    name: str = 'base'

    def __init__(self, targets, num_cycles: int,
                 settle_time: float,
                 return_home_between_targets: bool,
                 return_to_initial_between_visits: bool = True,
                 verify_home_with_tag: bool = False):
        self.targets = list(targets)
        self.num_cycles = num_cycles
        self.settle_time = settle_time
        self.return_home_between_targets = return_home_between_targets
        # When True (default), the runner moves the arm back to the
        # initial pose after every captured visit. Single-pose tests
        # (accuracy / repeatability) want this so each iteration
        # starts from a known state. The workspace_coverage sweep
        # sets this False to walk the envelope without doubling back.
        self.return_to_initial_between_visits = return_to_initial_between_visits
        # When True, the runner also waits after every return-to-home
        # for the apriltag detector to confirm the EE marker has reached
        # the URDF-predicted home pose within tolerance. Repeatability
        # uses this to guarantee each iteration starts from a verified
        # physical state, not just a commanded one.
        self.verify_home_with_tag = verify_home_with_tag

    @abstractmethod
    def iter_visits(self) -> Iterable[Target]:
        """Yield each target visit in execution order across all cycles."""
        ...

    def total_visits(self) -> int:
        """Number of *captured* visits (progress reflects measurements,
        not moves; motion-only pre-points are excluded)."""
        return sum(1 for v in self.iter_visits() if v.capture)
