"""Base class for calibration tests."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Target:
    """A single EE position the runner should visit."""
    y: float
    z: float
    label: str = ''


class BaseTest(ABC):
    """Abstract test definition consumed by CalibrationRunner.

    A test owns the visit pattern (which targets, in what order) and any
    per-test summary logic. The runner handles ROS plumbing.

    Sampling is one detection per visit, gated on a freshly progressed
    TF stamp post-settle. Multi-sample averaging was dropped because the
    arm is stationary by the time we read; repeated lookups added latency
    confounds without reducing the fundamental detector pixel noise floor.
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
        return len(self.targets) * self.num_cycles
