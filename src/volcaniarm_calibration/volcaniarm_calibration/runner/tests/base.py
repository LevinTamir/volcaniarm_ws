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

    A test owns the visit pattern (which targets, in what order, how
    many samples per visit) and any per-test summary logic. The runner
    handles ROS plumbing.
    """

    name: str = 'base'

    def __init__(self, targets, num_cycles: int, samples_per_visit: int,
                 settle_time: float, sample_interval: float,
                 return_home_between_targets: bool,
                 return_to_initial_between_visits: bool = True):
        self.targets = list(targets)
        self.num_cycles = num_cycles
        self.samples_per_visit = samples_per_visit
        self.settle_time = settle_time
        self.sample_interval = sample_interval
        self.return_home_between_targets = return_home_between_targets
        # When True (default), the runner moves the arm back to the
        # initial pose after every captured visit. Single-pose tests
        # (accuracy / repeatability) want this so each iteration
        # starts from a known state. The workspace_coverage sweep
        # sets this False to walk the envelope without doubling back.
        self.return_to_initial_between_visits = return_to_initial_between_visits

    @abstractmethod
    def iter_visits(self) -> Iterable[Target]:
        """Yield each target visit in execution order across all cycles."""
        ...

    def total_visits(self) -> int:
        return len(self.targets) * self.num_cycles
