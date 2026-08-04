from .base import BaseTest, Target
from .static_accuracy import StaticAccuracyTest
from .repeatability import RepeatabilityTest
from .workspace_coverage import WorkspaceCoverageTest
from .noise_gate import NoiseGateTest
from .settle_probe import SettleProbeTest
from .backlash import BacklashTest

__all__ = [
    'BaseTest', 'Target',
    'StaticAccuracyTest', 'RepeatabilityTest', 'WorkspaceCoverageTest',
    'NoiseGateTest', 'SettleProbeTest', 'BacklashTest',
]

TEST_REGISTRY = {
    StaticAccuracyTest.name: StaticAccuracyTest,
    RepeatabilityTest.name: RepeatabilityTest,
    WorkspaceCoverageTest.name: WorkspaceCoverageTest,
    NoiseGateTest.name: NoiseGateTest,
    SettleProbeTest.name: SettleProbeTest,
    BacklashTest.name: BacklashTest,
}
