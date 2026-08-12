from .base import BaseTest, Target
from .pose_test import PoseTest
from .workspace_coverage import WorkspaceCoverageTest
from .noise_gate import NoiseGateTest
from .settle_probe import SettleProbeTest
from .backlash import BacklashTest

__all__ = [
    'BaseTest', 'Target',
    'PoseTest', 'WorkspaceCoverageTest',
    'NoiseGateTest', 'SettleProbeTest', 'BacklashTest',
]

TEST_REGISTRY = {
    PoseTest.name: PoseTest,
    WorkspaceCoverageTest.name: WorkspaceCoverageTest,
    NoiseGateTest.name: NoiseGateTest,
    SettleProbeTest.name: SettleProbeTest,
    BacklashTest.name: BacklashTest,
    # Legacy alias: pose_test was called 'repeatability' before it and
    # static_accuracy merged (one run yields both AP and RP, so the two
    # pages were one test in disguise). Keeps config.yaml files from
    # old runs resumable.
    'repeatability': PoseTest,
}
