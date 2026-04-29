from .base import BaseTest, Target
from .static_accuracy import StaticAccuracyTest
from .repeatability import RepeatabilityTest
from .workspace_coverage import WorkspaceCoverageTest

__all__ = [
    'BaseTest', 'Target',
    'StaticAccuracyTest', 'RepeatabilityTest', 'WorkspaceCoverageTest',
]

TEST_REGISTRY = {
    StaticAccuracyTest.name: StaticAccuracyTest,
    RepeatabilityTest.name: RepeatabilityTest,
    WorkspaceCoverageTest.name: WorkspaceCoverageTest,
}
