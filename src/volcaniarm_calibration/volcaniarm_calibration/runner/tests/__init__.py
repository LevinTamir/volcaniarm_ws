from .base import BaseTest, Target
from .static_accuracy import StaticAccuracyTest
from .repeatability import RepeatabilityTest

__all__ = ['BaseTest', 'Target', 'StaticAccuracyTest', 'RepeatabilityTest']

TEST_REGISTRY = {
    StaticAccuracyTest.name: StaticAccuracyTest,
    RepeatabilityTest.name: RepeatabilityTest,
}
