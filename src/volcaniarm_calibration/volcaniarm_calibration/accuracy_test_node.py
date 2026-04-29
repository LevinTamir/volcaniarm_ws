"""Headless CLI entrypoint for an accuracy test.

Thin shim around ``CalibrationRunner`` with ``auto_approve=True`` that
preserves the legacy `ros2 run volcaniarm_calibration accuracy_test`
workflow. The dashboard uses the same runner via the rqt plugin.
"""

from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .runner import (
    CalibrationRunner,
    RunRequest,
    StaticAccuracyTest,
    RepeatabilityTest,
    TEST_REGISTRY,
)


def _parse_targets(flat: list) -> list:
    return [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]


class AccuracyTestNode(Node):

    def __init__(self):
        super().__init__('accuracy_test_node')
        self.declare_parameter('test_type', 'static_accuracy')
        self.declare_parameter('targets', [0.0, 0.6, 0.15, 0.5, -0.15, 0.5])
        self.declare_parameter('num_cycles', 5)
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('samples_per_visit', 20)
        self.declare_parameter('sample_interval', 0.1)
        self.declare_parameter('joint_names',
            ['volcaniarm_right_elbow_joint', 'volcaniarm_left_elbow_joint'])
        self.declare_parameter('trajectory_duration', 2.0)
        self.declare_parameter('home_position', [0.0, 0.0])
        self.declare_parameter('return_home_between_targets', True)
        self.declare_parameter('base_tag_frame', 'apriltag_marker_base')
        self.declare_parameter('ee_tag_frame', 'apriltag_marker_ee')
        self.declare_parameter('output_dir', '~/workspaces/volcaniarm_ws/data')

        test_type = self.get_parameter('test_type').value
        cls = TEST_REGISTRY.get(test_type, StaticAccuracyTest)

        targets = _parse_targets(list(self.get_parameter('targets').value))
        test = cls(
            targets=targets,
            num_cycles=int(self.get_parameter('num_cycles').value),
            samples_per_visit=int(self.get_parameter('samples_per_visit').value),
            settle_time=float(self.get_parameter('settle_time').value),
            sample_interval=float(self.get_parameter('sample_interval').value),
            return_home_between_targets=bool(
                self.get_parameter('return_home_between_targets').value),
        )

        self.runner = CalibrationRunner(self)
        self.runner.finished_cb = self._on_finished

        self.request = RunRequest(
            test=test,
            output_root=Path(self.get_parameter('output_dir').value).expanduser(),
            auto_approve=True,
            joint_names=tuple(self.get_parameter('joint_names').value),
            home_position=tuple(self.get_parameter('home_position').value),
            trajectory_duration=float(self.get_parameter('trajectory_duration').value),
            base_tag_frame=self.get_parameter('base_tag_frame').value,
            ee_tag_frame=self.get_parameter('ee_tag_frame').value,
        )

        self.create_timer(3.0, self._start_once)
        self._started = False

    def _start_once(self):
        if self._started:
            return
        self._started = True
        self.get_logger().info(
            f'starting {self.request.test.name} '
            f'({self.request.test.total_visits()} visits)')
        self.runner.request_run(self.request)

    def _on_finished(self, run_dir, status):
        self.get_logger().info(f'run {status}: {run_dir}')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = AccuracyTestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    node.runner.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
