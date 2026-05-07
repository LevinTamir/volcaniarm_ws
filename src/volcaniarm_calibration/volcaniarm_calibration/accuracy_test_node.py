"""Headless CLI entrypoint for an accuracy / repeatability run.

Thin shim around ``CalibrationRunner`` for sim use without the
dashboard. The runner gates capture on a freshly progressed TF stamp
post-settle; the operator-Continue plumbing is replaced here by an
auto-proceed at every gate so the headless run advances unsupervised.
"""

from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .runner import (
    CalibrationRunner,
    RunRequest,
    StaticAccuracyTest,
    TEST_REGISTRY,
)


class AccuracyTestNode(Node):

    def __init__(self):
        super().__init__('accuracy_test_node')
        self.declare_parameter('test_type', 'static_accuracy')
        # Initial pose (workspace y, z metres) is where the arm parks
        # at run start and between visits.
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.5)
        # Goals: flat list of (y, z) pairs, e.g. [0.0, 0.5, 0.1, 0.5].
        # Single-pose tests (static_accuracy / repeatability) supply
        # one pair; the workspace_coverage test supplies the full
        # envelope.
        self.declare_parameter('goals', [0.0, 0.6])
        self.declare_parameter('num_cycles', 5)
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('joint_names',
            ['volcaniarm_right_elbow_joint', 'volcaniarm_left_elbow_joint'])
        self.declare_parameter('trajectory_duration', 2.0)
        self.declare_parameter('base_tag_frame', 'apriltag_marker_base')
        self.declare_parameter('ee_tag_frame', 'apriltag_marker_ee')
        self.declare_parameter('base_urdf_frame', 'apriltag_base_link')
        self.declare_parameter('ee_urdf_frame', 'apriltag_ee_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('detection_timeout_s', 1.0)
        self.declare_parameter('detection_max_age_s', 0.5)
        # Home-confirm gate (used by repeatability / any test that
        # opts in via verify_home_with_tag).
        self.declare_parameter('home_tol_m', 0.02)
        self.declare_parameter('home_hold_frames', 5)
        self.declare_parameter('home_timeout_s', 10.0)
        self.declare_parameter('output_dir', '~/workspaces/volcaniarm_ws/data')

        test_type = self.get_parameter('test_type').value
        cls = TEST_REGISTRY.get(test_type, StaticAccuracyTest)

        flat = list(self.get_parameter('goals').value)
        if len(flat) % 2 != 0 or len(flat) == 0:
            raise ValueError(
                f'goals must be a flat list of even length [y0, z0, y1, z1, ...]; '
                f'got {flat!r}')
        goals = [(float(flat[i]), float(flat[i + 1]))
                 for i in range(0, len(flat), 2)]

        test = cls(
            targets=goals,
            num_cycles=int(self.get_parameter('num_cycles').value),
            settle_time=float(self.get_parameter('settle_time').value),
            return_home_between_targets=True,
        )

        self.runner = CalibrationRunner(self)
        self.runner.finished_cb = self._on_finished
        # Headless: auto-proceed each gate as soon as the runner enters
        # it. Capture gating happens inside _capture_observations on a
        # fresh TF stamp.
        self.runner.awaiting_continue_cb = lambda i, n: self.runner.proceed()

        self.request = RunRequest(
            test=test,
            output_root=Path(self.get_parameter('output_dir').value).expanduser(),
            initial_pose=(float(self.get_parameter('initial_y').value),
                          float(self.get_parameter('initial_z').value)),
            goals=tuple(goals),
            joint_names=tuple(self.get_parameter('joint_names').value),
            trajectory_duration=float(self.get_parameter('trajectory_duration').value),
            base_tag_frame=self.get_parameter('base_tag_frame').value,
            ee_tag_frame=self.get_parameter('ee_tag_frame').value,
            base_urdf_frame=self.get_parameter('base_urdf_frame').value,
            ee_urdf_frame=self.get_parameter('ee_urdf_frame').value,
            world_frame=self.get_parameter('world_frame').value,
            detection_timeout_s=float(self.get_parameter('detection_timeout_s').value),
            detection_max_age_s=float(self.get_parameter('detection_max_age_s').value),
            home_tol_m=float(self.get_parameter('home_tol_m').value),
            home_hold_frames=int(self.get_parameter('home_hold_frames').value),
            home_timeout_s=float(self.get_parameter('home_timeout_s').value),
        )

        self.create_timer(3.0, self._start_once)
        self._started = False

    def _start_once(self):
        if self._started:
            return
        self._started = True
        self.get_logger().info(
            f'starting {self.request.test.name} '
            f'({self.request.test.num_cycles} iterations)')
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
