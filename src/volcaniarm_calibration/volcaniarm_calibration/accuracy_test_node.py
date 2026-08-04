"""Headless CLI entrypoint for calibration / Experiment 0 runs.

Thin shim around ``CalibrationRunner`` for batch use without the
dashboard. Reintroduced for Experiment 0 (robot characterization):
the noise-gate bursts, settle probes, grid sweeps and 30-cycle anchor
runs are long unattended sessions that want a params-file workflow,
not GUI clicking. The interactive dashboard remains the surface for
one-off runs; see config/exp0_params.yaml for the Exp0 run modes.

The runner gates capture on a freshly progressed TF stamp post-settle;
the operator-Continue plumbing is replaced here by an auto-proceed at
every gate so the headless run advances unsupervised.
"""

from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
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
        # one pair; multi-pose tests (workspace_coverage, noise_gate,
        # backlash) supply the full list.
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
        self.declare_parameter('detection_timeout_s', 5.0)
        self.declare_parameter('detection_max_age_s', 0.5)
        # Home-confirm gate (opt-in for repeatability: the URDF
        # apriltag mounts carry a placeholder bias, so enable it only
        # once the mounts are calibrated or with home_tol_m above the
        # known bias -- see the RepeatabilityTest docstring).
        self.declare_parameter('verify_home_with_tag', False)
        self.declare_parameter('home_tol_m', 0.02)
        self.declare_parameter('home_hold_frames', 5)
        self.declare_parameter('home_timeout_s', 10.0)
        self.declare_parameter(
            'output_dir', '~/workspaces/volcaniarm_ws/experiments/data')
        # -- Exp0 extensions ---------------------------------------
        # Samples per visit: 1 for accuracy/repeatability/sweeps,
        # ~500 for noise_gate, ~120 for settle_probe.
        self.declare_parameter('samples_per_capture', 1)
        self.declare_parameter('sample_min_period_s', 0.0)
        # Sweep pass metadata (recorded in config.yaml; pass 2 runs on
        # a different day / after power cycle).
        self.declare_parameter('pass_id', 1)
        self.declare_parameter('session_note', '')
        # Resume an interrupted sweep: number of already-captured
        # visits to skip (= target row count in the dead run's
        # fk_poses.csv).
        self.declare_parameter('skip_visits', 0)
        # Backlash approach pre-point offset [m].
        self.declare_parameter('approach_offset_m', 0.05)
        # Goals source: 'list' takes the flat `goals` parameter;
        # 'grid' generates a serpentine grid over the task rectangle
        # below, filtered with the in-process kinematics (IK validity,
        # joint limits, closure margin) -- see grid.py.
        self.declare_parameter('goals_source', 'list')
        self.declare_parameter('grid_y0', -0.40)
        self.declare_parameter('grid_y1', 0.40)
        self.declare_parameter('grid_z0', 0.55)
        self.declare_parameter('grid_z1', 0.85)
        self.declare_parameter('grid_spacing_m', 0.025)
        # Measured mechanical joint limit (protocol item 6b); the
        # +-1.13 planning value appears nowhere in the repo, so this
        # MUST be set deliberately when goals_source is 'grid'.
        self.declare_parameter('joint_limit_rad', 1.13)
        self.declare_parameter('limit_margin_rad', 0.05)
        self.declare_parameter('closure_margin_m', 0.02)

        test_type = self.get_parameter('test_type').value
        cls = TEST_REGISTRY.get(test_type, StaticAccuracyTest)

        if self.get_parameter('goals_source').value == 'grid':
            from .grid import serpentine, filter_grid
            pts = serpentine(
                float(self.get_parameter('grid_y0').value),
                float(self.get_parameter('grid_y1').value),
                float(self.get_parameter('grid_z0').value),
                float(self.get_parameter('grid_z1').value),
                float(self.get_parameter('grid_spacing_m').value))
            goals, stats = filter_grid(
                pts,
                float(self.get_parameter('joint_limit_rad').value),
                float(self.get_parameter('limit_margin_rad').value),
                float(self.get_parameter('closure_margin_m').value))
            self.get_logger().info(
                f'grid: {stats.total} candidates -> {stats.kept} kept '
                f'(ik {stats.ik_invalid}, limit {stats.joint_limit}, '
                f'closure {stats.closure_margin} rejected)')
            if not goals:
                raise ValueError('grid filter kept no goals; check the '
                                 'rectangle and joint_limit_rad')
        else:
            flat = list(self.get_parameter('goals').value)
            if len(flat) % 2 != 0 or len(flat) == 0:
                raise ValueError(
                    f'goals must be a flat list of even length '
                    f'[y0, z0, y1, z1, ...]; got {flat!r}')
            goals = [(float(flat[i]), float(flat[i + 1]))
                     for i in range(0, len(flat), 2)]

        test_kwargs = dict(
            targets=goals,
            num_cycles=int(self.get_parameter('num_cycles').value),
            settle_time=float(self.get_parameter('settle_time').value),
            return_home_between_targets=True,
        )
        if test_type == 'backlash':
            test_kwargs['approach_offset_m'] = float(
                self.get_parameter('approach_offset_m').value)
        if test_type == 'repeatability':
            test_kwargs['verify_home_with_tag'] = bool(
                self.get_parameter('verify_home_with_tag').value)
        test = cls(**test_kwargs)

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
            samples_per_capture=int(
                self.get_parameter('samples_per_capture').value),
            sample_min_period_s=float(
                self.get_parameter('sample_min_period_s').value),
            pass_id=int(self.get_parameter('pass_id').value),
            session_note=str(self.get_parameter('session_note').value),
            skip_visits=int(self.get_parameter('skip_visits').value),
        )

        self.create_timer(3.0, self._start_once)
        self._started = False
        self.exit_code = 1

    def _start_once(self):
        if self._started:
            return
        self._started = True
        self.get_logger().info(
            f'starting {self.request.test.name} '
            f'({self.request.test.total_visits()} captured visits)')
        self.runner.request_run(self.request)

    def _on_finished(self, run_dir, status):
        self.get_logger().info(f'run {status}: {run_dir}')
        self.exit_code = 0 if status == 'completed' else 1
        # Called from the runner's worker thread, where a SystemExit
        # would just kill that thread and leave the executor spinning
        # forever. Shutting the rclpy context down instead makes
        # executor.spin() in main() return, so unattended Exp0 runs
        # exit on completion.
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AccuracyTestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.runner.shutdown()
    try:
        node.destroy_node()
        rclpy.try_shutdown()
    except Exception:
        pass
    raise SystemExit(node.exit_code)


if __name__ == '__main__':
    main()
