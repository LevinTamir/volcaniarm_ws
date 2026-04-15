"""End-effector accuracy and repeatability test node.

Sends the arm to a sequence of target positions for multiple cycles,
records the AprilTag-detected EE pose at each visit, and saves results
to CSV files suitable for plotting in a thesis.

Output files (in output_dir):
  - <prefix>_raw_<timestamp>.csv      Per-sample raw data
  - <prefix>_summary_<timestamp>.csv  Per-target statistics
"""

import os
import csv
import math
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from volcaniarm_msgs.srv import ComputeIK
from builtin_interfaces.msg import Duration

import tf2_ros
import numpy as np


class AccuracyTestNode(Node):

    # States for the test state machine
    STATE_IDLE = 'idle'
    STATE_MOVING = 'moving'
    STATE_SETTLING = 'settling'
    STATE_SAMPLING = 'sampling'
    STATE_DONE = 'done'

    def __init__(self):
        super().__init__('accuracy_test_node')
        self.cb_group = ReentrantCallbackGroup()

        # -- Declare all parameters --
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
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('tag_frame', 'apriltag_marker')
        self.declare_parameter('output_dir', '~/volcaniarm_calibration_data')
        self.declare_parameter('output_prefix', 'ee_accuracy')

        # -- Read parameters --
        raw_targets = self.get_parameter('targets').value
        # Parse flat list into pairs: [y1, z1, y2, z2, ...] -> [(y1,z1), (y2,z2), ...]
        self.targets = []
        for i in range(0, len(raw_targets), 2):
            self.targets.append((raw_targets[i], raw_targets[i + 1]))

        self.num_cycles = self.get_parameter('num_cycles').value
        self.settle_time = self.get_parameter('settle_time').value
        self.samples_per_visit = self.get_parameter('samples_per_visit').value
        self.sample_interval = self.get_parameter('sample_interval').value
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.trajectory_duration = self.get_parameter('trajectory_duration').value
        self.home_position = list(self.get_parameter('home_position').value)
        self.return_home = self.get_parameter('return_home_between_targets').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value

        output_dir = self.get_parameter('output_dir').value
        self.output_dir = Path(os.path.expanduser(output_dir))
        self.output_prefix = self.get_parameter('output_prefix').value

        # -- TF2 --
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -- IK client --
        self.ik_client = self.create_client(
            ComputeIK, 'compute_ik', callback_group=self.cb_group)

        # -- Trajectory action client --
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            '/volcaniarm_controller/follow_joint_trajectory',
            callback_group=self.cb_group)

        # -- State --
        self.state = self.STATE_IDLE
        self.current_cycle = 0
        self.current_target_idx = 0
        self.current_samples = []
        self.all_measurements = []  # list of dicts for CSV

        # -- Prepare output directory --
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')

        # -- Start after a short delay to let TF and services come up --
        self.create_timer(3.0, self._start_test, callback_group=self.cb_group)

        self.get_logger().info(
            f'Accuracy test configured: {len(self.targets)} targets x '
            f'{self.num_cycles} cycles x {self.samples_per_visit} samples/visit')

    # ── Test orchestration ────────────────────────────────────────

    def _start_test(self):
        """Begin the test sequence (called once after startup delay)."""
        if self.state != self.STATE_IDLE:
            return
        self.get_logger().info('Starting accuracy test...')
        self._next_move()

    def _next_move(self):
        """Advance to the next target or cycle, or finish the test."""
        if self.current_cycle >= self.num_cycles:
            self._finish_test()
            return

        if self.current_target_idx >= len(self.targets):
            self.current_target_idx = 0
            self.current_cycle += 1
            if self.current_cycle >= self.num_cycles:
                self._finish_test()
                return

        target_y, target_z = self.targets[self.current_target_idx]
        self.get_logger().info(
            f'Cycle {self.current_cycle + 1}/{self.num_cycles}, '
            f'target {self.current_target_idx + 1}/{len(self.targets)}: '
            f'y={target_y:.3f}, z={target_z:.3f}')

        self.state = self.STATE_MOVING
        self._move_to_target(target_y, target_z)

    def _move_to_target(self, y, z):
        """Call IK and send trajectory to the target."""
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('IK service not available, aborting test')
            self.state = self.STATE_DONE
            return

        request = ComputeIK.Request()
        request.x = 0.0
        request.y = y
        request.z = z

        future = self.ik_client.call_async(request)
        future.add_done_callback(self._on_ik_result)

    def _on_ik_result(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'IK call failed: {e}')
            self._advance_target()
            return

        if not response.success:
            self.get_logger().error(f'IK failed: {response.message}')
            self._advance_target()
            return

        self._send_trajectory(response.theta1, response.theta2)

    def _send_trajectory(self, theta_right, theta_left):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Trajectory action server not available')
            self._advance_target()
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [theta_right, theta_left]
        point.velocities = [0.0, 0.0]
        sec = int(self.trajectory_duration)
        nsec = int((self.trajectory_duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal.trajectory.points.append(point)

        send_future = self.action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            self._advance_target()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_trajectory_done)

    def _on_trajectory_done(self, future):
        result = future.result().result
        if result.error_code != 0:
            self.get_logger().error(
                f'Trajectory failed (code={result.error_code})')
            self._advance_target()
            return

        # Wait for settle time, then start sampling
        self.state = self.STATE_SETTLING
        self.get_logger().info(
            f'Arrived at target, settling for {self.settle_time:.1f}s...')
        self._settle_timer = self.create_timer(
            self.settle_time, self._on_settled, callback_group=self.cb_group)

    def _on_settled(self):
        self._settle_timer.cancel()
        self._settle_timer.destroy()
        self.state = self.STATE_SAMPLING
        self.current_samples = []
        self._sample_count = 0
        self.get_logger().info(
            f'Collecting {self.samples_per_visit} pose samples...')
        self._sample_timer = self.create_timer(
            self.sample_interval, self._collect_sample,
            callback_group=self.cb_group)

    # ── Pose sampling ─────────────────────────────────────────────

    def _collect_sample(self):
        if self._sample_count >= self.samples_per_visit:
            self._sample_timer.cancel()
            self._sample_timer.destroy()
            self._on_sampling_done()
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, self.tag_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        t = transform.transform.translation
        r = transform.transform.rotation
        self.current_samples.append({
            'x': t.x, 'y': t.y, 'z': t.z,
            'qx': r.x, 'qy': r.y, 'qz': r.z, 'qw': r.w,
        })
        self._sample_count += 1

    def _on_sampling_done(self):
        target_y, target_z = self.targets[self.current_target_idx]
        n = len(self.current_samples)

        if n == 0:
            self.get_logger().warn('No samples collected for this visit!')
            self._advance_target()
            return

        # Compute mean position from samples
        xs = [s['x'] for s in self.current_samples]
        ys = [s['y'] for s in self.current_samples]
        zs = [s['z'] for s in self.current_samples]
        mean_x, mean_y, mean_z = np.mean(xs), np.mean(ys), np.mean(zs)
        std_x, std_y, std_z = np.std(xs), np.std(ys), np.std(zs)

        self.get_logger().info(
            f'Collected {n} samples -- '
            f'mean=({mean_x:.4f}, {mean_y:.4f}, {mean_z:.4f}), '
            f'std=({std_x:.4f}, {std_y:.4f}, {std_z:.4f})')

        # Store raw measurements
        for i, s in enumerate(self.current_samples):
            self.all_measurements.append({
                'cycle': self.current_cycle + 1,
                'target_idx': self.current_target_idx + 1,
                'target_y': target_y,
                'target_z': target_z,
                'sample_idx': i + 1,
                'measured_x': s['x'],
                'measured_y': s['y'],
                'measured_z': s['z'],
                'measured_qx': s['qx'],
                'measured_qy': s['qy'],
                'measured_qz': s['qz'],
                'measured_qw': s['qw'],
            })

        self._advance_target()

    def _advance_target(self):
        """Move to next target, optionally returning home first."""
        self.current_target_idx += 1

        if self.return_home and self.current_target_idx < len(self.targets):
            self._go_home_then_next()
        else:
            self._next_move()

    def _go_home_then_next(self):
        """Send arm home, then proceed to next target."""
        self.get_logger().info('Returning home...')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.home_position
        point.velocities = [0.0, 0.0]
        sec = int(self.trajectory_duration)
        nsec = int((self.trajectory_duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal.trajectory.points.append(point)

        send_future = self.action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_home_goal_response)

    def _on_home_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Home goal rejected, continuing anyway')
            self._next_move()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._next_move())

    # ── Output ────────────────────────────────────────────────────

    def _finish_test(self):
        if self.state == self.STATE_DONE:
            return
        self.state = self.STATE_DONE

        if not self.all_measurements:
            self.get_logger().error('No measurements collected!')
            return

        self._save_raw_csv()
        self._save_summary_csv()
        self.get_logger().info('Accuracy test complete.')

    def _save_raw_csv(self):
        filename = f'{self.output_prefix}_raw_{self.timestamp_str}.csv'
        filepath = self.output_dir / filename

        fieldnames = [
            'cycle', 'target_idx', 'target_y', 'target_z', 'sample_idx',
            'measured_x', 'measured_y', 'measured_z',
            'measured_qx', 'measured_qy', 'measured_qz', 'measured_qw',
        ]

        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.all_measurements)

        self.get_logger().info(
            f'Raw data saved: {filepath} ({len(self.all_measurements)} samples)')

    def _save_summary_csv(self):
        filename = f'{self.output_prefix}_summary_{self.timestamp_str}.csv'
        filepath = self.output_dir / filename

        fieldnames = [
            'target_idx', 'target_y', 'target_z', 'num_visits', 'total_samples',
            'mean_x', 'mean_y', 'mean_z',
            'std_x', 'std_y', 'std_z',
            'repeatability_xyz',
        ]

        # Group measurements by target
        targets_data = {}
        for m in self.all_measurements:
            idx = m['target_idx']
            if idx not in targets_data:
                targets_data[idx] = {
                    'target_y': m['target_y'],
                    'target_z': m['target_z'],
                    'xs': [], 'ys': [], 'zs': [],
                }
            targets_data[idx]['xs'].append(m['measured_x'])
            targets_data[idx]['ys'].append(m['measured_y'])
            targets_data[idx]['zs'].append(m['measured_z'])

        rows = []
        for idx in sorted(targets_data.keys()):
            d = targets_data[idx]
            xs, ys, zs = np.array(d['xs']), np.array(d['ys']), np.array(d['zs'])
            std_x, std_y, std_z = np.std(xs), np.std(ys), np.std(zs)
            # Repeatability = RMS of per-axis standard deviations
            repeatability = math.sqrt(std_x**2 + std_y**2 + std_z**2)

            rows.append({
                'target_idx': idx,
                'target_y': d['target_y'],
                'target_z': d['target_z'],
                'num_visits': self.num_cycles,
                'total_samples': len(xs),
                'mean_x': f'{np.mean(xs):.6f}',
                'mean_y': f'{np.mean(ys):.6f}',
                'mean_z': f'{np.mean(zs):.6f}',
                'std_x': f'{std_x:.6f}',
                'std_y': f'{std_y:.6f}',
                'std_z': f'{std_z:.6f}',
                'repeatability_xyz': f'{repeatability:.6f}',
            })

        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        self.get_logger().info(f'Summary saved: {filepath}')

        # Also print summary to terminal
        self.get_logger().info('=== ACCURACY TEST SUMMARY ===')
        for row in rows:
            self.get_logger().info(
                f"  Target {row['target_idx']} "
                f"(y={row['target_y']}, z={row['target_z']}): "
                f"mean=({row['mean_x']}, {row['mean_y']}, {row['mean_z']}), "
                f"repeatability={row['repeatability_xyz']} m")


def main(args=None):
    rclpy.init(args=args)
    node = AccuracyTestNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
