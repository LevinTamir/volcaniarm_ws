#!/usr/bin/env python3
"""
Motion planning node for the volcaniarm.

Features:
  - Subscribes to weed detection topic, transforms position to arm frame via tf2
  - Calls IK service to compute joint angles
  - Sends trajectory to the controller
  - Returns to home position after each operation
  - Queues incoming detections to avoid dropping targets
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from collections import deque

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from volcaniarm_interfaces.srv import ComputeIK
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planning_node')
        self.cb_group = ReentrantCallbackGroup()

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('trajectory_duration', 2.0)
        self.declare_parameter('dwell_time', 2.0)
        self.declare_parameter('home_duration', 2.0)
        self.declare_parameter('weed_topic', '/weed_position')
        self.declare_parameter('arm_frame', 'volcaniarm_base_link')
        self.declare_parameter('home_position', [0.0, 0.0])
        self.declare_parameter('return_home', True)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('joint_names',
            ['volcaniarm_right_elbow_joint', 'volcaniarm_left_elbow_joint'])
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)

        self.trajectory_duration = self.get_parameter('trajectory_duration').value
        self.dwell_time = self.get_parameter('dwell_time').value
        self.home_duration = self.get_parameter('home_duration').value
        weed_topic = self.get_parameter('weed_topic').value
        self.arm_frame = self.get_parameter('arm_frame').value
        self.home_position = list(self.get_parameter('home_position').value)
        self.return_home = self.get_parameter('return_home').value
        queue_size = self.get_parameter('queue_size').value
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.offset_y = self.get_parameter('offset_y').value
        self.offset_z = self.get_parameter('offset_z').value

        # ── TF2 ──────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── IK client ────────────────────────────────────────────
        self.ik_client = self.create_client(
            ComputeIK, 'compute_ik', callback_group=self.cb_group)

        # ── Trajectory action ────────────────────────────────────
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            '/volcaniarm_controller/follow_joint_trajectory',
            callback_group=self.cb_group)

        # ── Weed subscriber ──────────────────────────────────────
        self.create_subscription(
            PointStamped, weed_topic,
            self.weed_position_cb, 10,
            callback_group=self.cb_group)

        # ── State ────────────────────────────────────────────────
        self.busy = False
        self.target_queue = deque(maxlen=queue_size)

        # Timer to process queued targets
        self.create_timer(0.1, self._process_queue, callback_group=self.cb_group)

        self.get_logger().info(
            f'Motion planning ready — listening on {weed_topic}, '
            f'arm_frame={self.arm_frame}, return_home={self.return_home}')

    # ── Weed detection callback ───────────────────────────────────

    def weed_position_cb(self, msg: PointStamped):
        """Transform weed position to arm frame and queue it."""
        try:
            # Transform from detection frame to arm frame
            if msg.header.frame_id != self.arm_frame:
                transform = self.tf_buffer.lookup_transform(
                    self.arm_frame, msg.header.frame_id,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
                msg = do_transform_point(msg, transform)
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e} — using raw coordinates')

        y = msg.point.y + self.offset_y
        z = msg.point.z + self.offset_z
        self.target_queue.append((y, z))
        self.get_logger().info(
            f'Queued weed at y={y:.3f}, z={z:.3f} '
            f'({len(self.target_queue)} in queue)')

    # ── Queue processing ──────────────────────────────────────────

    def _process_queue(self):
        """Process next target from the queue if not busy."""
        if self.busy or not self.target_queue:
            return

        y, z = self.target_queue.popleft()
        self.get_logger().info(f'Processing target y={y:.3f}, z={z:.3f}')
        self._move_to_position(y, z)

    def _move_to_position(self, y, z):
        """Call IK and send trajectory to the target position."""
        if not self.ik_client.service_is_ready():
            self.get_logger().warn('IK service not available, re-queueing')
            self.target_queue.appendleft((y, z))
            return

        self.busy = True
        request = ComputeIK.Request()
        request.x = 0.0
        request.y = y
        request.z = z

        future = self.ik_client.call_async(request)
        future.add_done_callback(
            lambda f: self._on_ik_result(f, go_home_after=self.return_home))

    def _on_ik_result(self, future, go_home_after=False):
        """Handle IK response and send trajectory."""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'IK call failed: {e}')
            self.busy = False
            return

        if not response.success:
            self.get_logger().error(f'IK failed: {response.message}')
            self.busy = False
            return

        self.get_logger().info(
            f'IK → R={response.theta1:.3f}, L={response.theta2:.3f}')
        self._send_trajectory(
            response.theta1, response.theta2,
            self.trajectory_duration,
            go_home_after=go_home_after)

    # ── Trajectory execution ──────────────────────────────────────

    def _send_trajectory(self, theta_right, theta_left, duration,
                         go_home_after=False):
        """Send a single-point trajectory."""
        if not self.action_client.server_is_ready():
            self.get_logger().warn('Action server not ready')
            self.busy = False
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [theta_right, theta_left]
        point.velocities = [0.0, 0.0]
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal.trajectory.points.append(point)

        send_future = self.action_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, go_home_after))

    def _on_goal_response(self, future, go_home_after):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.busy = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_trajectory_done(f, go_home_after))

    def _on_trajectory_done(self, future, go_home_after):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Trajectory completed')
        else:
            self.get_logger().error(f'Trajectory failed (code={result.error_code})')

        if go_home_after and self.dwell_time > 0:
            self.get_logger().info(f'Dwelling at target for {self.dwell_time:.1f}s')
            self._dwell_timer = self.create_timer(
                self.dwell_time, lambda: self._on_dwell_done(),
                callback_group=self.cb_group)
        elif go_home_after:
            self._go_home()
        else:
            self.busy = False

    def _on_dwell_done(self):
        """Called after dwell time at target expires."""
        self._dwell_timer.cancel()
        self._dwell_timer.destroy()
        self._go_home()

    # ── Home position ─────────────────────────────────────────────

    def _go_home(self):
        """Send arm to home position."""
        self.get_logger().info('Returning to home position')
        self._send_trajectory(
            self.home_position[0], self.home_position[1],
            self.home_duration,
            go_home_after=False)


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
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
