#!/usr/bin/env python3
"""Joystick teleop for the volcaniarm EE position in the YZ plane.

Hold the deadman button and deflect the left stick to slew the EE
target along Y (arm plane) and Z (world up). Each tick the node
integrates velocity from the stick, clamps to the workspace, calls
ComputeIK, and streams a short-horizon JointTrajectory point to the
volcaniarm_controller.

The /volcaniarm_controller/joint_trajectory topic is used (not the
follow_joint_trajectory action) because the action handshake is too
heavy for a 20 Hz streaming setpoint — the controller preempts
whatever was in flight the moment a new point arrives.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from volcaniarm_interfaces.srv import ComputeIK


class JoystickTeleopNode(Node):
    def __init__(self):
        super().__init__('volcaniarm_joystick_teleop')

        p = self._declare_params()

        # Mutable EE target, seeded at home.
        self.target_y = p['home_y']
        self.target_z = p['home_z']

        # Latest joystick snapshot; updated in joy_cb, consumed on tick.
        self.deadman_held = False
        self.axis_y_val = 0.0
        self.axis_z_val = 0.0

        # Prevents overlapping IK calls when a previous one is in flight.
        self.ik_pending = False

        self.ik_client = self.create_client(ComputeIK, 'compute_ik')
        self.traj_pub = self.create_publisher(
            JointTrajectory, p['trajectory_topic'], 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_timer(1.0 / p['rate_hz'], self._tick)

        self.params = p
        self.dt = 1.0 / p['rate_hz']

        self.get_logger().info(
            f"Joystick teleop ready — deadman=btn{p['deadman_button']}, "
            f"home=({p['home_y']:.2f}, {p['home_z']:.2f}), "
            f"workspace Y[{p['y_min']}, {p['y_max']}] Z[{p['z_min']}, {p['z_max']}]"
        )

    def _declare_params(self):
        defaults = {
            'rate_hz': 20.0,
            'deadman_button': 5,
            'axis_y': 0,
            'axis_z': 1,
            'invert_y': False,
            'invert_z': False,
            'scale_y': 0.15,
            'scale_z': 0.15,
            'y_min': -0.35, 'y_max': 0.35,
            'z_min': 0.30, 'z_max': 1.00,
            'home_y': 0.0, 'home_z': 0.65,
            'trajectory_horizon': 0.2,
            'trajectory_topic': '/volcaniarm_controller/joint_trajectory',
            'joint_names': ['volcaniarm_right_elbow_joint',
                            'volcaniarm_left_elbow_joint'],
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        return {k: self.get_parameter(k).value for k in defaults}

    def _joy_cb(self, msg: Joy):
        btn_idx = self.params['deadman_button']
        self.deadman_held = (
            btn_idx < len(msg.buttons) and msg.buttons[btn_idx] == 1
        )
        y_idx, z_idx = self.params['axis_y'], self.params['axis_z']
        self.axis_y_val = msg.axes[y_idx] if y_idx < len(msg.axes) else 0.0
        self.axis_z_val = msg.axes[z_idx] if z_idx < len(msg.axes) else 0.0

    def _tick(self):
        if not self.deadman_held or self.ik_pending:
            return

        # Integrate velocity → new target
        vy = self.axis_y_val * self.params['scale_y']
        vz = self.axis_z_val * self.params['scale_z']
        if self.params['invert_y']:
            vy = -vy
        if self.params['invert_z']:
            vz = -vz

        new_y = self.target_y + vy * self.dt
        new_z = self.target_z + vz * self.dt

        new_y = max(self.params['y_min'], min(self.params['y_max'], new_y))
        new_z = max(self.params['z_min'], min(self.params['z_max'], new_z))

        # Skip IK round-trip if nothing changed (stick centered + already clamped).
        if new_y == self.target_y and new_z == self.target_z:
            return

        self.target_y, self.target_z = new_y, new_z

        if not self.ik_client.service_is_ready():
            self.get_logger().warn(
                'compute_ik service not ready — is volcaniarm_kinematics running?',
                throttle_duration_sec=2.0,
            )
            return

        req = ComputeIK.Request()
        req.x = 0.0
        req.y = self.target_y
        req.z = self.target_z
        self.ik_pending = True
        future = self.ik_client.call_async(req)
        future.add_done_callback(self._on_ik_done)

    def _on_ik_done(self, future):
        self.ik_pending = False
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'IK call raised: {e}')
            return

        if not resp.success:
            self.get_logger().warn(f'IK rejected: {resp.message}')
            return

        horizon = self.params['trajectory_horizon']
        sec = int(horizon)
        nsec = int((horizon - sec) * 1e9)

        traj = JointTrajectory()
        traj.joint_names = list(self.params['joint_names'])
        pt = JointTrajectoryPoint()
        pt.positions = [resp.theta1, resp.theta2]
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
        self.traj_pub.publish(traj)


def main():
    rclpy.init()
    node = JoystickTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
