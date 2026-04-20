#!/usr/bin/env python3
"""Joystick teleop for the volcaniarm EE position in the YZ plane.

Hold the deadman button and deflect the left stick to slew the EE
target along Y (arm plane) and Z (world up). Each tick the node
integrates velocity from the stick, clamps to the workspace, calls
ComputeIK, and streams a short-horizon JointTrajectory point to the
volcaniarm_controller.

On every deadman rising edge the node snaps its internal target to
the arm's current EE position (computed via ComputeFK on the latest
/joint_states), so pressing the deadman never teleports the arm —
motion continues from wherever the arm already is.

The /volcaniarm_controller/joint_trajectory topic is used (not the
follow_joint_trajectory action) because the action handshake is too
heavy for a 20 Hz streaming setpoint — the controller preempts
whatever was in flight the moment a new point arrives.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from volcaniarm_msgs.srv import ComputeIK, ComputeFK


class JoystickTeleopNode(Node):
    def __init__(self):
        super().__init__('volcaniarm_joystick_teleop')

        p = self._declare_params()

        # EE target is None until the first deadman-press syncs it to
        # the arm's current pose. While None, ticks are no-ops.
        self.target_y = None
        self.target_z = None

        # Latest joystick snapshot; updated in joy_cb, consumed on tick.
        self.deadman_held = False
        self.deadman_prev = False
        self.axis_y_val = 0.0
        self.axis_z_val = 0.0

        # Latest elbow joint positions, keyed by name (populated from
        # /joint_states). Needed to FK → current EE when deadman is
        # first pressed.
        self.joint_positions = {}

        # Prevents overlapping service calls.
        self.ik_pending = False
        self.fk_pending = False

        self.ik_client = self.create_client(ComputeIK, 'compute_ik')
        self.fk_client = self.create_client(ComputeFK, 'compute_fk')
        self.traj_pub = self.create_publisher(
            JointTrajectory, p['trajectory_topic'], 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_state_cb, 10)
        self.create_timer(1.0 / p['rate_hz'], self._tick)

        self.params = p
        self.dt = 1.0 / p['rate_hz']
        self.right_joint = p['joint_names'][0]
        self.left_joint = p['joint_names'][1]

        self.get_logger().info(
            f"Joystick teleop ready — deadman=btn{p['deadman_button']}, "
            f"workspace Y[{p['y_min']}, {p['y_max']}] "
            f"Z[{p['z_min']}, {p['z_max']}]. "
            "Target will sync to arm's current pose on first deadman press."
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

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def _tick(self):
        # Detect deadman rising edge → resync target to current EE.
        rising_edge = self.deadman_held and not self.deadman_prev
        self.deadman_prev = self.deadman_held

        if rising_edge:
            self._resync_target_from_current_pose()
            return  # Skip this tick; next one commands from synced target.

        if not self.deadman_held or self.ik_pending:
            return

        # No target yet (first deadman press didn't complete / FK failed) →
        # don't send anything.
        if self.target_y is None or self.target_z is None:
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

    def _resync_target_from_current_pose(self):
        """Snap internal target to current EE via FK on live joint states."""
        if self.fk_pending:
            return
        if self.right_joint not in self.joint_positions or \
           self.left_joint not in self.joint_positions:
            self.get_logger().warn(
                'No /joint_states yet — deadman press ignored, '
                'target not synced.', throttle_duration_sec=2.0)
            return
        if not self.fk_client.service_is_ready():
            self.get_logger().warn(
                'compute_fk service not ready — is volcaniarm_kinematics running?',
                throttle_duration_sec=2.0,
            )
            return

        # ComputeFK convention (see volcaniarm_kinematics.py): theta1 is
        # the URDF right_elbow angle, theta2 is the URDF left_elbow angle.
        req = ComputeFK.Request()
        req.theta1 = self.joint_positions[self.right_joint]
        req.theta2 = self.joint_positions[self.left_joint]
        self.fk_pending = True
        future = self.fk_client.call_async(req)
        future.add_done_callback(self._on_fk_done)

    def _on_fk_done(self, future):
        self.fk_pending = False
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'FK call raised: {e}')
            return
        if not resp.success:
            self.get_logger().warn(f'FK rejected: {resp.message}')
            return
        self.target_y, self.target_z = resp.y, resp.z
        self.get_logger().info(
            f'Target synced to current EE: y={resp.y:.3f}, z={resp.z:.3f}')

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
