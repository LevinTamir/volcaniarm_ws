#!/usr/bin/env python3
"""Joystick teleop for the volcaniarm EE position in the YZ plane.

Hold the deadman button and deflect the left stick to slew the EE
target along Y (arm plane) and Z (world up). Each tick the node
integrates velocity from the stick, clamps to the workspace, solves
IK in-process (volcaniarm_kinematics), and streams a short-horizon
JointTrajectory point to the volcaniarm_controller.

On every deadman rising edge the node snaps its internal target to
the arm's current EE position (forward_ee on the latest /joint_states),
so pressing the deadman never teleports the arm — motion continues from
wherever the arm already is.

The /volcaniarm_controller/joint_trajectory topic is used (not the
follow_joint_trajectory action) because the action handshake is too
heavy for a 20 Hz streaming setpoint — the controller preempts
whatever was in flight the moment a new point arrives.

Kinematics come from the volcaniarm_kinematics C++ library via its pybind
module (the single source of truth); IK/FK are computed in-process, so
there is no compute_ik/compute_fk service dependency.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import volcaniarm_kinematics_py as vk


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
        self.home_btn_held = False
        self.home_btn_prev = False
        self.axis_y_val = 0.0
        self.axis_z_val = 0.0

        # Latest elbow joint positions, keyed by name (populated from
        # /joint_states). Needed to FK → current EE when deadman is
        # first pressed, and to seed IK branch selection.
        self.joint_positions = {}

        # Linkage geometry. Defaults mirror the URDF exactly (guarded by the
        # Pinocchio oracle); IK/FK run in-process via the C++ library.
        self.kin = vk.Params()

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
            # Home: deadman + this button publishes a single trajectory
            # point sending both elbows to home_positions over
            # home_duration_sec. Default button 0 = X (Xbox) / square (PS).
            'home_button': 0,
            'home_positions': [0.0, 0.0],
            'home_duration_sec': 3.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        return {k: self.get_parameter(k).value for k in defaults}

    def _joy_cb(self, msg: Joy):
        btn_idx = self.params['deadman_button']
        self.deadman_held = (
            btn_idx < len(msg.buttons) and msg.buttons[btn_idx] == 1
        )
        h_idx = self.params['home_button']
        self.home_btn_held = (
            h_idx < len(msg.buttons) and msg.buttons[h_idx] == 1
        )
        y_idx, z_idx = self.params['axis_y'], self.params['axis_z']
        self.axis_y_val = msg.axes[y_idx] if y_idx < len(msg.axes) else 0.0
        self.axis_z_val = msg.axes[z_idx] if z_idx < len(msg.axes) else 0.0

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def _tick(self):
        # Detect deadman + home-button rising edges. Both _prev flags
        # advance every tick so holding either button does not re-fire.
        rising_edge = self.deadman_held and not self.deadman_prev
        home_rising = self.home_btn_held and not self.home_btn_prev
        self.deadman_prev = self.deadman_held
        self.home_btn_prev = self.home_btn_held

        if rising_edge:
            self._resync_target_from_current_pose()
            return  # Skip this tick; next one commands from synced target.

        if self.deadman_held and home_rising:
            self._publish_home()
            return

        if not self.deadman_held:
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

        # Seed IK with the current elbow angles for branch continuity.
        seed_left = self.joint_positions.get(self.left_joint, 0.0)
        seed_right = self.joint_positions.get(self.right_joint, 0.0)
        ik = vk.inverse_ee(self.kin, self.target_y, self.target_z, seed_left, seed_right)
        if not ik.valid:
            self.get_logger().warn(
                f'IK unreachable for y={self.target_y:.3f}, z={self.target_z:.3f}',
                throttle_duration_sec=2.0)
            return
        # joint_names order is [right_elbow, left_elbow].
        self._publish_point([ik.theta_right, ik.theta_left],
                            self.params['trajectory_horizon'])

    def _publish_point(self, positions, horizon):
        sec = int(horizon)
        nsec = int((horizon - sec) * 1e9)
        traj = JointTrajectory()
        traj.joint_names = list(self.params['joint_names'])
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
        self.traj_pub.publish(traj)

    def _publish_home(self):
        """Send both elbows to the configured home positions."""
        positions = list(self.params['home_positions'])
        self.get_logger().info(f"Going to home position {tuple(positions)}")
        self._publish_point(positions, float(self.params['home_duration_sec']))
        # Force the next deadman press to re-FK from the post-home pose,
        # so stick teleop doesn't drag the arm back to the stale target.
        self.target_y = None
        self.target_z = None

    def _resync_target_from_current_pose(self):
        """Snap internal target to current EE via forward_ee on live states."""
        if self.right_joint not in self.joint_positions or \
           self.left_joint not in self.joint_positions:
            self.get_logger().warn(
                'No /joint_states yet — deadman press ignored, '
                'target not synced.', throttle_duration_sec=2.0)
            return
        tl = self.joint_positions[self.left_joint]
        tr = self.joint_positions[self.right_joint]
        ee = vk.forward_ee(self.kin, tl, tr)
        if not ee.valid:
            self.get_logger().warn('forward_ee invalid at current pose',
                                   throttle_duration_sec=2.0)
            return
        self.target_y, self.target_z = ee.y, ee.z
        self.get_logger().info(
            f'Target synced to current EE: y={ee.y:.3f}, z={ee.z:.3f}')


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
