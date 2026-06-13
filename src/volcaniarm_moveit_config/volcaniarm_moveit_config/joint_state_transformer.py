#!/usr/bin/env python3
"""joint_state_transformer: bridge MoveIt's phantom EE plan to the real motors.

MoveIt plans the open phantom chain (tool_joint_y, tool_joint_z = EE displacement
from home in Y, Z) and *executes* it to this node as a FollowJointTrajectory
(MoveIt thinks it's an ordinary controller, see moveit_controllers.yaml). Each
waypoint is converted to real elbow angles via volcaniarm_kinematics.inverse_ee
and forwarded to the JTC at /volcaniarm_controller/follow_joint_trajectory. The
node waits for the JTC result and reports it back to MoveIt.

The passive_broadcaster already publishes the full /joint_states, so this node
does not need to (unlike the reference's robot_model-based transformer).
"""

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import volcaniarm_kinematics_py as vk

# Full real-joint order for the preview ghost (planning model joint names).
REAL_JOINTS = [
    'volcaniarm_left_elbow_joint', 'volcaniarm_right_elbow_joint',
    'volcaniarm_left_arm_joint', 'volcaniarm_right_arm_joint', 'closure_joint',
]


class JointStateTransformer(Node):
    def __init__(self):
        super().__init__('joint_state_transformer')

        self.declare_parameter(
            'output_action', '/volcaniarm_controller/follow_joint_trajectory')
        # Real JTC joint order is [right, left] (matches volcaniarm_controllers.yaml).
        self.declare_parameter(
            'real_joint_names',
            ['volcaniarm_right_elbow_joint', 'volcaniarm_left_elbow_joint'])
        # Home EE in volcaniarm_base_link (forward_ee at q=0); phantom joints are
        # the displacement from this point.
        self.declare_parameter('home_ee_y', 0.0)
        self.declare_parameter('home_ee_z', 0.7489)

        out_action = self.get_parameter('output_action').value
        self.real_joints = list(self.get_parameter('real_joint_names').value)
        self.home_y = self.get_parameter('home_ee_y').value
        self.home_z = self.get_parameter('home_ee_z').value
        self.kin = vk.Params()

        self._left_joint = 'volcaniarm_left_elbow_joint'
        self._right_joint = 'volcaniarm_right_elbow_joint'

        cb = ReentrantCallbackGroup()
        self._client = ActionClient(
            self, FollowJointTrajectory, out_action, callback_group=cb)
        self._server = ActionServer(
            self, FollowJointTrajectory, '~/follow_joint_trajectory',
            execute_callback=self._execute, callback_group=cb)

        # Publish the phantom joints (tool_joint_y/z) derived from the real
        # elbow state, so MoveIt's current-state monitor tracks the actual EE
        # (the goal marker starts on the real EE; the planned start is correct).
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10, callback_group=cb)

        # Ghost preview: MoveIt's planned path only moves the phantom tool0 (it
        # can't solve the loop), so expand the planned phantom trajectory into a
        # full real-joint trajectory and republish it for a ghost-arm display.
        self._preview_pub = self.create_publisher(
            DisplayTrajectory, '/display_planned_path_full', 1)
        self.create_subscription(
            DisplayTrajectory, '/display_planned_path', self._on_planned_path, 1,
            callback_group=cb)

        self.get_logger().info(
            f'joint_state_transformer ready: phantom EE plan -> inverse_ee -> {out_action}')

    def _phantom_point_to_full(self, names, point, seed):
        """Map one phantom point -> full real-joint positions (or None)."""
        iy = names.index('tool_joint_y')
        iz = names.index('tool_joint_z')
        ee_y = self.home_y + point.positions[iy]
        ee_z = self.home_z + point.positions[iz]
        ik = vk.inverse_ee(self.kin, ee_y, ee_z, seed[0], seed[1])
        if not ik.valid:
            return None, seed
        pa = vk.passive_angles(self.kin, ik.theta_left, ik.theta_right)
        if not pa.valid:
            return None, seed
        # Order must match REAL_JOINTS.
        pos = [ik.theta_left, ik.theta_right, pa.left_arm, pa.right_arm, pa.closure]
        return pos, (ik.theta_left, ik.theta_right)

    def _on_planned_path(self, msg):
        out = DisplayTrajectory()
        out.model_id = msg.model_id
        out.trajectory_start = msg.trajectory_start
        for rt in msg.trajectory:
            jt = rt.joint_trajectory
            if not jt.points or 'tool_joint_y' not in jt.joint_names \
                    or 'tool_joint_z' not in jt.joint_names:
                continue
            full = JointTrajectory()
            full.header = jt.header
            full.joint_names = list(REAL_JOINTS)
            seed = (0.0, 0.0)
            for p in jt.points:
                pos, seed = self._phantom_point_to_full(jt.joint_names, p, seed)
                if pos is None:
                    continue
                fp = JointTrajectoryPoint()
                fp.positions = pos
                fp.time_from_start = p.time_from_start
                full.points.append(fp)
            new_rt = RobotTrajectory()
            new_rt.joint_trajectory = full
            out.trajectory.append(new_rt)
        self._preview_pub.publish(out)

    def _on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if self._left_joint not in pos or self._right_joint not in pos:
            return  # not the real-elbow message (skips our own phantom echo)
        ee = vk.forward_ee(self.kin, pos[self._left_joint], pos[self._right_joint])
        if not ee.valid:
            return
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = ['tool_joint_y', 'tool_joint_z']
        out.position = [ee.y - self.home_y, ee.z - self.home_z]
        self._js_pub.publish(out)

    def _phantom_to_real(self, traj_in):
        """Convert a phantom (tool_joint_y/z) trajectory to a real elbow trajectory."""
        names = list(traj_in.joint_names)
        if 'tool_joint_y' not in names or 'tool_joint_z' not in names:
            self.get_logger().error(f'Unexpected phantom joints: {names}')
            return None
        iy = names.index('tool_joint_y')
        iz = names.index('tool_joint_z')

        out = JointTrajectory()
        out.joint_names = self.real_joints
        seed_l, seed_r = 0.0, 0.0
        for pt in traj_in.points:
            ee_y = self.home_y + pt.positions[iy]
            ee_z = self.home_z + pt.positions[iz]
            ik = vk.inverse_ee(self.kin, ee_y, ee_z, seed_l, seed_r)
            if not ik.valid:
                self.get_logger().error(
                    f'inverse_ee unreachable at EE y={ee_y:.3f} z={ee_z:.3f}')
                return None
            seed_l, seed_r = ik.theta_left, ik.theta_right
            rp = JointTrajectoryPoint()
            rp.positions = [ik.theta_right, ik.theta_left]  # [right, left]
            rp.time_from_start = pt.time_from_start
            out.points.append(rp)
        return out

    async def _execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        real = self._phantom_to_real(goal_handle.request.trajectory)
        if real is None:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Real JTC action server not available')
            goal_handle.abort()
            return result

        fwd = FollowJointTrajectory.Goal()
        fwd.trajectory = real
        handle = await self._client.send_goal_async(fwd)
        if not handle.accepted:
            self.get_logger().error('Real JTC rejected the forwarded goal')
            goal_handle.abort()
            return result

        wrapped = await handle.get_result_async()
        result = wrapped.result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = JointStateTransformer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
