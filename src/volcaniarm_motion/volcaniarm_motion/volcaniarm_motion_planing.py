#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from volcaniarm_interfaces.srv import ComputeIK
from builtin_interfaces.msg import Duration


class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planning_node')
        
        # Create a client for the IK service
        self.ik_client = self.create_client(ComputeIK, 'compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        
        # Create an action client for the joint trajectory controller
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.joint_names = ['joint_1', 'joint_2']
        self.get_logger().info('Motion Planning Node initialized')
        
    def call_ik_service(self, x, y, z):
        """Call the inverse kinematics service"""
        request = ComputeIK.Request()
        request.x = x
        request.y = y
        request.z = z
        
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                return response.theta1, response.theta2
            else:
                self.get_logger().error(f"IK failed: {response.message}")
                return None
        else:
            self.get_logger().error('IK service call failed')
            return None
    
    def execute_trajectory(self, waypoints):
        """
        Execute a trajectory through multiple waypoints.
        
        Parameters
        ----------
        waypoints : list of tuples
            List of (x, y, z) positions for the end effector to go through
            Example: [(0.0, 0.3, 0.2), (0.0, 0.4, 0.3), (0.0, 0.35, 0.25)]
        """
        if not waypoints:
            self.get_logger().warn('No waypoints provided')
            return
        
        self.get_logger().info(f'Planning trajectory through {len(waypoints)} waypoint(s)')
        
        # Compute IK for all waypoints
        joint_positions = []
        for i, (x, y, z) in enumerate(waypoints):
            self.get_logger().info(f'Computing IK for waypoint {i+1}: ({x}, {y}, {z})')
            result = self.call_ik_service(x, y, z)
            if result is None:
                self.get_logger().error(f'Failed to compute IK for waypoint {i+1}')
                return
            joint_positions.append(result)
        
        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Add points to trajectory
        time_offset = 2.0  # seconds between waypoints
        for i, (theta1, theta2) in enumerate(joint_positions):
            point = JointTrajectoryPoint()
            point.positions = [theta1, theta2]
            point.velocities = [0.0, 0.0]
            point.accelerations = [0.0, 0.0]
            
            # Set time from start
            total_seconds = time_offset * (i + 1)
            point.time_from_start = Duration(sec=int(total_seconds), nanosec=0)
            
            trajectory.points.append(point)
        
        # Create goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.get_logger().info('Sending trajectory goal to controller...')
        
        # Wait for action server
        self.action_client.wait_for_server()
        
        # Send goal
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by controller')
            return
        
        self.get_logger().info('Goal accepted by controller')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        self.get_logger().info(f'Trajectory execution completed with error code: {result.error_code}')
        
    def feedback_callback(self, feedback_msg):
        """Callback for action feedback"""
        feedback = feedback_msg.feedback
        pass


def main(args=None):
    rclpy.init(args=args)
    motion_planner = MotionPlanningNode()
    
    # Example usage: Move through 3 waypoints
    # Format: (x, y, z) - X is fixed at 0.0 for now
    waypoints = [
        (0.0, 0.3, 0.2),
        (0.0, 0.4, 0.3),
        (0.0, 0.35, 0.15)
    ]
    
    # Or single point:
    # waypoints = [(0.0, 0.4, 0.2)]
    
    try:
        motion_planner.execute_trajectory(waypoints)
    except KeyboardInterrupt:
        pass
    
    motion_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
