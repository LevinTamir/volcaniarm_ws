#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from volcaniarm_interfaces.srv import ComputeIK
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped


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
            '/volcaniarm_controller/follow_joint_trajectory'
        )
        
        # Subscribe to weed position topic
        self.weed_position_sub = self.create_subscription(
            PointStamped,
            '/weed_position',
            self.weed_position_callback,
            10
        )
        
        self.joint_names = ['right_elbow_joint', 'left_elbow_joint']
        self.get_logger().info('Motion Planning Node initialized')
        self.get_logger().info('Waiting for weed positions on /weed_position...')
    
    def call_ik_service_async(self, x, y, z):
        """Call IK service asynchronously with callback"""
        request = ComputeIK.Request()
        request.x = x
        request.y = y
        request.z = z
        
        future = self.ik_client.call_async(request)
        # Add done callback - will be called when response arrives
        future.add_done_callback(lambda f: self.ik_service_callback(f, x, y, z))
    
    def ik_service_callback(self, future, x, y, z):
        """Callback when IK service response arrives"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'IK result: theta1={response.theta1:.3f}, theta2={response.theta2:.3f}')
                # Now execute trajectory with these joint angles
                self.execute_trajectory_with_angles([(response.theta1, response.theta2)])
            else:
                self.get_logger().error(f"IK failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f'IK service call failed: {e}')
    
    def weed_position_callback(self, msg):
        """Callback when a new weed position is received"""
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        
        self.get_logger().info(f'Received weed position: ({x:.3f}, {y:.3f}, {z:.3f})')
        
        # Spawn async task to compute IK and execute trajectory
        # Don't block in the callback!
        self.call_ik_service_async(x, y, z)
        
  
    def execute_trajectory_with_angles(self, joint_positions):
        """
        Execute trajectory with pre-computed joint angles.
        
        Parameters
        ----------
        joint_positions : list of tuples
            List of (theta1, theta2) joint angles
        """
        


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
        if result.error_code == 0:
            self.get_logger().info('Trajectory execution completed: SUCCESS')
        else:
            self.get_logger().error(f'Trajectory execution failed with error code: {result.error_code}')
        
    def feedback_callback(self, feedback_msg):
        """Callback for action feedback"""
        feedback = feedback_msg.feedback
        pass


def main(args=None):
    rclpy.init(args=args)
    motion_planner = MotionPlanningNode()
    
    try:
        rclpy.spin(motion_planner)
    except KeyboardInterrupt:
        pass
    
    motion_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
