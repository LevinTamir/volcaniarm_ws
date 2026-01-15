#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class SetInitialPosition(Node):
    def __init__(self):
        super().__init__('set_initial_position')
        
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/volcaniarm_controller/follow_joint_trajectory'
        )
        self.goal_done = False
        
        self.get_logger().info('Waiting for action server...')
        self.server_ready = self.action_client.wait_for_server(timeout_sec=10.0)
        
        if self.server_ready:
            self.get_logger().info('Action server ready, sending initial position')
            self.send_initial_position()
        else:
            self.get_logger().error('Action server not available')
            self.goal_done = True
    
    def send_initial_position(self):
        """Send initial joint positions (1.57, 1.57)"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['right_elbow_joint', 'left_elbow_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [1.57, 1.57]
        point.velocities = [0.0, 0.0]
        point.accelerations = [0.0, 0.0]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        goal.trajectory.points.append(point)
        
        self.get_logger().info('Sending initial position: [1.57, 1.57]')
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Initial position goal accepted')
                # Shutdown after goal is accepted
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self.result_callback)
            else:
                self.get_logger().error('Initial position goal rejected')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def result_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info('Initial position reached')
        except Exception as e:
            self.get_logger().error(f'Goal failed: {e}')
        finally:
            self.goal_done = True


def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPosition()
    
    # Spin with timeout to allow callbacks to process
    start_time = time.time()
    timeout = 15.0  # Maximum 15 seconds to complete
    
    try:
        while not node.goal_done and (time.time() - start_time) < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
