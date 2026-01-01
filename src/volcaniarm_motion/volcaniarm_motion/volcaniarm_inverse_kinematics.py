#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from volcaniarm_interfaces.srv import ComputeIK
import math

class InverseKinematicsService(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_service')
        
        # Create the service
        self.srv = self.create_service(ComputeIK, 'compute_ik', self.compute_ik_callback)
        self.get_logger().info('Inverse Kinematics service is ready')
        
        # Robot parameters
        self.l1 = 0.25  # Upper arm length
        self.l2 = 0.25  # Forearm length
        self.d = 0.25   # Distance between base joints
        self.x_fixed = 0.0  # Fixed X coordinate (not used in IK for now)

    def compute_ik_callback(self, request, response):
        """Service callback to compute inverse kinematics"""

        try:
            # X is fixed for now, use Y and Z for planar IK calculation
            theta1, theta2 = self.inverse_kinematics(request.y, request.z)
            response.theta1 = theta1
            response.theta2 = theta2
            response.success = True
            response.message = f"IK computed successfully for position ({request.x}, {request.y}, {request.z})"
            self.get_logger().info(f"IK: ({request.x}, {request.y}, {request.z}) -> ({theta1:.3f}, {theta2:.3f})")
        except Exception as e:
            response.theta1 = 0.0
            response.theta2 = 0.0
            response.success = False
            response.message = f"IK computation failed: {str(e)}"
            self.get_logger().error(response.message)
        
        return response

    def inverse_kinematics(self, x, y):
        """
        Inverse kinematics for a planar 2-D delta arm.

        Parameters
        ----------
        x, y : float
            End-effector position

        Returns
        -------
        theta1, theta2 : float
            Base joint angles in radians
        """
        def theta_calc(x, y, l1, l2, d, side):
            if side == "left":
                k_1 = x + d / 2
            else:
                k_1 = x - d / 2
            k_2 = y
            k_3 = (k_1**2 + k_2**2 + l1**2 - l2**2) / (2 * l1)
            if side == "left":
                theta = math.atan2(k_2, k_1) + math.acos(k_3 / math.sqrt(k_1**2 + k_2**2))
            else:
                theta = math.atan2(k_2, k_1) - math.acos(k_3 / math.sqrt(k_1**2 + k_2**2))
            return theta

        theta1 = theta_calc(x, y, self.l1, self.l2, self.d, "left")
        theta2 = theta_calc(x, y, self.l1, self.l2, self.d, "right")

        return theta1, theta2


def main(args=None):
    rclpy.init(args=args)
    ik_service = InverseKinematicsService()
    rclpy.spin(ik_service)
    ik_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
