#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from volcaniarm_interfaces.srv import ComputeIK
import math

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('volcaniarm_inverse_kinematics')
        
        # Create the service
        self.srv = self.create_service(ComputeIK, 'compute_ik', self.ik_cb)
        self.get_logger().info('Inverse Kinematics service is ready')
        
        # Robot parameters
        self.link_lengths = [0.50, 0.75]  # Link lengths [L1, L2]

    def ik_cb(self, request, response):
        """Service callback to compute inverse kinematics."""

        try:
            # X is fixed for now, use Y and Z for planar IK calculation
            q_left, q_right = self.ik_2R_YZ(request.y, request.z)
            response.theta1 = q_left
            response.theta2 = q_right
            response.success = True
            response.message = f"IK computed successfully for position ({request.x}, {request.y}, {request.z})"
            self.get_logger().info(f"IK: ({request.x}, {request.y}, {request.z}) -> ({q_left:.3f}, {q_right:.3f})")
        except Exception as e:
            response.theta1 = 0.0
            response.theta2 = 0.0
            response.success = False
            response.message = f"IK computation failed: {str(e)}"
            self.get_logger().error(response.message)
        
        return response

    def ik_2R_YZ(self, y: float, z: float):
        """
        Inverse kinematics for a planar 2-D delta arm in YZ plane.

        Parameters
        ----------
        y, z : float
            End-effector position

        Returns
        -------
        q_left, q_right : float
            Base joint angles in radians
        """
        L1, L2 = self.link_lengths
        
        # Fixed joints in base frame
        right_elbow_y = -0.215
        left_elbow_y = 0.215
        base_z = -0.0582
        
        def theta_calc(y, z, elbow_y, base_z, side):
            L1 = self.link_lengths[0]
            L2 = self.link_lengths[1]
            
            if (side == 'right'):
                R = math.sqrt((y - right_elbow_y)**2 + (z - base_z)**2)
                phi1 = math.atan2(-(z-base_z), (y - right_elbow_y))


            else:
                R = math.sqrt((y - right_elbow_y)**2 + (z - base_z)**2)
                phi1 = math.atan2(-(z-base_z), left_elbow_y - y)


            phi2  = math.acos(((R**2)+(L1**2)-(L2**2))/(2*R*L1))

            return phi1 + phi2

        q_left = theta_calc(y, z, left_elbow_y, base_z, "left")
        q_right = theta_calc(y, z, right_elbow_y, base_z, "right")

        return q_left, q_right


def main(args=None):
    rclpy.init(args=args)
    ik_service = InverseKinematics()
    rclpy.spin(ik_service)
    ik_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
