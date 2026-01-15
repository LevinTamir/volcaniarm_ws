#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from volcaniarm_interfaces.srv import ComputeIK
import math

def ik_2R_YZ_test( y, z, l0=0.215, l1=0.41621, l2=0.65):

    """Standalone planar 2R IK for testing without ROS context.

    Args:
        y (float): End-effector Y coordinate in meters.
        z (float): End-effector Z coordinate in meters.
        l0 (float, optional): Half shoulder width (distance from center to each shoulder). Defaults to 0.2085.
        l1 (float, optional): Length of the first link. Defaults to 0.413.
        l2 (float, optional): Length of the second link. Defaults to 0.622.

    Returns:
        tuple[float, float]: Shoulder joint angles (left, right) in radians.

    Raises:
        ValueError: If the target point is unreachable given link geometry.
    """



    eps = 1e-9

    # Angle from left shoulder to end effector
    beta1 = math.atan2(z, (l0 + y))

    # Angle from right shoulder to end effector
    beta2 = math.atan2(z, (l0 - y))

    dy1 = l0 + y
    dy2 = l0 - y
    r1 = math.hypot(dy1, z)
    r2 = math.hypot(dy2, z)

    if r1 < eps or r2 < eps:
        raise ValueError("Unreachable coordinates")

    # Reachability checks (triangle inequality for each 2-link arm)
    min_r = abs(l1 - l2)
    max_r = l1 + l2
    if (r1 < min_r - eps) or (r1 > max_r + eps) or (r2 < min_r - eps) or (r2 > max_r + eps):
        raise ValueError("Unreachable coordinates")

    # Alpha angle pre-calculations (law of cosines)
    alpha1_calc = (l1**2 + r1**2 - l2**2) / (2 * l1 * r1)
    alpha2_calc = (l1**2 + r2**2 - l2**2) / (2 * l1 * r2)

    # Guard against floating-point roundoff pushing value slightly outside [-1, 1]
    if alpha1_calc < -1 - eps or alpha1_calc > 1 + eps or alpha2_calc < -1 - eps or alpha2_calc > 1 + eps:
        raise ValueError("Unreachable coordinates")
    alpha1_calc = max(-1.0, min(1.0, alpha1_calc))
    alpha2_calc = max(-1.0, min(1.0, alpha2_calc))

    # Angle of left shoulder - beta1 and right shoulder - beta2
    alpha1 = math.acos(alpha1_calc)
    alpha2 = math.acos(alpha2_calc)

    # Angles of left and right shoulders
    shoulder1 = beta1 + alpha1
    shoulder2 = math.pi - beta2 - alpha2
    
    return(shoulder1, shoulder2)





class InverseKinematics(Node):
    def __init__(self):
        super().__init__('volcaniarm_inverse_kinematics')
        
        # Create the service
        self.srv = self.create_service(ComputeIK, 'compute_ik', self.ik_cb)
        self.get_logger().info('Inverse Kinematics service is ready')
        
        # Robot parameters
        self.link_lengths = [0.41621, 0.65]  # Link lengths [L1, L2] - must match FK

    def ik_cb(self, request, response):
        """Service callback to compute inverse kinematics."""

        try:
            # X is fixed for now, use Y and Z for planar IK calculation
            q_left, q_right = self.ik_2R_YZ(request.y, request.z)
            response.theta1 = q_right
            response.theta2 = q_left
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
        eps = 1e-9
        l0 = 0.215  # Shoulder separation (distance from center to each shoulder)
        base_z = 0.0582  # Base height (shoulder Z offset)
        l1, l2 = self.link_lengths

        # Account for base_z offset
        z_offset = z - base_z

        beta1 = math.atan2(z_offset, (l0 + y))
        beta2 = math.atan2(z_offset, (l0 - y))

        dy1 = l0 + y
        dy2 = l0 - y
        r1 = math.hypot(dy1, z_offset)
        r2 = math.hypot(dy2, z_offset)

        if r1 < eps or r2 < eps:
            raise ValueError("Unreachable coordinates")

        min_r = abs(l1 - l2)
        max_r = l1 + l2
        if (r1 < min_r - eps) or (r1 > max_r + eps) or (r2 < min_r - eps) or (r2 > max_r + eps):
            raise ValueError("Unreachable coordinates")

        alpha1_calc = (l1**2 + r1**2 - l2**2) / (2 * l1 * r1)
        alpha2_calc = (l1**2 + r2**2 - l2**2) / (2 * l1 * r2)

        if alpha1_calc < -1 - eps or alpha1_calc > 1 + eps or alpha2_calc < -1 - eps or alpha2_calc > 1 + eps:
            raise ValueError("Unreachable coordinates")
        alpha1_calc = max(-1.0, min(1.0, alpha1_calc))
        alpha2_calc = max(-1.0, min(1.0, alpha2_calc))

        alpha1 = math.acos(alpha1_calc)
        alpha2 = math.acos(alpha2_calc)

        shoulder1 = beta1 + alpha1
        shoulder2 = math.pi - beta2 - alpha2

        return shoulder1, shoulder2


def main(args=None):
    rclpy.init(args=args)
    ik_service = InverseKinematics()
    rclpy.spin(ik_service)
    ik_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
