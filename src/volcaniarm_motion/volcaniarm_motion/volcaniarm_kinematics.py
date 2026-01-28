#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from volcaniarm_interfaces.srv import ComputeIK, ComputeFK
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





class VolcaniarmKinematics(Node):
    def __init__(self):
        super().__init__('volcaniarm_kinematics')
        
        # Create the services
        self.ik_srv = self.create_service(ComputeIK, 'compute_ik', self.ik_cb)
        self.fk_srv = self.create_service(ComputeFK, 'compute_fk', self.fk_cb)
        self.get_logger().info('Kinematics services ready: IK and FK')
        
        # Robot parameters
        self.link_lengths = [0.41621, 0.65]  # Link lengths [L1, L2]
        self.l0 = 0.215  # Shoulder separation
        self.base_z = 0.0582  # Base height

    def ik_cb(self, request, response):
        """Service callback to compute inverse kinematics."""

        try:
            # X is fixed for now, use Y and Z for planar IK calculation
            q_left, q_right = self.ik_2R_YZ(request.y, request.z)
            response.theta1 = q_right
            response.theta2 = q_left
            response.success = True
            response.message = f"IK computed successfully for position ({request.x}, {request.y}, {request.z})"
            self.get_logger().info(f"IK: ({request.x}, {request.y}, {request.z}) -> theta1={response.theta1:.3f}, theta2={response.theta2:.3f}")
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
        l1, l2 = self.link_lengths

        # Account for base_z offset
        z_offset = z - self.base_z

        beta1 = math.atan2(z_offset, (self.l0 + y))
        beta2 = math.atan2(z_offset, (self.l0 - y))

        dy1 = self.l0 + y
        dy2 = self.l0 - y
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

    def fk_cb(self, request, response):
        """Service callback to compute forward kinematics."""
        try:
            # Compute FK for the given joint angles
            y, z = self.fk_2R_YZ(request.theta1, request.theta2)
            response.x = 0.0  # X is fixed for planar arm
            response.y = y
            response.z = z
            response.success = True
            response.message = f"FK computed successfully for angles ({request.theta1:.3f}, {request.theta2:.3f})"
            self.get_logger().info(f"FK: theta1={request.theta1:.3f}, theta2={request.theta2:.3f} -> ({response.x}, {y:.3f}, {z:.3f})")
        except Exception as e:
            response.x = 0.0
            response.y = 0.0
            response.z = 0.0
            response.success = False
            response.message = f"FK computation failed: {str(e)}"
            self.get_logger().error(response.message)
        
        return response

    def fk_2R_YZ(self, theta1: float, theta2: float):
        """Forward kinematics for planar 2-D delta arm in YZ plane.
        
        Parameters
        ----------
        theta1, theta2 : float
            Right and left shoulder joint angles in radians
            
        Returns
        -------
        y, z : float
            End-effector position in meters
        """
        l1, l2 = self.link_lengths
        
        # Left shoulder at (y=-l0, z=base_z)
        # theta2 is left shoulder angle
        elbow_left_y = -self.l0 + l1 * math.cos(theta2)
        elbow_left_z = self.base_z + l1 * math.sin(theta2)
        
        # Right shoulder at (y=l0, z=base_z)
        # theta1 is right shoulder angle
        elbow_right_y = self.l0 + l1 * math.cos(theta1)
        elbow_right_z = self.base_z + l1 * math.sin(theta1)
        
        # Debug
        self.get_logger().debug(f'Elbows: left=({elbow_left_y:.3f}, {elbow_left_z:.3f}), right=({elbow_right_y:.3f}, {elbow_right_z:.3f})')
        
        # The end effector is where both l2 links meet
        # We need to find the intersection point of two circles:
        # Circle 1: center at right elbow, radius l2
        # Circle 2: center at left elbow, radius l2
        
        dy = elbow_left_y - elbow_right_y
        dz = elbow_left_z - elbow_right_z
        d = math.hypot(dy, dz)
        
        # Check if intersection exists
        if d > 2 * l2 or d < 1e-9:
            raise ValueError(f"Invalid configuration: elbows are too far apart (d={d:.3f}m)")
        
        # Find intersection point (law of cosines)
        a = d / 2.0
        h = math.sqrt(l2**2 - a**2)
        
        # Point along the line from right elbow to left elbow
        my = elbow_right_y + a * dy / d
        mz = elbow_right_z + a * dz / d
        
        # Perpendicular offset (choose forward solution)
        y = my + h * dz / d
        z = mz - h * dy / d
        
        return y, z


def main(args=None):
    rclpy.init(args=args)
    kinematics_node = VolcaniarmKinematics()
    rclpy.spin(kinematics_node)
    kinematics_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
