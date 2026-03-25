#!/usr/bin/env python3
"""
Kinematics service node for the volcaniarm 5-bar closed-loop linkage.

Provides IK and FK services that accept/return URDF joint angles.
Internally converts between the planar FK model convention and URDF joint space.

Includes configurable workspace limits as a safety envelope.
"""

import math
import rclpy
from rclpy.node import Node
from volcaniarm_interfaces.srv import ComputeIK, ComputeFK


class VolcaniarmKinematics(Node):
    def __init__(self):
        super().__init__('volcaniarm_kinematics')

        # Kinematic parameters
        self.declare_parameter('L1', 0.41621)
        self.declare_parameter('L2', 0.65)
        self.declare_parameter('l0', 0.215)
        self.declare_parameter('base_z', 0.0582)
        self.declare_parameter('left_fk_offset', 3.0 * math.pi / 4.0)
        self.declare_parameter('right_fk_offset', math.pi / 4.0)

        # Workspace safety limits (in the arm's YZ plane)
        # Workspace limits: Z positive = downward (arm extends in +Z)
        self.declare_parameter('workspace.y_min', -0.4)
        self.declare_parameter('workspace.y_max', 0.4)
        self.declare_parameter('workspace.z_min', 0.1)
        self.declare_parameter('workspace.z_max', 0.9)

        # Joint limits (URDF joint angles)
        self.declare_parameter('joint_limits.min', -3.14)
        self.declare_parameter('joint_limits.max', 3.14)

        self.L1 = self.get_parameter('L1').value
        self.L2 = self.get_parameter('L2').value
        self.l0 = self.get_parameter('l0').value
        self.base_z = self.get_parameter('base_z').value
        self.left_fk_offset = self.get_parameter('left_fk_offset').value
        self.right_fk_offset = self.get_parameter('right_fk_offset').value

        self.ws_y_min = self.get_parameter('workspace.y_min').value
        self.ws_y_max = self.get_parameter('workspace.y_max').value
        self.ws_z_min = self.get_parameter('workspace.z_min').value
        self.ws_z_max = self.get_parameter('workspace.z_max').value

        self.jl_min = self.get_parameter('joint_limits.min').value
        self.jl_max = self.get_parameter('joint_limits.max').value

        # Services
        self.create_service(ComputeIK, 'compute_ik', self.ik_cb)
        self.create_service(ComputeFK, 'compute_fk', self.fk_cb)

        self.get_logger().info(
            f'Kinematics services ready (L1={self.L1}, L2={self.L2}, '
            f'workspace Y[{self.ws_y_min}, {self.ws_y_max}] '
            f'Z[{self.ws_z_min}, {self.ws_z_max}])')

    # ── angle conversion ──────────────────────────────────────────

    def _urdf_to_fk(self, urdf_left, urdf_right):
        return urdf_left + self.left_fk_offset, urdf_right + self.right_fk_offset

    @staticmethod
    def _normalize(a):
        """Normalize angle to [-pi, pi]."""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _fk_to_urdf(self, fk_left, fk_right):
        return (self._normalize(fk_left - self.left_fk_offset),
                self._normalize(fk_right - self.right_fk_offset))

    # ── workspace check ───────────────────────────────────────────

    def _check_workspace(self, y, z):
        if y < self.ws_y_min or y > self.ws_y_max:
            raise ValueError(
                f'Y={y:.3f} outside workspace [{self.ws_y_min}, {self.ws_y_max}]')
        if z < self.ws_z_min or z > self.ws_z_max:
            raise ValueError(
                f'Z={z:.3f} outside workspace [{self.ws_z_min}, {self.ws_z_max}]')

    def _check_joint_limits(self, urdf_left, urdf_right):
        for name, val in [('left', urdf_left), ('right', urdf_right)]:
            if val < self.jl_min or val > self.jl_max:
                raise ValueError(
                    f'{name} joint angle {val:.3f} outside limits '
                    f'[{self.jl_min}, {self.jl_max}]')

    # ── FK ────────────────────────────────────────────────────────

    def fk_cb(self, request, response):
        try:
            # request.theta1 = right URDF, request.theta2 = left URDF
            fk_left, fk_right = self._urdf_to_fk(request.theta2, request.theta1)
            y, z = self._fk_planar(fk_right, fk_left)
            response.x = 0.0
            response.y = y
            response.z = z
            response.success = True
            response.message = 'OK'
            self.get_logger().info(
                f'FK: urdf({request.theta1:.3f}, {request.theta2:.3f}) → '
                f'pos({y:.4f}, {z:.4f})')
        except Exception as e:
            response.x = response.y = response.z = 0.0
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'FK failed: {e}')
        return response

    def _fk_planar(self, fk_right, fk_left):
        y_l = -self.l0 + self.L1 * math.cos(fk_left)
        z_l = self.base_z + self.L1 * math.sin(fk_left)
        y_r = self.l0 + self.L1 * math.cos(fk_right)
        z_r = self.base_z + self.L1 * math.sin(fk_right)

        dy = y_l - y_r
        dz = z_l - z_r
        d = math.hypot(dy, dz)

        if d > 2.0 * self.L2 or d < 1e-9:
            raise ValueError(f'Elbows too far apart (d={d:.3f} m)')

        a = d / 2.0
        h = math.sqrt(self.L2**2 - a**2)
        my = y_r + a * dy / d
        mz = z_r + a * dz / d

        return my + h * dz / d, mz - h * dy / d

    # ── IK ────────────────────────────────────────────────────────

    def ik_cb(self, request, response):
        try:
            self._check_workspace(request.y, request.z)

            fk_left, fk_right = self._ik_planar(request.y, request.z)
            urdf_left, urdf_right = self._fk_to_urdf(fk_left, fk_right)

            self._check_joint_limits(urdf_left, urdf_right)

            response.theta1 = urdf_right
            response.theta2 = urdf_left
            response.success = True
            response.message = 'OK'
            self.get_logger().info(
                f'IK: pos({request.y:.4f}, {request.z:.4f}) → '
                f'urdf(R={urdf_right:.3f}, L={urdf_left:.3f})')
        except Exception as e:
            response.theta1 = response.theta2 = 0.0
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'IK failed: {e}')
        return response

    def _ik_planar(self, y, z):
        eps = 1e-9
        z_off = z - self.base_z

        dy1 = self.l0 + y
        dy2 = self.l0 - y
        r1 = math.hypot(dy1, z_off)
        r2 = math.hypot(dy2, z_off)

        if r1 < eps or r2 < eps:
            raise ValueError('Target at shoulder — unreachable')

        min_r = abs(self.L1 - self.L2)
        max_r = self.L1 + self.L2
        if r1 < min_r - eps or r1 > max_r + eps or \
           r2 < min_r - eps or r2 > max_r + eps:
            raise ValueError(f'Unreachable (r1={r1:.3f}, r2={r2:.3f})')

        def _clamp_acos(val):
            return math.acos(max(-1.0, min(1.0, val)))

        alpha1 = _clamp_acos((self.L1**2 + r1**2 - self.L2**2) / (2 * self.L1 * r1))
        alpha2 = _clamp_acos((self.L1**2 + r2**2 - self.L2**2) / (2 * self.L1 * r2))

        beta1 = math.atan2(z_off, dy1)
        beta2 = math.atan2(z_off, dy2)

        return beta1 + alpha1, math.pi - beta2 - alpha2


def main(args=None):
    rclpy.init(args=args)
    node = VolcaniarmKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
