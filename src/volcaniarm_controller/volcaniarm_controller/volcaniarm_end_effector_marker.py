#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class EndEffectorMarker(Node):
    def __init__(self):
        super().__init__('volcaniarm_end_effector_marker')

        # Parameters
        joints = self.declare_parameter('joints', ['left_elbow_joint', 'right_elbow_joint']).value
        link_lengths = self.declare_parameter('link_lengths', [0.50, 0.75]).value
        base_frame = self.declare_parameter('base_frame', 'delta_arm_base_link').value

        self.joint_names = joints
        self.link_lengths = link_lengths
        self.base_frame = base_frame

        # Publishers
        self.marker_pub = self.create_publisher(Marker, 'ee_marker', 10)
        self.path_pub = self.create_publisher(Marker, 'ee_path', 10)
        
        # Subscription
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        # State
        self.state = {}
        self.trail = deque(maxlen=300)

    def fk_2R_YZ(self, q_left: float, q_right: float):
        """
        Closed-loop 2D delta FK.
        Solves for the intersection of two 2-link arms meeting at end-effector.
        """
        L1, L2 = self.link_lengths
        
        # Fixed joints in base frame
        right_elbow_y = -0.215
        left_elbow_y = 0.215
        base_z = -0.0582
        
        # Compute elbow A position (left side, using left_elbow_joint angle)
        A_y = left_elbow_y + L1 * math.sin(q_left)
        A_z = base_z - L1 * math.cos(q_left)
        
        # Compute elbow B position (right side, using right_elbow_joint angle)
        B_y = right_elbow_y + L1 * math.sin(q_right)
        B_z = base_z - L1 * math.cos(q_right)
        
        # Solve for intersection of two circles:
        # Circle 1: center A, radius L2
        # Circle 2: center B, radius L2
        dy = B_y - A_y
        dz = B_z - A_z
        d = math.sqrt(dy**2 + dz**2)
        
        if d > 2 * L2 or d < 0.001:  # No solution or coincident
            # Fallback: midpoint
            ee_y = (A_y + B_y) / 2.0
            ee_z = (A_z + B_z) / 2.0
        else:
            # Intersection of two circles
            a = d / 2.0
            h = math.sqrt(L2**2 - a**2)
            
            # Midpoint between A and B
            mx = A_y + a * dy / d
            mz = A_z + a * dz / d
            
            # Perpendicular offset (take lower solution)
            ee_y = mx - h * dz / d
            ee_z = mz + h * dy / d
        
        
        ee_x = 0.15 # Offset marker along X axis

        return ee_x, ee_y, ee_z

    def joint_cb(self, msg: JointState):
        """Update state and publish marker."""
        for name, pos in zip(msg.name, msg.position):
            self.state[name] = pos

        try:
            q_l = self.state[self.joint_names[0]]
            q_r = self.state[self.joint_names[1]]
        except KeyError:
            return

        x, y, z = self.fk_2R_YZ(q_l, q_r)
        self.publish_markers(x, y, z)

    def publish_markers(self, x: float, y: float, z: float):
        """Publish EE marker and trail."""
        now = self.get_clock().now().to_msg()

        # EE sphere
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = now
        m.ns = 'ee'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 1.0
        self.marker_pub.publish(m)

        # Trail line
        self.trail.append(Point(x=x, y=y, z=z))
        p = Marker()
        p.header.frame_id = self.base_frame
        p.header.stamp = now
        p.ns = 'trail'
        p.id = 1
        p.type = Marker.LINE_STRIP
        p.action = Marker.ADD
        p.scale.x = 0.005
        p.color.r, p.color.g, p.color.b, p.color.a = 0.1, 0.6, 1.0, 0.9
        p.points = list(self.trail)
        self.path_pub.publish(p)


def main():
    rclpy.init()
    rclpy.spin(EndEffectorMarker())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
