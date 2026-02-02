#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from volcaniarm_interfaces.srv import ComputeFK


class EndEffectorMarker(Node):
    def __init__(self):
        super().__init__('volcaniarm_end_effector_marker')

        # Parameters
        joints = self.declare_parameter('joints', ['left_elbow_joint', 'right_elbow_joint']).value
        base_frame = self.declare_parameter('base_frame', 'delta_arm_base_link').value

        self.joint_names = joints
        self.base_frame = base_frame
        
        # FK service client
        self.fk_client = self.create_client(ComputeFK, 'compute_fk')
        self.use_fk_service = False  # Start with local FK, switch to service when available
        
        # Check for FK service in background
        self.create_timer(1.0, self.check_fk_service)

        # Publishers
        self.marker_pub = self.create_publisher(Marker, 'ee_marker', 10)
        self.path_pub = self.create_publisher(Marker, 'ee_path', 10)
        
        # Subscription
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        # State
        self.state = {}
        self.trail = deque(maxlen=300)
    
    def check_fk_service(self):
        """Check if FK service is available and switch to using it."""
        if not self.use_fk_service and self.fk_client.service_is_ready():
            self.use_fk_service = True

    def fk_2R_YZ(self, q_left: float, q_right: float):
        """
        Planar 5-bar linkage FK in Y-Z plane.
        Two actuated upper links from base, two passive lower links meeting at EE.
        Matches IK equations: motors at y=±0.215, z_offset=0.0582
        """
        L1 = 0.41621  # Upper link length (elbow link)
        L2 = 0.65     # Lower link length (forearm link)
        l0 = 0.215    # Motor separation
        z_offset = 0.0582  # Z coordinate offset
        
        # Passive joint positions (elbow tips) in Y-Z plane
        left_elbow_y = -l0 + L1 * math.cos(q_left)
        left_elbow_z = L1 * math.sin(q_left) + z_offset
        
        right_elbow_y = l0 + L1 * math.cos(q_right)
        right_elbow_z = L1 * math.sin(q_right) + z_offset
        
        # Solve for end effector: intersection of two circles in Y-Z plane
        dy = left_elbow_y - right_elbow_y
        dz = left_elbow_z - right_elbow_z
        d = math.sqrt(dy**2 + dz**2)
        
        eps = 1e-9
        if d > 2 * L2 + eps or d < eps:
            # No solution - return midpoint
            ee_y = (right_elbow_y + left_elbow_y) / 2.0
            ee_z = (right_elbow_z + left_elbow_z) / 2.0
        else:
            # Find intersection point in Y-Z plane (law of cosines)
            a = d / 2.0
            h = math.sqrt(max(0, L2**2 - a**2))
            
            # Midpoint between two elbow tips
            my = right_elbow_y + a * dy / d
            mz = right_elbow_z + a * dz / d
            
            # Perpendicular offset (choose the other solution)
            ee_y = my + h * dz / d
            ee_z = mz - h * dy / d
        
        # X position is constant (planar mechanism)
        ee_x = 0.1825 + 0.042  # Base X + small offset

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

        if self.use_fk_service:
            # Use FK service (theta1=right, theta2=left)
            request = ComputeFK.Request()
            request.theta1 = q_r
            request.theta2 = q_l
            future = self.fk_client.call_async(request)
            future.add_done_callback(lambda f: self.handle_fk_response(f))
        else:
            # Use local FK computation
            x, y, z = self.fk_2R_YZ(q_l, q_r)
            self.publish_markers(x, y, z)
    
    def handle_fk_response(self, future):
        """Handle FK service response."""
        try:
            response = future.result()
            if response.success:
                # Add X offset for visualization
                x = 0.1825 + 0.042
                self.publish_markers(x, response.y, response.z)
        except Exception as e:
            self.get_logger().error(f'FK service call failed: {e}')

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
