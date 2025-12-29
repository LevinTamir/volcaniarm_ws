#!/usr/bin/env python3
import math, ast
from collections import deque
from typing import Dict, List, Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def _as_str_list(val: Any, default: List[str]) -> List[str]:
    if isinstance(val, (list, tuple)):
        return [str(x) for x in val]
    if isinstance(val, str):
        try:
            parsed = ast.literal_eval(val)
            if isinstance(parsed, (list, tuple)):
                return [str(x) for x in parsed]
        except Exception:
            pass
    return default

def _as_float_list(val: Any, default: List[float]) -> List[float]:
    if isinstance(val, (list, tuple)):
        return [float(x) for x in val]
    if isinstance(val, str):
        try:
            parsed = ast.literal_eval(val)
            if isinstance(parsed, (list, tuple)):
                return [float(x) for x in parsed]
        except Exception:
            pass
    return default

class EndEffectorMarker(Node):
    def __init__(self):
        super().__init__('volcaniarm_end_effector_marker')

        # Parameters
        self.declare_parameter('joints', ['left_elbow_joint','right_elbow_joint'])
        self.declare_parameter('link_lengths', [0.20, 0.15])   # [L1, L2] (meters)
        self.declare_parameter('base_frame', 'delta_arm_base_link')
        self.declare_parameter('marker_ns', 'volcaniarm_ee')
        self.declare_parameter('marker_scale', 0.03)           # sphere diameter (m)
        self.declare_parameter('trail_length', 300)
        self.declare_parameter('zero_along_z', True)           # if False, zero points along +Y

        # Robust param reads
        self.joint_names: List[str] = _as_str_list(self.get_parameter('joints').value,
                                                   ['left_elbow_joint','right_elbow_joint'])
        self.link_lengths: List[float] = _as_float_list(self.get_parameter('link_lengths').value, [0.20, 0.15])
        if len(self.link_lengths) != 2:
            self.get_logger().warn('Expected 2 link lengths; padding/truncating to 2.')
            self.link_lengths = (self.link_lengths + [0.0, 0.0])[:2]

        self.base_frame  = str(self.get_parameter('base_frame').value)
        self.marker_ns   = str(self.get_parameter('marker_ns').value)
        self.marker_scale= float(self.get_parameter('marker_scale').value)
        self.trail_len   = int(self.get_parameter('trail_length').value)
        self.zero_along_z = bool(self.get_parameter('zero_along_z').value)

        # State & pubs/subs
        self.state: Dict[str, float] = {}
        self.trail = deque(maxlen=max(10, self.trail_len))
        self.marker_pub = self.create_publisher(Marker, 'ee_marker', 10)
        self.trail_pub  = self.create_publisher(Marker, 'ee_path', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 20)

        orient = 'Z' if self.zero_along_z else 'Y'
        self.get_logger().info(
            f'EE marker up. 2R planar FK, joints={self.joint_names}, L={self.link_lengths}, zero_along={orient}, frame="{self.base_frame}"'
        )

    # Planar 2R FK in the YZ plane:
    # joint order: [q_left (rad), q_right (rad)]
    # Y/Z come from 2R FK, X is always 0
    def fk_2R_YZ(self, q_left: float, q_right: float):
        L1, L2 = self.link_lengths
        th1 = q_right
        th2 = q_right + q_left

        # Decide whether q=0 points along +Z (default) or +Y
        if self.zero_along_z:
            # q=0 along +Z
            y = L1*math.sin(th1) + L2*math.sin(th2)
            z = L1*math.cos(th1) + L2*math.cos(th2)
        else:
            # q=0 along +Y
            y = L1*math.cos(th1) + L2*math.cos(th2)
            z = L1*math.sin(th1) + L2*math.sin(th2)

        x = 0.0
        return x, y, z

    def joint_cb(self, msg: JointState):
        self.get_logger().debug(f'Joint callback triggered. Received {len(msg.name)} joints: {msg.name}')
        for name, pos in zip(msg.name, msg.position):
            self.state[name] = pos

        try:
            q_l = float(self.state[self.joint_names[0]])  # radians (left_elbow_joint)
            q_r = float(self.state[self.joint_names[1]])  # radians (right_elbow_joint)
        except KeyError as e:
            self.get_logger().debug(f'Missing joint: {e}. Available: {list(self.state.keys())}, Need: {self.joint_names}')
            return  # wait until all are available

        x, y, z = self.fk_2R_YZ(q_l, q_r)
        self.publish_markers(x, y, z)

    def publish_markers(self, x: float, y: float, z: float):
        now = self.get_clock().now().to_msg()

        # End-effector sphere
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = now
        m.ns = self.marker_ns
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = self.marker_scale
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.95
        self.marker_pub.publish(m)

        # Path (LINE_STRIP)
        self.trail.append(Point(x=x, y=y, z=z))
        p = Marker()
        p.header.frame_id = self.base_frame
        p.header.stamp = now
        p.ns = self.marker_ns
        p.id = 1
        p.type = Marker.LINE_STRIP
        p.action = Marker.ADD
        p.scale.x = max(self.marker_scale * 0.3, 0.002)
        p.color.r, p.color.g, p.color.b, p.color.a = 0.1, 0.6, 1.0, 0.9
        p.points = list(self.trail)
        self.trail_pub.publish(p)

def main():
    rclpy.init()
    rclpy.spin(EndEffectorMarker())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
