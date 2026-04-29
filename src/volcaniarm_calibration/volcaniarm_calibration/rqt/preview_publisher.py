"""Publishes RViz preview markers for the upcoming calibration move.

Renders a green sphere at the FK-reachable target pose plus a yellow
line strip showing the joint-space interpolation between the current
and goal joints. Cleared when a move is approved/rejected.
"""

from __future__ import annotations

from typing import Iterable, Tuple

from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


PREVIEW_TOPIC = '/calibration/preview_markers'
TARGET_FRAME = 'target_ee_link'


class PreviewPublisher:

    def __init__(self, node: Node, base_frame: str = 'volcaniarm_base_link'):
        self.node = node
        self.base_frame = base_frame
        self._pub = node.create_publisher(MarkerArray, PREVIEW_TOPIC, 1)
        self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster(node)

    def publish_preview(self,
                        target_xyz: Tuple[float, float, float],
                        joint_path_xyz: Iterable[Tuple[float, float, float]],
                        label: str):
        markers = MarkerArray()
        markers.markers.append(self._sphere_marker(target_xyz))
        markers.markers.append(self._path_marker(list(joint_path_xyz)))
        markers.markers.append(self._text_marker(target_xyz, label))
        self._pub.publish(markers)
        self._publish_target_tf(target_xyz)

    def clear_preview(self):
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.base_frame
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self._pub.publish(markers)

    def _sphere_marker(self, xyz):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = 'calibration_target'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.65)
        return m

    def _path_marker(self, points):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = 'calibration_path'
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.005
        m.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.85)
        for x, y, z in points:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), float(z)
            m.points.append(p)
        return m

    def _text_marker(self, xyz, label):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = 'calibration_label'
        m.id = 2
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = xyz[0]
        m.pose.position.y = xyz[1]
        m.pose.position.z = xyz[2] + 0.06
        m.pose.orientation.w = 1.0
        m.scale.z = 0.03
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text = label
        return m

    def _publish_target_tf(self, xyz):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = TARGET_FRAME
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)
