import rclpy
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
import tf2_ros


class ArucoTfPublisher(Node):
    """Bridges ArUco marker detections to TF transforms for hand-eye calibration.

    Subscribes to aruco_opencv detections and publishes the marker pose
    as a TF transform from the camera frame to the marker frame.
    This is the glue between aruco_opencv and easy_handeye2.
    """

    def __init__(self):
        super().__init__('aruco_tf_publisher')

        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_frame', 'aruco_marker')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.sub = self.create_subscription(
            ArucoDetection, '/aruco_detections', self.aruco_callback, 10)

        self.marker_pub = self.create_publisher(
            Marker, '/calibration_marker', 10)

        self.get_logger().info(
            f'Waiting for ArUco marker ID {self.marker_id} detections...')

    def aruco_callback(self, msg: ArucoDetection):
        for marker_pose in msg.markers:
            if marker_pose.marker_id == self.marker_id:
                break
        else:
            return

        pose = marker_pose.pose

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.marker_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)

        self._publish_marker_visualization(msg.header.stamp, pose)

    def _publish_marker_visualization(self, stamp, pose):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.camera_frame
        marker.ns = 'calibration_aruco'
        marker.id = self.marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.005
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
