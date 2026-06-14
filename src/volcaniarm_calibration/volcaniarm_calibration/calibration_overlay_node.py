#!/usr/bin/env python3
"""Reprojection-overlay node for camera calibration verification.

Draws, on the live camera image:
  - DETECTED EE-tag corners (from apriltag_ros), and
  - PREDICTED EE-tag corners: the URDF tag's 4 corners (FK) projected through
    the CANDIDATE calibration (TF frame camera_link_calibrated, broadcast by the
    dashboard after a solve) and the camera intrinsics.

Overlap within ~1-2 px across the FoV means the calibration is good -- judgeable
by eye with zero transform knowledge. Per-corner pixel error + RMS are printed.
Publishes ~/calibration_overlay (sensor_msgs/Image). Before any solve it falls
back to the current URDF camera_link, so it always shows the live fit.
"""

import numpy as np

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
import message_filters
import tf2_ros

from sensor_msgs.msg import CameraInfo, Image
from apriltag_msgs.msg import AprilTagDetectionArray


def _quat_to_mat(x, y, z, w):
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n == 0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tf_to_se3(t):
    m = np.eye(4)
    q = t.transform.rotation
    m[:3, :3] = _quat_to_mat(q.x, q.y, q.z, q.w)
    tr = t.transform.translation
    m[:3, 3] = [tr.x, tr.y, tr.z]
    return m


def _se3_inv(m):
    r = m[:3, :3]
    p = m[:3, 3]
    out = np.eye(4)
    out[:3, :3] = r.T
    out[:3, 3] = -r.T @ p
    return out


class CalibrationOverlayNode(Node):
    def __init__(self):
        super().__init__('calibration_overlay')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('ee_tag_id', 20)
        self.declare_parameter('tag_size', 0.064)
        self.declare_parameter('optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('camera_link_frame', 'camera_link')
        self.declare_parameter('candidate_frame', 'camera_link_calibrated')
        self.declare_parameter('ee_urdf_frame', 'apriltag_ee_link')

        self._ee_id = int(self.get_parameter('ee_tag_id').value)
        s = float(self.get_parameter('tag_size').value) / 2.0
        # Tag corners in the marker frame (z out of the tag); ordering only
        # affects the line drawing, per-corner error uses nearest-neighbour.
        self._corners3d = np.array([
            [-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]])
        self._optical = self.get_parameter('optical_frame').value
        self._camlink = self.get_parameter('camera_link_frame').value
        self._candidate = self.get_parameter('candidate_frame').value
        self._ee_urdf = self.get_parameter('ee_urdf_frame').value

        self._bridge = CvBridge()
        self._K = None
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_info, 10)
        img_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('image_topic').value)
        det_sub = message_filters.Subscriber(
            self, AprilTagDetectionArray,
            self.get_parameter('detections_topic').value)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, det_sub], queue_size=10, slop=0.1)
        self._sync.registerCallback(self._on_pair)

        self._pub = self.create_publisher(Image, 'calibration_overlay', 1)
        self.get_logger().info('calibration_overlay ready (/calibration_overlay)')

    def _on_info(self, msg: CameraInfo):
        self._K = np.array(msg.k, dtype=float).reshape(3, 3)

    def _optical_to_ee(self):
        """T(optical_candidate -> apriltag_ee_link) under the candidate
        calibration, falling back to the current URDF camera_link."""
        try:
            t_co = self._tf.lookup_transform(
                self._camlink, self._optical, rclpy.time.Time())
            t_co = _tf_to_se3(t_co)  # camera_link -> optical (static offset)
            t_cand_ee = self._tf.lookup_transform(
                self._candidate, self._ee_urdf, rclpy.time.Time())
            # optical_candidate -> ee = inv(camlink->optical) @ (camlinkcand->ee)
            return _se3_inv(t_co) @ _tf_to_se3(t_cand_ee), 'candidate'
        except Exception:
            pass
        try:
            t = self._tf.lookup_transform(
                self._optical, self._ee_urdf, rclpy.time.Time())
            return _tf_to_se3(t), 'current'
        except Exception:
            return None, None

    def _on_pair(self, img_msg: Image, det_msg: AprilTagDetectionArray):
        if self._K is None:
            return
        det = next((d for d in det_msg.detections if int(d.id) == self._ee_id), None)
        if det is None:
            return
        T, which = self._optical_to_ee()
        if T is None:
            return

        # Predicted corners: tag corners -> optical -> pixels.
        pred = []
        for c in self._corners3d:
            p = T @ np.array([c[0], c[1], c[2], 1.0])
            if p[2] <= 1e-6:
                return
            uv = self._K @ (p[:3] / p[2])
            pred.append((float(uv[0]), float(uv[1])))
        detected = [(float(c.x), float(c.y)) for c in det.corners]

        img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        # detected = green, predicted = red
        for poly, color in ((detected, (0, 255, 0)), (pred, (0, 0, 255))):
            pts = np.array(poly, dtype=np.int32)
            cv2.polylines(img, [pts], True, color, 2)
            for u, v in poly:
                cv2.circle(img, (int(u), int(v)), 4, color, -1)
        # per-corner error by nearest detected corner
        errs = []
        det_np = np.array(detected)
        for (u, v) in pred:
            d = np.hypot(det_np[:, 0] - u, det_np[:, 1] - v)
            e = float(d.min())
            errs.append(e)
            j = int(d.argmin())
            cv2.line(img, (int(u), int(v)),
                     (int(detected[j][0]), int(detected[j][1])), (0, 255, 255), 1)
            cv2.putText(img, f'{e:.1f}', (int(u) + 5, int(v) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        rms = float(np.sqrt(np.mean(np.square(errs)))) if errs else 0.0
        cv2.putText(img, f'[{which}] corner RMS: {rms:.1f} px',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        out = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = img_msg.header
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
