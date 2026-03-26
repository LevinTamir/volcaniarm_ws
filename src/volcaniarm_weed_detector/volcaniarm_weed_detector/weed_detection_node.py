import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import message_filters
import cv2
import numpy as np

class Weed3DDetector(Node):
    def __init__(self):
        super().__init__('weed_detection_node')
        self.bridge = CvBridge()

        # --- Filter Parameters ---
        self.ema_alpha = 0.15  # EMA smoothing factor (0.1 = very smooth, 0.5 = responsive)
        self.outlier_threshold = 0.1  # Max allowed jump in meters per frame
        self.declare_parameter('min_contour_area', 100)
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.outlier_reset_count = 10  # Reset filter after this many consecutive outliers

        # --- Filter State ---
        self.filtered_position = None  # (X, Y, Z) filtered position
        self.position_initialized = False
        self.consecutive_outliers = 0  # Counter for consecutive outlier rejections

        # --- Target frame (publish in this frame) ---
        self.declare_parameter('target_frame', 'volcaniarm_base_link')
        self.target_frame = self.get_parameter('target_frame').value

        # --- Height offset (applied after TF, in target frame Z) ---
        self.declare_parameter('offset_z', -0.05)
        self.offset_z = self.get_parameter('offset_z').value

        # --- TF2 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Synchronized RGB + Point Cloud subscribers
        rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        cloud_sub = message_filters.Subscriber(self, PointCloud2, '/camera/depth/color/points')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, cloud_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synced_callback)

        # Publishers
        self.position_publisher = self.create_publisher(PointStamped, '/weed_position_raw', 10)
        self.marker_publisher = self.create_publisher(Marker, '/weed_marker', 10)

    def synced_callback(self, rgb_msg, cloud_msg):
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        self.latest_cloud = cloud_msg
        self.cloud_frame_id = cloud_msg.header.frame_id
        self.process(rgb_image, cloud_msg)

    def process(self, rgb_image, cloud_msg):

        # --- Segment Green (Weed) in RGB ---
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 60, 60])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # --- Morphological cleanup: close gaps, then erode edge noise ---
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.erode(mask, kernel, iterations=1)

        # --- Find largest contour for height, mean of all pixels for lateral ---
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().info('No weed segment found')
            return

        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) < self.min_contour_area:
            self.get_logger().debug('Contour too small, ignoring')
            return

        # Use centroid of the largest green contour for both X and Y
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return
        cx_pixel = int(M['m10'] / M['m00'])
        cy_pixel = int(M['m01'] / M['m00'])

        # --- Look up 3D point from the point cloud at the detected pixel ---
        # The point cloud is organized (same width/height as the image).
        # Read a small patch around the centroid and take the median valid point.
        half = 5  # 11x11 patch
        img_h, img_w = rgb_image.shape[:2]

        # Point cloud may have different dimensions than the RGB image
        cloud_w = cloud_msg.width
        cloud_h = cloud_msg.height

        # Scale pixel coordinates if image and cloud resolutions differ
        scale_x = cloud_w / img_w
        scale_y = cloud_h / img_h
        cloud_cx = int(cx_pixel * scale_x)
        cloud_cy = int(cy_pixel * scale_y)

        # Clamp patch bounds
        y0 = max(0, cloud_cy - half)
        y1 = min(cloud_h, cloud_cy + half + 1)
        x0 = max(0, cloud_cx - half)
        x1 = min(cloud_w, cloud_cx + half + 1)

        # Build flat indices into the organized cloud (row-major: index = v * width + u)
        indices = [v * cloud_w + u for v in range(y0, y1) for u in range(x0, x1)]

        # Read the patch of points
        patch = point_cloud2.read_points(
            cloud_msg, field_names=('x', 'y', 'z'),
            skip_nans=True, uvs=indices)

        if len(patch) == 0:
            self.get_logger().warn('No valid 3D points at weed location')
            return

        points_x = patch['x']
        points_y = patch['y']
        points_z = patch['z']

        # Filter out zero-depth points
        valid = points_z > 0
        points_x = points_x[valid]
        points_y = points_y[valid]
        points_z = points_z[valid]

        if len(points_x) == 0:
            self.get_logger().warn('No valid 3D points at weed location')
            return

        # Median of the patch for robustness
        X = float(np.median(points_x))
        Y = float(np.median(points_y))
        Z = float(np.median(points_z))

        # --- Apply Outlier Rejection and EMA Filter ---
        raw_position = np.array([X, Y, Z])

        if not self.position_initialized:
            self.filtered_position = raw_position
            self.position_initialized = True
            self.consecutive_outliers = 0
        else:
            distance = np.linalg.norm(raw_position - self.filtered_position)
            if distance > self.outlier_threshold:
                self.consecutive_outliers += 1
                if self.consecutive_outliers >= self.outlier_reset_count:
                    self.get_logger().info(f'Filter reset: new target detected at distance={distance:.3f}m')
                    self.filtered_position = raw_position
                    self.consecutive_outliers = 0
                else:
                    self.get_logger().debug(f'Outlier rejected: distance={distance:.3f}m ({self.consecutive_outliers}/{self.outlier_reset_count})')
                    return
            else:
                self.consecutive_outliers = 0
                self.filtered_position = (self.ema_alpha * raw_position +
                                          (1 - self.ema_alpha) * self.filtered_position)

        X, Y, Z = self.filtered_position

        # Build point in the cloud's native frame
        cloud_frame = cloud_msg.header.frame_id or 'camera_link'
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = cloud_frame
        point_msg.point.x = X
        point_msg.point.y = Y
        point_msg.point.z = Z

        # Transform to target frame (e.g. volcaniarm_base_link)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, cloud_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            point_msg = do_transform_point(point_msg, transform)
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e} — publishing in {cloud_frame}')

        # Apply height offset in target frame
        point_msg.point.z += self.offset_z

        self.get_logger().info(
            f"Weed 3D Position ({point_msg.header.frame_id}): "
            f"X={point_msg.point.x:.3f} m, Y={point_msg.point.y:.3f} m, Z={point_msg.point.z:.3f} m")
        self.position_publisher.publish(point_msg)

        # --- Publish Marker for RViz (in target frame) ---
        marker = Marker()
        marker.header.stamp = point_msg.header.stamp
        marker.header.frame_id = point_msg.header.frame_id
        marker.ns = "weed_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point_msg.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)

        # --- Optional Visualization ---
        vis_image = rgb_image.copy()
        if largest_contour is not None:
            cv2.drawContours(vis_image, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(vis_image, (cx_pixel, cy_pixel), 7, (0, 255, 255), -1)
        cv2.imshow('Weed Detection', vis_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Weed3DDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
