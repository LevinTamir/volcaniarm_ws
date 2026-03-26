import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np

class Weed3DDetector(Node):
    def __init__(self):
        super().__init__('weed_3d_detector_node')
        self.bridge = CvBridge()
        self.camera_info_received = False

        # --- Filter Parameters ---
        self.ema_alpha = 0.15  # EMA smoothing factor (0.1 = very smooth, 0.5 = responsive)
        self.outlier_threshold = 0.1  # Max allowed jump in meters per frame
        self.min_contour_area = 500  # Minimum contour area to consider valid
        self.outlier_reset_count = 10  # Reset filter after this many consecutive outliers

        # --- Filter State ---
        self.filtered_position = None  # (X, Y, Z) filtered position
        self.position_initialized = False
        self.consecutive_outliers = 0  # Counter for consecutive outlier rejections

        # Synchronized RGB + Depth subscribers
        rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.synced_callback)

        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.position_publisher = self.create_publisher(PointStamped, '/weed_position_raw', 10)
        self.marker_publisher = self.create_publisher(Marker, '/weed_marker', 10)

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info('Camera Intrinsics Received')

    def synced_callback(self, rgb_msg, depth_msg):
        if not self.camera_info_received:
            return
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        self.process(rgb_image, depth_image)

    def process(self, rgb_image, depth_image):

        # --- Segment Green (Weed) in RGB ---
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # --- Find Largest Contour ---
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().info('No weed segment found')
            return

        largest_contour = max(contours, key=cv2.contourArea)
        
        # --- Filter out small contours ---
        if cv2.contourArea(largest_contour) < self.min_contour_area:
            self.get_logger().debug('Contour too small, ignoring')
            return
            
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return
        cx_pixel = int(M['m10'] / M['m00'])
        cy_pixel = int(M['m01'] / M['m00'])

        # --- Get Median Depth over a patch around the target pixel ---
        half = 5  # 11x11 patch
        h, w = depth_image.shape[:2]
        y0 = max(0, cy_pixel - half)
        y1 = min(h, cy_pixel + half + 1)
        x0 = max(0, cx_pixel - half)
        x1 = min(w, cx_pixel + half + 1)
        depth_patch = depth_image[y0:y1, x0:x1].astype(float).flatten()
        depth_patch = depth_patch[(depth_patch > 0) & ~np.isnan(depth_patch)]
        if len(depth_patch) == 0:
            self.get_logger().warn('Invalid depth at weed target')
            return
        depth = float(np.median(depth_patch))

        # --- Convert depth to meters if in millimeters (RealSense uses mm, Gazebo uses m) ---
        if depth > 100.0:  # If depth > 100, assume it's in millimeters
            depth = depth / 1000.0

        # --- Compute 3D Coordinates ---
        X = float((cx_pixel - self.cx) * depth / self.fx)
        Y = float((cy_pixel - self.cy) * depth / self.fy)
        Z = float(depth)

        # --- Apply Outlier Rejection and EMA Filter ---
        raw_position = np.array([X, Y, Z])
        
        if not self.position_initialized:
            # First valid reading - initialize filter
            self.filtered_position = raw_position
            self.position_initialized = True
            self.consecutive_outliers = 0
        else:
            # Check for outliers (large jumps)
            distance = np.linalg.norm(raw_position - self.filtered_position)
            if distance > self.outlier_threshold:
                self.consecutive_outliers += 1
                
                # If we keep getting "outliers", it means we moved to a new target - reset filter
                if self.consecutive_outliers >= self.outlier_reset_count:
                    self.get_logger().info(f'Filter reset: new target detected at distance={distance:.3f}m')
                    self.filtered_position = raw_position
                    self.consecutive_outliers = 0
                else:
                    self.get_logger().debug(f'Outlier rejected: distance={distance:.3f}m ({self.consecutive_outliers}/{self.outlier_reset_count})')
                    return  # Reject this reading
            else:
                # Valid reading - reset outlier counter and apply EMA
                self.consecutive_outliers = 0
                self.filtered_position = (self.ema_alpha * raw_position + 
                                          (1 - self.ema_alpha) * self.filtered_position)
        
        # Use filtered position
        X, Y, Z = self.filtered_position

        self.get_logger().info(f"Weed 3D Position: X={X:.3f} m, Y={Y:.3f} m, Z={Z:.3f} m")

        # --- Publish Position as PointStamped ---
        point_msg = PointStamped()
        point_msg.header.stamp.sec = 0
        point_msg.header.stamp.nanosec = 0
        point_msg.header.frame_id = 'camera_link_optical'
        point_msg.point.x = X
        point_msg.point.y = Y
        point_msg.point.z = Z
        self.position_publisher.publish(point_msg)

        # --- Publish Marker for RViz ---
        marker = Marker()
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.header.frame_id = 'camera_link_optical'
        marker.ns = "weed_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point_msg.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)

        # --- Optional Visualization ---
        vis_image = rgb_image.copy()
        cv2.drawContours(vis_image, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(vis_image, (cx_pixel, cy_pixel), 5, (255, 0, 0), -1)
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
