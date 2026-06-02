"""One-shot base-tag camera locator for tests mode.

Recovers the stand camera's X/Y/Z in the world frame from a single
detection of the fixed base marker (ID 5), assuming the camera is level
and facing along the robot's central axis (orientation = identity). No
arm motion, no orientation solve -- this is the lightweight alternative
to the EE-sweep ``solve_with_marker_orientation_prior`` flow.

Math (``T(a, b)`` = pose of frame ``b`` expressed in frame ``a``):

    p_truth = T(world, apriltag_base_link).translation
        URDF-fixed base-marker origin in world. No camera in the chain.
    p_cam   = T(camera_link, apriltag_marker_base).translation
        Detected base-marker origin in camera_link. apriltag_ros
        publishes optical -> marker; camera_link -> optical is static,
        so this lookup is INDEPENDENT of the camera's (unknown) world
        pose -- which is exactly why we can solve for that pose.

    camera_xyz_world = p_truth - R_world_camera_link @ p_cam
                     = p_truth - p_cam            (R assumed identity)

Caveats (also printed at run time):
  * Level-camera assumption: roll=pitch=yaw=0. A tilted/yawed camera
    degrades the estimate with distance; use the EE-sweep flow for full
    6-DoF.
  * apriltag_base_link is a placeholder mount value, so the absolute
    camera position inherits any base-mount bias. RViz stays
    self-consistent (detected base marker overlays the URDF one).
"""

from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.time import Time as RclpyTime

import tf2_ros

from volcaniarm_calibration.runner.ee_sweep_calibration import (
    CAMERA_LINK_FRAME,
    MODE_STAND,
    WORLD_FRAME,
    write_camera_pose_config,
)

# Camera-link orientation in world is assumed identity (level, facing +X).
IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


class CameraLocatorNode(Node):
    def __init__(self):
        super().__init__('camera_locator')

        self.declare_parameter('num_samples', 20)
        self.declare_parameter('detection_timeout_s', 10.0)
        self.declare_parameter('base_tag_frame', 'apriltag_marker_base')
        self.declare_parameter('base_urdf_frame', 'apriltag_base_link')
        self.declare_parameter('world_frame', WORLD_FRAME)
        self.declare_parameter('camera_link_frame', CAMERA_LINK_FRAME)
        self.declare_parameter('write_config', True)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    # -- TF helpers --------------------------------------------------

    def _translation(self, target: str, source: str, wait_s: float):
        """Return T(target, source).translation as a 3-vector, or None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                target, source, RclpyTime(),
                timeout=RclpyDuration(seconds=wait_s))
        except Exception as exc:
            self.get_logger().warn(f'lookup {target}<-{source} failed: {exc}')
            return None, None
        t = tf.transform.translation
        stamp = tf.header.stamp
        return np.array([t.x, t.y, t.z]), (stamp.sec, stamp.nanosec)

    def _collect_detections(self, target: str, source: str,
                            num_samples: int, timeout_s: float):
        """Collect ``num_samples`` fresh T(target, source) translations.

        Freshness is gated on header.stamp progression so we never log
        the same buffered detection twice. ``timeout_s`` is a per-sample
        wall-clock deadline (monotonic, independent of sim time).
        """
        samples = []
        last_stamp = None
        while len(samples) < num_samples:
            deadline = time.monotonic() + timeout_s
            got = False
            while time.monotonic() < deadline:
                vec, stamp = self._translation(target, source, 0.2)
                if vec is not None and stamp != last_stamp:
                    samples.append(vec)
                    last_stamp = stamp
                    got = True
                    break
                time.sleep(0.02)
            if not got:
                self.get_logger().error(
                    f'timed out after {len(samples)}/{num_samples} fresh '
                    f'detections of {source} in {target}')
                return None
            self.get_logger().info(
                f'  detection {len(samples)}/{num_samples}')
        return np.array(samples)

    # -- main solve --------------------------------------------------

    def run(self) -> bool:
        num_samples = int(self.get_parameter('num_samples').value)
        timeout_s = float(self.get_parameter('detection_timeout_s').value)
        base_tag = self.get_parameter('base_tag_frame').value
        base_urdf = self.get_parameter('base_urdf_frame').value
        world = self.get_parameter('world_frame').value
        camera_link = self.get_parameter('camera_link_frame').value
        write_config = bool(self.get_parameter('write_config').value)

        # 1. URDF truth: base-marker origin in world (static, one lookup).
        p_truth, _ = self._translation(world, base_urdf, timeout_s)
        if p_truth is None:
            self.get_logger().error(
                f'could not look up URDF truth {base_urdf} in {world}; '
                f'is robot_state_publisher up with mode:=tests?')
            return False

        # 2. Detected: base-marker origin in camera_link (fresh frames).
        self.get_logger().info(
            f'collecting {num_samples} fresh detections of {base_tag}...')
        samples = self._collect_detections(
            camera_link, base_tag, num_samples, timeout_s)
        if samples is None:
            return False

        p_cam = np.median(samples, axis=0)
        spread_std = samples.std(axis=0)
        spread_range = samples.max(axis=0) - samples.min(axis=0)

        # 3. Solve camera_link world position (R = identity).
        camera_xyz = p_truth - p_cam

        self.get_logger().info('=' * 56)
        self.get_logger().info(
            f'camera_link in {world}: '
            f'x={camera_xyz[0]:.4f} y={camera_xyz[1]:.4f} '
            f'z={camera_xyz[2]:.4f}  (rpy=0,0,0 assumed)')
        self.get_logger().info(
            f'sample spread (m): std=({spread_std[0]:.4f}, '
            f'{spread_std[1]:.4f}, {spread_std[2]:.4f}) '
            f'range=({spread_range[0]:.4f}, {spread_range[1]:.4f}, '
            f'{spread_range[2]:.4f})')
        if float(spread_range.max()) > 0.01:
            self.get_logger().warn(
                'sample spread > 1 cm: detection is noisy (motion blur, '
                'poor lighting, or marker too far). Re-run when settled.')
        self.get_logger().info(
            'NOTE: assumes a level camera facing along world +X. If the '
            'stand is tilted/yawed, use the EE-sweep "Calibrate camera" '
            'flow for full 6-DoF instead.')
        self.get_logger().info('=' * 56)

        # 4. Persist (same schema as the EE-sweep solver).
        if write_config:
            path = write_camera_pose_config(
                MODE_STAND, world, camera_xyz, IDENTITY_QUAT,
                source='base_tag_locator')
            self.get_logger().info(f'wrote {path}')
            self.get_logger().info(
                'relaunch mode:=tests to apply it in RViz')
        else:
            self.get_logger().info(
                'write_config=false: not writing camera_pose.yaml')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = CameraLocatorNode()

    # Spin in the background so the TF listener fills the buffer while
    # run() does its blocking lookups on this thread.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ok = False
    try:
        ok = node.run()
    finally:
        # Stop the executor and JOIN the spin thread before tearing down
        # the node / rclpy. Destroying entities while the spin thread is
        # still in a callback aborts with SIGABRT (std::terminate).
        executor.shutdown()
        spin_thread.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    main()
