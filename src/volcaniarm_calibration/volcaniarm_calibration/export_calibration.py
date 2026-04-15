"""Export easy_handeye2 calibration result to a YAML config file.

Reads the calibration from ~/.ros2/easy_handeye2/calibrations/ and writes
it to a YAML file that can be loaded by a static_transform_publisher in
a launch file, or used as a reference for updating the URDF.

Usage:
  ros2 run volcaniarm_calibration export_calibration
  ros2 run volcaniarm_calibration export_calibration --ros-args \
      -p calibration_name:=volcaniarm_calibration \
      -p output_path:=~/volcaniarm_calibration_data/camera_transform.yaml
"""

import os
import sys
import yaml
import math
from pathlib import Path

import rclpy
from rclpy.node import Node


def quaternion_to_rpy(qx, qy, qz, qw):
    """Convert quaternion to roll-pitch-yaw (extrinsic XYZ)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ExportCalibrationNode(Node):

    def __init__(self):
        super().__init__('export_calibration')

        self.declare_parameter('calibration_name', 'volcaniarm_calibration')
        self.declare_parameter(
            'output_path',
            '~/volcaniarm_calibration_data/camera_transform.yaml')

        calib_name = self.get_parameter('calibration_name').value
        output_path = os.path.expanduser(
            self.get_parameter('output_path').value)

        # Find calibration file
        calib_dir = Path.home() / '.ros2' / 'easy_handeye2' / 'calibrations'
        calib_file = calib_dir / f'{calib_name}.calib'

        if not calib_file.exists():
            self.get_logger().error(
                f'Calibration file not found: {calib_file}\n'
                f'Run the hand-eye calibration first.')
            return

        # Read calibration
        with open(calib_file, 'r') as f:
            calib_data = yaml.safe_load(f)

        self.get_logger().info(f'Loaded calibration from {calib_file}')

        # Extract transform
        transform = calib_data.get('transformation', {})
        tx = transform.get('x', 0.0)
        ty = transform.get('y', 0.0)
        tz = transform.get('z', 0.0)
        qx = transform.get('qx', 0.0)
        qy = transform.get('qy', 0.0)
        qz = transform.get('qz', 0.0)
        qw = transform.get('qw', 1.0)

        roll, pitch, yaw = quaternion_to_rpy(qx, qy, qz, qw)

        # Build output
        output = {
            'hand_eye_calibration': {
                'calibration_name': calib_name,
                'parent_frame': calib_data.get(
                    'robot_base_frame', 'volcaniarm_base_link'),
                'child_frame': calib_data.get(
                    'tracking_base_frame', 'camera_color_optical_frame'),
                'calibration_type': calib_data.get(
                    'calibration_type', 'eye_on_base'),
                'translation': {
                    'x': float(f'{tx:.6f}'),
                    'y': float(f'{ty:.6f}'),
                    'z': float(f'{tz:.6f}'),
                },
                'rotation_quaternion': {
                    'x': float(f'{qx:.6f}'),
                    'y': float(f'{qy:.6f}'),
                    'z': float(f'{qz:.6f}'),
                    'w': float(f'{qw:.6f}'),
                },
                'rotation_rpy_rad': {
                    'roll': float(f'{roll:.6f}'),
                    'pitch': float(f'{pitch:.6f}'),
                    'yaw': float(f'{yaw:.6f}'),
                },
                'rotation_rpy_deg': {
                    'roll': float(f'{math.degrees(roll):.4f}'),
                    'pitch': float(f'{math.degrees(pitch):.4f}'),
                    'yaw': float(f'{math.degrees(yaw):.4f}'),
                },
            },
            'static_transform_publisher_args': {
                'description': (
                    'Use these args with: ros2 run tf2_ros '
                    'static_transform_publisher --ros-args '
                    '-p x:=... -p y:=... etc.'),
                'x': float(f'{tx:.6f}'),
                'y': float(f'{ty:.6f}'),
                'z': float(f'{tz:.6f}'),
                'qx': float(f'{qx:.6f}'),
                'qy': float(f'{qy:.6f}'),
                'qz': float(f'{qz:.6f}'),
                'qw': float(f'{qw:.6f}'),
            },
            'urdf_xacro_origin': {
                'description': (
                    'Paste this into your URDF/xacro joint origin: '
                    '<origin xyz="x y z" rpy="r p y" />'),
                'xyz': f'{tx:.6f} {ty:.6f} {tz:.6f}',
                'rpy': f'{roll:.6f} {pitch:.6f} {yaw:.6f}',
            },
        }

        # Save
        output_dir = os.path.dirname(output_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        with open(output_path, 'w') as f:
            yaml.dump(output, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f'Calibration exported to: {output_path}')
        self.get_logger().info(
            f'Transform ({output["hand_eye_calibration"]["parent_frame"]} '
            f'-> {output["hand_eye_calibration"]["child_frame"]}):')
        self.get_logger().info(
            f'  xyz: {tx:.6f} {ty:.6f} {tz:.6f}')
        self.get_logger().info(
            f'  rpy: {math.degrees(roll):.4f} {math.degrees(pitch):.4f} '
            f'{math.degrees(yaw):.4f} deg')
        self.get_logger().info(
            f'  quat: {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}')


def main(args=None):
    rclpy.init(args=args)
    node = ExportCalibrationNode()
    # This is a one-shot node, no need to spin
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
