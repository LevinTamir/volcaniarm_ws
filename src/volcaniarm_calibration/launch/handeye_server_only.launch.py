"""Spawn easy_handeye2's handeye_server with no rqt calibrator UI.

Used by the dashboard's automated camera calibration. We can't pass
the server's typed parameters via `ros2 run --ros-args -p ...` on
Jazzy (rclpy rejects the overrides as ``PARAMETER_NOT_SET``), so we
go through ros2 launch which materialises the parameter dict as a
proper params YAML for the node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'name', default_value='volcaniarm_calibration'),
        DeclareLaunchArgument(
            'calibration_type', default_value='eye_on_base'),
        DeclareLaunchArgument(
            'tracking_base_frame',
            default_value='camera_color_optical_frame'),
        DeclareLaunchArgument(
            'tracking_marker_frame', default_value='apriltag_marker_ee'),
        DeclareLaunchArgument(
            'robot_base_frame', default_value='volcaniarm_base_link'),
        DeclareLaunchArgument(
            'robot_effector_frame', default_value='right_arm_tip_link'),
    ]

    handeye_server = Node(
        package='easy_handeye2',
        executable='handeye_server',
        name='handeye_server',
        parameters=[{
            'name': LaunchConfiguration('name'),
            'calibration_type': LaunchConfiguration('calibration_type'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
            'freehand_robot_movement': True,
        }],
        output='screen',
    )

    return LaunchDescription(args + [handeye_server])
