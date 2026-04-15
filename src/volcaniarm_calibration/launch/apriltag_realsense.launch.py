"""Launch AprilTag detector with RealSense camera for testing.

Starts the RealSense D435i (color only), apriltag_ros detector, and RViz.
Use this to verify the tag is detected before running calibration.

Usage:
  ros2 launch volcaniarm_calibration apriltag_realsense.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibration_share = get_package_share_directory('volcaniarm_calibration')

    apriltag_config = os.path.join(calibration_share, 'config', 'apriltag_params.yaml')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Open RViz on launch')

    # RealSense D435i camera (color only)
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',
            'align_depth.enable': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'rgb_camera.color_profile': '1920x1080x30',
            'pointcloud.enable': 'false',
            'initial_reset': 'false',
        }.items(),
    )

    # AprilTag detector
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[apriltag_config],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        output='screen',
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        realsense,
        apriltag_node,
        rviz,
    ])
