"""Launch AprilTag detection with RealSense camera for basic testing.

Starts the RealSense color stream, the apriltag_ros detector node,
a pose printer for live feedback, and optionally RViz.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_share = get_package_share_directory('volcaniarm_calibration')
    apriltag_share = get_package_share_directory('apriltag_ros')

    # Load realsense config
    rs_config_path = os.path.join(calibration_share, 'config', 'realsense_params.yaml')
    with open(rs_config_path, 'r') as f:
        rs_args = yaml.safe_load(f)['realsense']

    apriltag_config = os.path.join(calibration_share, 'config', 'apriltag_params.yaml')
    rviz_config = os.path.join(apriltag_share, 'rviz', 'apriltag.rviz')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz')

    # RealSense camera
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py')
        ]),
        launch_arguments=rs_args.items(),
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

    # Pose printer for live terminal feedback
    pose_printer = Node(
        package='apriltag_ros',
        executable='pose_printer',
        name='pose_printer',
        parameters=[{
            'camera_frame': 'camera_color_optical_frame',
            'tag_frame': 'apriltag_marker',
            'rate_hz': 5.0,
        }],
        output='screen',
    )

    # RViz (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_rviz_arg,
        realsense_camera,
        apriltag_node,
        pose_printer,
        rviz,
    ])
