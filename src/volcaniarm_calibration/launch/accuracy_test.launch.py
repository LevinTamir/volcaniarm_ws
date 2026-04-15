"""Launch EE accuracy and repeatability test.

Starts the RealSense camera, AprilTag detector, and the accuracy test node.
The arm must already be running (bringup launched separately).
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    calibration_share = get_package_share_directory('volcaniarm_calibration')

    # Load realsense config
    rs_config_path = os.path.join(calibration_share, 'config', 'realsense_params.yaml')
    with open(rs_config_path, 'r') as f:
        rs_args = yaml.safe_load(f)['realsense']

    apriltag_config = os.path.join(calibration_share, 'config', 'apriltag_params.yaml')
    accuracy_config = os.path.join(calibration_share, 'config', 'accuracy_test_params.yaml')

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

    # Accuracy test node
    accuracy_test = Node(
        package='volcaniarm_calibration',
        executable='accuracy_test',
        parameters=[accuracy_config],
        output='screen',
    )

    return LaunchDescription([
        realsense_camera,
        apriltag_node,
        accuracy_test,
    ])
