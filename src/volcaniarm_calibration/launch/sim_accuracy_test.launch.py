"""Launch AprilTag detector and accuracy test node for simulation.

This launch file only starts the calibration-specific nodes.
It expects the simulation (Gazebo, controllers, etc.) to already be running,
typically launched via volcaniarm_bringup/calibration_bringup.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    calibration_share = get_package_share_directory('volcaniarm_calibration')

    apriltag_config = os.path.join(calibration_share, 'config', 'apriltag_params.yaml')
    accuracy_config = os.path.join(calibration_share, 'config', 'accuracy_test_params.yaml')

    # AprilTag detector -- sim camera publishes to /camera/color/image_raw
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[apriltag_config, {'use_sim_time': True}],
        remappings=[
            ('image_rect', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info'),
        ],
        output='screen',
    )

    # Accuracy test node
    # Override camera_frame for sim (Gazebo uses camera_link, not camera_color_optical_frame)
    accuracy_test = Node(
        package='volcaniarm_calibration',
        executable='accuracy_test',
        parameters=[accuracy_config, {
            'use_sim_time': True,
            'camera_frame': 'camera_link',
        }],
        output='screen',
    )

    return LaunchDescription([
        apriltag_node,
        accuracy_test,
    ])
