"""Simulation bringup for calibration testing.

Launches the sim stack (Gazebo, controllers, RViz, motion planning) with
the AprilTag on the EE. The camera is placed at a fixed position in front
of the robot (instead of on the arm mount) for calibration.

Calibration test nodes (e.g. accuracy test) should be launched separately:
  ros2 launch volcaniarm_calibration sim_accuracy_test.launch.py

Usage:
  ros2 launch volcaniarm_bringup sim_calibration_bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('volcaniarm_bringup')

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='calibration',
        description='Gazebo world name')

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_share, 'launch', 'sim_bringup.launch.py')
        ]),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'calibration': 'true',
        }.items(),
    )

    return LaunchDescription([
        world_name_arg,
        sim_bringup,
    ])
