"""Full simulation bringup for calibration testing.

Launches the complete sim stack (Gazebo with AprilTag on EE, controllers,
RViz, motion planning) then includes the calibration-specific nodes
from volcaniarm_calibration.

Usage:
  ros2 launch volcaniarm_bringup sim_calibration_bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('volcaniarm_bringup')
    calibration_share = get_package_share_directory('volcaniarm_calibration')

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='lab',
        description='Gazebo world name')

    # Full sim bringup with calibration flag (adds AprilTag to EE)
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_share, 'launch', 'sim_bringup.launch.py')
        ]),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'calibration': 'true',
        }.items(),
    )

    # Calibration nodes (apriltag detector + accuracy test), delayed for Gazebo startup
    calibration_nodes = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(calibration_share, 'launch',
                                 'sim_accuracy_test.launch.py')
                ]),
            ),
        ],
    )

    return LaunchDescription([
        world_name_arg,
        sim_bringup,
        calibration_nodes,
    ])
