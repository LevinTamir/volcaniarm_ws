"""Joystick driver for LeRobot teleop/record on the volcaniarm.

Starts ONLY joy_node (publishes /joy). Unlike joystick_teleop.launch.py, it does
NOT start the joystick_teleop_node integrator: with LeRobot, the LeRobot gamepad
teleop (volcaniarm_gamepad, in the lerobot_robot_volcaniarm plugin) is the driver
and reads /joy itself, so it can record the action. Run this alongside the sim or
real stack, then run lerobot-teleoperate / lerobot-record.

Usage:
  ros2 launch volcaniarm_controllers lerobot_joy.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('volcaniarm_controllers')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulated time (pass true when running alongside sim_bringup)',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        parameters=[
            os.path.join(pkg_share, 'config', 'joy_config.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([use_sim_time_arg, joy_node])
