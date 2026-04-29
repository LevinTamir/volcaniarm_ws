"""Joystick teleop for volcaniarm EE position (YZ plane).

Starts joy_node (reads the gamepad) and the custom teleop integrator
node (stick → velocity → EE target → IK → JointTrajectory).

Assumes the sim or real stack is already running so /compute_ik service
and /volcaniarm_controller/joint_trajectory topic exist. Typically
launched after sim_bringup.launch.py.

Usage:
  ros2 launch volcaniarm_controller joystick_teleop.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('volcaniarm_controller')

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

    teleop_node = Node(
        package='volcaniarm_controller',
        executable='joystick_teleop_node.py',
        name='volcaniarm_joystick_teleop',
        parameters=[
            os.path.join(pkg_share, 'config', 'joy_teleop.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    return LaunchDescription([use_sim_time_arg, joy_node, teleop_node])
