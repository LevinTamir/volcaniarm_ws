"""Joystick teleop for volcaniarm EE position (YZ plane).

Starts joy_node (reads the gamepad) and the custom teleop integrator
node (stick → velocity → EE target → IK → JointTrajectory).

Assumes the sim or real stack is already running so the
/volcaniarm_controller/joint_trajectory topic exists (IK is solved
in-process via volcaniarm_kinematics). Typically launched after
sim_bringup.launch.py / real_bringup.launch.py.

The gamepad enumerates differently over USB vs Bluetooth, so the
button/axis mapping is split into profiles selected by joy_profile.

Usage:
  ros2 launch volcaniarm_controllers joystick_teleop.launch.py                  # Bluetooth (default)
  ros2 launch volcaniarm_controllers joystick_teleop.launch.py joy_profile:=usb # USB
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('volcaniarm_controllers'), 'config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulated time (pass true when running alongside sim_bringup)',
    )

    joy_profile_arg = DeclareLaunchArgument(
        'joy_profile',
        default_value='bt',
        choices=['usb', 'bt'],
        description='Joystick mapping profile: bt (Bluetooth, default) or usb. '
                    "The gamepad's button/axis indices differ between the two.",
    )

    # Pick config/joy_teleop_<profile>.yaml at launch time.
    joy_teleop_params = PathJoinSubstitution([
        config_dir, ['joy_teleop_', LaunchConfiguration('joy_profile'), '.yaml'],
    ])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        parameters=[
            os.path.join(config_dir, 'joy_config.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    teleop_node = Node(
        package='volcaniarm_controllers',
        executable='joystick_teleop_node.py',
        name='volcaniarm_joystick_teleop',
        parameters=[
            joy_teleop_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    return LaunchDescription(
        [use_sim_time_arg, joy_profile_arg, joy_node, teleop_node]
    )
