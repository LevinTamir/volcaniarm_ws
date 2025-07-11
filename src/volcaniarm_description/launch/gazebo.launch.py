#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "volcaniarm_description"
    pkg_share = get_package_share_directory(pkg_name)

    # Paths
    urdf_file = os.path.join(pkg_share, "urdf", "volcaniarm.xacro")
    world_file = os.path.join(pkg_share, "worlds", "volcaniarm.world")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_arg = DeclareLaunchArgument(
        "world", default_value=world_file, description="Full path to world file to load"
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": open(urdf_file).read()}
        ],
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": ["-r ", LaunchConfiguration("world")],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "volcaniarm",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    # Bridge for joint states
    joint_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="joint_state_bridge",
        arguments=[
            "/world/default/model/volcaniarm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            robot_state_publisher,
            gazebo_launch,
            spawn_robot,
            joint_state_bridge,
        ]
    )
