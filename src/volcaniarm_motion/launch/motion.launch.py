import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    # Kinematics node (IK and FK services)
    kinematics_node = Node(
        package="volcaniarm_motion",
        executable="volcaniarm_kinematics.py",
        name="volcaniarm_kinematics",
        output="screen",
    )

    # Motion Planning node
    motion_planning_node = Node(
        package="volcaniarm_motion",
        executable="volcaniarm_motion_planning.py",
        name="motion_planning_node",
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            kinematics_node,
            motion_planning_node,
        ]
    )
