import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('volcaniarm_motion'),
        'config', 'motion_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    kinematics_node = Node(
        package="volcaniarm_motion",
        executable="volcaniarm_kinematics.py",
        name="volcaniarm_kinematics",
        output="screen",
        parameters=[config_file],
    )

    motion_planning_node = Node(
        package="volcaniarm_motion",
        executable="volcaniarm_motion_planning.py",
        name="motion_planning_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            kinematics_node,
            motion_planning_node,
        ]
    )
