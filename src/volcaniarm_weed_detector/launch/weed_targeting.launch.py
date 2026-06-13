import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('volcaniarm_weed_detector'),
        'config', 'weed_targeting_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="True")

    weed_targeting_node = Node(
        package="volcaniarm_weed_detector",
        executable="weed_targeting_node",
        name="weed_targeting_node",
        output="screen",
        parameters=[config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([use_sim_time_arg, weed_targeting_node])
