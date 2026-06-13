import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    desc_share = get_package_share_directory("volcaniarm_description")
    urdf_xacro = os.path.join(desc_share, "urdf", "volcaniarm.urdf.xacro")
    config_share = get_package_share_directory("volcaniarm_moveit_config")
    rviz_config = os.path.join(config_share, "config", "moveit.rviz")

    moveit_config = (
        MoveItConfigsBuilder("volcaniarm", package_name="volcaniarm_moveit_config")
        .robot_description(file_path=urdf_xacro, mappings={"planning": "true"})
        .robot_description_semantic(file_path="srdf/volcaniarm.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([use_sim_time_arg, rviz_node])
