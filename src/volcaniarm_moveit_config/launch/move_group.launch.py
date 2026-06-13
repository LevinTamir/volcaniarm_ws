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

    # Build the RobotModel from the same description xacro the bringup uses. Only
    # the arm kinematics matter for planning; camera args keep their defaults.
    desc_share = get_package_share_directory("volcaniarm_description")
    urdf_xacro = os.path.join(desc_share, "urdf", "volcaniarm.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("volcaniarm", package_name="volcaniarm_moveit_config")
        .robot_description(file_path=urdf_xacro, mappings={"use_sim": "true"})
        .robot_description_semantic(file_path="srdf/volcaniarm.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .planning_pipelines(
            pipelines=["pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            {"publish_robot_description_semantic": True},
        ],
    )

    return LaunchDescription([use_sim_time_arg, move_group_node])
