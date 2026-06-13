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

    # move_group's robot_description is the PLANNING render: it adds the open
    # phantom Y-Z chain to tool0 (planning:=true) so MoveIt can plan the
    # otherwise-unplannable closed loop. The real/sim robot uses planning:=false.
    moveit_config = (
        MoveItConfigsBuilder("volcaniarm", package_name="volcaniarm_moveit_config")
        .robot_description(file_path=urdf_xacro, mappings={"planning": "true"})
        .robot_description_semantic(file_path="srdf/volcaniarm.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )

    # Bridge: MoveIt executes the planned phantom (EE Y-Z) trajectory to this
    # node, which solves inverse_ee per waypoint and forwards to the JTC.
    transformer_node = Node(
        package="volcaniarm_moveit_config",
        executable="joint_state_transformer",
        name="joint_state_transformer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([use_sim_time_arg, move_group_node, transformer_node])
