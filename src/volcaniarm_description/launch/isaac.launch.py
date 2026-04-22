import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    volcaniarm_description = get_package_share_directory("volcaniarm_description")
    volcaniarm_controller = get_package_share_directory("volcaniarm_controller")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(volcaniarm_description, "urdf", "volcaniarm.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation time (Isaac Sim publishes /clock when configured)",
    )

    robot_description = ParameterValue(
        Command(
            ["xacro ", LaunchConfiguration("model"), " sim_engine:=isaac"]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    controllers_yaml = os.path.join(
        volcaniarm_controller, "config", "volcaniarm_controllers.yaml"
    )
    rl_controllers_yaml = os.path.join(
        volcaniarm_controller, "config", "volcaniarm_rl_controller.yaml"
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
            controllers_yaml,
            rl_controllers_yaml,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            model_arg,
            use_sim_time_arg,
            robot_state_publisher_node,
            controller_manager_node,
        ]
    )
