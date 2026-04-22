import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _build_controller_manager(context, robot_description):
    volcaniarm_controller = get_package_share_directory("volcaniarm_controller")
    mode = LaunchConfiguration("controller").perform(context)

    yamls = []
    if mode in ("traj", "all"):
        yamls.append(os.path.join(
            volcaniarm_controller, "config", "volcaniarm_controllers.yaml"))
    if mode in ("policy", "all"):
        yamls.append(os.path.join(
            volcaniarm_controller, "config", "volcaniarm_rl_controller.yaml"))

    return [Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
            *yamls,
        ],
        output="screen",
    )]


def generate_launch_description():
    volcaniarm_description = get_package_share_directory("volcaniarm_description")

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

    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="traj",
        choices=["traj", "policy", "all"],
        description="Which controller(s) to load into controller_manager",
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

    return LaunchDescription(
        [
            model_arg,
            use_sim_time_arg,
            controller_arg,
            robot_state_publisher_node,
            OpaqueFunction(
                function=lambda ctx: _build_controller_manager(ctx, robot_description)
            ),
        ]
    )
