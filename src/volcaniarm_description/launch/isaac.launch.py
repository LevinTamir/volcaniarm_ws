import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _build_controller_manager(context, robot_description):
    volcaniarm_controller = get_package_share_directory("volcaniarm_controllers")
    mode = LaunchConfiguration("controller").perform(context)

    yamls = []
    if mode in ("traj", "all"):
        yamls.append(os.path.join(
            volcaniarm_controller, "config", "volcaniarm_controllers.yaml"))
    if mode in ("policy", "all"):
        yamls.append(os.path.join(
            volcaniarm_controller, "config", "volcaniarm_rl_controller.yaml"))
    if mode == "vision_policy":
        yamls.append(os.path.join(
            volcaniarm_controller, "config", "volcaniarm_rl_vision_controller.yaml"))

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
        choices=["traj", "policy", "vision_policy", "all"],
        description="Which controller(s) to load into controller_manager",
    )

    # Forward `controller:=` to xacro so this RSP publishes a URDF
    # whose <parameters> block matches the YAML that
    # _build_controller_manager actually loads. Without this the URDF
    # races display.launch.py's RSP on /robot_description with the
    # wrong YAML reference (same bug we hit on the Gazebo path).
    robot_description = ParameterValue(
        Command(
            [
                "xacro ", LaunchConfiguration("model"),
                " sim_engine:=isaac",
                " controller:=", LaunchConfiguration("controller"),
            ]
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
