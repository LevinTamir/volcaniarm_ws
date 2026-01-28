import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation time (False for real hardware)",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port for hardware interface",
    )

    # Get package paths
    volcaniarm_description_share = get_package_share_directory("volcaniarm_description")
    volcaniarm_controller_share = get_package_share_directory("volcaniarm_controller")

    # Robot state publisher with real hardware (use_sim=false)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command([
                    "xacro ",
                    os.path.join(
                        volcaniarm_description_share,
                        "urdf",
                        "volcaniarm.urdf.xacro",
                    ),
                    " use_sim:=false",
                ]),
            }
        ],
        output="screen",
    )

    # Controller manager node (runs on PC, not in Gazebo)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": Command([
                    "xacro ",
                    os.path.join(
                        volcaniarm_description_share,
                        "urdf",
                        "volcaniarm_ros2_control.xacro",
                    ),
                    " use_sim:=false",
                ]),
            }
        ],
        output="screen",
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Main arm controller spawner
    volcaniarm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "volcaniarm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Load controllers config
    controller_config_file = os.path.join(
        volcaniarm_controller_share,
        "config",
        "volcaniarm_controllers.yaml",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            serial_port_arg,
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            volcaniarm_controller_spawner,
        ]
    )
