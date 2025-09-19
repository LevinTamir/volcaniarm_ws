import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("volcaniarm_description"),
                    "urdf",
                    "volcaniarm.xacro",
                ),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        condition=UnlessCondition(is_sim),
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("volcaniarm_controller"),
                "config",
                "volcaniarm_controllers.yaml",
            ),
        ],
        condition=UnlessCondition(is_sim),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    volcaniarm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["volcaniarm_controller", "--controller-manager", "/controller_manager"],
    )

    # --- end-effector marker node ---
    end_effector_marker = Node(
        package="volcaniarm_controller",
        executable="volcaniarm_end_effector_marker.py",
        name="volcaniarm_end_effector_marker",
        output="screen",
        parameters=[{
            "joints": ["delta_arm_base_joint", "right_elbow_joint", "left_elbow_joint"],
            "link_lengths": [0.20, 0.15],
            "base_frame": "world"
        }],
    )
    # -------------------------------------

    return LaunchDescription(
        [
            is_sim_arg,
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            volcaniarm_controller_spawner,
            end_effector_marker,
        ]
    )
