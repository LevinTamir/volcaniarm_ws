import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    """JSB + RL vision controller spawner.

    Sibling of `rl_controller.launch.py`. Passes `--param-file` to the
    vision spawner explicitly because the gz_ros2_control plugin's
    URDF `<parameters>` loader was observed to skip the
    `volcaniarm_rl_vision_controller.type` entry (JSB from the same
    YAML still loaded fine — root cause unclear, but the explicit
    param-file is robust against whatever the plugin is doing).
    """

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    vision_yaml = os.path.join(
        get_package_share_directory("volcaniarm_controller"),
        "config",
        "volcaniarm_rl_vision_controller.yaml",
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

    rl_vision_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "volcaniarm_rl_vision_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            vision_yaml,
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            joint_state_broadcaster_spawner,
            rl_vision_controller_spawner,
        ]
    )
