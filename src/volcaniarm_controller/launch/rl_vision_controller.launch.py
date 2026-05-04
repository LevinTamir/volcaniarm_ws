from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    """JSB + RL vision controller spawner.

    Sibling of `rl_controller.launch.py` — same shape, just spawns
    `volcaniarm_rl_vision_controller` instead of `volcaniarm_rl_controller`.
    """

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
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
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            joint_state_broadcaster_spawner,
            rl_vision_controller_spawner,
        ]
    )
