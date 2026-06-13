from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    # Publishes the full joint state (actives + solved passives); replaces the
    # stock joint_state_broadcaster as the single /joint_states source.
    passive_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "volcaniarm_passive_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    volcaniarm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "volcaniarm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            passive_broadcaster_spawner,
            volcaniarm_controller_spawner,
        ]
    )
