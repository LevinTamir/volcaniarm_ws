import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

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

    volcaniarm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "volcaniarm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # End-effector marker node
    end_effector_marker = Node(
        package="volcaniarm_controller",
        executable="volcaniarm_end_effector_marker.py",
        name="volcaniarm_end_effector_marker",
        output="screen",
    )

    # Set initial position node with delay to ensure controllers are ready
    set_initial_position = Node(
        package="volcaniarm_controller",
        executable="set_initial_position.py",
        name="set_initial_position",
        output="screen",
    )

    # Wait for controller spawner to exit before starting initial position
    delayed_set_initial_position = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=volcaniarm_controller_spawner,
            on_exit=[set_initial_position],
        )
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            joint_state_broadcaster_spawner,
            volcaniarm_controller_spawner,
            delayed_set_initial_position,
            end_effector_marker,
        ]
    )
