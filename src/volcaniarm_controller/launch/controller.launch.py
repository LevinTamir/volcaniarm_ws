import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    # --- end-effector marker node ---
    # end_effector_marker = Node(
    #     package="volcaniarm_controller",
    #     executable="volcaniarm_end_effector_marker.py",
    #     name="volcaniarm_end_effector_marker",
    #     output="screen",
    #     parameters=[{
    #         "joints": ["left_elbow_joint", "right_elbow_joint"],
    #         "link_lengths": [0.30, 0.2227],
    #         "base_frame": "delta_arm_base_link",
    #         "zero_along_z": False
    #     }],
    # )
    # -------------------------------------

    return LaunchDescription(
        [
            use_sim_time_arg,
            joint_state_broadcaster_spawner,
            volcaniarm_controller_spawner,
            # end_effector_marker,
        ]
    )
