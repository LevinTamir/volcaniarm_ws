from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    """JSB + RL vision controller spawner.

    Sibling of `rl_controller.launch.py`. The vision YAML is loaded
    into gz_ros2_control's controller_manager via the URDF
    `<parameters>` block (see volcaniarm_gazebo.xacro); the spawner
    just invokes load + activate.
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
