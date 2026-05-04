import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    volcaniarm_description_dir = get_package_share_directory("volcaniarm_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            volcaniarm_description_dir, "urdf", "volcaniarm.urdf.xacro"
        ),
        description="Absolute path to robot urdf file",
    )

    # Forward the parent bringup's controller arg so this RSP's URDF
    # loads the right gz_ros2_control <parameters> block. Without this
    # the second RSP publishes a URDF with the xacro default
    # (controller=traj) and races the gazebo.launch.py RSP on the
    # transient_local /robot_description topic — the trajectory YAML
    # then wins and a different controller's type-config never reaches
    # gz_ros2_control. Default kept at 'traj' so display.launch.py
    # works standalone.
    controller_arg = DeclareLaunchArgument(
        name="controller",
        default_value="traj",
        choices=["traj", "policy", "vision_policy", "all"],
        description="Mirror of bringup's controller arg",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ", LaunchConfiguration("model"),
                " controller:=", LaunchConfiguration("controller"),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui", executable="joint_state_publisher_gui"
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(volcaniarm_description_dir, "rviz", "display.rviz"),
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            controller_arg,
            # joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
