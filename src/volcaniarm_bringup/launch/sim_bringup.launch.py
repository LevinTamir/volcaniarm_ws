import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation time",
    )

    # Get package paths
    volcaniarm_description_share = get_package_share_directory("volcaniarm_description")
    volcaniarm_controller_share = get_package_share_directory("volcaniarm_controller")
    volcaniarm_motion_share = get_package_share_directory("volcaniarm_motion")

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_description_share,
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("world_name", "feild"),
        ],
    )

    # Controller launch
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_controller_share,
                "launch",
                "controller.launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
    )

    # Display (RViz) launch
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_description_share,
                "launch",
                "display.launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
    )

    # Motion launch
    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_motion_share,
                "launch",
                "motion.launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            gazebo_launch,
            controller_launch,
            display_launch,
            motion_launch,
        ]
    )
