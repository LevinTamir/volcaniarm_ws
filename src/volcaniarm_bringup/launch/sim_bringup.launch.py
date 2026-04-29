import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation time",
    )

    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="lab",
        description="Gazebo world name (without .sdf extension)",
    )

    calibration_arg = DeclareLaunchArgument(
        "calibration",
        default_value="false",
        description="Add AprilTag to EE for calibration testing",
    )

    camera_mount_x_arg = DeclareLaunchArgument(
        "camera_mount_x",
        default_value="0.25",
        description="Camera mount position along X (range: 0.155 to 0.655)",
    )

    camera_mount_pitch_arg = DeclareLaunchArgument(
        "camera_mount_pitch",
        default_value="1.3",
        description="Camera mount pitch in radians",
    )

    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="gazebo",
        choices=["gazebo", "isaac"],
        description="Simulator backend: 'gazebo' (auto-launches Gazebo) or 'isaac' "
                    "(expects Isaac Sim already running with the ROS2 bridge and the scene loaded)",
    )

    is_gazebo = IfCondition(
        PythonExpression(["'", LaunchConfiguration("sim"), "' == 'gazebo'"])
    )
    is_isaac = IfCondition(
        PythonExpression(["'", LaunchConfiguration("sim"), "' == 'isaac'"])
    )

    # Get package paths
    volcaniarm_description_share = get_package_share_directory("volcaniarm_description")
    volcaniarm_controller_share = get_package_share_directory("volcaniarm_controller")
    volcaniarm_motion_share = get_package_share_directory("volcaniarm_motion")
    volcaniarm_calibration_share = get_package_share_directory("volcaniarm_calibration")

    # Gazebo launch (simulator + robot spawn + gz bridge + robot_state_publisher)
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
            ("world_name", LaunchConfiguration("world_name")),
            ("calibration", LaunchConfiguration("calibration")),
            ("camera_mount_x", LaunchConfiguration("camera_mount_x")),
            ("camera_mount_pitch", LaunchConfiguration("camera_mount_pitch")),
        ],
        condition=is_gazebo,
    )

    # Isaac launch (robot_state_publisher + standalone controller_manager with TopicBasedSystem)
    isaac_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_description_share,
                "launch",
                "isaac.launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
        condition=is_isaac,
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

    # Display (RViz) launch. Skipped in calibration mode so the
    # calibration dashboard's RViz takes over instead.
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
        condition=UnlessCondition(LaunchConfiguration("calibration")),
    )

    # Calibration dashboard (apriltag detector + RViz + rqt plugin).
    calibration_dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_calibration_share,
                "launch",
                "dashboard.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("calibration")),
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
            world_name_arg,
            calibration_arg,
            camera_mount_x_arg,
            camera_mount_pitch_arg,
            sim_arg,
            gazebo_launch,
            isaac_launch,
            controller_launch,
            display_launch,
            motion_launch,
            calibration_dashboard,
        ]
    )
