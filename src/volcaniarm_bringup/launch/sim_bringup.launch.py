import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation time",
    )

    calibration_arg = DeclareLaunchArgument(
        "calibration",
        default_value="false",
        description="Add AprilTag to EE for calibration testing",
    )

    # Default world depends on calibration mode: the lab world contains
    # a potted-weed visual that occludes the apriltag scene, so
    # calibration runs use a stripped-down world. User can still
    # override with world_name:=<name>.
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value=PythonExpression([
            "'calibration' if '",
            LaunchConfiguration("calibration"),
            "' == 'true' else 'lab'",
        ]),
        description="Gazebo world name (without .sdf extension); "
                    "defaults to 'calibration' when calibration:=true, "
                    "else 'lab'",
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

    # Controller mode:
    #   traj   → only trajectory controller loaded + active (default)
    #   policy → only RL policy controller loaded + active
    #   all    → both loaded; trajectory active, policy inactive
    #            (available to claim via `ros2 control switch_controllers`)
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="traj",
        choices=["traj", "policy", "all"],
        description="Which controller(s) to load",
    )

    is_gazebo = IfCondition(
        PythonExpression(["'", LaunchConfiguration("sim"), "' == 'gazebo'"])
    )
    is_isaac = IfCondition(
        PythonExpression(["'", LaunchConfiguration("sim"), "' == 'isaac'"])
    )
    is_traj_active = IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("controller"), "' in ('traj', 'all')"]
        )
    )
    is_policy_only = IfCondition(
        PythonExpression(["'", LaunchConfiguration("controller"), "' == 'policy'"])
    )
    is_all = IfCondition(
        PythonExpression(["'", LaunchConfiguration("controller"), "' == 'all'"])
    )

    # Get package paths
    volcaniarm_description_share = get_package_share_directory("volcaniarm_description")
    volcaniarm_controller_share = get_package_share_directory("volcaniarm_controller")
    volcaniarm_motion_share = get_package_share_directory("volcaniarm_motion")
    volcaniarm_calibration_share = get_package_share_directory("volcaniarm_calibration")

    # Gazebo launch — passes `controller` through so the xacro emits
    # the right <parameters> block(s) into the URDF.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_description_share, "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("world_name", LaunchConfiguration("world_name")),
            ("calibration", LaunchConfiguration("calibration")),
            ("camera_mount_x", LaunchConfiguration("camera_mount_x")),
            ("camera_mount_pitch", LaunchConfiguration("camera_mount_pitch")),
            ("controller", LaunchConfiguration("controller")),
        ],
        condition=is_gazebo,
    )

    # Isaac launch — loads the matching YAML(s) into its own controller_manager.
    isaac_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_description_share, "launch", "isaac.launch.py"
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("controller", LaunchConfiguration("controller")),
        ],
        condition=is_isaac,
    )

    # Trajectory sub-launch (JSB + trajectory active). Included for
    # `traj` and `all`.
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcaniarm_controller_share, "launch", "controller.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
        condition=is_traj_active,
    )

    # Policy sub-launch (JSB + policy active). Included only for `policy`.
    rl_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcaniarm_controller_share, "launch", "rl_controller.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
        condition=is_policy_only,
    )

    # For `all`: load the policy controller inactive so it can be claimed later.
    rl_inactive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "volcaniarm_rl_controller",
            "--controller-manager", "/controller_manager",
            "--inactive",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
        condition=is_all,
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
    # The sim mesh scales the tag to 0.2 m (volcaniarm_apriltag.xacro),
    # so override the detector's tag size to match.
    calibration_dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_calibration_share,
                "launch",
                "dashboard.launch.py",
            )
        ),
        launch_arguments=[
            ("tag_size", "0.200"),
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
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
            # calibration_arg must come before world_name_arg --
            # world_name's default is a PythonExpression that reads
            # LaunchConfiguration("calibration"), so the calibration
            # arg has to exist by the time launch evaluates it.
            calibration_arg,
            world_name_arg,
            camera_mount_x_arg,
            camera_mount_pitch_arg,
            sim_arg,
            controller_arg,
            gazebo_launch,
            isaac_launch,
            controller_launch,
            rl_controller_launch,
            rl_inactive_spawner,
            display_launch,
            motion_launch,
            calibration_dashboard,
        ]
    )
