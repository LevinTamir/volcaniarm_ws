import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Persisted camera pose written by the calibration dashboard's
# Calibrate camera button. Workspace-local; gitignored.
_CAMERA_POSE_CONFIG = (
    Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/'
         'config/camera_pose.yaml').expanduser())


def _load_camera_pose_config():
    """Read the persisted camera pose. Returns the parsed dict or None.

    Missing file is the normal first-launch case (no calibration yet).
    Malformed file gets a single warning and we fall back to URDF
    defaults rather than crashing the launch.
    """
    if not _CAMERA_POSE_CONFIG.exists():
        return None
    try:
        with _CAMERA_POSE_CONFIG.open() as f:
            cfg = yaml.safe_load(f) or {}
        for key in ('parent_frame', 'child_frame', 'xyz', 'quat'):
            if key not in cfg:
                raise ValueError(f'missing key {key!r}')
        if len(cfg['xyz']) != 3 or len(cfg['quat']) != 4:
            raise ValueError('xyz must have 3 values and quat must have 4')
        return cfg
    except Exception as exc:
        print(f'[real_bringup] WARNING: ignoring {_CAMERA_POSE_CONFIG} ({exc})')
        return None


def _build_camera_pose_publisher(context):
    """Construct a static_transform_publisher node for the saved
    camera pose, or return [] if the override is unwanted / missing."""
    use_override = LaunchConfiguration("use_camera_pose_config").perform(context)
    if use_override.lower() not in ('true', '1', 'yes'):
        return []
    cfg = _load_camera_pose_config()
    if cfg is None:
        return []
    x, y, z = (float(v) for v in cfg['xyz'])
    qx, qy, qz, qw = (float(v) for v in cfg['quat'])
    parent = str(cfg['parent_frame'])
    child = str(cfg['child_frame'])
    return [
        LogInfo(msg=(
            f'[real_bringup] applying camera_pose.yaml '
            f'({parent} -> {child}, last_updated '
            f'{cfg.get("last_updated", "?")})')),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_pose_override',
            arguments=[
                '--x', f'{x}', '--y', f'{y}', '--z', f'{z}',
                '--qx', f'{qx}', '--qy', f'{qy}',
                '--qz', f'{qz}', '--qw', f'{qw}',
                '--frame-id', parent,
                '--child-frame-id', child,
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
    ]


def _build_controller_manager(context, robot_description_content):
    volcaniarm_controller_share = get_package_share_directory("volcaniarm_controller")
    mode = LaunchConfiguration("controller").perform(context)

    yamls = []
    if mode in ("traj", "all"):
        yamls.append(os.path.join(
            volcaniarm_controller_share, "config", "volcaniarm_controllers.yaml"))
    if mode in ("policy", "all"):
        yamls.append(os.path.join(
            volcaniarm_controller_share, "config", "volcaniarm_rl_controller.yaml"))

    return [Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            *yamls,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )]


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation time (False for real hardware)",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/volcaniarm",
        description="Serial port for hardware interface (stable symlink "
                    "created by the udev rule in volcaniarm_hardware/udev/)",
    )

    auto_home_arg = DeclareLaunchArgument(
        "auto_home",
        default_value="false",
        choices=["true", "false"],
        description="If true, run limit-switch homing during hardware on_configure. "
                    "If false, boot without homing and use the volcaniarm_hardware/home "
                    "service to home manually when ready.",
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

    calibration_arg = DeclareLaunchArgument(
        "calibration",
        default_value="false",
        description="Add AprilTag links to the URDF and launch the calibration "
                    "dashboard (assumes physical AprilTags are mounted on the arm)",
    )

    # Pointcloud is off by default since it adds USB bandwidth + CPU
    # load and isn't needed for calibration (RGB only) or RL policy
    # control. Enable when running 3D perception (e.g. weed detector)
    # or when sim/real parity matters for the test you're running.
    pointcloud_arg = DeclareLaunchArgument(
        "pointcloud",
        default_value="false",
        choices=["true", "false"],
        description="Publish /camera/depth/color/points from the RealSense driver",
    )

    # Drives both the URDF mesh scale (via the apriltag xacro) and the
    # apriltag detector size param (via the calibration dashboard
    # include below). Default 0.064 matches the small printed tags on
    # the real arm; pass tag_size:=0.200 when using the larger prints.
    tag_size_arg = DeclareLaunchArgument(
        "tag_size",
        default_value="0.064",
        description="AprilTag edge length [m]",
    )

    # If true and a saved camera_pose.yaml exists, the launch publishes
    # base_link -> camera_link with those values via a static publisher,
    # overriding the URDF's hardcoded camera_joint origin. Set to false
    # to compare URDF defaults to the calibrated pose.
    use_camera_pose_config_arg = DeclareLaunchArgument(
        "use_camera_pose_config",
        default_value="true",
        choices=["true", "false"],
        description="Apply src/volcaniarm_calibration/config/camera_pose.yaml "
                    "(written by the dashboard's Calibrate camera button) "
                    "via a static_transform_publisher overriding the URDF's "
                    "base_link -> camera_link joint",
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

    volcaniarm_description_share = get_package_share_directory("volcaniarm_description")
    volcaniarm_controller_share = get_package_share_directory("volcaniarm_controller")
    volcaniarm_calibration_share = get_package_share_directory("volcaniarm_calibration")

    # Robot description with real hardware (use_sim=false). The
    # `calibration` arg toggles the AprilTag link block in the URDF;
    # `auto_home` toggles limit-switch homing in the hardware interface.
    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            os.path.join(volcaniarm_description_share, "urdf", "volcaniarm.urdf.xacro"),
            " use_sim:=false",
            " auto_home:=", LaunchConfiguration("auto_home"),
            " calibration:=", LaunchConfiguration("calibration"),
            " tag_size:=", LaunchConfiguration("tag_size"),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        output="screen",
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

    # For `all`: load the policy controller but leave it inactive so it
    # can be claimed later via `ros2 control switch_controllers`.
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
            os.path.join(volcaniarm_description_share, "launch", "display.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
        condition=UnlessCondition(LaunchConfiguration("calibration")),
    )

    # Calibration dashboard (apriltag detector + RViz + rqt plugin).
    # Assumes physical AprilTags are already mounted on the arm.
    calibration_dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcaniarm_calibration_share, "launch", "dashboard.launch.py")
        ),
        launch_arguments=[("tag_size", LaunchConfiguration("tag_size"))],
        condition=IfCondition(LaunchConfiguration("calibration")),
    )

    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("volcaniarm_motion"),
                "launch", "motion.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
    )

    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'depth_module.depth_profile': '848x480x30',
            'rgb_camera.color_profile': '848x480x30',
            'align_depth.enable': 'true',
            'pointcloud.enable': LaunchConfiguration("pointcloud"),
            'publish_tf': 'false',  # URDF handles all TF
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            serial_port_arg,
            auto_home_arg,
            controller_arg,
            calibration_arg,
            pointcloud_arg,
            tag_size_arg,
            use_camera_pose_config_arg,
            robot_state_publisher,
            OpaqueFunction(
                function=lambda ctx: _build_controller_manager(ctx, robot_description_content)
            ),
            OpaqueFunction(function=_build_camera_pose_publisher),
            display_launch,
            controller_launch,
            rl_controller_launch,
            rl_inactive_spawner,
            motion_launch,
            realsense_camera,
            calibration_dashboard,
        ]
    )
