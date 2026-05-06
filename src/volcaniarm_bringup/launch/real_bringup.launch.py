import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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

    Schema (camera-off-robot calibration; parent_frame must be world,
    child_frame must be camera_link, since those are the URDF's
    parameterised calibration_camera_joint endpoints):
        parent_frame: world
        child_frame: camera_link
        xyz: [x, y, z]
        rpy: [roll, pitch, yaw]   # radians, used for the URDF override
        quat: [qx, qy, qz, qw]    # informational
    """
    if not _CAMERA_POSE_CONFIG.exists():
        return None
    try:
        with _CAMERA_POSE_CONFIG.open() as f:
            cfg = yaml.safe_load(f) or {}
        for key in ('parent_frame', 'child_frame', 'xyz', 'rpy'):
            if key not in cfg:
                raise ValueError(f'missing key {key!r}')
        if cfg['parent_frame'] != 'world' \
                or cfg['child_frame'] != 'camera_link':
            raise ValueError(
                f'parent/child must be world/camera_link, '
                f'got {cfg["parent_frame"]!r}/{cfg["child_frame"]!r}')
        if len(cfg['xyz']) != 3 or len(cfg['rpy']) != 3:
            raise ValueError('xyz must have 3 values and rpy must have 3')
        return cfg
    except Exception as exc:
        print(f'[real_bringup] WARNING: ignoring {_CAMERA_POSE_CONFIG} ({exc})')
        return None


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

    # camera_pose.yaml (written by the dashboard's Calibrate camera
    # button in calibration mode) holds the off-robot stand pose:
    # world -> camera_link. Its xyz/rpy are forwarded to the URDF's
    # parameterised calibration_camera_joint via xacro args. If the file
    # is missing or malformed, the URDF defaults apply (stand 1.5 m in
    # front of the arm, 0.6 m above floor); rename / delete the YAML to
    # revert. Note: world_to_base has yaw=pi, so "in front of the arm"
    # is world x=-1.5, not +1.5.
    _camera_cfg_at_launch = _load_camera_pose_config()
    if _camera_cfg_at_launch is not None:
        _cam_xyz = _camera_cfg_at_launch['xyz']
        _cam_rpy = _camera_cfg_at_launch['rpy']
        print(f'[real_bringup] applying camera_pose.yaml '
              f'(xyz={_cam_xyz}, rpy={_cam_rpy}, last_updated '
              f'{_camera_cfg_at_launch.get("last_updated", "?")})')
        _cal_cam_defaults = {
            'calibration_camera_x':     f'{_cam_xyz[0]:.9f}',
            'calibration_camera_y':     f'{_cam_xyz[1]:.9f}',
            'calibration_camera_z':     f'{_cam_xyz[2]:.9f}',
            'calibration_camera_roll':  f'{_cam_rpy[0]:.9f}',
            'calibration_camera_pitch': f'{_cam_rpy[1]:.9f}',
            'calibration_camera_yaw':   f'{_cam_rpy[2]:.9f}',
        }
    else:
        # URDF defaults from volcaniarm_realsense.xacro xacro:arg blocks
        _cal_cam_defaults = {
            'calibration_camera_x':     '-1.55',
            'calibration_camera_y':     '0.0',
            'calibration_camera_z':     '0.6',
            'calibration_camera_roll':  '0.0',
            'calibration_camera_pitch': '0.0',
            'calibration_camera_yaw':   '0.0',
        }

    calibration_camera_x_arg = DeclareLaunchArgument(
        "calibration_camera_x",
        default_value=_cal_cam_defaults['calibration_camera_x'],
        description="calibration_camera_joint origin x in world frame "
                    "(off-robot stand pose; from camera_pose.yaml if present)")
    calibration_camera_y_arg = DeclareLaunchArgument(
        "calibration_camera_y",
        default_value=_cal_cam_defaults['calibration_camera_y'])
    calibration_camera_z_arg = DeclareLaunchArgument(
        "calibration_camera_z",
        default_value=_cal_cam_defaults['calibration_camera_z'])
    calibration_camera_roll_arg = DeclareLaunchArgument(
        "calibration_camera_roll",
        default_value=_cal_cam_defaults['calibration_camera_roll'])
    calibration_camera_pitch_arg = DeclareLaunchArgument(
        "calibration_camera_pitch",
        default_value=_cal_cam_defaults['calibration_camera_pitch'])
    calibration_camera_yaw_arg = DeclareLaunchArgument(
        "calibration_camera_yaw",
        default_value=_cal_cam_defaults['calibration_camera_yaw'])

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
            " calibration_camera_x:=", LaunchConfiguration("calibration_camera_x"),
            " calibration_camera_y:=", LaunchConfiguration("calibration_camera_y"),
            " calibration_camera_z:=", LaunchConfiguration("calibration_camera_z"),
            " calibration_camera_roll:=", LaunchConfiguration("calibration_camera_roll"),
            " calibration_camera_pitch:=", LaunchConfiguration("calibration_camera_pitch"),
            " calibration_camera_yaw:=", LaunchConfiguration("calibration_camera_yaw"),
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
            calibration_camera_x_arg,
            calibration_camera_y_arg,
            calibration_camera_z_arg,
            calibration_camera_roll_arg,
            calibration_camera_pitch_arg,
            calibration_camera_yaw_arg,
            robot_state_publisher,
            OpaqueFunction(
                function=lambda ctx: _build_controller_manager(ctx, robot_description_content)
            ),
            display_launch,
            controller_launch,
            rl_controller_launch,
            rl_inactive_spawner,
            motion_launch,
            realsense_camera,
            calibration_dashboard,
        ]
    )
