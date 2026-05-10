import math
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

    Schema:
        mode:         calibration_stand | on_robot_mount
        parent_frame: world (stand) | camera_mount_rev_link (on_robot)
        child_frame:  camera_link
        xyz:          [x, y, z]
        rpy:          [roll, pitch, yaw]   # radians
        quat:         [qx, qy, qz, qw]     # informational
    """
    if not _CAMERA_POSE_CONFIG.exists():
        return None
    try:
        with _CAMERA_POSE_CONFIG.open() as f:
            cfg = yaml.safe_load(f) or {}
        for key in ('mode', 'parent_frame', 'child_frame', 'xyz', 'rpy'):
            if key not in cfg:
                raise ValueError(f'missing key {key!r}')
        if cfg['mode'] not in ('calibration_stand', 'on_robot_mount'):
            raise ValueError(
                f'mode must be calibration_stand or on_robot_mount, '
                f'got {cfg["mode"]!r}')
        if cfg['child_frame'] != 'camera_link':
            raise ValueError(
                f'child_frame must be camera_link, got {cfg["child_frame"]!r}')
        expected_parent = ('world' if cfg['mode'] == 'calibration_stand'
                           else 'camera_mount_rev_link')
        if cfg['parent_frame'] != expected_parent:
            raise ValueError(
                f'parent_frame must be {expected_parent} for mode '
                f'{cfg["mode"]}, got {cfg["parent_frame"]!r}')
        if len(cfg['xyz']) != 3 or len(cfg['rpy']) != 3:
            raise ValueError('xyz must have 3 values and rpy must have 3')
        # Defensive: refuse non-finite values. A bug elsewhere (e.g. a
        # cv2 hand-eye solve that returned NaN on an underconstrained
        # 2-DOF arm) shouldn't be able to reach the URDF parser, which
        # would otherwise hang the entire launch.
        for key in ('xyz', 'rpy'):
            for v in cfg[key]:
                if not math.isfinite(float(v)):
                    raise ValueError(
                        f'{key} contains non-finite value {v!r}')
        return cfg
    except Exception as exc:
        print(f'[real_bringup] WARNING: ignoring {_CAMERA_POSE_CONFIG} ({exc})')
        return None


def _camera_xacro_defaults() -> dict:
    """Resolve xacro arg defaults for both `calibration_camera_*` (stand
    mount) and `camera_*` (on-robot mount) arg sets.

    The persisted camera_pose.yaml carries a `mode` field stamping which
    arg set its xyz/rpy belong in. We always emit *both* sets: the slot
    matching the YAML's mode receives the calibrated values; the other
    slot keeps URDF defaults. The xacro's `mode` arg picks which set
    actually drives the published TF, so a mismatch is structurally
    impossible.
    """
    # Stand defaults: from volcaniarm_realsense.xacro xacro:arg blocks.
    cal_cam = {
        'calibration_camera_x':     '-1.5',
        'calibration_camera_y':      '0.0',
        'calibration_camera_z':      '0.6',
        'calibration_camera_roll':   '0.0',
        'calibration_camera_pitch':  '0.0',
        'calibration_camera_yaw':    '0.0',
    }
    # On-robot defaults: from volcaniarm.urdf.xacro xacro:arg blocks.
    on_robot_cam = {
        'camera_x':     '0.0160004150359285',
        'camera_y':     '0.0',
        'camera_z':     '0.0',
        'camera_roll':  '3.14159',
        'camera_pitch': '0.0',
        'camera_yaw':   '0.0',
    }

    cfg = _load_camera_pose_config()
    if cfg is None:
        return {**cal_cam, **on_robot_cam}

    xyz = cfg['xyz']
    rpy = cfg['rpy']
    print(f'[real_bringup] applying camera_pose.yaml '
          f'(mode={cfg["mode"]}, xyz={xyz}, rpy={rpy}, last_updated '
          f'{cfg.get("last_updated", "?")})')

    if cfg['mode'] == 'calibration_stand':
        cal_cam = {
            'calibration_camera_x':     f'{xyz[0]:.9f}',
            'calibration_camera_y':     f'{xyz[1]:.9f}',
            'calibration_camera_z':     f'{xyz[2]:.9f}',
            'calibration_camera_roll':  f'{rpy[0]:.9f}',
            'calibration_camera_pitch': f'{rpy[1]:.9f}',
            'calibration_camera_yaw':   f'{rpy[2]:.9f}',
        }
    else:  # on_robot_mount
        on_robot_cam = {
            'camera_x':     f'{xyz[0]:.9f}',
            'camera_y':     f'{xyz[1]:.9f}',
            'camera_z':     f'{xyz[2]:.9f}',
            'camera_roll':  f'{rpy[0]:.9f}',
            'camera_pitch': f'{rpy[1]:.9f}',
            'camera_yaw':   f'{rpy[2]:.9f}',
        }

    return {**cal_cam, **on_robot_cam}


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

    # Physical configuration of the camera. Decoupled from `calibration`:
    #   work  -> camera mounted on the robot (URDF parent: camera_mount_rev_link)
    #   tests -> camera on a stand in front of the robot (URDF parent: world)
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="work",
        choices=["work", "tests"],
        description="Physical camera configuration. 'work' mounts the "
                    "camera on the robot (default); 'tests' puts the "
                    "camera on a stand in front of the robot for "
                    "accuracy/repeatability tests.",
    )

    # The (mode, calibration) tuple covers four launch configurations:
    #   mode=work, calibration=false   regular ops, no markers, no dashboard
    #   mode=work, calibration=true    on-robot camera, EE marker, calibrate camera_joint
    #   mode=tests, calibration=false  stand camera, both markers, full test runner
    #   mode=tests, calibration=true   stand camera, EE marker, calibrate calibration_camera_joint
    calibration_arg = DeclareLaunchArgument(
        "calibration",
        default_value="false",
        choices=["true", "false"],
        description="Open the calibration dashboard with only the camera-"
                    "pose calibration UI exposed. With mode=tests this "
                    "skips the test runner widgets; with mode=work it "
                    "enables the on-robot eye-in-hand calibration.",
    )

    # Pointcloud is on by default for parity with sim and so RViz / the
    # weed detector see depth without needing an extra arg. Disable on
    # bandwidth-constrained setups with pointcloud:=false.
    pointcloud_arg = DeclareLaunchArgument(
        "pointcloud",
        default_value="true",
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

    # Optional override for the EE marker world-orientation prior used
    # by the EE-sweep calibration solver. Format: comma-separated rpy
    # in radians (extrinsic XYZ Euler) of `apriltag_marker_ee` in
    # world. Empty -> the runner reads R(world, apriltag_marker_ee)
    # via TF at run start (uses URDF defaults). Set this when the URDF
    # apriltag mount is biased and you want to bypass it; e.g. for
    # "marker face up, image axes aligned with world" pass
    # `marker_world_rpy:=0.0,0.0,0.0`.
    marker_world_rpy_arg = DeclareLaunchArgument(
        "marker_world_rpy",
        default_value="",
        description="Override marker world-orientation prior. Format "
                    "'r,p,y' (rad), or empty to read from URDF.",
    )

    # camera_pose.yaml (written by the dashboard's Calibrate camera
    # button) holds either the off-robot stand pose (parent: world) or
    # the on-robot mount pose (parent: camera_mount_rev_link), keyed by
    # `mode`. Both arg sets are emitted; the YAML's mode-stamp decides
    # which set receives the calibrated values, and the URDF's `mode`
    # arg decides which set drives the published TF.
    _cam_defaults = _camera_xacro_defaults()

    calibration_camera_x_arg = DeclareLaunchArgument(
        "calibration_camera_x",
        default_value=_cam_defaults['calibration_camera_x'],
        description="calibration_camera_joint origin x in world frame "
                    "(off-robot stand pose; from camera_pose.yaml if mode matches)")
    calibration_camera_y_arg = DeclareLaunchArgument(
        "calibration_camera_y",
        default_value=_cam_defaults['calibration_camera_y'])
    calibration_camera_z_arg = DeclareLaunchArgument(
        "calibration_camera_z",
        default_value=_cam_defaults['calibration_camera_z'])
    calibration_camera_roll_arg = DeclareLaunchArgument(
        "calibration_camera_roll",
        default_value=_cam_defaults['calibration_camera_roll'])
    calibration_camera_pitch_arg = DeclareLaunchArgument(
        "calibration_camera_pitch",
        default_value=_cam_defaults['calibration_camera_pitch'])
    calibration_camera_yaw_arg = DeclareLaunchArgument(
        "calibration_camera_yaw",
        default_value=_cam_defaults['calibration_camera_yaw'])

    camera_x_arg = DeclareLaunchArgument(
        "camera_x",
        default_value=_cam_defaults['camera_x'],
        description="On-robot camera_joint origin x in camera_mount_rev_link "
                    "(from camera_pose.yaml if mode=on_robot_mount)")
    camera_y_arg = DeclareLaunchArgument(
        "camera_y", default_value=_cam_defaults['camera_y'])
    camera_z_arg = DeclareLaunchArgument(
        "camera_z", default_value=_cam_defaults['camera_z'])
    camera_roll_arg = DeclareLaunchArgument(
        "camera_roll", default_value=_cam_defaults['camera_roll'])
    camera_pitch_arg = DeclareLaunchArgument(
        "camera_pitch", default_value=_cam_defaults['camera_pitch'])
    camera_yaw_arg = DeclareLaunchArgument(
        "camera_yaw", default_value=_cam_defaults['camera_yaw'])

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
    # `mode` arg picks where the camera is parented (world for tests,
    # camera_mount_rev_link for work). `calibration` controls the marker
    # mounting + calibration dashboard scope. `auto_home` toggles
    # limit-switch homing in the hardware interface.
    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            os.path.join(volcaniarm_description_share, "urdf", "volcaniarm.urdf.xacro"),
            " use_sim:=false",
            " auto_home:=", LaunchConfiguration("auto_home"),
            " mode:=", LaunchConfiguration("mode"),
            " calibration:=", LaunchConfiguration("calibration"),
            " tag_size:=", LaunchConfiguration("tag_size"),
            " calibration_camera_x:=", LaunchConfiguration("calibration_camera_x"),
            " calibration_camera_y:=", LaunchConfiguration("calibration_camera_y"),
            " calibration_camera_z:=", LaunchConfiguration("calibration_camera_z"),
            " calibration_camera_roll:=", LaunchConfiguration("calibration_camera_roll"),
            " calibration_camera_pitch:=", LaunchConfiguration("calibration_camera_pitch"),
            " calibration_camera_yaw:=", LaunchConfiguration("calibration_camera_yaw"),
            " camera_x:=", LaunchConfiguration("camera_x"),
            " camera_y:=", LaunchConfiguration("camera_y"),
            " camera_z:=", LaunchConfiguration("camera_z"),
            " camera_roll:=", LaunchConfiguration("camera_roll"),
            " camera_pitch:=", LaunchConfiguration("camera_pitch"),
            " camera_yaw:=", LaunchConfiguration("camera_yaw"),
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

    # Display (RViz) launch. Skipped when the calibration dashboard is
    # up (the dashboard's RViz takes over instead) or when running the
    # full test workflow (mode=tests, calibration=false; the test
    # runner's RViz takes over).
    show_display = IfCondition(PythonExpression([
        "'", LaunchConfiguration("calibration"), "' == 'false' and ",
        "'", LaunchConfiguration("mode"), "' != 'tests'",
    ]))
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcaniarm_description_share, "launch", "display.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
        condition=show_display,
    )

    # Calibration dashboard.
    # Activated when calibration:=true (any mode) OR when mode=tests
    # with calibration:=false (the standard test workflow). The
    # `camera_calibration_only` arg restricts the dashboard UI to just
    # the camera-localization group when calibration:=true.
    show_dashboard = IfCondition(PythonExpression([
        "'", LaunchConfiguration("calibration"), "' == 'true' or ",
        "'", LaunchConfiguration("mode"), "' == 'tests'",
    ]))
    calibration_dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcaniarm_calibration_share, "launch", "dashboard.launch.py")
        ),
        launch_arguments=[
            ("tag_size", LaunchConfiguration("tag_size")),
            ("camera_calibration_only", LaunchConfiguration("calibration")),
            ("marker_world_rpy", LaunchConfiguration("marker_world_rpy")),
        ],
        condition=show_dashboard,
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
            mode_arg,
            calibration_arg,
            pointcloud_arg,
            tag_size_arg,
            marker_world_rpy_arg,
            calibration_camera_x_arg,
            calibration_camera_y_arg,
            calibration_camera_z_arg,
            calibration_camera_roll_arg,
            calibration_camera_pitch_arg,
            calibration_camera_yaw_arg,
            camera_x_arg,
            camera_y_arg,
            camera_z_arg,
            camera_roll_arg,
            camera_pitch_arg,
            camera_yaw_arg,
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
