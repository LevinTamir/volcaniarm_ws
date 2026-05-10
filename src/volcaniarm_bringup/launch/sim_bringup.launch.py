import math
import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


_CAMERA_POSE_CONFIG = (
    Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/'
         'config/camera_pose.yaml').expanduser())


def _camera_xacro_defaults() -> dict:
    """Mirror real_bringup: emit both `calibration_camera_*` and
    `camera_*` arg sets, applying camera_pose.yaml to the slot matching
    its `mode` field. The URDF's `mode` arg picks which set is used.
    """
    cal_cam = {
        'calibration_camera_x':     '-1.5',
        'calibration_camera_y':      '0.0',
        'calibration_camera_z':      '0.6',
        'calibration_camera_roll':   '0.0',
        'calibration_camera_pitch':  '0.0',
        'calibration_camera_yaw':    '0.0',
    }
    on_robot_cam = {
        'camera_x':     '0.0160004150359285',
        'camera_y':     '0.0',
        'camera_z':     '0.0',
        'camera_roll':  '3.14159',
        'camera_pitch': '0.0',
        'camera_yaw':   '0.0',
    }
    if not _CAMERA_POSE_CONFIG.exists():
        return {**cal_cam, **on_robot_cam}
    try:
        with _CAMERA_POSE_CONFIG.open() as f:
            cfg = yaml.safe_load(f) or {}
        for key in ('mode', 'parent_frame', 'child_frame', 'xyz', 'rpy'):
            if key not in cfg:
                raise ValueError(f'missing key {key!r}')
        for key in ('xyz', 'rpy'):
            for v in cfg[key]:
                if not math.isfinite(float(v)):
                    raise ValueError(
                        f'{key} contains non-finite value {v!r}')
    except Exception as exc:
        print(f'[sim_bringup] WARNING: ignoring {_CAMERA_POSE_CONFIG} ({exc})')
        return {**cal_cam, **on_robot_cam}
    xyz, rpy = cfg['xyz'], cfg['rpy']
    print(f'[sim_bringup] applying camera_pose.yaml '
          f'(mode={cfg["mode"]}, xyz={xyz}, rpy={rpy})')
    if cfg['mode'] == 'calibration_stand':
        cal_cam = {
            'calibration_camera_x':     f'{xyz[0]:.9f}',
            'calibration_camera_y':     f'{xyz[1]:.9f}',
            'calibration_camera_z':     f'{xyz[2]:.9f}',
            'calibration_camera_roll':  f'{rpy[0]:.9f}',
            'calibration_camera_pitch': f'{rpy[1]:.9f}',
            'calibration_camera_yaw':   f'{rpy[2]:.9f}',
        }
    elif cfg['mode'] == 'on_robot_mount':
        on_robot_cam = {
            'camera_x':     f'{xyz[0]:.9f}',
            'camera_y':     f'{xyz[1]:.9f}',
            'camera_z':     f'{xyz[2]:.9f}',
            'camera_roll':  f'{rpy[0]:.9f}',
            'camera_pitch': f'{rpy[1]:.9f}',
            'camera_yaw':   f'{rpy[2]:.9f}',
        }
    return {**cal_cam, **on_robot_cam}


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation time",
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

    # The (mode, calibration) tuple covers four configurations:
    #   mode=work, calibration=false   regular sim, no markers, no dashboard
    #   mode=work, calibration=true    on-robot camera, EE marker, calibrate camera_joint
    #   mode=tests, calibration=false  stand camera, both markers, full test runner
    #   mode=tests, calibration=true   stand camera, EE marker, calibrate calibration_camera_joint
    calibration_arg = DeclareLaunchArgument(
        "calibration",
        default_value="false",
        choices=["true", "false"],
        description="Open the calibration dashboard with only the camera-"
                    "pose calibration UI exposed.",
    )

    # Default world depends on (mode, calibration). Tests-mode runs use
    # a stripped-down world without lab clutter so the apriltags are
    # unobstructed; work-mode keeps the full lab world.
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value=PythonExpression([
            "'calibration' if '",
            LaunchConfiguration("mode"),
            "' == 'tests' else 'lab'",
        ]),
        description="Gazebo world name (without .sdf extension); "
                    "defaults to 'calibration' when mode:=tests, else 'lab'",
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

    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="traj",
        choices=["traj", "policy", "all"],
        description="Which controller(s) to load",
    )

    tag_size_arg = DeclareLaunchArgument(
        "tag_size",
        default_value="0.064",
        description="AprilTag edge length [m]",
    )

    pointcloud_arg = DeclareLaunchArgument(
        "pointcloud",
        default_value="true",
        choices=["true", "false"],
        description="Bridge the depth pointcloud topic from Gazebo",
    )

    marker_world_rpy_arg = DeclareLaunchArgument(
        "marker_world_rpy",
        default_value="",
        description="Override marker world-orientation prior used by "
                    "the EE-sweep calibration. Format 'r,p,y' (rad), "
                    "or empty to read from URDF at run start.",
    )

    _cam_defaults = _camera_xacro_defaults()
    cal_cam_x_arg = DeclareLaunchArgument(
        "calibration_camera_x", default_value=_cam_defaults['calibration_camera_x'])
    cal_cam_y_arg = DeclareLaunchArgument(
        "calibration_camera_y", default_value=_cam_defaults['calibration_camera_y'])
    cal_cam_z_arg = DeclareLaunchArgument(
        "calibration_camera_z", default_value=_cam_defaults['calibration_camera_z'])
    cal_cam_roll_arg = DeclareLaunchArgument(
        "calibration_camera_roll", default_value=_cam_defaults['calibration_camera_roll'])
    cal_cam_pitch_arg = DeclareLaunchArgument(
        "calibration_camera_pitch", default_value=_cam_defaults['calibration_camera_pitch'])
    cal_cam_yaw_arg = DeclareLaunchArgument(
        "calibration_camera_yaw", default_value=_cam_defaults['calibration_camera_yaw'])
    cam_x_arg = DeclareLaunchArgument(
        "camera_x", default_value=_cam_defaults['camera_x'])
    cam_y_arg = DeclareLaunchArgument(
        "camera_y", default_value=_cam_defaults['camera_y'])
    cam_z_arg = DeclareLaunchArgument(
        "camera_z", default_value=_cam_defaults['camera_z'])
    cam_roll_arg = DeclareLaunchArgument(
        "camera_roll", default_value=_cam_defaults['camera_roll'])
    cam_pitch_arg = DeclareLaunchArgument(
        "camera_pitch", default_value=_cam_defaults['camera_pitch'])
    cam_yaw_arg = DeclareLaunchArgument(
        "camera_yaw", default_value=_cam_defaults['camera_yaw'])

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

    volcaniarm_description_share = get_package_share_directory("volcaniarm_description")
    volcaniarm_controller_share = get_package_share_directory("volcaniarm_controller")
    volcaniarm_motion_share = get_package_share_directory("volcaniarm_motion")
    volcaniarm_calibration_share = get_package_share_directory("volcaniarm_calibration")

    # Gazebo launch — passes `mode`, `calibration`, and the full camera
    # xacro arg surface through so the URDF emits the right joints.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_description_share, "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("world_name", LaunchConfiguration("world_name")),
            ("mode", LaunchConfiguration("mode")),
            ("calibration", LaunchConfiguration("calibration")),
            ("camera_mount_x", LaunchConfiguration("camera_mount_x")),
            ("camera_mount_pitch", LaunchConfiguration("camera_mount_pitch")),
            ("controller", LaunchConfiguration("controller")),
            ("tag_size", LaunchConfiguration("tag_size")),
            ("calibration_camera_x", LaunchConfiguration("calibration_camera_x")),
            ("calibration_camera_y", LaunchConfiguration("calibration_camera_y")),
            ("calibration_camera_z", LaunchConfiguration("calibration_camera_z")),
            ("calibration_camera_roll", LaunchConfiguration("calibration_camera_roll")),
            ("calibration_camera_pitch", LaunchConfiguration("calibration_camera_pitch")),
            ("calibration_camera_yaw", LaunchConfiguration("calibration_camera_yaw")),
            ("camera_x", LaunchConfiguration("camera_x")),
            ("camera_y", LaunchConfiguration("camera_y")),
            ("camera_z", LaunchConfiguration("camera_z")),
            ("camera_roll", LaunchConfiguration("camera_roll")),
            ("camera_pitch", LaunchConfiguration("camera_pitch")),
            ("camera_yaw", LaunchConfiguration("camera_yaw")),
        ],
        condition=is_gazebo,
    )

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

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcaniarm_controller_share, "launch", "controller.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
        condition=is_traj_active,
    )

    rl_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcaniarm_controller_share, "launch", "rl_controller.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
        condition=is_policy_only,
    )

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

    # Display (RViz). Skipped when the calibration dashboard is up.
    show_display = IfCondition(PythonExpression([
        "'", LaunchConfiguration("calibration"), "' == 'false' and ",
        "'", LaunchConfiguration("mode"), "' != 'tests'",
    ]))
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
        condition=show_display,
    )

    # Calibration dashboard activated when calibration:=true (any mode)
    # OR when mode=tests with calibration:=false (the standard test
    # workflow). The `camera_calibration_only` arg restricts the dashboard
    # to just the camera-localization group when calibration:=true.
    show_dashboard = IfCondition(PythonExpression([
        "'", LaunchConfiguration("calibration"), "' == 'true' or ",
        "'", LaunchConfiguration("mode"), "' == 'tests'",
    ]))
    calibration_dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_calibration_share,
                "launch",
                "dashboard.launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("tag_size", LaunchConfiguration("tag_size")),
            ("camera_calibration_only", LaunchConfiguration("calibration")),
            ("marker_world_rpy", LaunchConfiguration("marker_world_rpy")),
        ],
        condition=show_dashboard,
    )

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
            # mode_arg / calibration_arg must come before world_name_arg
            # since world_name_arg's default reads `mode`.
            mode_arg,
            calibration_arg,
            world_name_arg,
            camera_mount_x_arg,
            camera_mount_pitch_arg,
            sim_arg,
            controller_arg,
            tag_size_arg,
            pointcloud_arg,
            marker_world_rpy_arg,
            cal_cam_x_arg, cal_cam_y_arg, cal_cam_z_arg,
            cal_cam_roll_arg, cal_cam_pitch_arg, cal_cam_yaw_arg,
            cam_x_arg, cam_y_arg, cam_z_arg,
            cam_roll_arg, cam_pitch_arg, cam_yaw_arg,
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
