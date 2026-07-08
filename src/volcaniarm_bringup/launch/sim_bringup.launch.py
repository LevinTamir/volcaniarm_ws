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

    # Calibration is real-hardware only now; sim always runs the camera on
    # the robot (work configuration). The calibration GUI + AprilTag
    # detector live in real_bringup + calibration_gui.launch.py.
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="lab",
        description="Gazebo world name (without .sdf extension); "
                    "defaults to the full lab world.",
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
    #   traj          → only trajectory controller loaded + active (default)
    #   policy        → only state-based RL policy controller loaded + active
    #   vision_policy → only vision RL policy controller loaded + active
    #                   (subscribes to /camera/color/image_raw, runs the
    #                    bundled ResNet18+actor ONNX exported from the
    #                    Volcaniarm-Reach-Vision-v0 task)
    #   all           → trajectory + state-based RL loaded; trajectory active,
    #                   policy inactive (claim via `ros2 control switch_controllers`)
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="traj",
        choices=["traj", "policy", "vision_policy", "all"],
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

    moveit_arg = DeclareLaunchArgument(
        "moveit",
        default_value="false",
        choices=["true", "false"],
        description="Launch MoveIt move_group + MotionPlanning RViz",
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
    is_vision_policy_only = IfCondition(
        PythonExpression(["'", LaunchConfiguration("controller"), "' == 'vision_policy'"])
    )
    is_all = IfCondition(
        PythonExpression(["'", LaunchConfiguration("controller"), "' == 'all'"])
    )

    volcaniarm_description_share = get_package_share_directory("volcaniarm_description")
    volcaniarm_controller_share = get_package_share_directory("volcaniarm_controllers")

    # Gazebo launch - sim always runs work configuration (camera on the
    # robot, no markers). Passes the camera xacro arg surface through so
    # the URDF emits the right joints.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_description_share, "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("world_name", LaunchConfiguration("world_name")),
            ("mode", "work"),
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

    # Vision policy sub-launch (JSB + vision policy active). Included
    # only for `vision_policy`.
    rl_vision_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_controller_share, "launch", "rl_vision_controller.launch.py"
            )
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
        condition=is_vision_policy_only,
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

    # Display (RViz). Skipped when moveit:=true (the MoveIt MotionPlanning
    # RViz replaces the plain display).
    show_display = IfCondition(PythonExpression([
        "'", LaunchConfiguration("moveit"), "' == 'false'",
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
            ("controller", LaunchConfiguration("controller")),
        ],
        condition=show_display,
    )

    # Weed-targeting behavior (formerly volcaniarm_motion/motion_planning_node,
    # now in volcaniarm_weed_detector, using the volcaniarm_kinematics binding).
    weed_targeting_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("volcaniarm_weed_detector"),
                "launch",
                "weed_targeting.launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
    )

    # MoveIt (opt-in): move_group + MotionPlanning RViz, reusing the running
    # robot_state_publisher, JTC and passive broadcaster.
    is_moveit = IfCondition(LaunchConfiguration("moveit"))
    volcaniarm_moveit_share = get_package_share_directory("volcaniarm_moveit_config")
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcaniarm_moveit_share, "launch", "move_group.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
        condition=is_moveit,
    )
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcaniarm_moveit_share, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
        condition=is_moveit,
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            world_name_arg,
            camera_mount_x_arg,
            camera_mount_pitch_arg,
            sim_arg,
            controller_arg,
            tag_size_arg,
            pointcloud_arg,
            moveit_arg,
            cal_cam_x_arg, cal_cam_y_arg, cal_cam_z_arg,
            cal_cam_roll_arg, cal_cam_pitch_arg, cal_cam_yaw_arg,
            cam_x_arg, cam_y_arg, cam_z_arg,
            cam_roll_arg, cam_pitch_arg, cam_yaw_arg,
            gazebo_launch,
            isaac_launch,
            controller_launch,
            rl_controller_launch,
            rl_vision_controller_launch,
            rl_inactive_spawner,
            display_launch,
            weed_targeting_launch,
            move_group_launch,
            moveit_rviz_launch,
        ]
    )
