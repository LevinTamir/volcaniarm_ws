import math
import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


_CAMERA_POSE_CONFIG = (
    Path('~/workspaces/volcaniarm_ws/src/volcaniarm_calibration/'
         'config/camera_pose.yaml').expanduser())

# Seconds between the sim's first data message and opening RViz (both
# backends: /isaac_joint_states for Isaac, /joint_states for Gazebo). The
# first message proves the sim is playing, but it can still be loading
# assets / compiling shaders for a moment — opening RViz into that just
# shows a stuttering scene. Bump if RViz still comes up before the sim
# feels responsive on a slower machine.
RVIZ_SETTLE_SEC = 1


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

    # All modes (work and tests/calibration) use the full lab world so the
    # scene is consistent. Override with world_name:=calibration for a
    # stripped-down world if lab clutter ever obstructs the apriltags.
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="lab",
        description="Gazebo world name (without .sdf extension); defaults to 'lab' "
                    "for all modes (use world_name:=calibration to strip lab clutter)",
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
                    "(auto-launches the Isaac Sim GUI with the lab stage unless "
                    "isaac_gui:=false, in which case Isaac Sim must already be "
                    "running with the ROS2 bridge and the scene loaded)",
    )

    # Isaac Sim GUI autostart. Ctrl-C on this launch also closes Isaac Sim
    # (which takes ~1 min to boot) — pass isaac_gui:=false while iterating
    # on the ROS side to keep a running Isaac Sim alive across relaunches.
    isaac_gui_arg = DeclareLaunchArgument(
        "isaac_gui",
        default_value="true",
        choices=["true", "false"],
        description="With sim:=isaac, start the Isaac Sim app with the lab "
                    "stage playing. false = attach to an already-running Isaac Sim.",
    )
    isaac_path_arg = DeclareLaunchArgument(
        "isaac_path",
        default_value=os.path.expanduser("~/isaac/isaac-sim"),
        description="Isaac Sim install directory (contains isaac-sim.sh)",
    )
    isaac_open_script_arg = DeclareLaunchArgument(
        "isaac_open_script",
        default_value=os.path.expanduser(
            "~/projects/volcaniarm_isaaclab/scripts/open_lab_gui.py"),
        description="Kit --exec script that opens the lab USD and presses Play",
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
        description="Compose /camera/depth/color/points (XYZRGB) from the "
                    "color + aligned-depth images via the shared "
                    "depth_image_proc pipeline (same code path as real "
                    "hardware, regardless of sim backend)",
    )

    moveit_arg = DeclareLaunchArgument(
        "moveit",
        default_value="false",
        choices=["true", "false"],
        description="Launch MoveIt move_group + MotionPlanning RViz",
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
    volcaniarm_description_share = get_package_share_directory("volcaniarm_description")
    volcaniarm_controller_share = get_package_share_directory("volcaniarm_controllers")
    volcaniarm_calibration_share = get_package_share_directory("volcaniarm_calibration")
    volcaniarm_bringup_share = get_package_share_directory("volcaniarm_bringup")

    # Shared colored-pointcloud composer — the single pipeline that turns
    # color + aligned depth into /camera/depth/color/points on every
    # backend (Gazebo, Isaac, and real hardware via real_bringup).
    camera_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_bringup_share, "launch", "camera_pointcloud.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
        condition=IfCondition(LaunchConfiguration("pointcloud")),
    )

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

    # Isaac Sim GUI itself — Gazebo-parity autostart. The --exec script
    # opens the volcaniarm lab USD and presses Play, which brings up the
    # ROS2 bridge topics (/isaac_joint_states, /joint_commands, camera).
    # The ros2_control TopicBasedSystem above just idles until those
    # topics appear (~1 min boot), so start order doesn't matter.
    isaac_gui_proc = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([LaunchConfiguration("isaac_path"), "isaac-sim.sh"]),
            "--/isaac/startup/create_new_stage=false",
            "--exec", LaunchConfiguration("isaac_open_script"),
        ],
        name="isaac_sim",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("sim"), "' == 'isaac' and ",
            "'", LaunchConfiguration("isaac_gui"), "' == 'true'",
        ])),
    )

    # Controller spawners. Everything below needs the controller_manager's
    # update loop to actually be running, and with use_sim_time:=true that
    # loop is driven by /clock — which Isaac only publishes once loaded and
    # playing. Spawning against a frozen clock times out the activate call
    # and strands the controller `inactive`. So these actions are built by
    # a factory: instantiated once for the gazebo path (immediate — Gazebo
    # publishes /clock within seconds) and once for the isaac path, where
    # they run only after the readiness waiter below sees real bridge data.
    def _controller_actions(gate):
        # gate: extra condition term ANDed onto each action's own condition.
        def cond(expr):
            return IfCondition(PythonExpression(expr + [" and ", *gate]))

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        volcaniarm_controller_share, "launch", "controller.launch.py")
                ),
                launch_arguments=[
                    ("use_sim_time", LaunchConfiguration("use_sim_time"))],
                condition=cond(
                    ["'", LaunchConfiguration("controller"), "' in ('traj', 'all')"]),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        volcaniarm_controller_share, "launch", "rl_controller.launch.py")
                ),
                launch_arguments=[
                    ("use_sim_time", LaunchConfiguration("use_sim_time"))],
                condition=cond(
                    ["'", LaunchConfiguration("controller"), "' == 'policy'"]),
            ),
            # Vision policy sub-launch (JSB + vision policy active).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        volcaniarm_controller_share,
                        "launch", "rl_vision_controller.launch.py")
                ),
                launch_arguments=[
                    ("use_sim_time", LaunchConfiguration("use_sim_time"))],
                condition=cond(
                    ["'", LaunchConfiguration("controller"), "' == 'vision_policy'"]),
            ),
            # For `all`: load the policy controller inactive so it can be
            # claimed later.
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "volcaniarm_rl_controller",
                    "--controller-manager", "/controller_manager",
                    "--inactive",
                ],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                output="screen",
                condition=cond(
                    ["'", LaunchConfiguration("controller"), "' == 'all'"]),
            ),
        ]

    gazebo_controller_actions = _controller_actions(
        ["'", LaunchConfiguration("sim"), "' == 'gazebo'"])
    # The isaac set carries no sim== term — it is only ever fired from the
    # isaac-only waiter's exit handler below.
    isaac_controller_actions = _controller_actions(["True"])

    # Display (RViz). Skipped when the calibration dashboard is up, or when
    # moveit:=true (the MoveIt MotionPlanning RViz replaces the plain display).
    _show_display_expr = [
        "'", LaunchConfiguration("calibration"), "' == 'false' and ",
        "'", LaunchConfiguration("mode"), "' != 'tests' and ",
        "'", LaunchConfiguration("moveit"), "' == 'false'",
    ]
    # Either RViz flavor wants the sim publishing before it opens, so the
    # readiness waiters below run for the plain display OR the MoveIt one.
    _wants_rviz_expr = [
        "(", *_show_display_expr,
        ") or '", LaunchConfiguration("moveit"), "' == 'true'",
    ]

    # MoveIt MotionPlanning RViz — instantiated per readiness handler below
    # (one for each sim backend) so it opens against live sim data with the
    # same settle delay as the plain display.
    volcaniarm_moveit_share = get_package_share_directory("volcaniarm_moveit_config")

    def _moveit_rviz_include():
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(volcaniarm_moveit_share, "launch", "moveit_rviz.launch.py")
            ),
            launch_arguments=[
                ("use_sim_time", LaunchConfiguration("use_sim_time"))],
            condition=IfCondition(LaunchConfiguration("moveit")),
        )
    _display_source = PythonLaunchDescriptionSource(
        os.path.join(
            volcaniarm_description_share,
            "launch",
            "display.launch.py",
        )
    )
    _display_args = [
        ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ("controller", LaunchConfiguration("controller")),
    ]
    # Gazebo boots in seconds, but the same sim-first-then-RViz order as
    # the isaac path still applies: wait for the first real /joint_states
    # message (broadcaster active → controllers spawned → Gazebo running),
    # then open RViz after the shared settle delay.
    gazebo_ready_waiter = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "echo '[sim_bringup] waiting for Gazebo to publish /joint_states...'; "
            "until timeout 5 ros2 topic echo /joint_states --once >/dev/null 2>&1; "
            "do :; done; "
            "echo '[sim_bringup] Gazebo is publishing — "
            "RViz in {}s'".format(RVIZ_SETTLE_SEC),
        ],
        name="gazebo_ready_waiter",
        output="screen",
        condition=IfCondition(PythonExpression(
            ["(", *_wants_rviz_expr, ") and '",
             LaunchConfiguration("sim"), "' == 'gazebo'"]
        )),
    )
    display_after_gazebo = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo_ready_waiter,
            on_exit=[TimerAction(
                period=float(RVIZ_SETTLE_SEC),
                actions=[
                    IncludeLaunchDescription(
                        _display_source,
                        launch_arguments=_display_args,
                        condition=IfCondition(
                            PythonExpression(_show_display_expr)),
                    ),
                    _moveit_rviz_include(),
                ],
            )],
        )
    )
    # Isaac Sim takes ~1 min to boot. Anything that needs a *live* sim —
    # controller spawners (their activate call needs the /clock-driven
    # controller_manager update loop to be running) and RViz (needs TF) —
    # is gated on the bridge actually *delivering data*: a throwaway
    # waiter blocks until a /isaac_joint_states message arrives, and the
    # gated actions fire on its exit. (`ros2 topic list` is NOT a valid
    # readiness signal here — the ros2_control TopicBasedSystem subscribes
    # to /isaac_joint_states at startup, which already makes the name
    # appear in the graph.) With isaac_gui:=false against an already-
    # running Isaac the first message lands within one cycle, so this
    # degrades to a no-delay start.
    isaac_ready_waiter = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "echo '[sim_bringup] waiting for Isaac Sim to publish /isaac_joint_states...'; "
            "until timeout 5 ros2 topic echo /isaac_joint_states --once >/dev/null 2>&1; "
            "do :; done; "
            "echo '[sim_bringup] Isaac Sim bridge is publishing — "
            "spawning controllers, RViz in {}s'".format(RVIZ_SETTLE_SEC),
        ],
        name="isaac_ready_waiter",
        output="screen",
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("sim"), "' == 'isaac'"])),
    )
    isaac_gated_actions = RegisterEventHandler(
        OnProcessExit(
            target_action=isaac_ready_waiter,
            on_exit=[
                *isaac_controller_actions,
                # Extra settle margin: first bridge message ≠ Isaac fully
                # responsive (asset loading / shader compile can still be
                # in flight). RViz is pure display, so err on the side of
                # opening late rather than against a stuttering sim.
                TimerAction(
                    period=float(RVIZ_SETTLE_SEC),
                    actions=[
                        IncludeLaunchDescription(
                            _display_source,
                            launch_arguments=_display_args,
                            condition=IfCondition(
                                PythonExpression(_show_display_expr)),
                        ),
                        _moveit_rviz_include(),
                    ],
                ),
            ],
        )
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

    # MoveIt (opt-in): move_group, reusing the running robot_state_publisher,
    # JTC and passive broadcaster. The MotionPlanning RViz is NOT started
    # here — it fires from the per-backend readiness handlers above, after
    # the sim publishes real data plus the shared settle delay, same as the
    # plain display.
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcaniarm_moveit_share, "launch", "move_group.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
        condition=IfCondition(LaunchConfiguration("moveit")),
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
            isaac_gui_arg,
            isaac_path_arg,
            isaac_open_script_arg,
            controller_arg,
            tag_size_arg,
            pointcloud_arg,
            moveit_arg,
            marker_world_rpy_arg,
            cal_cam_x_arg, cal_cam_y_arg, cal_cam_z_arg,
            cal_cam_roll_arg, cal_cam_pitch_arg, cal_cam_yaw_arg,
            cam_x_arg, cam_y_arg, cam_z_arg,
            cam_roll_arg, cam_pitch_arg, cam_yaw_arg,
            gazebo_launch,
            isaac_launch,
            isaac_gui_proc,
            camera_pointcloud_launch,
            *gazebo_controller_actions,
            gazebo_ready_waiter,
            display_after_gazebo,
            isaac_ready_waiter,
            isaac_gated_actions,
            weed_targeting_launch,
            calibration_dashboard,
            move_group_launch,
        ]
    )
