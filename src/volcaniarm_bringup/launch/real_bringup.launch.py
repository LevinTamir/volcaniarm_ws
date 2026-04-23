import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        default_value="/dev/ttyUSB0",
        description="Serial port for hardware interface",
    )

    homing_arg = DeclareLaunchArgument(
        "homing",
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

    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            os.path.join(volcaniarm_description_share, "urdf", "volcaniarm.urdf.xacro"),
            " use_sim:=false",
            " auto_home:=", LaunchConfiguration("homing"),
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

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(volcaniarm_description_share, "launch", "display.launch.py")
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
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
            'pointcloud.enable': 'false',
            'publish_tf': 'false',  # URDF handles all TF
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            serial_port_arg,
            homing_arg,
            controller_arg,
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
        ]
    )
