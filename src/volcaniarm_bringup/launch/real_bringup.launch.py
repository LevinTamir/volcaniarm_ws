import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Arguments
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

    # Controller selector. Both controllers' YAMLs are always loaded
    # into controller_manager, so runtime switching with
    # `ros2 control switch_controllers ...` works regardless of the
    # initial choice.
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="trajectory",
        choices=["trajectory", "rl"],
        description="Which controller to auto-spawn at startup",
    )
    is_trajectory = IfCondition(
        PythonExpression(["'", LaunchConfiguration("controller"), "' == 'trajectory'"])
    )
    is_rl = IfCondition(
        PythonExpression(["'", LaunchConfiguration("controller"), "' == 'rl'"])
    )

    # Get package paths
    volcaniarm_description_share = get_package_share_directory("volcaniarm_description")
    volcaniarm_controller_share = get_package_share_directory("volcaniarm_controller")

    # Load controllers config. Both files are passed so both controller
    # types are registered with controller_manager and runtime switching
    # is possible.
    controller_config_file = os.path.join(
        volcaniarm_controller_share,
        "config",
        "volcaniarm_controllers.yaml",
    )
    rl_controller_config_file = os.path.join(
        volcaniarm_controller_share,
        "config",
        "volcaniarm_rl_controller.yaml",
    )

    # Robot description with real hardware (use_sim=false)
    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            os.path.join(
                volcaniarm_description_share,
                "urdf",
                "volcaniarm.urdf.xacro",
            ),
            " use_sim:=false",
        ]),
        value_type=str,
    )

    # Robot state publisher
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

    # Controller manager node (runs on PC, not in Gazebo)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            controller_config_file,
            rl_controller_config_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    # Trajectory-controller launch (spawns joint_state_broadcaster +
    # volcaniarm_controller). Only included when controller:=trajectory.
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
        condition=is_trajectory,
    )

    # RL-controller launch (spawns joint_state_broadcaster +
    # volcaniarm_rl_controller). Only included when controller:=rl.
    rl_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                volcaniarm_controller_share,
                "launch",
                "rl_controller.launch.py",
            )
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
        condition=is_rl,
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

    # RealSense camera launch (D435i)
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
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
            'publish_tf': 'false',  # Disable - URDF handles all TF
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            serial_port_arg,
            controller_arg,
            robot_state_publisher,
            controller_manager,
            display_launch,
            controller_launch,
            rl_controller_launch,
            realsense_camera,
        ]
    )
