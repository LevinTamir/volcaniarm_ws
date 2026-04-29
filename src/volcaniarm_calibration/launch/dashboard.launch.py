"""Launch the calibration dashboard.

Brings up the arm (sim or real), the AprilTag detector, RViz with the
preview markers visible, and rqt_gui with the calibration dashboard
plugin pre-loaded.

Usage:
  ros2 launch volcaniarm_calibration dashboard.launch.py            # sim
  ros2 launch volcaniarm_calibration dashboard.launch.py sim:=false # real
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibration_share = get_package_share_directory('volcaniarm_calibration')
    bringup_share = get_package_share_directory('volcaniarm_bringup')

    apriltag_config = os.path.join(calibration_share, 'config', 'apriltag_params.yaml')
    rviz_config = os.path.join(
        calibration_share, 'resource', 'calibration_dashboard.rviz')

    sim_arg = DeclareLaunchArgument(
        'sim', default_value='true',
        description='Use simulation (true) or real hardware (false)')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Open RViz on launch')
    use_rqt_arg = DeclareLaunchArgument(
        'use_rqt', default_value='true',
        description='Open the rqt dashboard plugin on launch')

    # --- Sim path: calibration_bringup includes sim_bringup with calibration:=true.
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'calibration_bringup.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    # --- Real path: real_bringup + RealSense camera. AprilTag detector is
    # launched separately below so it works in either mode.
    real_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'real_bringup.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('sim')),
    )
    real_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'),
            '/launch/rs_launch.py',
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'rgb_camera.color_profile': '1280x720x30',
            'pointcloud.enable': 'false',
            'initial_reset': 'false',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('sim')),
    )

    # --- AprilTag detector (always on; image topic differs by mode).
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[apriltag_config],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        output='screen',
    )

    # --- Visualization.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_calibration',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # rqt_gui --standalone <plugin>: spawns just the dashboard widget.
    # The plugin is referenced by the qtgui label declared in plugin.xml.
    rqt = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_calibration_dashboard',
        arguments=[
            '--force-discover',
            '--standalone',
            'volcaniarm_calibration.rqt.calibration_dashboard_plugin.CalibrationDashboardPlugin',
        ],
        condition=IfCondition(LaunchConfiguration('use_rqt')),
    )

    return LaunchDescription([
        sim_arg,
        use_rviz_arg,
        use_rqt_arg,
        sim_bringup,
        real_bringup,
        real_camera,
        apriltag_node,
        rviz,
        rqt,
    ])
