"""Calibration dashboard stack (apriltag detector + RViz + rqt plugin).

Assumes the arm and camera are already running. This launch is normally
included by the bringup launches when `calibration:=true`, not run
standalone:

  ros2 launch volcaniarm_bringup sim_bringup.launch.py calibration:=true
  ros2 launch volcaniarm_bringup real_bringup.launch.py calibration:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_share = get_package_share_directory('volcaniarm_calibration')
    apriltag_config = os.path.join(
        calibration_share, 'config', 'apriltag_params.yaml')
    rviz_config = os.path.join(
        calibration_share, 'rviz', 'calibration_dashboard.rviz')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Open RViz with the calibration dashboard config')
    use_rqt_arg = DeclareLaunchArgument(
        'use_rqt', default_value='true',
        description='Open the rqt calibration dashboard plugin')

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[apriltag_config],
        remappings=[
            ('image_rect', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info'),
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_calibration',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

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
        use_rviz_arg,
        use_rqt_arg,
        apriltag_node,
        rviz,
        rqt,
    ])
