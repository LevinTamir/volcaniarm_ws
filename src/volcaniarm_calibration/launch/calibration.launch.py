"""Launch AprilTag detector and easy_handeye2 for hand-eye calibration.

This launch file only starts the calibration-specific nodes (apriltag + handeye).
The camera and arm should already be running via a bringup launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_share = get_package_share_directory('volcaniarm_calibration')
    easy_handeye2_share = get_package_share_directory('easy_handeye2')

    apriltag_config = os.path.join(calibration_share, 'config', 'apriltag_params.yaml')

    # -- Arguments --
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/camera/color/image_raw',
        description='Camera image topic (real: /camera/camera/..., sim: /camera/...)')

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera/camera/color/camera_info',
        description='Camera info topic')

    calibration_type_arg = DeclareLaunchArgument(
        'calibration_type', default_value='eye_on_base',
        description='eye_on_base (camera fixed, tag on EE) or eye_in_hand')

    robot_base_frame_arg = DeclareLaunchArgument(
        'robot_base_frame', default_value='volcaniarm_base_link',
        description='Robot base TF frame')

    robot_effector_frame_arg = DeclareLaunchArgument(
        'robot_effector_frame', default_value='right_arm_tip_link',
        description='Robot end-effector TF frame')

    tracking_base_frame_arg = DeclareLaunchArgument(
        'tracking_base_frame', default_value='camera_color_optical_frame',
        description='Camera TF frame')

    tracking_marker_frame_arg = DeclareLaunchArgument(
        'tracking_marker_frame', default_value='apriltag_marker',
        description='AprilTag marker TF frame (must match tag.frames in apriltag_params)')

    # -- AprilTag detector --
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[apriltag_config],
        remappings=[
            ('image_rect', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
        ],
        output='screen',
    )

    # -- easy_handeye2 calibration --
    handeye_calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(easy_handeye2_share, 'launch', 'calibrate.launch.py')
        ]),
        launch_arguments={
            'name': 'volcaniarm_calibration',
            'calibration_type': LaunchConfiguration('calibration_type'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'freehand_robot_movement': 'true',
        }.items(),
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        calibration_type_arg,
        robot_base_frame_arg,
        robot_effector_frame_arg,
        tracking_base_frame_arg,
        tracking_marker_frame_arg,
        apriltag_node,
        handeye_calibration,
    ])
