"""Launch hand-eye calibration using AprilTag and easy_handeye2.

Starts the RealSense camera, AprilTag detector (which publishes TF directly),
and the easy_handeye2 calibration GUI.
The arm must already be running (bringup launched separately).
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_share = get_package_share_directory('volcaniarm_calibration')
    easy_handeye2_share = get_package_share_directory('easy_handeye2')

    # Load realsense config
    rs_config_path = os.path.join(calibration_share, 'config', 'realsense_params.yaml')
    with open(rs_config_path, 'r') as f:
        rs_args = yaml.safe_load(f)['realsense']

    apriltag_config = os.path.join(calibration_share, 'config', 'apriltag_params.yaml')

    # -- Arguments --
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

    # -- RealSense camera --
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py')
        ]),
        launch_arguments=rs_args.items(),
    )

    # -- AprilTag detector (publishes TF: camera_optical -> apriltag_marker) --
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
        calibration_type_arg,
        robot_base_frame_arg,
        robot_effector_frame_arg,
        tracking_base_frame_arg,
        tracking_marker_frame_arg,
        realsense_camera,
        apriltag_node,
        handeye_calibration,
    ])
