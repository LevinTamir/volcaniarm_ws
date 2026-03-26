import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_share = get_package_share_directory('volcaniarm_calibration')

    # --- Arguments ---
    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.05',
        description='ArUco marker side length in meters')

    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='0',
        description='ArUco marker ID on the end effector')

    # --- Config files ---
    aruco_config = os.path.join(calibration_share, 'config', 'aruco_params.yaml')
    calibration_config = os.path.join(calibration_share, 'config', 'calibration_params.yaml')

    # --- RealSense camera ---
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py')
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
            'publish_tf': 'false',
        }.items(),
    )

    # --- ArUco marker detector (aruco_opencv) ---
    aruco_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        parameters=[aruco_config, {
            'marker_size': LaunchConfiguration('marker_size'),
        }],
        output='screen',
    )

    # --- ArUco TF bridge (publishes marker pose as TF) ---
    aruco_tf_publisher = Node(
        package='volcaniarm_calibration',
        executable='aruco_tf_publisher',
        parameters=[calibration_config, {
            'marker_id': LaunchConfiguration('marker_id'),
        }],
        output='screen',
    )

    # --- easy_handeye2 calibration ---
    easy_handeye2_share = get_package_share_directory('easy_handeye2')
    handeye_calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(easy_handeye2_share, 'launch', 'calibrate.launch.py')
        ]),
        launch_arguments={
            'name': 'volcaniarm_calibration',
            'calibration_type': 'eye_on_base',
            'robot_base_frame': 'volcaniarm_base_link',
            'robot_effector_frame': 'right_arm_tip_link',
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': 'aruco_marker',
            'freehand_robot_movement': 'true',
        }.items(),
    )

    return LaunchDescription([
        marker_size_arg,
        marker_id_arg,
        realsense_camera,
        aruco_node,
        aruco_tf_publisher,
        handeye_calibration,
    ])
