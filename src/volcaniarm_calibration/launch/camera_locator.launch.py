"""Run the one-shot base-tag camera locator (tests mode).

Expects the stack to already be running (camera_link in TF,
robot_state_publisher, and apriltag_ros detecting the base tag),
typically via:
    ros2 launch volcaniarm_bringup real_bringup.launch.py mode:=tests

Solves the stand camera's world X/Y/Z from a single base-marker
detection assuming a level camera, prints it, and (by default) writes
config/camera_pose.yaml so a relaunch reflects it in RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_samples = LaunchConfiguration('num_samples')
    write_config = LaunchConfiguration('write_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_samples', default_value='20',
            description='Fresh base-tag detections to median-average.'),
        DeclareLaunchArgument(
            'write_config', default_value='true',
            description='Write the solved pose to config/camera_pose.yaml.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Set true when running against Gazebo/sim time.'),
        Node(
            package='volcaniarm_calibration',
            executable='camera_locator',
            name='camera_locator',
            output='screen',
            parameters=[{
                'num_samples': num_samples,
                'write_config': write_config,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
