"""Calibration GUI (rqt dashboard).

Second-terminal launch, MoveIt-Setup-Assistant style: bring the robot up
first (which also starts the AprilTag detector and RViz), then run this to
open the calibration GUI on top of it.

  # terminal 1 -- robot + camera + apriltag detector + RViz + TF
  ros2 launch volcaniarm_bringup real_bringup.launch.py mode:=tests calibration:=true

  # terminal 2 -- calibration GUI
  ros2 launch volcaniarm_calibration calibration_gui.launch.py

The GUI exposes camera localization and every accuracy/repeatability/
workspace-coverage test through its left sidebar. RViz is not started here;
it comes up with the robot bringup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Operator override for the marker world-orientation prior used by the
    # EE-sweep calibration solver. Format: comma-separated rpy in radians,
    # e.g. `marker_world_rpy:=0.0,0.0,0.0` for "marker face up, image axes
    # aligned with world". Empty string => use URDF lookup at run start.
    marker_world_rpy_arg = DeclareLaunchArgument(
        'marker_world_rpy', default_value='',
        description='Override the marker world-orientation prior used by '
                    'the EE-sweep calibration. Format: "r,p,y" (radians, '
                    'extrinsic XYZ Euler) of apriltag_marker_ee in world. '
                    'Empty -> read from URDF at run start.')

    rqt = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_calibration_dashboard',
        arguments=[
            '--force-discover',
            '--standalone',
            'volcaniarm_calibration.rqt.calibration_dashboard_plugin.CalibrationDashboardPlugin',
        ],
        parameters=[{
            'marker_world_rpy': LaunchConfiguration('marker_world_rpy'),
        }],
    )

    return LaunchDescription([
        marker_world_rpy_arg,
        rqt,
    ])
