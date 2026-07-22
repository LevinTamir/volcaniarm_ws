"""Shared colored-pointcloud composer — one pipeline for every camera backend.

Publishes /camera/depth/color/points (XYZRGB) by fusing the aligned depth
image with the color image via depth_image_proc. This is the ONLY place a
pointcloud is generated in the stack: the RealSense driver's own cloud, the
Gazebo rgbd_camera native cloud, and Isaac's depth_pcl publisher are all
disabled/unused so that real hardware, Gazebo, and Isaac Sim exercise the
exact same code path.

Input contract (every backend publishes these):
    /camera/color/image_raw                     rgb8
    /camera/color/camera_info
    /camera/aligned_depth_to_color/image_raw    32FC1 [m] (sims)
                                                or 16UC1 [mm] (RealSense);
                                                depth_image_proc handles both

Included by sim_bringup.launch.py (both sims) and real_bringup.launch.py.
Requires: ros-jazzy-depth-image-proc.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="True under a simulator, False on real hardware",
    )

    container = ComposableNodeContainer(
        name="camera_pointcloud_container",
        namespace="camera",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="point_cloud_xyzrgb",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=[
                    ("rgb/image_rect_color", "/camera/color/image_raw"),
                    ("rgb/camera_info", "/camera/color/camera_info"),
                    ("depth_registered/image_rect",
                     "/camera/aligned_depth_to_color/image_raw"),
                    ("points", "/camera/depth/color/points"),
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([use_sim_time_arg, container])
