import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    volcaniarm_description = get_package_share_directory("volcaniarm_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(volcaniarm_description, "urdf", "volcaniarm.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    mode_arg = DeclareLaunchArgument(
        name="mode", default_value="work",
        choices=["work", "tests"],
        description="Physical camera configuration: 'work' mounts the "
                    "camera on the robot, 'tests' on a stand in front."
    )

    calibration_arg = DeclareLaunchArgument(
        name="calibration", default_value="false",
        description="Open the calibration dashboard with only the camera-"
                    "pose calibration UI; gates marker mounting in URDF."
    )

    # Both arg sets are emitted; the URDF's `mode` arg picks which is
    # actually used. Defaults match the URDF xacro:arg declarations and
    # are overridden by sim_bringup when it forwards camera_pose.yaml.
    cal_cam_x_arg = DeclareLaunchArgument("calibration_camera_x", default_value="-1.5")
    cal_cam_y_arg = DeclareLaunchArgument("calibration_camera_y", default_value="0.0")
    cal_cam_z_arg = DeclareLaunchArgument("calibration_camera_z", default_value="0.6")
    cal_cam_roll_arg = DeclareLaunchArgument("calibration_camera_roll", default_value="0.0")
    cal_cam_pitch_arg = DeclareLaunchArgument("calibration_camera_pitch", default_value="0.0")
    cal_cam_yaw_arg = DeclareLaunchArgument("calibration_camera_yaw", default_value="0.0")
    cam_x_arg = DeclareLaunchArgument("camera_x", default_value="0.0160004150359285")
    cam_y_arg = DeclareLaunchArgument("camera_y", default_value="0.0")
    cam_z_arg = DeclareLaunchArgument("camera_z", default_value="0.0")
    cam_roll_arg = DeclareLaunchArgument("camera_roll", default_value="3.14159")
    cam_pitch_arg = DeclareLaunchArgument("camera_pitch", default_value="0.0")
    cam_yaw_arg = DeclareLaunchArgument("camera_yaw", default_value="0.0")

    camera_mount_x_arg = DeclareLaunchArgument(
        name="camera_mount_x", default_value="0.25",
        description="Camera mount position along X (range: 0.155 to 0.655)"
    )

    camera_mount_pitch_arg = DeclareLaunchArgument(
        name="camera_mount_pitch", default_value="1.3",
        description="Camera mount pitch in radians"
    )

    controller_arg = DeclareLaunchArgument(
        name="controller",
        default_value="traj",
        choices=["traj", "policy", "vision_policy", "all"],
        description="Which controller(s) to load into gz_ros2_control. "
                    "'vision_policy' loads the image-based RL controller "
                    "(matches volcaniarm_rl_vision_controller.yaml).",
    )

    tag_size_arg = DeclareLaunchArgument(
        name="tag_size", default_value="0.064",
        description="AprilTag edge length [m]; scales the apriltag mesh "
                    "in volcaniarm_apriltag.xacro proportionally",
    )

    world_path = PathJoinSubstitution(
        [
            volcaniarm_description,
            "worlds",
            PythonExpression(
                expression=["'", LaunchConfiguration("world_name"), "'", " + '.sdf'"]
            ),
        ]
    )

    model_path = str(Path(volcaniarm_description).parent.resolve())
    model_path += pathsep + os.path.join(
        get_package_share_directory("volcaniarm_description"), "models"
    )

    gazebo_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    ros_distro = os.environ["ROS_DISTRO"]

    robot_description = ParameterValue(
        Command(
            [
                "xacro ", LaunchConfiguration("model"),
                " mode:=", LaunchConfiguration("mode"),
                " calibration:=", LaunchConfiguration("calibration"),
                " camera_mount_x:=", LaunchConfiguration("camera_mount_x"),
                " camera_mount_pitch:=", LaunchConfiguration("camera_mount_pitch"),
                " controller:=", LaunchConfiguration("controller"),
                " tag_size:=", LaunchConfiguration("tag_size"),
                " calibration_camera_x:=", LaunchConfiguration("calibration_camera_x"),
                " calibration_camera_y:=", LaunchConfiguration("calibration_camera_y"),
                " calibration_camera_z:=", LaunchConfiguration("calibration_camera_z"),
                " calibration_camera_roll:=", LaunchConfiguration("calibration_camera_roll"),
                " calibration_camera_pitch:=", LaunchConfiguration("calibration_camera_pitch"),
                " calibration_camera_yaw:=", LaunchConfiguration("calibration_camera_yaw"),
                " camera_x:=", LaunchConfiguration("camera_x"),
                " camera_y:=", LaunchConfiguration("camera_y"),
                " camera_z:=", LaunchConfiguration("camera_z"),
                " camera_roll:=", LaunchConfiguration("camera_roll"),
                " camera_pitch:=", LaunchConfiguration("camera_pitch"),
                " camera_yaw:=", LaunchConfiguration("camera_yaw"),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments={
            "gz_args": PythonExpression(["' ", world_path, " -v 4 -r'"])
        }.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "volcaniarm",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0",
            "-r",
            "0",
            "-p",
            "0",
            "-Y",
            "0",
            "-J",
            "volcaniarm_right_elbow_joint",
            "0.0",
            "-J",
            "volcaniarm_left_elbow_joint",
            "0.0",
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/rgbd_camera/depth_image_camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            ('/rgbd_camera/image', '/camera/color/image_raw'),
            ('/rgbd_camera/camera_info', '/camera/color/camera_info'),
            ('/rgbd_camera/depth_image', '/camera/aligned_depth_to_color/image_raw'),
            ('/rgbd_camera/depth_image_camera_info', '/camera/aligned_depth_to_color/camera_info'),
            ('/rgbd_camera/points', '/camera/depth/color/points'),
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            world_name_arg,
            mode_arg,
            calibration_arg,
            camera_mount_x_arg,
            camera_mount_pitch_arg,
            controller_arg,
            tag_size_arg,
            cal_cam_x_arg, cal_cam_y_arg, cal_cam_z_arg,
            cal_cam_roll_arg, cal_cam_pitch_arg, cal_cam_yaw_arg,
            cam_x_arg, cam_y_arg, cam_z_arg,
            cam_roll_arg, cam_pitch_arg, cam_yaw_arg,
            robot_state_publisher_node,
            gazebo_resource_path,
            gazebo,
            gz_spawn_entity,
            gz_ros2_bridge,
        ]
    )
