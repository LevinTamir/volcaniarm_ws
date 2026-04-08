# Volcaniarm Workspace

ROS 2 workspace for the Volcaniarm — a dual stepper motor robotic arm for agricultural weed detection research.

## Packages

| Package | Description |
|---------|-------------|
| `volcaniarm_bringup` | Launch files for simulation and real robot |
| `volcaniarm_controller` | Closed-loop trajectory controller (ros2_control) |
| `volcaniarm_description` | URDF/Xacro, meshes, Gazebo worlds, RViz configs |
| `volcaniarm_hardware` | ros2_control hardware interface (serial to MCU) |
| `volcaniarm_interfaces` | Custom service/message definitions |
| `volcaniarm_motion` | Motion planning and kinematics |
| `volcaniarm_weed_detector` | Weed detection node (OpenCV + depth camera) |

## Quick Start

```bash
# Build
colcon build --symlink-install

# Source
source install/setup.bash

# Launch simulation
ros2 launch volcaniarm_bringup sim_bringup.launch.py

# Launch real robot
ros2 launch volcaniarm_bringup real_bringup.launch.py
```

## Requirements

- ROS 2 Jazzy
- Ubuntu 24.04
- Python 3.12
