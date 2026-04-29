# Volcaniarm Workspace

ROS 2 workspace for the Volcaniarm, a 2-DOF stepper motor arm for weed detection in agriculture, as part of my Thesis.

Firmware lives in a separate repo: [volcaniarm_firmware](https://github.com/LevinTamir/volcaniarm_firmware)

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- `python3-vcstool`

## Packages

| Package | Description |
|---------|-------------|
| `volcaniarm_bringup` | Launch files (sim + real) |
| `volcaniarm_calibration` | Hand-eye calibration |
| `volcaniarm_controller` | Trajectory controller (ros2_control) |
| `volcaniarm_description` | URDF, meshes, worlds, RViz configs |
| `volcaniarm_hardware` | Hardware interface, serial to MCU (ros2_control) |
| `volcaniarm_msgs` | Custom srv/msg definitions |
| `volcaniarm_motion` | Motion planning and kinematics |
| `volcaniarm_weed_detector` | Weed detection (OpenCV + depth camera) |

Third-party / vendor packages are pulled into `src/` via [vcstool](https://github.com/dirk-thomas/vcstool) and gitignored from this repo:

| Package | Source |
|---------|--------|
| `apriltag_ros` | https://github.com/christianrauch/apriltag_ros (pinned) |
| `easy_handeye2` | https://github.com/marcoesposito1988/easy_handeye2 (pinned) |
| `onnxruntime_vendor` | https://github.com/LevinTamir/onnxruntime_vendor |

## Setup

```bash
mkdir -p <your_ws_path>/volcaniarm_ws
cd <your_ws_path>/volcaniarm_ws
git clone git@github.com:LevinTamir/volcaniarm_ws.git .

# Install vcstool and pull third-party packages
sudo apt install python3-vcstool
vcs import < third_party.repos

# Install rosdep deps
rosdep install --from-paths src --ignore-src -ry
```

## Build

```bash
colcon build --symlink-install
source install/setup.bash

# Simulation
ros2 launch volcaniarm_bringup sim_bringup.launch.py

# Real robot
ros2 launch volcaniarm_bringup real_bringup.launch.py
```

## Updating third-party packages

```bash
# Pull latest on every third-party repo
vcs pull src

# Refresh pinned versions in third_party.repos
vcs export src --exact > third_party.repos
```
