# Volcaniarm Workspace

ROS 2 workspace for the Volcaniarm, a 2-DOF delta type robotic arm for precision weeding in agriculture, as part of my Thesis.

<p align="center">
  <img src="docs/images/volcaniarm_urdf_img.jpeg" alt="Volcaniarm URDF" width="400">
</p>

---

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- `python3-vcstool`

## Packages

| Package | Description |
|---------|-------------|
| `volcaniarm_bringup` | Launch files (sim + real) |
| `volcaniarm_calibration` | Hand-eye calibration |
| `volcaniarm_controller` | Trajectory & Policy-based controllers (ros2_control) |
| `volcaniarm_description` | URDF, meshes, worlds, RViz configs |
| `volcaniarm_hardware` | Hardware interface, serial to MCU (ros2_control) |
| `volcaniarm_msgs` | Custom srv/msg definitions |
| `volcaniarm_motion` | Motion planning and kinematics |
| `volcaniarm_weed_detector` | Weed detection algorithm |

Third-party / vendor packages are pulled into `src/` via [vcstool](https://github.com/dirk-thomas/vcstool) and gitignored from this repo:

| Package | Source |
|---------|--------|
| `apriltag_ros` | https://github.com/christianrauch/apriltag_ros (pinned) |
| `easy_handeye2` | https://github.com/marcoesposito1988/easy_handeye2 (pinned) |
| `onnxruntime_vendor` | https://github.com/LevinTamir/onnxruntime_vendor (prebuilt ONNX Runtime, used by the policy-based controller) |

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

### Real Hardware

The hardware interface expects the ESP32 at `/dev/volcaniarm`. Plug in the ESP and run the installer once: it auto-detects the device's serial number, fills in the udev rule, and reloads udev.

```bash
cd <your_ws_path>/volcaniarm_ws
src/volcaniarm_hardware/udev/install.sh
```

To remove: `src/volcaniarm_hardware/udev/install.sh --uninstall`.

## Build

Build and source:

```bash
cd <your_ws_path>/volcaniarm_ws
colcon build --symlink-install
source install/setup.bash
```

Simulation:

```bash
ros2 launch volcaniarm_bringup sim_bringup.launch.py
```

Real robot:

```bash
ros2 launch volcaniarm_bringup real_bringup.launch.py
```

> **Note:** Common launch args (run with `--show-args` for the full list):
> - **sim**: `sim:=gazebo/isaac`, `controller:=traj/policy/all`, `world_name:=<name>`, `calibration:=true/false`
> - **real**: `controller:=traj/policy/all`, `homing:=true/false`, `calibration:=true/false`

## Updating third-party packages

```bash
# Pull latest on every third-party repo
vcs pull src

# Refresh pinned versions in third_party.repos
vcs export src --exact > third_party.repos
```

### Related repos
- Firmware: [volcaniarm_firmware](https://github.com/LevinTamir/volcaniarm_firmware)
- Isaac Lab: [volcaniarm_isaaclab](https://github.com/LevinTamir/volcaniarm_isaaclab)
