# Volcaniarm Workspace

ROS 2 workspace for the Volcaniarm, a 2-DOF delta type robotic arm for precision weeding in agriculture, as part of my Thesis.

<p align="center">
  <img src="docs/images/volcaniarm_urdf_img.jpeg" alt="Volcaniarm URDF" width="400">
</p>

---

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy


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

`easy_handeye2` and `apriltag_ros` are included as git submodules. `onnxruntime_vendor` is a local vendor package wrapping the prebuilt ONNX Runtime as part of policy-based controller.

## Setup

```bash
mkdir -p <your_ws_path>/volcaniarm_ws
cd <your_ws_path>/volcaniarm_ws
git clone --recurse-submodules git@github.com:LevinTamir/volcaniarm_ws.git .
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
> - **real**: `controller:=traj/policy/all`, `homing:=true/false`

### Related repos
- Firmware: [volcaniarm_firmware](https://github.com/LevinTamir/volcaniarm_firmware)
- Isaac Lab: [volcaniarm_isaaclab](https://github.com/LevinTamir/volcaniarm_isaaclab)
