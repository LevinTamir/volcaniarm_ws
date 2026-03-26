# Volcaniarm Workspace — Claude Code Project Guide

## Project Overview
This is a **ROS 2 Jazzy** workspace for the **Volcaniarm** — a dual stepper motor robotic arm used for agricultural weed detection research (thesis project). The system combines computer vision (weed detection via depth camera) with motion planning and real-time motor control.

**ROS distro:** Jazzy | **Python:** 3.12 | **Platform:** Ubuntu 24.04

## Workspace Layout

```
volcaniarm_ws/
├── src/
│   ├── volcaniarm_bringup/        # Launch files (sim + real)
│   ├── volcaniarm_controller/     # C++ — closed-loop trajectory controller (ros2_control)
│   ├── volcaniarm_description/    # URDF/Xacro, meshes, Gazebo worlds, RViz configs
│   ├── volcaniarm_hardware/       # C++ — ros2_control hardware interface (dual stepper)
│   ├── volcaniarm_interfaces/     # Custom srv/msg definitions
│   ├── volcaniarm_motion/         # Python — motion planning + kinematics
│   └── volcaniarm_weed_detector/  # Python — weed detection node (CV, depth camera)
├── volcaniarm_firmware/           # PlatformIO — microcontroller firmware (COLCON_IGNORE)
├── build/ install/ log/           # Colcon artifacts (gitignored)
└── .claude/                       # Claude Code config
```

## Key Commands

```bash
# Build
colcon build --symlink-install
colcon build --symlink-install --packages-select <pkg>

# Source
source install/setup.bash

# Launch
ros2 launch volcaniarm_bringup sim_bringup.launch.py    # simulation
ros2 launch volcaniarm_bringup real_bringup.launch.py   # real robot

# Introspect
ros2 topic list / ros2 topic echo <topic>
ros2 service list / ros2 service call <srv> <type> <args>
ros2 node list / ros2 node info <node>
ros2 param list / ros2 param get <node> <param>
```

## Architecture Notes

- **Hardware interface** (`volcaniarm_hardware`): C++ plugin for ros2_control. Communicates with PlatformIO firmware over serial.
- **Controller** (`volcaniarm_controller`): C++ custom controller using ros2_control controller_interface. Implements closed-loop trajectory control.
- **Motion planning** (`volcaniarm_motion`): Python node with forward/inverse kinematics. Sends joint trajectories to the controller.
- **Weed detector** (`volcaniarm_weed_detector`): Python node using OpenCV + depth camera (cv_bridge, message_filters). Publishes weed positions.
- **Firmware** (`volcaniarm_firmware`): PlatformIO project — NOT built by colcon. Runs on the motor controller MCU.

## Conventions

- Python packages use `ament_python` build type with `setup.py`
- C++ packages use `ament_cmake` build type with `CMakeLists.txt`
- Launch files are Python-based (`.launch.py`)
- Parameters live in `config/*.yaml` files within each package
- Custom interfaces go in `volcaniarm_interfaces`

## Working With This Repo

- **Current branch:** Check `git branch` — active development happens on feature branches, merged to `main`
- **Before editing C++ code:** Understand the ros2_control plugin architecture (hardware_interface::SystemInterface, controller_interface::ControllerInterface)
- **Before editing motion planning:** Check `volcaniarm_motion/config/` for calibration offsets and kinematic parameters
- **Firmware changes** require PlatformIO — they don't affect the ROS workspace build

## Available Skills
- `/ros2` — ROS 2 command reference and cheat sheet

## Rules
Claude Code rules for this project are in `.claude/rules/`. They cover:
- ROS 2 general conventions (package structure, naming, launch files)
- ROS 2 node patterns (lifecycle, publishers, subscribers)
- ROS 2 communication (topics, services, QoS profiles, TF2)
- Robot-specific patterns (URDF, Nav2, sensor/motor control)
