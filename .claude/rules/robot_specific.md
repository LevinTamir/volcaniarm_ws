---
description: Volcaniarm-specific robot patterns — URDF, hardware, control
globs: src/**
---

# Robot-Specific Rules

## URDF/Xacro
- Robot description lives in `volcaniarm_description/urdf/`
- Use Xacro macros for repeated structures (joints, links)
- ros2_control hardware interface is declared in the URDF via `<ros2_control>` tags
- Meshes are in `volcaniarm_description/meshes/` (STL/DAE format)

## Hardware Interface
- Dual stepper motors controlled via serial communication to MCU
- The `volcaniarm_hardware` package implements `hardware_interface::SystemInterface`
- State interfaces: joint positions (and optionally velocities)
- Command interfaces: joint position commands
- Serial communication happens in `read()` and `write()` — keep these non-blocking

## Motor Control Pipeline
```
Motion Planning → JointTrajectory → Controller → Command Interfaces → Hardware Interface → Serial → MCU → Steppers
```

## Gazebo Simulation
- World files in `volcaniarm_description/worlds/`
- Use `small_world.sdf` for faster simulation
- `sim_bringup.launch.py` spawns the robot in Gazebo with simulated controllers
- Simulated hardware uses `gz_ros2_control` plugin (not the real hardware interface)

## Weed Detection Pipeline
```
Depth Camera → Image Topics → weed_3d_detector_node → Weed Positions (3D) → Motion Planning
```
- Uses `cv_bridge` for ROS ↔ OpenCV conversion
- Uses `message_filters` for time-synchronized depth + color images

## Calibration
- Kinematic calibration offsets are in `volcaniarm_motion/config/`
- These are critical — always preserve existing calibration values when editing config files
- Joint limits and physical parameters are defined in the URDF
