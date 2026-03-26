---
description: ROS 2 communication patterns — topics, services, QoS, TF2
globs: src/**/*.py, src/**/*.cpp, src/**/*.hpp, src/**/*.msg, src/**/*.srv, src/**/*.action
---

# ROS 2 Communication Rules

## Topic Naming
- Use descriptive, namespaced names: `/volcaniarm/joint_states`, `/weed_detector/detections`
- Sensor data: `/sensor_name/data_type` (e.g., `/camera/depth/image_raw`)
- Commands: `/controller/command`

## QoS Profiles
Choose QoS based on data semantics:

| Use Case | Reliability | Durability | History |
|----------|-------------|------------|---------|
| Sensor data | BEST_EFFORT | VOLATILE | KEEP_LAST(5) |
| Control commands | RELIABLE | VOLATILE | KEEP_LAST(1) |
| State/status | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST(1) |
| Parameters/config | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST(1) |

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5
)
```

## Custom Interfaces
- Define in `volcaniarm_interfaces` package
- Messages: `msg/MyMessage.msg`
- Services: `srv/MyService.srv`
- Actions: `action/MyAction.action`
- Always add interface dependencies to both `package.xml` and `CMakeLists.txt`

## TF2 Transforms
- Follow REP-105 frame conventions: `base_link`, `odom`, `map`
- Static transforms go in URDF or a static broadcaster
- Dynamic transforms published by the hardware interface or state estimator
- Use `tf2_ros::Buffer` + `TransformListener` to look up transforms
