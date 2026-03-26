---
description: ROS 2 node implementation patterns and best practices
globs: src/**/*.py, src/**/*.cpp, src/**/*.hpp
---

# ROS 2 Node Rules

## Python Node Pattern
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # 1. Declare parameters
        self.declare_parameter('param_name', default_value)
        # 2. Create publishers/subscribers
        self.pub = self.create_publisher(MsgType, 'topic', 10)
        self.sub = self.create_subscription(MsgType, 'topic', self.callback, 10)
        # 3. Create timers
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## C++ Node Pattern
```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("node_name") {
        // Declare parameters, create pubs/subs/timers
    }
private:
    // Callbacks, members
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```

## ros2_control Nodes (Volcaniarm-specific)

### Hardware Interface
- Extends `hardware_interface::SystemInterface`
- Implements: `on_init`, `export_state_interfaces`, `export_command_interfaces`, `read`, `write`
- `read()` and `write()` are called in the real-time control loop — keep them fast, no blocking I/O

### Controller
- Extends `controller_interface::ControllerInterface`
- Implements: `on_init`, `command_interface_configuration`, `state_interface_configuration`, `on_activate`, `on_deactivate`, `update`
- `update()` is the real-time loop — no allocations, no blocking calls

## Best Practices
- Always call `declare_parameter()` before `get_parameter()`
- Use QoS profiles appropriate for the data type (see ros2_communication rules)
- Handle node shutdown gracefully — clean up resources
- Prefer composition over inheritance for mixing capabilities
