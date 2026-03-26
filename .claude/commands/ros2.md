# ROS 2 Command Reference

## Colcon Build System
| Command | Description |
|---------|-------------|
| `colcon build --symlink-install` | Build all packages with symlinks |
| `colcon build --packages-select <pkg>` | Build specific package |
| `colcon build --packages-up-to <pkg>` | Build package and its dependencies |
| `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug` | Debug build |
| `colcon test --packages-select <pkg>` | Run tests for a package |
| `colcon test-result --verbose` | Show test results |

## Environment
| Command | Description |
|---------|-------------|
| `source install/setup.bash` | Source the workspace |
| `printenv | grep ROS` | Check ROS environment variables |
| `ros2 doctor` | Check ROS 2 system health |

## Node Introspection
| Command | Description |
|---------|-------------|
| `ros2 node list` | List active nodes |
| `ros2 node info <node>` | Show node details (pubs, subs, services) |
| `ros2 topic list -t` | List topics with types |
| `ros2 topic echo <topic>` | Print topic messages |
| `ros2 topic hz <topic>` | Show publish rate |
| `ros2 topic info <topic> --verbose` | Show topic QoS info |

## Services & Actions
| Command | Description |
|---------|-------------|
| `ros2 service list -t` | List services with types |
| `ros2 service call <srv> <type> <args>` | Call a service |
| `ros2 action list -t` | List actions with types |
| `ros2 action send_goal <action> <type> <args>` | Send action goal |

## Parameters
| Command | Description |
|---------|-------------|
| `ros2 param list <node>` | List node parameters |
| `ros2 param get <node> <param>` | Get parameter value |
| `ros2 param set <node> <param> <value>` | Set parameter value |
| `ros2 param dump <node>` | Dump all parameters as YAML |

## Debugging
| Command | Description |
|---------|-------------|
| `ros2 run rqt_graph rqt_graph` | Visualize node/topic graph |
| `ros2 run tf2_tools view_frames` | Generate TF tree PDF |
| `ros2 run tf2_ros tf2_echo <from> <to>` | Print transform between frames |
| `ros2 run rqt_console rqt_console` | View log messages |

## ros2_control (Volcaniarm-specific)
| Command | Description |
|---------|-------------|
| `ros2 control list_controllers` | List loaded controllers |
| `ros2 control list_hardware_interfaces` | List HW interfaces |
| `ros2 control list_hardware_components` | List HW components |
| `ros2 control set_controller_state <ctrl> active` | Activate controller |
