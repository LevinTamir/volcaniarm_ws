---
description: ROS 2 general conventions for package structure, naming, and configuration
globs: src/**
---

# ROS 2 General Rules

## Package Naming
- Use `snake_case` for package names, prefixed with `volcaniarm_`
- Node executables: `snake_case`
- Topic/service names: `snake_case` with `/` namespace separators

## C++ Package Structure (ament_cmake)
```
pkg_name/
├── package.xml
├── CMakeLists.txt
├── config/
├── launch/
├── include/pkg_name/
│   └── header.hpp
└── src/
    └── source.cpp
```

## Python Package Structure (ament_cmake_python)
```
pkg_name/
├── package.xml
├── CMakeLists.txt
├── config/              # Parameter YAML files
├── launch/              # Launch files
├── pkg_name/
│   ├── __init__.py
│   └── node_module.py
└── src/
    └── node_executable  # Thin wrapper: calls node_module:main()
```
- Uses `ament_cmake` build type with `ament_cmake_python` for Python module install
- Executable wrappers go in `src/`, installed via `install(PROGRAMS src/... DESTINATION lib/${PROJECT_NAME})`
- Do NOT use `ament_python` / `setup.py` / `resource/` marker files

## Launch Files
- Use Python launch files (`.launch.py`)
- Declare parameters via YAML files loaded in launch
- Use `LaunchConfiguration` for runtime arguments
- Group related nodes with `GroupAction` or `ComposableNodeContainer`

## Parameters
- Store defaults in `config/<node_name>_params.yaml`
- Load via launch file, not hardcoded in node source
- Use `declare_parameter()` with default values in node constructor

## Logging
- Python: `self.get_logger().info/warn/error/debug()`
- C++: `RCLCPP_INFO/WARN/ERROR/DEBUG(this->get_logger(), ...)`
- Use appropriate log levels — don't spam INFO in control loops
