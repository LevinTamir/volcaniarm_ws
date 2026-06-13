cmake_minimum_required(VERSION 3.8)
project(volcaniarm_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(onnxruntime_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(volcaniarm_kinematics REQUIRED)

# Passive-joint broadcaster: solves the closed-loop passives from the actuated
# elbows (via volcaniarm_kinematics) and publishes the full joint state.
add_library(passive_broadcaster SHARED
  src/passive_broadcaster.cpp
)
target_include_directories(passive_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(passive_broadcaster
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  visualization_msgs
  geometry_msgs
  urdf
  volcaniarm_kinematics
)
pluginlib_export_plugin_description_file(
  controller_interface passive_broadcaster.xml
)

# RL policy controller library (runs Isaac-Lab-trained ONNX policy)
add_library(rl_policy_controller SHARED
  src/rl_policy_controller.cpp
)
target_include_directories(rl_policy_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rl_policy_controller
  ament_index_cpp
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
)
target_link_libraries(rl_policy_controller onnxruntime::onnxruntime)
pluginlib_export_plugin_description_file(
  controller_interface rl_policy_controller.xml
)

install(
  TARGETS passive_broadcaster rl_policy_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  volcaniarm_controllers/joystick_teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(passive_broadcaster rl_policy_controller)
ament_export_include_directories(include)
ament_export_dependencies(
  ament_index_cpp
  controller_interface
  hardware_interface
  onnxruntime_vendor
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  visualization_msgs
  geometry_msgs
  urdf
  volcaniarm_kinematics
)

ament_package()
