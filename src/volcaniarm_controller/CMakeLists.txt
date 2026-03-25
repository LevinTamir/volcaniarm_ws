cmake_minimum_required(VERSION 3.8)
project(volcaniarm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

# C++ closed-loop trajectory controller library
add_library(closed_loop_trajectory_controller SHARED
  src/closed_loop_trajectory_controller.cpp
)
target_include_directories(closed_loop_trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(closed_loop_trajectory_controller
  controller_interface
  joint_trajectory_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)
pluginlib_export_plugin_description_file(
  controller_interface closed_loop_trajectory_controller.xml
)

install(
  TARGETS closed_loop_trajectory_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/volcaniarm_end_effector_marker.py
  ${PROJECT_NAME}/set_initial_position.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(closed_loop_trajectory_controller)
ament_export_include_directories(include)
ament_export_dependencies(
  controller_interface
  joint_trajectory_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

ament_package()
