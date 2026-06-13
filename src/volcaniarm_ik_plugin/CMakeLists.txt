cmake_minimum_required(VERSION 3.8)
project(volcaniarm_ik_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(volcaniarm_kinematics REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/volcaniarm_ik_plugin.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  moveit_core
  pluginlib
  rclcpp
  geometry_msgs
  moveit_msgs
  tf2_eigen
  volcaniarm_kinematics
)

# Register as a MoveIt kinematics plugin (base package: moveit_core).
pluginlib_export_plugin_description_file(moveit_core volcaniarm_ik_plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES volcaniarm_ik_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(
  moveit_core
  pluginlib
  rclcpp
  geometry_msgs
  moveit_msgs
  tf2_eigen
  volcaniarm_kinematics
)

ament_package()
