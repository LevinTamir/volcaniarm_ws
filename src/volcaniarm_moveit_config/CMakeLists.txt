cmake_minimum_required(VERSION 3.8)
project(volcaniarm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python: the joint_state_transformer node (bridges MoveIt phantom plans to the
# real JTC via volcaniarm_kinematics).
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/joint_state_transformer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
