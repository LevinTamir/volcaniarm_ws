cmake_minimum_required(VERSION 3.8)
project(volcaniarm_calibration)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (recurses into rqt/ and runner/ subpackages)
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/accuracy_test
  src/export_calibration
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install rqt plugin manifest and resources
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)
install(DIRECTORY notebooks/
  DESTINATION share/${PROJECT_NAME}/notebooks
)

ament_package()
