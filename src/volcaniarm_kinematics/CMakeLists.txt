cmake_minimum_required(VERSION 3.8)
project(volcaniarm_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# The core library has no ROS runtime deps. The optional Python module below
# links pybind only; the C++ library and its downstream linkers stay ROS-free.
find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/five_bar.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Python bindings so ROS Python nodes use this one source of truth for IK/FK.
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
pybind11_add_module(volcaniarm_kinematics_py src/python_bindings.cpp)
target_link_libraries(volcaniarm_kinematics_py PRIVATE ${PROJECT_NAME})
# colcon/ament adds <prefix>/lib/pythonX.Y/site-packages to PYTHONPATH.
set(_py_site "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
install(TARGETS volcaniarm_kinematics_py DESTINATION "${_py_site}")

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_five_bar test/test_five_bar.cpp)
  target_link_libraries(test_five_bar ${PROJECT_NAME})

  # Pinocchio CI oracle: validates the analytic solver against the live URDF.
  # Optional - only built when pinocchio, the description package, and xacro are
  # all available (e.g. a developer/CI machine with the workspace sourced).
  find_package(pinocchio QUIET)
  find_package(volcaniarm_description QUIET)
  find_program(XACRO_EXECUTABLE xacro)
  if(pinocchio_FOUND AND volcaniarm_description_FOUND AND XACRO_EXECUTABLE)
    set(_oracle_xacro "${volcaniarm_description_DIR}/../urdf/volcaniarm.urdf.xacro")
    set(_oracle_urdf "${CMAKE_CURRENT_BINARY_DIR}/volcaniarm_oracle.urdf")
    execute_process(
      COMMAND ${XACRO_EXECUTABLE} ${_oracle_xacro} -o ${_oracle_urdf}
      RESULT_VARIABLE _oracle_rc
      ERROR_VARIABLE _oracle_err)
    if(_oracle_rc EQUAL 0 AND EXISTS ${_oracle_urdf})
      ament_add_gtest(test_pinocchio_oracle test/test_pinocchio_oracle.cpp)
      target_link_libraries(test_pinocchio_oracle ${PROJECT_NAME} pinocchio::pinocchio)
      target_compile_definitions(test_pinocchio_oracle PRIVATE
        ORACLE_URDF_PATH="${_oracle_urdf}")
    else()
      message(STATUS
        "volcaniarm_kinematics: skipping Pinocchio oracle (xacro expand failed: ${_oracle_err})")
    endif()
  else()
    message(STATUS
      "volcaniarm_kinematics: skipping Pinocchio oracle (pinocchio/description/xacro not all found)")
  endif()
endif()

ament_package()
