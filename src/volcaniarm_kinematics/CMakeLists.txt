cmake_minimum_required(VERSION 3.8)
project(volcaniarm_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Pure C++ library: ament_cmake is the only build dependency, no ROS runtime deps.
find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/five_bar.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

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
