cmake_minimum_required(VERSION 3.14)
project(imu_preproc_ihw)

# ------------------------------------------------------------------------------
# Build options
# ------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# ------------------------------------------------------------------------------
# Includes
# ------------------------------------------------------------------------------
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------
# Library (optional core processing; combine .cpp sources)
# ------------------------------------------------------------------------------
add_library(imu_preproc_core
  src/imu_preproc.cpp
)
ament_target_dependencies(imu_preproc_core
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
)

target_include_directories(imu_preproc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------------------
# Executable node
# ------------------------------------------------------------------------------
add_executable(imu_preproc_node
  src/imu_preproc_node.cpp
)
ament_target_dependencies(imu_preproc_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
)
target_link_libraries(imu_preproc_node imu_preproc_core)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(TARGETS
  imu_preproc_core
  imu_preproc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Docs (optional; only if file exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/PIPELINE_OVERVIEW.md")
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/PIPELINE_OVERVIEW.md"
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ------------------------------------------------------------------------------
# Testing / lint
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
