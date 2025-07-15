cmake_minimum_required(VERSION 3.14)
project(gps_imu_fusion_ihw)

# ----------------------------------------------------------------------------
# Compile options & C++ standard
# ----------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# ----------------------------------------------------------------------------
# Include directories
# ----------------------------------------------------------------------------
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ----------------------------------------------------------------------------
# Library: kalman_filter
# ----------------------------------------------------------------------------
add_library(kalman_filter
  src/kalman_filter.cpp
)
ament_target_dependencies(kalman_filter
  Eigen3
)

# ----------------------------------------------------------------------------
# Executable: sensor_fusion_node
# ----------------------------------------------------------------------------
add_executable(sensor_fusion_node
  src/sensor_fusion_node.cpp
)
ament_target_dependencies(sensor_fusion_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
)

target_link_libraries(sensor_fusion_node
  kalman_filter
)

# ----------------------------------------------------------------------------
# Install targets & headers
# ----------------------------------------------------------------------------
install(TARGETS
  kalman_filter
  sensor_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# ----------------------------------------------------------------------------
# Tests (clang-format, cpplint, etc.)
# ----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
