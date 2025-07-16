cmake_minimum_required(VERSION 3.14)
project(imu_preproc_ihw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(imu_preproc_lib
  src/imu_preproc.cpp
)

ament_target_dependencies(imu_preproc_lib
  Eigen3
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

add_executable(imu_preproc_node
  src/imu_preproc_node.cpp
)
target_link_libraries(imu_preproc_node imu_preproc_lib)

ament_target_dependencies(imu_preproc_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS
  imu_preproc_lib
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

install(FILES PIPELINE_OVERVIEW.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
