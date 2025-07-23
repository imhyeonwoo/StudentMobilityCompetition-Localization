cmake_minimum_required(VERSION 3.8)
project(gps_imu_fusion)

find_package(ament_cmake REQUIRED)

# launch/ , config/ 폴더를 install/share/${PROJECT_NAME}/ 로 복사
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
