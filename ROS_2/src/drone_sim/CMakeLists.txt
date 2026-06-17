cmake_minimum_required(VERSION 3.8)
project(drone_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models worlds launch px4
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
