cmake_minimum_required(VERSION 3.8)
project(uav_mapping)

find_package(ament_cmake REQUIRED)

# No C++ nodes in this package — just config and launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()