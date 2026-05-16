cmake_minimum_required(VERSION 3.8)
project(uav_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

set(DEPS rclcpp geometry_msgs std_msgs px4_msgs)

add_executable(setpoint_publisher_node src/setpoint_publisher_node.cpp)
ament_target_dependencies(setpoint_publisher_node ${DEPS})

install(TARGETS setpoint_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()