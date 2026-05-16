cmake_minimum_required(VERSION 3.8)
project(uav_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(uav_planner_interface REQUIRED)

set(DEPS
  rclcpp rclcpp_action nav_msgs std_msgs geometry_msgs
  visualization_msgs octomap_msgs uav_planner_interface)

add_executable(frontier_explorer_node src/frontier_explorer_node.cpp)
ament_target_dependencies(frontier_explorer_node ${DEPS})
target_include_directories(frontier_explorer_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(frontier_explorer_node
  ${OCTOMAP_LIBRARIES})

install(TARGETS frontier_explorer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()