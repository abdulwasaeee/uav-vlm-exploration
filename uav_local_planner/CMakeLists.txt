cmake_minimum_required(VERSION 3.8)
project(uav_local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(uav_planner_interface REQUIRED)

set(DEPS
  rclcpp rclcpp_action nav_msgs geometry_msgs std_msgs
  octomap_msgs uav_planner_interface)

set(DEPS_MP
  rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs
  pcl_conversions pcl_ros)

# ── VFH3D library ─────────────────────────────────────────────────────
add_library(vfh3d SHARED src/vfh3d.cpp)
target_include_directories(vfh3d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(vfh3d ${OCTOMAP_LIBRARIES})
ament_target_dependencies(vfh3d rclcpp)

# ── VFH3D node ────────────────────────────────────────────────────────
add_executable(vfh3d_node src/vfh3d_node.cpp)
ament_target_dependencies(vfh3d_node ${DEPS})
target_include_directories(vfh3d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(vfh3d_node vfh3d ${OCTOMAP_LIBRARIES})

# ── Waypoint manager node ─────────────────────────────────────────────
add_executable(waypoint_manager_node src/waypoint_manager_node.cpp)
ament_target_dependencies(waypoint_manager_node ${DEPS})

# ── Motion Primitive library ──────────────────────────────────────────
add_library(motion_primitives SHARED src/motion_primitives.cpp)
target_include_directories(motion_primitives PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(motion_primitives ${PCL_LIBRARIES})
ament_target_dependencies(motion_primitives rclcpp)

# ── Motion Primitive node ─────────────────────────────────────────────
add_executable(mp_node src/mp_node.cpp)
ament_target_dependencies(mp_node ${DEPS_MP})
target_include_directories(mp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(mp_node motion_primitives ${PCL_LIBRARIES})

install(TARGETS vfh3d vfh3d_node waypoint_manager_node motion_primitives mp_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_export_include_directories(include)
ament_export_libraries(vfh3d motion_primitives)
ament_package()
