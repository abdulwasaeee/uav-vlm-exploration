cmake_minimum_required(VERSION 3.8)
project(uav_depth_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(px4_msgs REQUIRED)

set(DEPS
  rclcpp sensor_msgs nav_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
  message_filters pcl_ros pcl_conversions px4_msgs)

# cloud_merge_node
add_executable(cloud_merge_node src/cloud_merge_node.cpp)
ament_target_dependencies(cloud_merge_node ${DEPS})
# pcl_ros must also be linked directly for pcl::transformPointCloud
target_link_libraries(cloud_merge_node ${PCL_LIBRARIES})

# tf_static_broadcaster
add_executable(tf_static_broadcaster src/tf_static_broadcaster.cpp)
ament_target_dependencies(tf_static_broadcaster
  rclcpp geometry_msgs tf2 tf2_ros)

# px4_odom_bridge
add_executable(px4_odom_bridge src/px4_odom_bridge.cpp)
ament_target_dependencies(px4_odom_bridge
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs px4_msgs)

# stereo_camera_info_publisher
add_executable(stereo_camera_info_publisher src/stereo_camera_info_publisher.cpp)
ament_target_dependencies(stereo_camera_info_publisher
  rclcpp sensor_msgs)

install(TARGETS
    cloud_merge_node
    tf_static_broadcaster
    px4_odom_bridge
    stereo_camera_info_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()