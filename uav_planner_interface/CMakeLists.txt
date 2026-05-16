cmake_minimum_required(VERSION 3.8)
project(uav_planner_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Generate action types ────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToGoal.action"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs)

# Need generated headers before building nodes
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

set(DEPS
  rclcpp rclcpp_action nav_msgs geometry_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
  octomap_msgs pluginlib)

# ── Planner server node ───────────────────────────────────────────────
add_executable(planner_server_node src/planner_server_node.cpp)
ament_target_dependencies(planner_server_node ${DEPS})
target_include_directories(planner_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(planner_server_node
  ${cpp_typesupport_target}
  ${OCTOMAP_LIBRARIES})

# ── A* plugin (shared library — loaded by pluginlib) ─────────────────
add_library(astar_planner SHARED plugins/astar_planner.cpp)
ament_target_dependencies(astar_planner rclcpp nav_msgs geometry_msgs pluginlib)
target_include_directories(astar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(astar_planner ${OCTOMAP_LIBRARIES})
pluginlib_export_plugin_description_file(uav_planner_interface planner_plugins.xml)

install(TARGETS planner_server_node astar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(FILES planner_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_export_include_directories(include)
ament_export_libraries(astar_planner)
ament_package()