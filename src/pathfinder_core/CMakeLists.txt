cmake_minimum_required(VERSION 3.16)
project(pathfinder_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pathfinder_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OCTOMAP REQUIRED octomap)

add_library(planner_action_server_base
  src/planner_action_server_base.cpp
)
target_include_directories(planner_action_server_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planner_action_server_base
  rclcpp
  rclcpp_action
  pathfinder_msgs
  octomap_msgs
)

add_executable(scheduler_node src/scheduler_node.cpp)
ament_target_dependencies(scheduler_node
  rclcpp
  rclcpp_action
  pathfinder_msgs
  octomap_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(gz_pose_relay_node src/gz_pose_relay_node.cpp)
ament_target_dependencies(gz_pose_relay_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(map_publisher_node src/map_publisher_node.cpp)
ament_target_dependencies(map_publisher_node
  rclcpp
  octomap_msgs
  octomap_ros
)
target_include_directories(map_publisher_node PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(map_publisher_node ${OCTOMAP_LIBRARIES})
target_link_directories(map_publisher_node PRIVATE ${OCTOMAP_LIBRARY_DIRS})

add_executable(generate_demo_map tools/generate_demo_map.cpp)
target_include_directories(generate_demo_map PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(generate_demo_map ${OCTOMAP_LIBRARIES})
target_link_directories(generate_demo_map PRIVATE ${OCTOMAP_LIBRARY_DIRS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS scheduler_node gz_pose_relay_node map_publisher_node generate_demo_map
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS planner_action_server_base
  EXPORT export_pathfinder_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(planner_action_server_base)
ament_export_targets(export_pathfinder_core HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  pathfinder_msgs
  octomap_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
