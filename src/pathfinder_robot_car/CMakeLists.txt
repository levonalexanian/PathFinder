cmake_minimum_required(VERSION 3.16)
project(pathfinder_robot_car)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pathfinder_core REQUIRED)

add_executable(path_follower_node src/path_follower_node.cpp)
ament_target_dependencies(path_follower_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  pathfinder_core
)

install(
  TARGETS path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf gazebo launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
