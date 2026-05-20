cmake_minimum_required(VERSION 3.16)
project(pathfinder_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AlgorithmSelection.msg
  msg/PlannerStatus.msg
  action/RequestPath.action
  DEPENDENCIES geometry_msgs nav_msgs visualization_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs nav_msgs visualization_msgs action_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
