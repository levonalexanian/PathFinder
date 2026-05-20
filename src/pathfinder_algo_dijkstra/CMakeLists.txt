cmake_minimum_required(VERSION 3.16)
project(pathfinder_algo_dijkstra)

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
find_package(pathfinder_core REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OCTOMAP REQUIRED octomap)

add_library(dijkstra_core
  src/dijkstra_core.cpp
)
target_include_directories(dijkstra_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)
ament_target_dependencies(dijkstra_core
  rclcpp
  pathfinder_core
)
target_link_libraries(dijkstra_core ${OCTOMAP_LIBRARIES})
target_link_directories(dijkstra_core PUBLIC ${OCTOMAP_LIBRARY_DIRS})

add_executable(dijkstra_node
  src/dijkstra_node.cpp
)
target_include_directories(dijkstra_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OCTOMAP_INCLUDE_DIRS}
)
ament_target_dependencies(dijkstra_node
  rclcpp
  rclcpp_action
  pathfinder_msgs
  pathfinder_core
  octomap_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
)
target_link_libraries(dijkstra_node
  dijkstra_core
  ${OCTOMAP_LIBRARIES}
)
target_link_directories(dijkstra_node PRIVATE ${OCTOMAP_LIBRARY_DIRS})

install(
  TARGETS dijkstra_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS dijkstra_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
