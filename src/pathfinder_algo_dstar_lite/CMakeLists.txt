cmake_minimum_required(VERSION 3.16)
project(pathfinder_algo_dstar_lite)

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

add_library(dstar_lite_core
  src/dstar_lite_core.cpp
)
target_include_directories(dstar_lite_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)
ament_target_dependencies(dstar_lite_core
  rclcpp
  pathfinder_core
)
target_link_libraries(dstar_lite_core ${OCTOMAP_LIBRARIES})
target_link_directories(dstar_lite_core PUBLIC ${OCTOMAP_LIBRARY_DIRS})

add_executable(dstar_lite_node
  src/dstar_lite_node.cpp
)
target_include_directories(dstar_lite_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OCTOMAP_INCLUDE_DIRS}
)
ament_target_dependencies(dstar_lite_node
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
target_link_libraries(dstar_lite_node
  dstar_lite_core
  ${OCTOMAP_LIBRARIES}
)
target_link_directories(dstar_lite_node PRIVATE ${OCTOMAP_LIBRARY_DIRS})

install(
  TARGETS dstar_lite_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS dstar_lite_node
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
