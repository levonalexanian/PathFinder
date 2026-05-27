cmake_minimum_required(VERSION 3.16)
project(pathfinder_algo_astar)

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

add_library(astar_core src/astar_core.cpp)
target_include_directories(astar_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)
ament_target_dependencies(astar_core
  pathfinder_core
)
target_link_libraries(astar_core ${OCTOMAP_LIBRARIES})
target_link_directories(astar_core PUBLIC ${OCTOMAP_LIBRARY_DIRS})

add_executable(astar_node src/astar_node.cpp)
target_include_directories(astar_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OCTOMAP_INCLUDE_DIRS}
)
ament_target_dependencies(astar_node
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
target_link_libraries(astar_node astar_core ${OCTOMAP_LIBRARIES})
target_link_directories(astar_node PRIVATE ${OCTOMAP_LIBRARY_DIRS})

install(
  TARGETS astar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS astar_core
  EXPORT export_astar_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linters whose conventions this project doesn't follow (C++ style + copyright).
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
