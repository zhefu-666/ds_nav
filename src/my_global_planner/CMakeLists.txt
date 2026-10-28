cmake_minimum_required(VERSION 3.8)
project(my_global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(planner_node
  src/planner_node.cpp
  src/astar.cpp
)

add_executable(demo_map_server
  src/demo_map_server.cpp
)

add_executable(static_map_server
  src/static_map_server.cpp
)

target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)

ament_target_dependencies(demo_map_server
  rclcpp
  nav_msgs
)

ament_target_dependencies(static_map_server
  rclcpp
  nav_msgs
)
target_link_libraries(static_map_server yaml-cpp)

install(TARGETS
  planner_node
  demo_map_server
  static_map_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
