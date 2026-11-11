cmake_minimum_required(VERSION 3.8)
project(huananhu_sim_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_planner REQUIRED)

add_executable(simulation_planner_server src/planner_server_main.cpp)
ament_target_dependencies(simulation_planner_server rclcpp nav2_planner)
install(TARGETS simulation_planner_server DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config maps rviz behavior_tree runtime DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_prior_map.py DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
