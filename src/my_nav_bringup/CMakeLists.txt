cmake_minimum_required(VERSION 3.8)
project(my_nav_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf maps pcd rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_to_chassis_cmd.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
