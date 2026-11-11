cmake_minimum_required(VERSION 3.8)
project(huananhu_sim_resources)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models worlds ign urdf scripts config
  DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

ament_package()
