cmake_minimum_required(VERSION 3.5)
project(huananhu_sim_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER 6)

add_library(MecanumDrive2 SHARED plugins/MecanumDrive2.cc)
target_link_libraries(MecanumDrive2
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)

add_library(WheelTractionDrive SHARED plugins/WheelTractionDrive.cc)
target_link_libraries(WheelTractionDrive
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)

add_library(BodyContactEvidence SHARED plugins/BodyContactEvidence.cc)
target_link_libraries(BodyContactEvidence
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)

install(TARGETS MecanumDrive2 WheelTractionDrive BodyContactEvidence DESTINATION plugins)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

ament_package()
