cmake_minimum_required(VERSION 3.8)
project(huananhu_sim_bridge)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/sim_lidar_to_livox.py
  scripts/sim_state_adapter.py
  scripts/sim_command_gate.py
  scripts/sim_posture_controller.py
  scripts/sim_physics_test_gate.py
  scripts/sim_nav_command_adapter.py
  scripts/sim_wait_ready.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/sensor_state_math.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/cad_self_filter.py DESTINATION lib/${PROJECT_NAME})

ament_package()
