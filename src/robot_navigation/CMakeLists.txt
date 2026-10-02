cmake_minimum_required(VERSION 3.10)
project(robot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/nav_monitor.py scripts/virtual_wall_manager.py scripts/safety_supervisor.py
  scripts/sim_power_system.py scripts/docking_manager.py scripts/battery_monitor.py scripts/robot_state_manager.py scripts/robot_health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
