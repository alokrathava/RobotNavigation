cmake_minimum_required(VERSION 3.10)
project(robot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/nav_monitor.py scripts/virtual_wall_manager.py scripts/safety_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
