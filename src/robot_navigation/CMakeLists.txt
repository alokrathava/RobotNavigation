cmake_minimum_required(VERSION 3.10)
project(robot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/nav_monitor.py scripts/virtual_wall_manager.py scripts/safety_supervisor.py
  scripts/sim_battery.py scripts/sim_dock_contact.py scripts/sim_dock_detector.py
  scripts/docking_manager.py scripts/battery_monitor.py scripts/robot_state_manager.py scripts/robot_health_monitor.py
  scripts/navigation_runtime_coordinator.py
  scripts/map_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(map_manager_transitions test/test_map_manager_transitions.py)
  ament_add_pytest_test(map_manager_persistence test/test_map_manager_persistence.py)
  ament_add_pytest_test(safety_supervisor test/test_safety_supervisor.py)
endif()

ament_package()
