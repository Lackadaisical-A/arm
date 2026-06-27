cmake_minimum_required(VERSION 3.8)
project(so101_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/display_trajectory_udp_bridge.py
    scripts/so101_trajectory_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
