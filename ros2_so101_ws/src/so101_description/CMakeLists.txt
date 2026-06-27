cmake_minimum_required(VERSION 3.8)
project(so101_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY assets launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/joint_states_udp_bridge.py
    scripts/calibrated_joint_slider_gui.py
    scripts/play_recording_joint_states.py
    scripts/real_state_udp_receiver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
