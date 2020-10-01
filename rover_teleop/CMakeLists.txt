cmake_minimum_required(VERSION 3.5)
project(rover_teleop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  scripts/modules
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/gamepad_mapper_node.py
  DESTINATION lib/${PROJECT_NAME})


ament_package()
