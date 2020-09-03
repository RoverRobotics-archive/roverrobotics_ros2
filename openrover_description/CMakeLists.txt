cmake_minimum_required(VERSION 3.5)
project(openrover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  meshes
  urdf
  scripts
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/urdf_spawner.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
