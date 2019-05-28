cmake_minimum_required(VERSION 3.5)
project(openrover_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  meshes
  urdf
  maps
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
    scripts/urdf_spawner.py
    DESTINATION lib/${PROJECT_NAME})


ament_package()
