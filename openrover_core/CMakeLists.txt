cmake_minimum_required(VERSION 3.5)
project(openrover_core)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(openrover_core_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(openrover
  src/openrover_node.cpp
  src/openrover_node.hpp
  src/openrover_serial.cpp
  src/openrover_serial.hpp
  src/openrover.cpp
  src/openrover.hpp
)

ament_target_dependencies(openrover
  rclcpp 
  std_msgs 
  serial
  geometry_msgs
  openrover_core_msgs
)

install(TARGETS
  openrover
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

