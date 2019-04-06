cmake_minimum_required(VERSION 3.5)
project(rr_openrover_core)

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

add_executable(openrover_node src/openrover_node.cpp)
ament_target_dependencies(openrover_node rclcpp std_msgs serial)

install(TARGETS openrover_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

