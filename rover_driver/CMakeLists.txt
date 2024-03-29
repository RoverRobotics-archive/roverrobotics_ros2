cmake_minimum_required(VERSION 3.5)
project(rover_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(include/)
file(GLOB_RECURSE AllHeaders ${PROJECT_SOURCE_DIR}/*.hpp)

add_executable(rover_driver
  src/composition.cpp
  src/rover_serial.cpp
  src/rover.cpp
  src/data.cpp
  src/pi_controller.cpp
  ${AllHeaders})

target_link_libraries(rover_driver pthread)

target_include_directories(rover_driver
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(rover_driver
  rclcpp
  std_msgs
  geometry_msgs
  rover_msgs
  nav_msgs
  tf2_geometry_msgs
  diagnostic_updater
  )

install(TARGETS
  rover_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
