cmake_minimum_required(VERSION 3.5)
project(openrover_core)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(openrover_core_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include/ src/)
file(GLOB_RECURSE AllHeaders ${PROJECT_SOURCE_DIR}/*.hpp)

add_executable(rover
        src/composition.cpp
        src/rover_serial.cpp
        src/rover.cpp
        src/data.cpp
        src/pi_controller.cpp
        ${AllHeaders}
        )

target_include_directories(rover
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(rover
        rclcpp
        std_msgs
        serial
        geometry_msgs
        openrover_core_msgs
        diagnostic_msgs
        nav_msgs
        tf2
        tf2_geometry_msgs
        )

install(TARGETS
  rover
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
