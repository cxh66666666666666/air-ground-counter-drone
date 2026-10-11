cmake_minimum_required(VERSION 3.8)
project(agc_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(ground_odom_adapter src/ground_odom_adapter.cpp)
ament_target_dependencies(ground_odom_adapter
  rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_ros)
install(TARGETS ground_odom_adapter DESTINATION lib/${PROJECT_NAME})

ament_package()