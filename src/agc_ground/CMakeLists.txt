cmake_minimum_required(VERSION 3.8)
project(agc_ground)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(agc_msgs REQUIRED)

add_executable(ground_policy_placeholder src/ground_policy_placeholder.cpp)
ament_target_dependencies(ground_policy_placeholder
  rclcpp geometry_msgs nav_msgs std_msgs agc_msgs)

add_executable(ground_safety_stub src/ground_safety_stub.cpp)
ament_target_dependencies(ground_safety_stub rclcpp geometry_msgs)

add_executable(ground_drive_relay src/ground_drive_relay.cpp)
ament_target_dependencies(ground_drive_relay rclcpp geometry_msgs)

install(TARGETS
  ground_policy_placeholder
  ground_safety_stub
  ground_drive_relay
  DESTINATION lib/${PROJECT_NAME})

ament_package()