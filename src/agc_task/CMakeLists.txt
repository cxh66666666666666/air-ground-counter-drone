cmake_minimum_required(VERSION 3.8)
project(agc_task)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(agc_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(task_manager src/task_manager.cpp)
ament_target_dependencies(task_manager
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs agc_msgs)

add_executable(scenario_reset src/scenario_reset.cpp)
ament_target_dependencies(scenario_reset
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs agc_msgs ros_gz_interfaces)

add_executable(run_logger src/run_logger.cpp)
ament_target_dependencies(run_logger
  rclcpp geometry_msgs nav_msgs sensor_msgs nav2_msgs std_msgs agc_msgs ros_gz_interfaces)

add_executable(capture_adjudicator src/capture_adjudicator.cpp)
ament_target_dependencies(capture_adjudicator
  rclcpp nav_msgs std_msgs agc_msgs)

install(TARGETS task_manager scenario_reset run_logger capture_adjudicator
  DESTINATION lib/${PROJECT_NAME})

# W6 前置 P / 拦截裁决：Python 节点（入侵者驱动 + 干扰注入）
install(PROGRAMS scripts/intruder_driver.py DESTINATION lib/${PROJECT_NAME} RENAME intruder_driver)
install(PROGRAMS scripts/jammer.py DESTINATION lib/${PROJECT_NAME} RENAME jammer)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()