cmake_minimum_required(VERSION 3.8)
project(agc_ground)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(agc_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# W5-c 离线基线：复用 Nav2 planner 插件 + agc_control LQR 控制器插件（ADR-0008/ADR-0018）
find_package(agc_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)

# W5-b：命令源为 agc_rl 的推理节点（Python）；原 ground_policy_placeholder 已退役（spec R7）。
add_executable(ground_drive_relay src/ground_drive_relay.cpp)
ament_target_dependencies(ground_drive_relay rclcpp geometry_msgs)

# W5 · R-007：自体过滤（自返回置 inf，供包围式旋转/后向 zone 使用）
add_executable(scan_self_filter src/scan_self_filter.cpp)
ament_target_dependencies(scan_self_filter rclcpp sensor_msgs)

# W5-c：离线对照基线（Nav2 planner 插件 + agc_control LQR 插件 + 内建 costmap）
add_executable(ground_baseline src/ground_baseline.cpp)
target_link_libraries(ground_baseline Eigen3::Eigen)
ament_target_dependencies(ground_baseline
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs agc_msgs
  agc_control pluginlib tf2_ros nav2_core nav2_costmap_2d nav2_util)

install(TARGETS
  ground_drive_relay
  ground_baseline
  scan_self_filter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
