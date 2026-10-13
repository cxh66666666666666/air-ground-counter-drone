cmake_minimum_required(VERSION 3.8)
project(agc_air)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(agc_msgs REQUIRED)
find_package(agc_control REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPS
  rclcpp rclcpp_lifecycle nav_msgs geometry_msgs std_msgs std_srvs trajectory_msgs
  agc_msgs agc_control px4_msgs pluginlib)

# 共享：PX4 uORB 语义映射（R2）＋ offboard 接口 ＋ 最小安全层（R7）
add_library(agc_air_lib STATIC
  src/offboard_interface.cpp
  src/safety_shield.cpp)
target_include_directories(agc_air_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(agc_air_lib ${DEPS})
target_link_libraries(agc_air_lib Eigen3::Eigen)

# 空中链路节点（R1/R2/R4/R5/R6/R7/R8）
# 注：W3 占位目标源节点已在 W4 退役（真值源改由 agc_perception 的 gt_adapter /
# air_fusion 提供，见 .trae/specs/w4-perception-fusion/spec.md R3）。
set(NODES
  px4_adapter
  takeoff_controller
  trajectory_tracker
  standoff_follower)

foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} agc_air_lib)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_px4_conversions test/test_px4_conversions.cpp)
  target_link_libraries(test_px4_conversions agc_air_lib)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${DEPS} Eigen3)
ament_package()
