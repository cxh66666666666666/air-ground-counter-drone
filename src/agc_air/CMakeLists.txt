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
# 生命周期失联检测：复用 Nav2 bond 惯用法（topic "bond" / id=节点名；ADR-0049）
find_package(bond REQUIRED)
find_package(bondcpp REQUIRED)

set(DEPS
  rclcpp rclcpp_lifecycle nav_msgs geometry_msgs std_msgs std_srvs trajectory_msgs
  agc_msgs agc_control px4_msgs pluginlib bond bondcpp)

# 共享：PX4 uORB 语义映射（R2）＋ offboard 接口 ＋ 最小安全层（R7）
# 注：agc_air_lib 为**纯内部实现**（仅本包节点 / 测试使用），不对外承诺公共接口，
# 不安装、不导出（对齐 Nav2 惯例：公共库需同时安装 include 并 ament_export_*；
# 纯内部 / 仅 launch 的包零导出）。共享通用控制以 agc_control 为唯一公共库家（ADR-0018）。
add_library(agc_air_lib STATIC
  src/offboard_interface.cpp
  src/safety_shield.cpp)
target_include_directories(agc_air_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
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
  # 节点 TU 直接包含 agc_lifecycle_node.hpp（含 bondcpp）→ 需显式拿到 bond 的 include / link
  ament_target_dependencies(${node} ${DEPS})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# ADR-0080：lifecycle 服务预热（manager 启动前执行；仅预热，不改节点状态）
install(PROGRAMS scripts/air_lifecycle_prewarm.py
  DESTINATION lib/${PROJECT_NAME} RENAME air_lifecycle_prewarm)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_px4_conversions test/test_px4_conversions.cpp)
  target_link_libraries(test_px4_conversions agc_air_lib)
  # R-038：安全层障碍配置**失败型**验证（平行数组长度一致 / 几何合法性；不写镜像实现测试）
  ament_add_gtest(test_safety_shield_validation test/test_safety_shield_validation.cpp)
  target_link_libraries(test_safety_shield_validation agc_air_lib rclcpp::rclcpp)
endif()

# agc_air 不对外承诺公共接口：不安装 include/，不 ament_export_*（纯内部实现 + 节点可执行 + launch/config）。
# 下游无对本包的 ament 消费者（find_package 本包者为零）；共享控制经 agc_control（唯一公共库家，ADR-0018）。
ament_package()
