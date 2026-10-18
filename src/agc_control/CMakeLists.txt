cmake_minimum_required(VERSION 3.8)
project(agc_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# 通用控制层：纯数学接口（头文件），与执行器解耦（ADR-0018）。
# 接口 = state(pos,vel) + ref(pos,vel,acc) → acc_cmd（世界系）；控制频率由消费者 pump。
add_library(agc_control INTERFACE)
target_include_directories(agc_control INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(agc_control INTERFACE Eigen3::Eigen)
install(DIRECTORY include/ DESTINATION include)

# 插件①：轨迹生成器（linear/circular/helical/figure8/hover/step）
add_library(agc_trajectory_plugins SHARED
  src/trajectory_generators.cpp)
target_link_libraries(agc_trajectory_plugins
  agc_control rclcpp::rclcpp pluginlib::pluginlib)
install(TARGETS agc_trajectory_plugins
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

# 插件②：控制器（LQR）
add_library(agc_controller_plugins SHARED
  src/lqr_controller.cpp)
target_link_libraries(agc_controller_plugins
  agc_control rclcpp::rclcpp pluginlib::pluginlib)
install(TARGETS agc_controller_plugins
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

pluginlib_export_plugin_description_file(agc_control plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_trajectory_plugins test/test_trajectory_plugins.cpp)
  target_link_libraries(test_trajectory_plugins
    agc_control agc_trajectory_plugins rclcpp::rclcpp)

  ament_add_gtest(test_lqr_controller test/test_lqr_controller.cpp)
  target_link_libraries(test_lqr_controller
    agc_control agc_controller_plugins agc_trajectory_plugins rclcpp::rclcpp)

  # R-038：LQR 配置合法性 / DARE 收敛**失败型**验证（不写镜像实现测试）
  ament_add_gtest(test_lqr_validation test/test_lqr_validation.cpp)
  target_link_libraries(test_lqr_validation
    agc_control agc_controller_plugins rclcpp::rclcpp)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp pluginlib Eigen3)
ament_package()
