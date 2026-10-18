cmake_minimum_required(VERSION 3.8)
project(agc_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
# GZ 侧末端失效停车守门人（R-009，ADR-0025）：gz-sim System 插件，直连 gz-transport
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(agc_cmd_deadman SHARED src/agc_cmd_deadman.cpp)
target_include_directories(agc_cmd_deadman PRIVATE
  ${gz-sim8_INCLUDE_DIRS} ${gz-plugin2_INCLUDE_DIRS}
  ${gz-transport13_INCLUDE_DIRS} ${gz-msgs10_INCLUDE_DIRS})
target_link_libraries(agc_cmd_deadman
  ${gz-sim8_LIBRARIES} ${gz-plugin2_LIBRARIES}
  ${gz-transport13_LIBRARIES} ${gz-msgs10_LIBRARIES})
# 插件库落 install/agc_sim/lib（gz 经 GZ_SIM_SYSTEM_PLUGIN_PATH 加载，见 world_bringup.launch.py）
install(TARGETS agc_cmd_deadman DESTINATION lib)

# 世界 / 模型 / 桥配置 / launch 的安装（W2）
install(DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
