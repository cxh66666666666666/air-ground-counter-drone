cmake_minimum_required(VERSION 3.8)
project(agc_sim)

find_package(ament_cmake REQUIRED)

# 世界 / 模型 / 桥配置 / launch 的安装（W2）
install(DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()