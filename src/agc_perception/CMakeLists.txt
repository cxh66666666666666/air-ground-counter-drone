cmake_minimum_required(VERSION 3.8)
project(agc_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(agc_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# gz 真值旁路（gt_bypass）：经 gz-transport 直读 dynamic_pose/info（见 spec R1）
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

set(DEPS rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_ros agc_msgs)

# ---- W4-a · 适配层 ----
add_executable(gt_bypass src/gt_bypass.cpp)
ament_target_dependencies(gt_bypass rclcpp geometry_msgs)
target_include_directories(gt_bypass PRIVATE
  ${gz-transport13_INCLUDE_DIRS} ${gz-msgs10_INCLUDE_DIRS})
target_link_libraries(gt_bypass ${gz-transport13_LIBRARIES} ${gz-msgs10_LIBRARIES})

add_executable(ground_odom_adapter src/ground_odom_adapter.cpp)
ament_target_dependencies(ground_odom_adapter ${DEPS})
target_include_directories(ground_odom_adapter PRIVATE include)

add_executable(gt_adapter src/gt_adapter.cpp)
ament_target_dependencies(gt_adapter ${DEPS})
target_include_directories(gt_adapter PRIVATE include)

# ---- W4-b · 感知与融合 ----
add_executable(air_perception src/air_perception.cpp)
ament_target_dependencies(air_perception ${DEPS})
target_include_directories(air_perception PRIVATE include)
target_link_libraries(air_perception yaml-cpp)

add_executable(air_fusion src/air_fusion.cpp)
ament_target_dependencies(air_fusion ${DEPS})
target_include_directories(air_fusion PRIVATE include)
target_link_libraries(air_fusion yaml-cpp)

# ---- W4-c · 地图 ----
add_executable(map_adapter src/map_adapter.cpp)
ament_target_dependencies(map_adapter ${DEPS})
target_include_directories(map_adapter PRIVATE include)
target_link_libraries(map_adapter yaml-cpp)

install(TARGETS
  gt_bypass ground_odom_adapter gt_adapter
  air_perception air_fusion map_adapter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
