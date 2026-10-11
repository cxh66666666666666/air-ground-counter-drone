cmake_minimum_required(VERSION 3.8)
project(agc_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgcTaskState.msg"
  "msg/AirObservationArray.msg"
  "msg/ObstacleObservation.msg"
  "msg/ObstacleTrack.msg"
  "msg/ObstacleTrackArray.msg"
  "msg/SensorObservation.msg"
  "msg/TargetObservation.msg"
  "msg/TargetTrack.msg"
  "msg/TargetTrackArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(interface_contract "test/test_interface_contract.py")
endif()

ament_package()
