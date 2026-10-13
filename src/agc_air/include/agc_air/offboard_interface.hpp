// agc_air/offboard_interface.hpp —— PX4 Offboard 接口（起飞控制器与轨迹跟踪器共用）
//
// 契约 [唯一执行口]（空中控制权）：任一时刻**只有一个** setpoint 发布者。
// 本类把「订阅归一化状态 + 发布 offboard setpoint / 命令」收在一处，供两个控制者复用；
// setpoint 世界系为 ENU（契约 #namespace），发布前转 PX4 NED（ADR-0005）。

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "agc_air/px4_conversions.hpp"

namespace agc_air
{

struct OffboardParams
{
  std::string odom_topic{"/agc/air/odom"};
  std::string status_topic{"/fmu/out/vehicle_status"};
  std::string ocm_topic{"/fmu/in/offboard_control_mode"};
  std::string setpoint_topic{"/fmu/in/trajectory_setpoint"};
  std::string command_topic{"/fmu/in/vehicle_command"};
  double setpoint_dt{0.02};   // setpoint 前馈步长 = 1/控制频率
  double odom_timeout_s{0.5};
  std::uint8_t target_system{1};
  std::uint8_t source_system{1};
  std::uint8_t source_component{1};
};

class OffboardInterface
{
public:
  OffboardInterface(rclcpp_lifecycle::LifecycleNode & node, const OffboardParams & params);

  void configure();    // 建订阅
  void activate();     // 建 / 激活发布器
  void deactivate();   // 停发
  void cleanup();      // 释放
  /// 移交控制权：**销毁** setpoint / 控制模式 发布器（真正让出"唯一发布者"地位；
  /// 仅置停发标志会把发布器留在 DDS 图里，使 `count_publishers` 冲突检测误报）
  void releaseSetpointPublishers();

  bool odomValid() const {return have_odom_ && (node_.now() - odom_stamp_).seconds() < params_.odom_timeout_s;}
  const nav_msgs::msg::Odometry & odom() const {return odom_;}
  bool armed() const {return status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;}
  bool offboard() const {return status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;}
  bool failsafe() const {return status_.failsafe;}

  /// 世界系（ENU）加速度 → PX4：offboard_control_mode（position + acceleration）
  void publishControlMode();
  /// 世界系（ENU）setpoint；位置项含 `0.5·a·dt²` 前馈（spec R5）
  void publishSetpoint(const Vector3 & pos_enu, const Vector3 & vel_enu, const Vector3 & acc_cmd_enu,
    double yaw_enu);
  /// 直接把 offboard setpoint 落为 PX4 NED 形式（内部使用，亦可复用于 NaN 通道）
  void sendSetpointNed(const Vector3 & pos_ned, const Vector3 & vel_ned, const Vector3 & acc_ned,
    double yaw_ned);

  void sendArm(bool arm);
  void sendOffboardMode();

  /// setpoint 发布者计数（用于 `control_conflict` 检测，图 API 内省）
  std::size_t countSetpointPublishers() const;

  double setpointDt() const {return params_.setpoint_dt;}

private:
  void stampHeader(std::uint64_t & px4_stamp);
  std::uint64_t nowMicros() const;

  rclcpp_lifecycle::LifecycleNode & node_;
  OffboardParams params_;

  nav_msgs::msg::Odometry odom_;
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  bool have_odom_{false};
  px4_msgs::msg::VehicleStatus status_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;

  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sp_pub_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
};

}  // namespace agc_air
