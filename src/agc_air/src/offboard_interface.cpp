// agc_air/offboard_interface.cpp

#include "agc_air/offboard_interface.hpp"

#include <chrono>
#include <limits>
#include <vector>

namespace agc_air
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}

OffboardInterface::OffboardInterface(rclcpp_lifecycle::LifecycleNode & node, const OffboardParams & params)
: node_(node), params_(params)
{
}

void OffboardInterface::configure()
{
  // 归一化话题：odom 走 SENSOR_DATA 档；状态由 px4_adapter 以可靠发布，此处按内部数据订阅
  const auto reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const auto sensor = rclcpp::SensorDataQoS();

  odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    params_.odom_topic, sensor,
    [this](nav_msgs::msg::Odometry::SharedPtr msg) {
      odom_ = *msg;
      odom_stamp_ = node_.now();
      have_odom_ = true;
    });

  status_sub_ = node_.create_subscription<px4_msgs::msg::VehicleStatus>(
    params_.status_topic, reliable,
    [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {status_ = *msg;});
}

void OffboardInterface::activate()
{
  const auto reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  // PX4 uXRCE-DDS 侧的默认 writer 为 reliable，故用 reliable keep_last(1) 匹配
  ocm_pub_ = node_.create_publisher<px4_msgs::msg::OffboardControlMode>(params_.ocm_topic, reliable);
  sp_pub_ = node_.create_publisher<px4_msgs::msg::TrajectorySetpoint>(params_.setpoint_topic, reliable);
  cmd_pub_ = node_.create_publisher<px4_msgs::msg::VehicleCommand>(params_.command_topic, reliable);
  ocm_pub_->on_activate();
  sp_pub_->on_activate();
  cmd_pub_->on_activate();
}

void OffboardInterface::deactivate()
{
  if (ocm_pub_) {ocm_pub_->on_deactivate();}
  if (sp_pub_) {sp_pub_->on_deactivate();}
  if (cmd_pub_) {cmd_pub_->on_deactivate();}
}

void OffboardInterface::releaseSetpointPublishers()
{
  if (ocm_pub_) {ocm_pub_->on_deactivate();}
  if (sp_pub_) {sp_pub_->on_deactivate();}
  ocm_pub_.reset();
  sp_pub_.reset();
}

void OffboardInterface::cleanup()
{
  odom_sub_.reset();
  status_sub_.reset();
  ocm_pub_.reset();
  sp_pub_.reset();
  cmd_pub_.reset();
  have_odom_ = false;
}

std::uint64_t OffboardInterface::nowMicros() const
{
  return static_cast<std::uint64_t>(node_.now().nanoseconds() / 1000);
}

void OffboardInterface::publishControlMode()
{
  if (!ocm_pub_ || !ocm_pub_->is_activated()) {return;}
  px4_msgs::msg::OffboardControlMode m{};
  m.timestamp = nowMicros();
  m.position = true;          // 位置 + 加速度前馈通道
  m.velocity = false;
  m.acceleration = true;
  ocm_pub_->publish(m);
}

void OffboardInterface::sendSetpointNed(const Vector3 & pos_ned, const Vector3 & vel_ned,
  const Vector3 & acc_ned, double yaw_ned)
{
  if (!sp_pub_ || !sp_pub_->is_activated()) {return;}
  px4_msgs::msg::TrajectorySetpoint m{};
  m.timestamp = nowMicros();
  m.position = {static_cast<float>(pos_ned.x()), static_cast<float>(pos_ned.y()),
    static_cast<float>(pos_ned.z())};
  m.velocity = {static_cast<float>(vel_ned.x()), static_cast<float>(vel_ned.y()),
    static_cast<float>(vel_ned.z())};
  m.acceleration = {static_cast<float>(acc_ned.x()), static_cast<float>(acc_ned.y()),
    static_cast<float>(acc_ned.z())};
  m.jerk = {kNaN, kNaN, kNaN};
  m.yaw = static_cast<float>(yaw_ned);
  m.yawspeed = kNaN;
  sp_pub_->publish(m);
}

void OffboardInterface::publishSetpoint(const Vector3 & pos_enu, const Vector3 & vel_enu,
  const Vector3 & acc_cmd_enu, double yaw_enu)
{
  // 位置项含 0.5·a·dt² 前馈（spec R5）；世界系 ENU → PX4 NED
  const Vector3 pos_enu_ff = pos_enu + 0.5 * acc_cmd_enu * params_.setpoint_dt * params_.setpoint_dt;

  const auto enuToNed = [](const Vector3 & v) {return Vector3(v.y(), v.x(), -v.z());};
  sendSetpointNed(enuToNed(pos_enu_ff), enuToNed(vel_enu), enuToNed(acc_cmd_enu),
    enuYawToPx4(yaw_enu));
}

void OffboardInterface::sendArm(bool arm)
{
  if (!cmd_pub_ || !cmd_pub_->is_activated()) {return;}
  px4_msgs::msg::VehicleCommand c{};
  c.timestamp = nowMicros();
  c.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  c.param1 = arm ? 1.0F : 0.0F;
  c.target_system = params_.target_system;
  c.target_component = 1;
  c.source_system = params_.source_system;
  c.source_component = params_.source_component;
  c.from_external = true;
  cmd_pub_->publish(c);
}

void OffboardInterface::sendOffboardMode()
{
  if (!cmd_pub_ || !cmd_pub_->is_activated()) {return;}
  px4_msgs::msg::VehicleCommand c{};
  c.timestamp = nowMicros();
  // 与 PX4 自身 `commander mode offboard` 等价：DO_SET_MODE(custom_enabled, MAIN_MODE_OFFBOARD)
  // 注意 param2 用 PX4 的 **自定义主模式号**（6），而非 NAVIGATION_STATE_*（14）。
  c.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  c.param1 = 1.0F;    // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
  c.param2 = 6.0F;    // PX4_CUSTOM_MAIN_MODE_OFFBOARD
  c.target_system = params_.target_system;
  c.target_component = 1;
  c.source_system = params_.source_system;
  c.source_component = params_.source_component;
  c.from_external = true;
  cmd_pub_->publish(c);
}

std::size_t OffboardInterface::countSetpointPublishers() const
{
  return node_.count_publishers(params_.setpoint_topic);
}

}  // namespace agc_air
