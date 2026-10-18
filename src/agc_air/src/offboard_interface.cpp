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
  // 局部原点（R-003）：PX4 `TrajectorySetpoint.position` 在**局部 NED**（相对 EKF 原点），
  // 而本接口对外收发的都是世界 ENU；发布前须减去该原点。取值与 px4_adapter 同源（launch 注入 spawn 位姿）。
  // R-034：**幂等**声明（先 has_parameter）→ `cleanup→configure` 二轮不抛重复声明异常。
  const auto origin_pose = node_.has_parameter("origin_pose") ?
    node_.get_parameter("origin_pose").as_string() :
    node_.declare_parameter<std::string>("origin_pose", "0,0,0.2,0,0,0");
  if (!parseSpawnPositionXyz(origin_pose, origin_enu_)) {
    RCLCPP_WARN(
      node_.get_logger(), "origin_pose='%s' 解析失败 → 局部原点按 (0,0,0) 处理", origin_pose.c_str());
    origin_enu_ = Vector3::Zero();
  }
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
  // R-035：写入记账清零（可再 configure/activate）。
  setpoint_write_count_ = 0;
  first_setpoint_write_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_setpoint_write_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
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
  // R-035：setpoint 写入记账（诊断）——首次写记录起点，末次写记录终点。
  const auto t_write = node_.now();
  if (setpoint_write_count_ == 0) {first_setpoint_write_ = t_write;}
  last_setpoint_write_ = t_write;
  ++setpoint_write_count_;
}

void OffboardInterface::publishSetpoint(const Vector3 & pos_enu, const Vector3 & vel_enu,
  const Vector3 & acc_cmd_enu, double yaw_enu)
{
  // 位置项含 0.5·a·dt² 前馈（spec R5）；
  // 世界 ENU → 局部 ENU（减 EKF 局部原点，R-003）→ PX4 NED。速度/加速度是与原点无关的自由矢量。
  const Vector3 pos_local_enu = pos_enu - origin_enu_;
  const Vector3 pos_local_ff =
    pos_local_enu + 0.5 * acc_cmd_enu * params_.setpoint_dt * params_.setpoint_dt;

  const auto enuToNed = [](const Vector3 & v) {return Vector3(v.y(), v.x(), -v.z());};
  sendSetpointNed(enuToNed(pos_local_ff), enuToNed(vel_enu), enuToNed(acc_cmd_enu),
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
  // R-035：**仅作告警**——图内省计数含**静默 publisher**、且 DDS 发现延迟不保证事件顺序，
  // 故**不**单独决定控制权威（唯一权威由显式控制许可判定，见 ADR-0050）。
  return node_.count_publishers(params_.setpoint_topic);
}

double OffboardInterface::setpointRateHz() const
{
  // R-035：自写入速率（Hz）= 计数 /（末次 − 首次）；跨度 ≤ 0 → 0（单次写入无速率）。
  const double span = (last_setpoint_write_ - first_setpoint_write_).seconds();
  if (setpoint_write_count_ == 0 || span <= 0.0) {return 0.0;}
  return static_cast<double>(setpoint_write_count_) / span;
}

}  // namespace agc_air
