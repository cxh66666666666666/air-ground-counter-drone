// px4_adapter —— PX4 uORB → 契约口径 的归一化适配（spec R2，ADR-0005）
//
// 唯一直接消费 `/fmu/out/*` 的节点：集中完成微秒时间戳→ROS 时间、NED/FRD↔ENU/FLU、
// QoS 标定，并归一化到契约口径 `/agc/air/*`。上层控制节点只消费归一化话题。
// 帧名依契约 [命名空间与坐标系]：`frame_id=world`、`child_frame_id=drone/base_link`（禁裸 base_link）。

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "agc_air/agc_lifecycle_node.hpp"
#include "agc_air/px4_conversions.hpp"

using agc_air::AgcLifecycleNode;

namespace
{
const char * kOdomOut = "/agc/air/odom";
const char * kStatusOut = "/agc/air/status";
}  // namespace

class Px4Adapter : public AgcLifecycleNode
{
public:
  Px4Adapter()
  : AgcLifecycleNode("px4_adapter")
  {
  }

protected:
  CallbackReturn on_configure_impl() override
  {
    odom_in_ = declare_parameter<std::string>("odom_in_topic", "/fmu/out/vehicle_odometry");
    status_in_ = declare_parameter<std::string>("status_in_topic", "/fmu/out/vehicle_status");
    odom_out_ = declare_parameter<std::string>("odom_out_topic", kOdomOut);
    status_out_ = declare_parameter<std::string>("status_out_topic", kStatusOut);
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    base_frame_ = declare_parameter<std::string>("base_frame", "drone/base_link");
    time_alpha_ = declare_parameter<double>("time_offset_alpha", 0.05);

    mapper_ = agc_air::Px4TimeMapper(time_alpha_);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      odom_out_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    status_pub_ = create_publisher<px4_msgs::msg::VehicleStatus>(
      status_out_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    // PX4 uXRCE-DDS 的 `/fmu/out/*` writer 为 best_effort，订阅端必须同档（否则 QoS 不兼容、收不到数据）；
    // 归一化后的对外话题再按契约口径给 QoS（odom→SENSOR_DATA 档、状态→内部可靠）。
    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      odom_in_, rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleOdometry::SharedPtr m) {onOdom(*m);});

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      status_in_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](px4_msgs::msg::VehicleStatus::SharedPtr m) {status_pub_->publish(*m);});

    RCLCPP_INFO(get_logger(), "px4_adapter 就绪：%s → %s", odom_in_.c_str(), odom_out_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate_impl() override
  {
    odom_pub_->on_activate();
    status_pub_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate_impl() override
  {
    odom_pub_->on_deactivate();
    status_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup_impl() override
  {
    odom_sub_.reset();
    status_sub_.reset();
    odom_pub_.reset();
    status_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void onOdom(const px4_msgs::msg::VehicleOdometry & m)
  {
    const std::uint64_t px4_us = m.timestamp;
    mapper_.observe(px4_us, now().nanoseconds() / 1000);
    if (!mapper_.valid()) {return;}

    const agc_air::Vector3 pos_ned(m.position[0], m.position[1], m.position[2]);
    const agc_air::Vector3 vel_ned(m.velocity[0], m.velocity[1], m.velocity[2]);
    const agc_air::Vector3 ang_frd(m.angular_velocity[0], m.angular_velocity[1],
      m.angular_velocity[2]);
    const auto q_enu = agc_air::px4AttitudeToEnu(m.q);

    nav_msgs::msg::Odometry out;
    out.header.stamp = rclcpp::Time(mapper_.toRosMicros(px4_us));
    out.header.frame_id = world_frame_;
    out.child_frame_id = base_frame_;
    const agc_air::Vector3 p = agc_air::nedToEnu(pos_ned);
    out.pose.pose.position.x = p.x();
    out.pose.pose.position.y = p.y();
    out.pose.pose.position.z = p.z();
    out.pose.pose.orientation.x = q_enu.x();
    out.pose.pose.orientation.y = q_enu.y();
    out.pose.pose.orientation.z = q_enu.z();
    out.pose.pose.orientation.w = q_enu.w();

    const agc_air::Vector3 v = agc_air::nedToEnu(vel_ned);
    out.twist.twist.linear.x = v.x();
    out.twist.twist.linear.y = v.y();
    out.twist.twist.linear.z = v.z();

    const agc_air::Vector3 w = agc_air::frdToFlu(ang_frd);
    out.twist.twist.angular.x = w.x();
    out.twist.twist.angular.y = w.y();
    out.twist.twist.angular.z = w.z();

    odom_pub_->publish(out);
  }

  std::string odom_in_, status_in_, odom_out_, status_out_, world_frame_, base_frame_;
  double time_alpha_{0.05};
  agc_air::Px4TimeMapper mapper_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<Px4Adapter>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
