// ground_odom_adapter —— 最小地面 odom 归一（M0 提前落地，W4 加厚）
//
// 职责：
//   * 订阅 GZ DiffDrive 原始 odom（/agc/sim/ground_odom_raw，AGV 自身 odom 帧），
//     以及场景复位发布的 car_start（/agc/scenario/car_start）。
//   * 归一为 world 系的 /agc/ground/odom，并发布 TF world→car/base_link 与 odom_valid。
//
// 世界位姿来源：DiffDrive odom 只积分不自更新（teleport 不会重置），故以
//   world = offset ∘ odom 组合；offset 在收到 car_start 时按
//   offset = car_start ∘ inverse(odom_now) 重算（teleport 安全）。
//
// 帧名遵循契约：机体系固定 car/base_link（禁止裸 base_link）。

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
}  // namespace

class GroundOdomAdapter : public rclcpp::Node
{
public:
  GroundOdomAdapter()
  : Node("ground_odom_adapter")
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    base_frame_ = declare_parameter<std::string>("base_frame", "car/base_link");
    odom_raw_topic_ =
      declare_parameter<std::string>("odom_raw_topic", "/agc/sim/ground_odom_raw");
    car_start_topic_ =
      declare_parameter<std::string>("car_start_topic", "/agc/scenario/car_start");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/agc/ground/odom");
    valid_topic_ = declare_parameter<std::string>("valid_topic", "/agc/ground/odom_valid");
    car_start_x_ = declare_parameter<double>("car_start_x", -4.0);
    car_start_y_ = declare_parameter<double>("car_start_y", -3.0);
    car_start_yaw_ = declare_parameter<double>("car_start_yaw", 0.0);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(valid_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    car_start_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      car_start_topic_, 10,
      std::bind(&GroundOdomAdapter::onCarStart, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_raw_topic_, 20,
      std::bind(&GroundOdomAdapter::onOdom, this, std::placeholders::_1));
  }

private:
  void onCarStart(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    computeOffset(
      msg->pose.position.x, msg->pose.position.y,
      yawFromQuat(msg->pose.orientation), "car_start msg");
  }

  // offset = (cx,cy,cyaw) ∘ inverse(odom_now)，使 world(odom_now) = (cx,cy,cyaw)
  void computeOffset(double cx, double cy, double cyaw, const char * src)
  {
    if (have_odom_) {
      const double inv_x = -(std::cos(last_yaw_) * last_x_ + std::sin(last_yaw_) * last_y_);
      const double inv_y = -(-std::sin(last_yaw_) * last_x_ + std::cos(last_yaw_) * last_y_);
      off_x_ = cx + (std::cos(cyaw) * inv_x - std::sin(cyaw) * inv_y);
      off_y_ = cy + (std::sin(cyaw) * inv_x + std::cos(cyaw) * inv_y);
      off_yaw_ = cyaw - last_yaw_;
    } else {
      off_x_ = cx;
      off_y_ = cy;
      off_yaw_ = cyaw;
    }
    have_offset_ = true;
    RCLCPP_INFO(
      get_logger(), "odom offset applied from %s: (%.3f, %.3f, yaw=%.3f)",
      src, cx, cy, cyaw);
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_x_ = msg->pose.pose.position.x;
    last_y_ = msg->pose.pose.position.y;
    last_yaw_ = yawFromQuat(msg->pose.pose.orientation);
    have_odom_ = true;

    std_msgs::msg::Bool valid;
    if (!have_offset_) {
      // 首次 odom：用 car_start 参数初始化 offset（odom 从 0 起）
      computeOffset(car_start_x_, car_start_y_, car_start_yaw_, "params");
    }

    const double wx = off_x_ + std::cos(off_yaw_) * last_x_ - std::sin(off_yaw_) * last_y_;
    const double wy = off_y_ + std::sin(off_yaw_) * last_x_ + std::cos(off_yaw_) * last_y_;
    const double wyaw = off_yaw_ + last_yaw_;

    nav_msgs::msg::Odometry out = *msg;
    out.header.frame_id = world_frame_;
    out.child_frame_id = base_frame_;
    out.pose.pose.position.x = wx;
    out.pose.pose.position.y = wy;
    out.pose.pose.position.z = msg->pose.pose.position.z;
    out.pose.pose.orientation = quatFromYaw(wyaw);
    odom_pub_->publish(out);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = wx;
    tf.transform.translation.y = wy;
    tf.transform.translation.z = msg->pose.pose.position.z;
    tf.transform.rotation = quatFromYaw(wyaw);
    tf_broadcaster_->sendTransform(tf);

    valid.data = true;
    valid_pub_->publish(valid);
  }

  std::string world_frame_, base_frame_, odom_raw_topic_, car_start_topic_;
  std::string odom_topic_, valid_topic_;
  bool have_odom_{false};
  bool have_offset_{false};
  double last_x_{0.0}, last_y_{0.0}, last_yaw_{0.0};
  double off_x_{0.0}, off_y_{0.0}, off_yaw_{0.0};
  double car_start_x_{-4.0}, car_start_y_{-3.0}, car_start_yaw_{0.0};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr car_start_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundOdomAdapter>());
  rclcpp::shutdown();
  return 0;
}