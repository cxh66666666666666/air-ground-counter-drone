// ground_odom_adapter —— 地面 odom 归一（W4-a 加固，spec R2）
//
// 位姿来源：**W4 真值旁路**（`/agc/gt/agc_car`，gz 物理引擎产生的世界真值）。
// 归一为 world 系 `/agc/ground/odom`（`world`→`car/base_link`）+ TF + `/agc/ground/odom_valid`。
//
// 与 M0 版差异（W4-a）：
//   * 位姿改用真值旁路（契约 world 基准）；**去除** car_start offset 标定路径；
//   * DiffDrive odom 不再是位姿来源（速度可由真值差分得到）。
//   * twist 仍按 ROS Odometry 约定表达在 child frame：world 速度短窗差分后旋转到车体系。
//
// 帧名遵循契约 #namespace：机体系固定 car/base_link（禁止裸 base_link）。

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "agc_perception/math_utils.hpp"

namespace
{
const size_t kHistoryMax = 500;  // 与旧树行为对齐（deque(maxlen=500)）

struct Sample
{
  double t, x, y, yaw;
};
}  // namespace

class GroundOdomAdapter : public rclcpp::Node
{
public:
  GroundOdomAdapter()
  : Node("ground_odom_adapter")
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    base_frame_ = declare_parameter<std::string>("base_frame", "car/base_link");
    gt_topic_ = declare_parameter<std::string>("gt_topic", "/agc/gt/agc_car");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/agc/ground/odom");
    valid_topic_ = declare_parameter<std::string>("valid_topic", "/agc/ground/odom_valid");
    rate_hz_ = declare_parameter<double>("rate_hz", 30.0);
    diff_window_s_ = declare_parameter<double>("diff_window_s", 0.15);
    odom_max_age_s_ = declare_parameter<double>("odom_max_age_s", 0.5);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(valid_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    gt_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      gt_topic_, 10,
      std::bind(&GroundOdomAdapter::onGt, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1e-3));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GroundOdomAdapter::tick, this));

    RCLCPP_INFO(
      get_logger(), "ground_odom_adapter：真值源 %s → %s（%.1f Hz）",
      gt_topic_.c_str(), odom_topic_.c_str(), rate_hz_);
  }

private:
  void onGt(const geometry_msgs::msg::PoseStamped & msg)
  {
    cur_pos_ = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
    cur_quat_ = {
      msg.pose.orientation.x, msg.pose.orientation.y,
      msg.pose.orientation.z, msg.pose.orientation.w};
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    history_.push_back(Sample{t, cur_pos_[0], cur_pos_[1], agc::yawFromQuat(msg.pose.orientation)});
    if (history_.size() > kHistoryMax) {history_.pop_front();}
    last_rx_sim_ = t;
    have_cur_ = true;
  }

  double age() const
  {
    const double now_s = now().seconds();
    if (now_s > 0.0 && last_rx_sim_ > 0.0) {return std::max(0.0, now_s - last_rx_sim_);}
    return 0.0;
  }

  void velocity(std::array<double, 3> * v_world, double * wz) const
  {
    *v_world = {{0.0, 0.0, 0.0}};
    *wz = 0.0;
    if (history_.size() < 2) {return;}
    const Sample & s1 = history_.back();
    Sample s0 = history_.front();
    for (const Sample & e : history_) {
      if (s1.t - e.t <= diff_window_s_) {s0 = e; break;}
    }
    const double dt = s1.t - s0.t;
    if (dt <= 1e-6) {return;}
    (*v_world)[0] = (s1.x - s0.x) / dt;
    (*v_world)[1] = (s1.y - s0.y) / dt;
    (*v_world)[2] = 0.0;
    *wz = agc::normalizeAngle(s1.yaw - s0.yaw) / dt;
  }

  void publishValid(bool ok)
  {
        std_msgs::msg::Bool m;
    m.data = ok;
    valid_pub_->publish(m);
  }

  void tick()
  {
    if (!have_cur_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "ground odom：等待真值 %s", gt_topic_.c_str());
      publishValid(false);
      return;
    }
    if (age() > odom_max_age_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ground odom stale: age=%.3f s > %.3f s",
        age(), odom_max_age_s_);
      publishValid(false);
      return;
    }

    std::array<double, 3> v_world;
    double wz = 0.0;
    velocity(&v_world, &wz);

    // world 速度 → 车体系（R(q)^T · v_world）
    tf2::Quaternion q(cur_quat_[0], cur_quat_[1], cur_quat_[2], cur_quat_[3]);
    tf2::Matrix3x3 R(q);
    const tf2::Vector3 v_w(v_world[0], v_world[1], v_world[2]);
    const tf2::Vector3 v_b = R.transpose() * v_w;

    // 同一仿真时刻重复发布 TF 会触发 tf2 TF_REPEATED_DATA，去重
    const rclcpp::Time stamp = now();
    if (has_last_pub_ && stamp == last_pub_stamp_) {return;}
    last_pub_stamp_ = stamp;
    has_last_pub_ = true;

    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = world_frame_;
    o.child_frame_id = base_frame_;
    o.pose.pose.position.x = cur_pos_[0];
    o.pose.pose.position.y = cur_pos_[1];
    o.pose.pose.position.z = cur_pos_[2];
    o.pose.pose.orientation.x = cur_quat_[0];
    o.pose.pose.orientation.y = cur_quat_[1];
    o.pose.pose.orientation.z = cur_quat_[2];
    o.pose.pose.orientation.w = cur_quat_[3];
    o.twist.twist.linear.x = v_b.x();
    o.twist.twist.linear.y = v_b.y();
    o.twist.twist.linear.z = v_b.z();
    o.twist.twist.angular.x = 0.0;
    o.twist.twist.angular.y = 0.0;
    o.twist.twist.angular.z = wz;
    odom_pub_->publish(o);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = cur_pos_[0];
    tf.transform.translation.y = cur_pos_[1];
    tf.transform.translation.z = cur_pos_[2];
    tf.transform.rotation.x = cur_quat_[0];
    tf.transform.rotation.y = cur_quat_[1];
    tf.transform.rotation.z = cur_quat_[2];
    tf.transform.rotation.w = cur_quat_[3];
    tf_broadcaster_->sendTransform(tf);

    publishValid(true);
  }

  std::string world_frame_, base_frame_, gt_topic_, odom_topic_, valid_topic_;
  double rate_hz_{30.0}, diff_window_s_{0.15}, odom_max_age_s_{0.5};

  bool have_cur_{false};
  std::array<double, 3> cur_pos_{{0.0, 0.0, 0.0}};
  std::array<double, 4> cur_quat_{{0.0, 0.0, 0.0, 1.0}};
  double last_rx_sim_{0.0};
  std::deque<Sample> history_;
  rclcpp::Time last_pub_stamp_;
  bool has_last_pub_{false};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundOdomAdapter>());
  rclcpp::shutdown();
  return 0;
}