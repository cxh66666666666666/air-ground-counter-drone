// ground_policy_placeholder —— M0 占位策略（RL 替身）
//
// 最简 go-to-goal：订阅 /agc/ground/goal 与 /agc/ground/odom，输出 TwistStamped 到
// /agc/ground/cmd_vel_raw（车体系）。仅当 task state == GROUND_APPROACH 时输出非零，
// 否则输出零速（契约：仅 GROUND_APPROACH 允许地面驱动）。W5 用真 RL + collision_monitor 替换。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <agc_msgs/msg/agc_task_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace
{
double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
double wrapPi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}
}  // namespace

class GroundPolicyPlaceholder : public rclcpp::Node
{
public:
  GroundPolicyPlaceholder()
  : Node("ground_policy_placeholder")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "car/base_link");
    linear_speed_ = declare_parameter<double>("linear_speed", 0.3);
    yaw_gain_ = declare_parameter<double>("yaw_gain", 1.5);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 1.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.2);

    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/agc/ground/goal");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/agc/ground/odom");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/agc/task/state");
    const auto cmd_raw_topic =
      declare_parameter<std::string>("cmd_raw_topic", "/agc/ground/cmd_vel_raw");

    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_raw_topic, 10);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
        goal_x_ = m->pose.position.x; goal_y_ = m->pose.position.y; have_goal_ = true;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        x_ = m->pose.pose.position.x; y_ = m->pose.pose.position.y;
        yaw_ = yawFromQuat(m->pose.pose.orientation); have_odom_ = true;
      });
    state_sub_ = create_subscription<agc_msgs::msg::AgcTaskState>(
      state_topic, 10,
      [this](agc_msgs::msg::AgcTaskState::SharedPtr m) {state_ = m->state;});

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&GroundPolicyPlaceholder::tick, this));
  }

private:
  void tick()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = base_frame_;

    const bool active =
      have_goal_ && have_odom_ && state_ == agc_msgs::msg::AgcTaskState::GROUND_APPROACH;
    if (active) {
      const double dx = goal_x_ - x_;
      const double dy = goal_y_ - y_;
      const double dist = std::hypot(dx, dy);
      if (dist > goal_tolerance_) {
        const double desired = std::atan2(dy, dx);
        const double err = wrapPi(desired - yaw_);
        cmd.twist.angular.z = std::clamp(yaw_gain_ * err, -max_yaw_rate_, max_yaw_rate_);
        // 航向偏差大时先原地转向，否则匀速前进（避免画小圈不收敛）
        cmd.twist.linear.x = (std::fabs(err) < 0.5) ? linear_speed_ : 0.0;
      }
    }
    pub_->publish(cmd);
  }

  std::string base_frame_;
  double linear_speed_{0.3}, yaw_gain_{1.5}, max_yaw_rate_{1.0}, goal_tolerance_{0.2};
  bool have_goal_{false}, have_odom_{false};
  double goal_x_{0.0}, goal_y_{0.0}, x_{0.0}, y_{0.0}, yaw_{0.0};
  uint8_t state_{agc_msgs::msg::AgcTaskState::IDLE};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<agc_msgs::msg::AgcTaskState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundPolicyPlaceholder>());
  rclcpp::shutdown();
  return 0;
}