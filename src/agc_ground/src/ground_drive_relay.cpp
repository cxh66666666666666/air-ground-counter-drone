// ground_drive_relay —— 地面执行中继（M0）
//
// 契约唯一命令口 /agc/ground/cmd_vel（TwistStamped，车体系）→ GZ /model/agc_car/cmd_vel
// （geometry_msgs/Twist，无戳、模型系），经 ros_gz 桥下行。本节点是**唯一**命令车体运动者。
// 看门狗：上游超时（默认 0.3 s）发零速。
//
// R-009（评审风险）：接收年龄改为**单调钟**（与 ROS 时钟完全解耦，时钟冻结/回拨不影响超时发零）；
// 并对 Header（frame_id / 采样年龄 / stamp 倒退）与值（有限性 / 模型上限）做前置校验，
// 非法 / 陈旧命令一律**拒收**（不更新已接受值），避免用旧/非法值覆盖。末端（GZ DiffDrive）失效停车
// 机制**不在本节点**，由后续子任务负责。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class GroundDriveRelay : public rclcpp::Node
{
public:
  GroundDriveRelay()
  : Node("ground_drive_relay")
  {
    in_topic_ = declare_parameter<std::string>("in_topic", "/agc/ground/cmd_vel");
    out_topic_ = declare_parameter<std::string>("out_topic", "/model/agc_car/cmd_vel");
    watchdog_s_ = declare_parameter<double>("watchdog_s", 0.3);
    // —— R-009 新增（默认保守值；由 launch/config 单一注入，契约 #params）——
    expected_frame_id_ = declare_parameter<std::string>("expected_frame_id", "car/base_link");
    max_sample_age_s_ = declare_parameter<double>("max_sample_age_s", 0.5);
    frame_id_tolerance_ = declare_parameter<double>("frame_id_tolerance", 0.05);
    max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 1.5);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 2.0);

    pub_ = create_publisher<geometry_msgs::msg::Twist>(out_topic_, 10);
    sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      in_topic_, 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr m) {on_command(*m);});
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&GroundDriveRelay::tick, this));
  }

private:
  void on_command(const geometry_msgs::msg::TwistStamped & m)
  {
    // (b-1) frame_id 必须为期望车体系（默认 car/base_link，契约 #namespace）；错帧判为链路异常。
    if (m.header.frame_id != expected_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), throttle_clock_, 1000, "reject: unexpected frame_id='%s'",
        m.header.frame_id.c_str());
      return;
    }

    // (b-2) 采样年龄（ROS 钟）：防「上游给出陈旧采样」（延迟/积压）。
    // 注意：ROS 时钟冻结 / 回拨时 (now - stamp) 恒为 0，本规则单独**不足以防时钟异常**，
    // 故须与下面的「stamp 倒退」规则并用。
    const double age = (now() - rclcpp::Time(m.header.stamp)).seconds();
    if (age > max_sample_age_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), throttle_clock_, 1000, "reject: sample age %.3f s > %.3f s",
        age, max_sample_age_s_);
      return;
    }

    // (b-3) stamp 倒退（与时钟无关）：防「时钟冻结/回拨时旧命令重放」（ROS 年龄规则失效的场景）。
    // 维护已接受的最大 stamp；新 stamp 明显小于之（超过容差）即视为旧命令重放 → 拒收。
    const double stamp = rclcpp::Time(m.header.stamp).seconds();
    if (have_stamp_ && stamp < max_stamp_ - frame_id_tolerance_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), throttle_clock_, 1000, "reject: stamp %.3f s behind max %.3f s (replay?)",
        stamp, max_stamp_);
      return;
    }

    // (c) 有限性：任一分量 NaN/±inf → 上游链路异常，整体拒收。
    const auto & t = m.twist;
    if (!std::isfinite(t.linear.x) || !std::isfinite(t.linear.y) || !std::isfinite(t.linear.z) ||
      !std::isfinite(t.angular.x) || !std::isfinite(t.angular.y) || !std::isfinite(t.angular.z))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 1000, "reject: non-finite twist");
      return;
    }

    // holonomic=false（差速车）：本车只消费 linear.x / angular.z。linear.y≠0 意味着上游用错
    // 模型（holonomic 规划）→ 判为链路异常并**拒收**（保守；若改为清零须在此注明理由）。
    if (std::fabs(t.linear.y) > 1e-6) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), throttle_clock_, 1000, "reject: linear.y=%.3f on holonomic=false car",
        t.linear.y);
      return;
    }

    // (d) 按模型上限限幅（v_max / ω_max 与 agc_sim/config/car.yaml 共享同一事实）。
    const double v = std::clamp(t.linear.x, -max_linear_velocity_, max_linear_velocity_);
    const double w = std::clamp(t.angular.z, -max_angular_velocity_, max_angular_velocity_);
    if (v != t.linear.x || w != t.angular.z) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), throttle_clock_, 1000, "clamp: v %.3f->%.3f, w %.3f->%.3f",
        t.linear.x, v, t.angular.z, w);
    }

    // 接受：仅保留被消费分量（差速车 linear.x / angular.z），其余置零。
    last_.linear.x = v;
    last_.angular.z = w;
    last_.linear.y = last_.linear.z = 0.0;
    last_.angular.x = last_.angular.y = 0.0;
    have_ = true;
    have_stamp_ = true;
    max_stamp_ = std::max(max_stamp_, stamp);
    last_rx_mono_ = std::chrono::steady_clock::now();
  }

  void tick()
  {
    // 看门狗：单调钟判接收年龄（与 ROS 时钟无关）→ 超时 / 从未收到即发零速。
    const double age_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_rx_mono_).count();
    geometry_msgs::msg::Twist out;
    if (have_ && age_s < watchdog_s_) {
      out = last_;
    }
    pub_->publish(out);
  }

  std::string in_topic_, out_topic_, expected_frame_id_;
  // 节流日志用**稳定钟**：不随 ROS 时钟冻结/回拨而失效（R-009 场景下仍能给出告警）。
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
  double watchdog_s_{0.3};
  double max_sample_age_s_{0.5};
  double frame_id_tolerance_{0.05};
  double max_linear_velocity_{1.5};
  double max_angular_velocity_{2.0};
  bool have_{false};
  bool have_stamp_{false};
  double max_stamp_{0.0};
  std::chrono::steady_clock::time_point last_rx_mono_{};
  geometry_msgs::msg::Twist last_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundDriveRelay>());
  rclcpp::shutdown();
  return 0;
}
