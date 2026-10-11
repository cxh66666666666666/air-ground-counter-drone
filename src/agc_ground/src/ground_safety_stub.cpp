// ground_safety_stub —— M0 安全桩（collision_monitor 的恒等透传替身）
//
// 订阅 /agc/ground/cmd_vel_raw，原样发布到唯一执行口 /agc/ground/cmd_vel；
// 带看门狗：上游超时（默认 0.3 s）则发零速。W5 用真 Nav2 collision_monitor 替换。

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class GroundSafetyStub : public rclcpp::Node
{
public:
  GroundSafetyStub()
  : Node("ground_safety_stub")
  {
    in_topic_ = declare_parameter<std::string>("in_topic", "/agc/ground/cmd_vel_raw");
    out_topic_ = declare_parameter<std::string>("out_topic", "/agc/ground/cmd_vel");
    base_frame_ = declare_parameter<std::string>("base_frame", "car/base_link");
    watchdog_s_ = declare_parameter<double>("watchdog_s", 0.3);

    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(out_topic_, 10);
    sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      in_topic_, 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr m) {
        last_ = *m; have_ = true; last_rx_ = now();
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&GroundSafetyStub::tick, this));
  }

private:
  void tick()
  {
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = now();
    out.header.frame_id = base_frame_;
    const bool fresh = have_ && (now() - last_rx_).seconds() < watchdog_s_;
    if (fresh) {
      out.twist = last_.twist;
    }
    pub_->publish(out);
  }

  std::string in_topic_, out_topic_, base_frame_;
  double watchdog_s_{0.3};
  bool have_{false};
  rclcpp::Time last_rx_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::TwistStamped last_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundSafetyStub>());
  rclcpp::shutdown();
  return 0;
}