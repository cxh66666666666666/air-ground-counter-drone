// ground_drive_relay —— 地面执行中继（M0）
//
// 契约唯一命令口 /agc/ground/cmd_vel（TwistStamped，车体系）→ GZ /model/agc_car/cmd_vel
// （geometry_msgs/Twist，无戳、模型系），经 ros_gz 桥下行。本节点是**唯一**命令车体运动者。
// 看门狗：上游超时（默认 0.3 s）发零速。

#include <chrono>
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

    pub_ = create_publisher<geometry_msgs::msg::Twist>(out_topic_, 10);
    sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      in_topic_, 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr m) {
        last_ = m->twist; have_ = true; last_rx_ = now();
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&GroundDriveRelay::tick, this));
  }

private:
  void tick()
  {
    geometry_msgs::msg::Twist out;
    const bool fresh = have_ && (now() - last_rx_).seconds() < watchdog_s_;
    if (fresh) {
      out = last_;
    }
    pub_->publish(out);
  }

  std::string in_topic_, out_topic_;
  double watchdog_s_{0.3};
  bool have_{false};
  rclcpp::Time last_rx_{0, 0, RCL_ROS_TIME};
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