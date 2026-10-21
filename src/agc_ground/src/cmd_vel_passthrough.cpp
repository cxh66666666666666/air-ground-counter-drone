// cmd_vel_passthrough —— W7-b（T1 / E1）「无 shield 对照臂」的恒等透传（**纯仿真**）。
//
// 用途：对照臂 `shield:=false`（[ADR-0070](../../../docs/adr/0070-w7b-research-propositions-and-rl-role-adjudication.md)(d)(f)）
// ——把安全层**输入** `/agc/ground/cmd_vel_raw` **原样**转发到唯一执行口 `/agc/ground/cmd_vel`
// （**保持** header.stamp / frame_id），使「无安全层修正」臂与「有安全层」臂在**同一命令链结构**下可比。
//
// 范围声明（**不得外推**）：本节点仅用于**可控仿真研究对照**，**不**用于实际部署；
// 该臂无安全层，碰撞风险由仿真承担。属 F-06「with-shield / 等预算对照臂」的**无 shield 一支**。
//
// 唯一执行口纪律：本节点是 `/agc/ground/cmd_vel` 的发布者（有 shield 时该角色属 `collision_monitor`）；
// 上游只有命令源，**不**构成第二命令者（契约 #execution-port）。

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class CmdVelPassthrough : public rclcpp::Node
{
public:
  CmdVelPassthrough()
  : Node("cmd_vel_passthrough")
  {
    const auto in_topic = declare_parameter<std::string>("in_topic", "/agc/ground/cmd_vel_raw");
    const auto out_topic = declare_parameter<std::string>("out_topic", "/agc/ground/cmd_vel");

    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(out_topic, 10);
    sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      in_topic, 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr m) {pub_->publish(*m);});

    RCLCPP_WARN(get_logger(),
      "无 shield 对照臂（纯仿真，**不**用于部署）：%s → %s（恒等透传）",
      in_topic.c_str(), out_topic.c_str());
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelPassthrough>());
  rclcpp::shutdown();
  return 0;
}
