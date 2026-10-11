// task_manager —— M0 最小任务门控（W6 加厚）
//
// * 服务 /agc/task/start（std_srvs/Trigger）；状态 /agc/task/state（AgcTaskState），1 Hz。
// * 目标 /agc/ground/goal（world）。
// * 门禁五项：{single_world, clock, odom_valid, cmd_vel 回环, reset 完成}。
// * 竞态修复（契约 #lessons）：**先 reset，后校验 odom**。
// * M0 不做成功/失败裁定；门控通过置 GROUND_APPROACH，仅此态允许地面驱动。

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <agc_msgs/msg/agc_task_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using Trigger = std_srvs::srv::Trigger;

class TaskManager : public rclcpp::Node
{
public:
  TaskManager()
  : Node("task_manager")
  {
    world_name_ = declare_parameter<std::string>("world_name", "agc_world");
    goal_x_ = declare_parameter<double>("goal_x", 4.0);
    goal_y_ = declare_parameter<double>("goal_y", 3.0);

    const auto odom_valid_topic =
      declare_parameter<std::string>("odom_valid_topic", "/agc/ground/odom_valid");
    const auto cmd_topic =
      declare_parameter<std::string>("cmd_topic", "/agc/ground/cmd_vel");
    const auto state_topic =
      declare_parameter<std::string>("state_topic", "/agc/task/state");
    const auto goal_topic =
      declare_parameter<std::string>("goal_topic", "/agc/ground/goal");

    odom_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      odom_valid_topic, 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {odom_valid_ = m->data;});
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_topic, 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr) {last_cmd_rx_ = now(); have_cmd_ = true;});

    state_pub_ = create_publisher<agc_msgs::msg::AgcTaskState>(state_topic, 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

    reset_client_node_ = std::make_shared<rclcpp::Node>("task_manager_reset_client");
    reset_client_ = reset_client_node_->create_client<Trigger>("/agc/scenario/reset");

    start_srv_ = create_service<Trigger>(
      "/agc/task/start",
      std::bind(&TaskManager::onStart, this, std::placeholders::_1, std::placeholders::_2));

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&TaskManager::publishState, this));
    goal_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&TaskManager::publishGoal, this));
  }

private:
  static const char * nameOf(uint8_t s)
  {
    switch (s) {
      case agc_msgs::msg::AgcTaskState::IDLE: return "IDLE";
      case agc_msgs::msg::AgcTaskState::GROUND_APPROACH: return "GROUND_APPROACH";
      case agc_msgs::msg::AgcTaskState::INFRA_FAILURE: return "INFRA_FAILURE";
      default: return "OTHER";
    }
  }

  void onStart(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res)
  {
    // 1) 先 reset（契约竞态修复）
    if (!reset_client_->wait_for_service(std::chrono::seconds(2))) {
      fail(res, "reset service unavailable");
      return;
    }
    auto future = reset_client_->async_send_request(std::make_shared<Trigger::Request>());
    if (rclcpp::spin_until_future_complete(reset_client_node_, future, std::chrono::seconds(8)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      fail(res, "reset call failed/timeout");
      return;
    }
    const bool reset_ok = future.get()->success;

    // 2) 再校验（reset 之后）
    std::string reason;
    if (!reset_ok) {reason = "reset_failed";}
    else if (now().seconds() <= 0.0) {reason = "clock_not_synced";}
    else if (!odom_valid_) {reason = "odom_invalid";}
    else if (!have_cmd_ || (now() - last_cmd_rx_).seconds() > 0.5) {reason = "cmd_vel_loopback_missing";}

    if (!reason.empty()) {
      fail(res, reason);
      return;
    }
    if (world_name_ != "agc_world") {fail(res, "single_world_mismatch"); return;}

    setState(agc_msgs::msg::AgcTaskState::GROUND_APPROACH, "ok");
    res->success = true;
    res->message = "started: GROUND_APPROACH";
    RCLCPP_INFO(get_logger(), "start OK -> GROUND_APPROACH");
  }

  void fail(const std::shared_ptr<Trigger::Response> & res, const std::string & reason)
  {
    setState(agc_msgs::msg::AgcTaskState::INFRA_FAILURE, reason);
    res->success = false;
    res->message = "start rejected: " + reason;
    RCLCPP_WARN(get_logger(), "start rejected: %s", reason.c_str());
  }

  void setState(uint8_t s, const std::string & reason)
  {
    state_ = s;
    reason_ = reason;
    publishState();
  }

  void publishState()
  {
    agc_msgs::msg::AgcTaskState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.state = state_;
    msg.state_name = nameOf(state_);
    msg.reason_code = reason_;
    msg.air_controller = "none";
    msg.ground_controller = state_ == agc_msgs::msg::AgcTaskState::GROUND_APPROACH ?
      "ground_policy_placeholder" : "none";
    msg.target_id = "";
    msg.target_xy_distance_m = std::nanf("");
    msg.target_obs_age_s = std::nanf("");
    msg.tracking_milestone_met = false;
    state_pub_->publish(msg);
  }

  void publishGoal()
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "world";
    goal.pose.position.x = goal_x_;
    goal.pose.position.y = goal_y_;
    goal.pose.orientation.w = 1.0;
    goal_pub_->publish(goal);
  }

  std::string world_name_;
  double goal_x_{4.0}, goal_y_{3.0};
  uint8_t state_{agc_msgs::msg::AgcTaskState::IDLE};
  std::string reason_;
  bool odom_valid_{false};
  bool have_cmd_{false};
  rclcpp::Time last_cmd_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odom_valid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Publisher<agc_msgs::msg::AgcTaskState>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  std::shared_ptr<rclcpp::Node> reset_client_node_;
  rclcpp::Client<Trigger>::SharedPtr reset_client_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_;
  rclcpp::TimerBase::SharedPtr state_timer_, goal_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManager>());
  rclcpp::shutdown();
  return 0;
}