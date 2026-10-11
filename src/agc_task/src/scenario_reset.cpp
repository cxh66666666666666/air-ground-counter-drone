// scenario_reset —— M0 最小场景复位（W6 加厚）
//
// * 服务 /agc/scenario/reset（std_srvs/Trigger）：经 GZ set_pose 把 agc_car / intruder_uav
//   置回 spawn；受运行期门控（仅 IDLE / 终态可复位，契约 #lessons）。
// * 启动时执行一次初始复位。
// * 复位后发布 /agc/scenario/car_start（PoseStamped），供 ground_odom_adapter 重算 odom offset。

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <agc_msgs/msg/agc_task_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <std_srvs/srv/trigger.hpp>

using Trigger = std_srvs::srv::Trigger;
using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;

class ScenarioReset : public rclcpp::Node
{
public:
  ScenarioReset()
  : Node("scenario_reset")
  {
    world_name_ = declare_parameter<std::string>("world_name", "agc_world");
    car_x_ = declare_parameter<double>("car_x", -4.0);
    car_y_ = declare_parameter<double>("car_y", -3.0);
    car_z_ = declare_parameter<double>("car_z", 0.05);
    car_yaw_ = declare_parameter<double>("car_yaw", 0.0);
    target_x_ = declare_parameter<double>("target_x", 6.5);
    target_y_ = declare_parameter<double>("target_y", 2.5);
    target_z_ = declare_parameter<double>("target_z", 2.0);
    target_yaw_ = declare_parameter<double>("target_yaw", 0.0);

    const auto state_topic =
      declare_parameter<std::string>("state_topic", "/agc/task/state");
    const auto car_start_topic =
      declare_parameter<std::string>("car_start_topic", "/agc/scenario/car_start");

    state_sub_ = create_subscription<agc_msgs::msg::AgcTaskState>(
      state_topic, 10, [this](agc_msgs::msg::AgcTaskState::SharedPtr m) {state_ = m->state;});
    car_start_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(car_start_topic, 10);

    client_node_ = std::make_shared<rclcpp::Node>("scenario_reset_client");
    set_pose_client_ = client_node_->create_client<SetEntityPose>(
      "/world/" + world_name_ + "/set_pose");

    reset_srv_ = create_service<Trigger>(
      "/agc/scenario/reset",
      std::bind(&ScenarioReset::onReset, this, std::placeholders::_1, std::placeholders::_2));

    // 启动初始复位（延迟等待 gz / 桥就绪）
    startup_timer_ = create_wall_timer(
      std::chrono::seconds(4), [this]() {
        startup_timer_->cancel();
        RCLCPP_INFO(get_logger(), "initial reset: %s", doReset(true) ? "ok" : "FAILED");
      });
  }

private:
  bool isResettable() const
  {
    return state_ == agc_msgs::msg::AgcTaskState::IDLE ||
           state_ == agc_msgs::msg::AgcTaskState::TRACKING_SUCCESS ||
           state_ == agc_msgs::msg::AgcTaskState::MISSION_FAILED ||
           state_ == agc_msgs::msg::AgcTaskState::ABORTED ||
           state_ == agc_msgs::msg::AgcTaskState::INFRA_FAILURE;
  }

  bool setPose(const std::string & name, double x, double y, double z, double yaw)
  {
    if (!set_pose_client_->wait_for_service(std::chrono::seconds(2))) {
      return false;
    }
    auto req = std::make_shared<SetEntityPose::Request>();
    req->entity.name = name;
    req->entity.type = 2;  // MODEL
    req->pose.position.x = x;
    req->pose.position.y = y;
    req->pose.position.z = z;
    req->pose.orientation.z = std::sin(yaw * 0.5);
    req->pose.orientation.w = std::cos(yaw * 0.5);
    auto future = set_pose_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }
    return future.get()->success;
  }

  bool doReset(bool bypass_gate)
  {
    if (!bypass_gate && !isResettable()) {
      RCLCPP_WARN(get_logger(), "reset denied: not in resettable state (%u)", state_);
      return false;
    }
    const bool car_ok = setPose("agc_car", car_x_, car_y_, car_z_, car_yaw_);
    const bool tgt_ok = setPose("intruder_uav", target_x_, target_y_, target_z_, target_yaw_);
    publishCarStart();
    return car_ok && tgt_ok;
  }

  void publishCarStart()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.pose.position.x = car_x_;
    msg.pose.position.y = car_y_;
    msg.pose.position.z = car_z_;
    msg.pose.orientation.z = std::sin(car_yaw_ * 0.5);
    msg.pose.orientation.w = std::cos(car_yaw_ * 0.5);
    car_start_pub_->publish(msg);
  }

  void onReset(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res)
  {
    const bool ok = doReset(false);
    res->success = ok;
    res->message = ok ? "reset ok" : "reset denied/failed";
  }

  std::string world_name_;
  double car_x_, car_y_, car_z_, car_yaw_;
  double target_x_, target_y_, target_z_, target_yaw_;
  uint8_t state_{agc_msgs::msg::AgcTaskState::IDLE};
  rclcpp::Subscription<agc_msgs::msg::AgcTaskState>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr car_start_pub_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<SetEntityPose>::SharedPtr set_pose_client_;
  rclcpp::Service<Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScenarioReset>());
  rclcpp::shutdown();
  return 0;
}