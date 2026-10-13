// takeoff_controller —— 起飞 / 悬停 / Offboard 接管 + 控制权交接（spec R4）
//
// 契约 [唯一执行口]（空中控制权）：起飞阶段归本节点；`/agc/air/release_control` 后交轨迹跟踪器。
// 流程对齐旧 `drone_launch_controller` 的**行为事实**（先持续发 setpoint 流，再解锁、切 Offboard、爬升），
// 但链路为全新 uXRCE-DDS 实现（ADR-0005），不继承旧实现。
// lifecycle managed node（Nav2 吸收点）。

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "agc_air/agc_lifecycle_node.hpp"
#include "agc_air/offboard_interface.hpp"

using agc_air::AgcLifecycleNode;
using agc_air::OffboardInterface;
using agc_air::OffboardParams;

class TakeoffController : public AgcLifecycleNode
{
public:
  TakeoffController()
  : AgcLifecycleNode("takeoff_controller")
  {
  }

protected:
  CallbackReturn on_configure_impl() override
  {
    takeoff_altitude_m_ = declare_parameter<double>("takeoff_altitude_m", 2.5);
    hover_tolerance_m_ = declare_parameter<double>("hover_tolerance_m", 0.25);
    rate_hz_ = declare_parameter<double>("setpoint_rate_hz", 50.0);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.5);
    arm_after_n_ = declare_parameter<int>("arm_after_setpoints", 50);
    offboard_after_n_ = declare_parameter<int>("offboard_after_setpoints", 60);
    arm_retry_ticks_ = declare_parameter<int>("arm_retry_ticks", 100);
    active_controller_topic_ =
      declare_parameter<std::string>("active_controller_topic", "/agc/air/active_controller");
    release_service_ =
      declare_parameter<std::string>("release_service", "/agc/air/release_control");

    OffboardParams p;
    p.odom_topic = declare_parameter<std::string>("odom_topic", "/agc/air/odom");
    p.status_topic = declare_parameter<std::string>("status_topic", "/agc/air/status");
    p.setpoint_dt = 1.0 / rate_hz_;
    p.odom_timeout_s = odom_timeout_s_;
    offboard_ = std::make_unique<OffboardInterface>(*this, p);
    offboard_->configure();

    // active_controller 需让后加入的跟踪器拿到当前控制者 → transient_local
    active_pub_ = create_publisher<std_msgs::msg::String>(
      active_controller_topic_, rclcpp::QoS(1).transient_local().reliable());

    release_srv_ = create_service<std_srvs::srv::Trigger>(
      release_service_,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {onRelease(res);});

    RCLCPP_INFO(get_logger(), "takeoff_controller 就绪（起飞高度 %.1f m）", takeoff_altitude_m_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate_impl() override
  {
    offboard_->activate();
    active_pub_->on_activate();
    publishActive("takeoff_controller");

    state_ = State::kWaitOdom;
    counter_ = 0;
    released_ = false;
    arm_sent_ = false;
    offboard_requested_ = false;

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_), [this]() {tick();});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate_impl() override
  {
    if (timer_) {timer_->cancel();}
    timer_.reset();
    offboard_->deactivate();
    if (active_pub_) {active_pub_->on_deactivate();}
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup_impl() override
  {
    timer_.reset();
    release_srv_.reset();
    active_pub_.reset();
    if (offboard_) {offboard_->cleanup();}
    offboard_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  enum class State {kWaitOdom, kStream, kClimb, kHover, kReleased};

  void onRelease(const std::shared_ptr<std_srvs::srv::Trigger::Response> & res)
  {
    if (released_) {
      res->success = false;
      res->message = "控制权已移交";
      return;
    }
    released_ = true;
    state_ = State::kReleased;   // 立即停止发 setpoint：跟踪器在下一拍接管（交接窗口 ≈ 一拍）
    offboard_->releaseSetpointPublishers();   // 真正让出唯一发布者地位（见该函数注释）
    publishActive("trajectory_tracker");
    res->success = true;
    res->message = "已移交控制权给 trajectory_tracker";
    RCLCPP_INFO(get_logger(), "release_control：控制权移交 trajectory_tracker");
  }

  void tick()
  {
    if (state_ == State::kReleased) {return;}
    if (!offboard_->odomValid()) {return;}

    const auto & odom = offboard_->odom();
    const agc_air::Vector3 pos(odom.pose.pose.position.x, odom.pose.pose.position.y,
      odom.pose.pose.position.z);

    if (state_ == State::kWaitOdom) {
      launch_point_ = pos;   // 机体着陆点（地面基准）
      state_ = State::kStream;
    }

    // 持续发 offboard_control_mode + setpoint 流（PX4 切 Offboard 的前置条件）
    offboard_->publishControlMode();

    // 高度以**起飞时刻机体高度**为地面基准：PX4 局部原点与世界原点不重合，
    // 直接给绝对高度会落在错误层高（见 evidence 记录）。
    const agc_air::Vector3 target(launch_point_.x(), launch_point_.y(),
      state_ == State::kStream ? launch_point_.z() : launch_point_.z() + takeoff_altitude_m_);
    offboard_->publishSetpoint(target, agc_air::Vector3::Zero(), agc_air::Vector3::Zero(), 0.0);

    ++counter_;

    if (state_ == State::kStream) {
      // 先持续发 setpoint 流（PX4 切 Offboard 的前置条件），再解锁、再切 Offboard；
      // 预检未过时 PX4 会拒绝，故**周期重试**直到就绪。
      if (counter_ >= arm_after_n_ && !offboard_->armed() && (counter_ % arm_retry_ticks_ == 0)) {
        offboard_->sendArm(true);
        arm_sent_ = true;
      }
      if (arm_sent_ && (counter_ >= arm_after_n_ + offboard_after_n_)
        && !offboard_->offboard() && (counter_ % arm_retry_ticks_ == 0))
      {
        offboard_->sendOffboardMode();
        offboard_requested_ = true;
      }
      // 已解锁且进入 Offboard → 进入爬升
      if (offboard_->armed() && offboard_->offboard()) {
        state_ = State::kClimb;
      }
    } else if (state_ == State::kClimb) {
      if (std::abs(pos.z() - (launch_point_.z() + takeoff_altitude_m_)) < hover_tolerance_m_) {
        state_ = State::kHover;
        RCLCPP_INFO(get_logger(), "起飞完成，悬停（相对地面 %.2f m）", pos.z() - launch_point_.z());
      }
    } else if (state_ == State::kHover) {
      // 保持悬停（setpoint 已在上方按高度给出）
    }
  }

  void publishActive(const std::string & name)
  {
    if (!active_pub_ || !active_pub_->is_activated()) {return;}
    std_msgs::msg::String msg;
    msg.data = name;
    active_pub_->publish(msg);
  }

  double takeoff_altitude_m_{2.5};
  double hover_tolerance_m_{0.25};
  double rate_hz_{50.0};
  double odom_timeout_s_{0.5};
  int arm_after_n_{50};
  int offboard_after_n_{60};
  int arm_retry_ticks_{100};
  std::string active_controller_topic_, release_service_;

  std::unique_ptr<OffboardInterface> offboard_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr active_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  State state_{State::kWaitOdom};
  agc_air::Vector3 launch_point_{agc_air::Vector3::Zero()};
  int counter_{0};
  bool released_{false};
  bool arm_sent_{false};
  bool offboard_requested_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<TakeoffController>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
