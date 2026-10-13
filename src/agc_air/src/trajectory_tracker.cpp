// trajectory_tracker —— 轨迹跟踪 + setpoint 唯一发布者 + 冲突检测（spec R3/R4/R5/R7）
//
// 控制层用 agc_control 的 **pluginlib** 插件（轨迹生成器 + LQR 控制器），本节只做
// 「取参考 → 安全层 → LQR → 世界系 ENU 转 PX4 setpoint」的消费者工作（ADR-0018）。
// 接管后才成为 setpoint 发布者；用图内省检测多发布者冲突 `control_conflict`（对齐契约 [唯一执行口]）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <agc_msgs/msg/target_track_array.hpp>

#include "agc_air/agc_lifecycle_node.hpp"
#include "agc_air/offboard_interface.hpp"
#include "agc_air/px4_conversions.hpp"
#include "agc_air/reference_msg.hpp"
#include "agc_air/safety_shield.hpp"

#include "agc_control/controller.hpp"
#include "agc_control/param_reader.hpp"
#include "agc_control/trajectory_generator.hpp"

using agc_air::AgcLifecycleNode;
using agc_air::OffboardInterface;
using agc_air::OffboardParams;
using agc_air::SafetyShield;

class TrajectoryTracker : public AgcLifecycleNode
{
public:
  TrajectoryTracker()
  : AgcLifecycleNode("trajectory_tracker"),
    traj_loader_("agc_control", "agc_control::TrajectoryGenerator"),
    ctrl_loader_("agc_control", "agc_control::Controller")
  {
  }

protected:
  CallbackReturn on_configure_impl() override
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    traj_plugin_ = declare_parameter<std::string>(
      "trajectory_plugin", "agc_control/Figure8Trajectory");
    ctrl_plugin_ = declare_parameter<std::string>("controller_plugin", "agc_control/LqrController");
    traj_ns_ = declare_parameter<std::string>("trajectory_param_ns", "traj");
    ctrl_ns_ = declare_parameter<std::string>("controller_param_ns", "ctrl");
    rate_hz_ = declare_parameter<double>("setpoint_rate_hz", 50.0);
    ref_topic_ = declare_parameter<std::string>("ref_topic", "/agc/air/ref");
    tracking_ref_topic_ =
      declare_parameter<std::string>("tracking_ref_topic", "/agc/air/tracking_ref");
    target_topic_ = declare_parameter<std::string>("target_topic", "/agc/targets");
    active_topic_ = declare_parameter<std::string>("active_controller_topic",
        "/agc/air/active_controller");
    conflict_topic_ = declare_parameter<std::string>("conflict_topic", "/agc/air/control_conflict");
    rl_offset_topic_ = declare_parameter<std::string>("rl_offset_topic", "/agc/air/rl_offset");
    own_name_ = declare_parameter<std::string>("controller_name", "trajectory_tracker");
    tracking_ref_timeout_s_ = declare_parameter<double>("tracking_ref_timeout_s", 0.5);
    conflict_persist_s_ = declare_parameter<double>("conflict_persist_s", 1.0);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 2.0);

    OffboardParams p;
    p.odom_topic = declare_parameter<std::string>("odom_topic", "/agc/air/odom");
    p.status_topic = declare_parameter<std::string>("status_topic", "/agc/air/status");
    p.setpoint_dt = 1.0 / rate_hz_;
    offboard_ = std::make_unique<OffboardInterface>(*this, p);
    offboard_->configure();

    shield_.configure_parameters(get_node_parameters_interface(), "safety");
    RCLCPP_INFO(get_logger(),
      "安全层：enabled=%d 障碍余量=%.2f 目标余量=%.2f target_min_dist=%.1f 静态障碍 %zu 个",
      static_cast<int>(shield_.enabled()), shield_.obstacleMargin(), shield_.targetMargin(),
      shield_.targetMinDist(), shield_.obstacles().size());

    // 加载插件（同一实现供 air / ground 共用；不复制）
    trajectory_ = traj_loader_.createSharedInstance(traj_plugin_);
    controller_ = ctrl_loader_.createSharedInstance(ctrl_plugin_);
    agc_control::NodeParamReader traj_params(get_node_parameters_interface(), traj_ns_);
    agc_control::NodeParamReader ctrl_params(get_node_parameters_interface(), ctrl_ns_);
    trajectory_->configure(traj_params);
    controller_->configure(ctrl_params);

    ref_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      ref_topic_, rclcpp::QoS(1).reliable());
    conflict_pub_ = create_publisher<std_msgs::msg::Bool>(
      conflict_topic_, rclcpp::QoS(1).transient_local().reliable());
    active_pub_ = create_publisher<std_msgs::msg::String>(
      active_topic_, rclcpp::QoS(1).transient_local().reliable());

    tracking_ref_sub_ =
      create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      tracking_ref_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr m) {
        tracking_ref_ = agc_air::fromMsg(*m);
        tracking_ref_stamp_ = now();
        have_tracking_ref_ = true;
      });

    target_sub_ = create_subscription<agc_msgs::msg::TargetTrackArray>(
      target_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](agc_msgs::msg::TargetTrackArray::SharedPtr m) {
        have_target_ = !m->tracks.empty();
        if (have_target_) {
          const auto & p = m->tracks.front().pose.pose.position;
          target_pos_ = agc_air::Vector3(p.x, p.y, p.z);
          target_stamp_ = now();
        }
      });

    active_sub_ = create_subscription<std_msgs::msg::String>(
      active_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::SharedPtr m) {commanded_controller_ = m->data;});

    rl_offset_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      rl_offset_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr m) {
        rl_offset_ = agc_air::Vector3(m->vector.x, m->vector.y, m->vector.z);
      });

    RCLCPP_INFO(get_logger(), "trajectory_tracker 就绪：轨迹 %s + 控制器 %s",
      traj_plugin_.c_str(), ctrl_plugin_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate_impl() override
  {
    offboard_->activate();
    ref_pub_->on_activate();
    conflict_pub_->on_activate();
    active_pub_->on_activate();

    have_control_ = false;
    conflict_ = false;
    conflict_since_ = now();
    last_yaw_ = 0.0;
    publishConflict(false);   // 发布初值，便于观测（冲突置位时再翻转）

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_), [this]() {tick();});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate_impl() override
  {
    if (timer_) {timer_->cancel();}
    timer_.reset();
    offboard_->deactivate();
    if (ref_pub_) {ref_pub_->on_deactivate();}
    if (conflict_pub_) {conflict_pub_->on_deactivate();}
    if (active_pub_) {active_pub_->on_deactivate();}
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup_impl() override
  {
    timer_.reset();
    tracking_ref_sub_.reset();
    target_sub_.reset();
    active_sub_.reset();
    rl_offset_sub_.reset();
    ref_pub_.reset();
    conflict_pub_.reset();
    active_pub_.reset();
    trajectory_.reset();
    controller_.reset();
    if (offboard_) {offboard_->cleanup();}
    offboard_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  bool trackingRefFresh() const
  {
    return have_tracking_ref_ &&
           (now() - tracking_ref_stamp_).seconds() < tracking_ref_timeout_s_;
  }

  void tick()
  {
    if (!offboard_->odomValid()) {return;}

    const auto & odom = offboard_->odom();
    agc_control::State state;
    state.pos = agc_air::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y,
        odom.pose.pose.position.z);
    state.vel = agc_air::Vector3(
      odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);

    // ① 接管判定：由 active_controller 指定；先在有效接管点重置相位（C5 语义：接管即 t=0）
    const bool should_have_control = (commanded_controller_ == own_name_);
    if (should_have_control && !have_control_) {
      trajectory_->reset(state);
      controller_->reset();
      t0_ = now();
      // 高度基准：PX4 局部原点与世界原点不重合，故参考高度以**接管时刻机体高度**对齐
      // （轨迹 h=2.5 m 即相对该基准的层高，C1 的"恒速"不受影响）。
      z_offset_ = state.pos.z() - trajectory_->sample(0.0).pos.z();
      have_control_ = true;
      last_yaw_ = agc_air::yawFromQuaternion(
        agc_air::Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
      RCLCPP_INFO(get_logger(), "接管控制权：期望轨迹 %s", trajectory_->name().c_str());
    }

    const double t = have_control_ ? (now() - t0_).seconds() : 0.0;
    const agc_control::Reference base_ref = trajectory_->sample(t);

    // ② 始终发布基础参考（对外话题，spec R8）
    if (ref_pub_ && ref_pub_->is_activated()) {
      ref_pub_->publish(agc_air::toMsg(base_ref, world_frame_, now()));
    }

    if (!have_control_) {return;}

    // ③ tracking_ref **覆盖**基础参考（spec R6）
    agc_control::Reference effective = trackingRefFresh() ? tracking_ref_ : base_ref;

    // ④ RL 局部偏移（默认零偏，W7 填充）
    effective.pos += rl_offset_;

    // ⑤' 高度基准对齐（见接管处说明）
    effective.pos.z() += z_offset_;

    // ⑤ 最小安全层：避障 + 对目标最小间距兜底（spec R7）
    const bool target_fresh = have_target_ && (now() - target_stamp_).seconds() < 1.0;
    effective = shield_.apply(effective, target_pos_, target_fresh);

    // ⑥ LQR → 世界系加速度指令
    const agc_air::Vector3 acc = controller_->computeAcceleration(state, effective);

    // ⑦ 偏航随速度方向（限速率），零速时保持
    double yaw = last_yaw_;
    const double vh = std::hypot(effective.vel.x(), effective.vel.y());
    if (vh > 0.05) {
      const double desired_yaw = std::atan2(effective.vel.y(), effective.vel.x());
      const double max_step = max_yaw_rate_ / rate_hz_;
      double dyaw = desired_yaw - last_yaw_;
      while (dyaw > M_PI) {dyaw -= 2.0 * M_PI;}
      while (dyaw < -M_PI) {dyaw += 2.0 * M_PI;}
      yaw = last_yaw_ + std::clamp(dyaw, -max_step, max_step);
    }
    last_yaw_ = yaw;

    offboard_->publishControlMode();
    offboard_->publishSetpoint(effective.pos, effective.vel, acc, yaw);

    // ⑧ 冲突检测：setpoint 发布者 >1 持续超过阈值 → control_conflict 置位
    const bool multi = offboard_->countSetpointPublishers() > 1;
    if (multi && !conflict_) {
      if ((now() - conflict_since_).seconds() > conflict_persist_s_) {
        conflict_ = true;
        publishConflict(true);
        RCLCPP_WARN(get_logger(), "检测到多个 setpoint 发布者 → control_conflict");
      }
    } else if (!multi) {
      conflict_since_ = now();
      if (conflict_) {
        conflict_ = false;
        publishConflict(false);
      }
    }
  }

  void publishConflict(bool value)
  {
    if (!conflict_pub_ || !conflict_pub_->is_activated()) {return;}
    std_msgs::msg::Bool msg;
    msg.data = value;
    conflict_pub_->publish(msg);
  }

  std::string world_frame_, traj_plugin_, ctrl_plugin_, traj_ns_, ctrl_ns_;
  std::string ref_topic_, tracking_ref_topic_, target_topic_, active_topic_, conflict_topic_;
  std::string rl_offset_topic_, own_name_;
  double rate_hz_{50.0};
  double tracking_ref_timeout_s_{0.5};
  double conflict_persist_s_{1.0};
  double max_yaw_rate_{2.0};

  pluginlib::ClassLoader<agc_control::TrajectoryGenerator> traj_loader_;
  pluginlib::ClassLoader<agc_control::Controller> ctrl_loader_;
  agc_control::TrajectoryGenerator::Ptr trajectory_;
  agc_control::Controller::Ptr controller_;
  SafetyShield shield_;

  std::unique_ptr<OffboardInterface> offboard_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr ref_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr conflict_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr active_pub_;

  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr tracking_ref_sub_;
  rclcpp::Subscription<agc_msgs::msg::TargetTrackArray>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr rl_offset_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  agc_control::Reference tracking_ref_;
  rclcpp::Time tracking_ref_stamp_{0, 0, RCL_ROS_TIME};
  bool have_tracking_ref_{false};
  agc_air::Vector3 target_pos_{agc_air::Vector3::Zero()};
  rclcpp::Time target_stamp_{0, 0, RCL_ROS_TIME};
  bool have_target_{false};
  agc_air::Vector3 rl_offset_{agc_air::Vector3::Zero()};
  std::string commanded_controller_;

  bool have_control_{false};
  double z_offset_{0.0};
  rclcpp::Time t0_{0, 0, RCL_ROS_TIME};
  double last_yaw_{0.0};
  bool conflict_{false};
  rclcpp::Time conflict_since_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<TrajectoryTracker>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
