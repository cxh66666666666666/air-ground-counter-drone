// trajectory_tracker —— 轨迹跟踪 + setpoint 唯一发布者 + 冲突检测（spec R3/R4/R5/R7）
//
// 控制层用 agc_control 的 **pluginlib** 插件（轨迹生成器 + LQR 控制器），本节只做
// 「取参考 → 安全层 → LQR → 世界系 ENU 转 PX4 setpoint」的消费者工作（ADR-0018）。
// 接管后才成为 setpoint 发布者；R-035：以**显式控制许可**（`active_controller==own_name` 的
// 唯一授权 + claim/ack 顺序）判定「本机是否唯一授权命令者」，发布者计数**仅作告警**（对齐契约 [唯一执行口]）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <random>
#include <string>

#include <unistd.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    self_uid_(makeSelfUid()),
    traj_loader_("agc_control", "agc_control::TrajectoryGenerator"),
    ctrl_loader_("agc_control", "agc_control::Controller")
  {
  }

protected:
  /// R-035：进程实例唯一标识（getpid + 随机）——用于区分**同名重复节点**（竞争者检出）。
  static std::string makeSelfUid()
  {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "tt-%ld-%08x",
      static_cast<long>(::getpid()), std::random_device{}());
    return std::string(buf);
  }

  CallbackReturn on_configure_impl() override
  {
    world_frame_ = declare_param_idempotent<std::string>("world_frame", "world");
    traj_plugin_ = declare_param_idempotent<std::string>(
      "trajectory_plugin", "agc_control/Figure8Trajectory");
    ctrl_plugin_ = declare_param_idempotent<std::string>("controller_plugin", "agc_control/LqrController");
    traj_ns_ = declare_param_idempotent<std::string>("trajectory_param_ns", "traj");
    ctrl_ns_ = declare_param_idempotent<std::string>("controller_param_ns", "ctrl");
    rate_hz_ = declare_param_idempotent<double>("setpoint_rate_hz", 50.0);
    ref_topic_ = declare_param_idempotent<std::string>("ref_topic", "/agc/air/ref");
    tracking_ref_topic_ =
      declare_param_idempotent<std::string>("tracking_ref_topic", "/agc/air/tracking_ref");
    target_topic_ = declare_param_idempotent<std::string>("target_topic", "/agc/targets");
    active_topic_ = declare_param_idempotent<std::string>("active_controller_topic",
        "/agc/air/active_controller");
    conflict_topic_ = declare_param_idempotent<std::string>("conflict_topic", "/agc/air/control_conflict");
    rl_offset_topic_ = declare_param_idempotent<std::string>("rl_offset_topic", "/agc/air/rl_offset");
    track_status_topic_ = declare_param_idempotent<std::string>(
      "tracking_status_topic", "/agc/air/tracking_status");
    own_name_ = declare_param_idempotent<std::string>("controller_name", "trajectory_tracker");
    tracking_ref_timeout_s_ = declare_param_idempotent<double>("tracking_ref_timeout_s", 0.5);
    // R-032：目标门同时校验**源时间窗**与 **confidence**（对齐契约 C10：ref_max_age_s / confidence≥0.5）。
    target_max_age_s_ = declare_param_idempotent<double>("target_max_age_s", 0.5);
    confidence_min_ = declare_param_idempotent<double>("confidence_min", 0.5);
    // R-032：RL 局部偏移的**寿命 / 生效窗口**与**能力限幅**（幅值上限）。
    rl_offset_timeout_s_ = declare_param_idempotent<double>("rl_offset_timeout_s", 0.5);
    rl_offset_max_m_ = declare_param_idempotent<double>("rl_offset_max_m", 2.0);
    conflict_persist_s_ = declare_param_idempotent<double>("conflict_persist_s", 1.0);
    // R-035：控制许可 —— 竞争者 claim 消失清冲突阈值（s）；许可 claim/ack 话题（配套诊断，**非** ABI、**非**命令者）。
    conflict_clear_s_ = declare_param_idempotent<double>("conflict_clear_s", 2.0);
    control_permit_topic_ = declare_param_idempotent<std::string>(
      "control_permit_topic", "/agc/air/control_permit");
    max_yaw_rate_ = declare_param_idempotent<double>("max_yaw_rate", 2.0);
    // W7-b（P1 甲方案）：观测偏航模式——P1 自变量 α 的落地（单一真源 air.yaml，缺省行为不变）。
    //   "velocity"（缺省）= 机头随速度方向（= 既有行为 / 非协同观测臂）；
    //   "target"          = 机头（固连前视相机）**对准目标**（= 空地协同观测臂）。
    observe_yaw_mode_ =
      declare_param_idempotent<std::string>("observe_yaw_mode", "velocity");
    // W7-b（P1 甲）：**指向信息来源**（仅 `observe_yaw_mode=target` 时创建订阅与使用）。
    //   如实标注：本段用**真值方位**（`/agc/gt/intruder_uav`）作**指向**信息 → 该臂 = **显式 oracle
    //   指向臂**（ADR-0070(i) 允许「显式 oracle 臂」并须标注）；**测量**仍走真实感知通道
    //   （噪声 / 丢包 / FOV 照常生效，不因指向而放宽）。
    //   为何不用 `/agc/targets`（融合航迹）：对准会依赖「相机已看见目标」→ **冷启动自锁**
    //   （实测：目标不在初始视场时两臂可见性同为 0，对不准 ⇒ 看不见 ⇒ 更对不准）。
    yaw_aim_gt_topic_ =
      declare_param_idempotent<std::string>("yaw_aim_gt_topic", "/agc/gt/intruder_uav");
    if (observe_yaw_mode_ == "target") {
      yaw_aim_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        yaw_aim_gt_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          yaw_aim_pos_ = agc_air::Vector3(
            m->pose.position.x, m->pose.position.y, m->pose.position.z);
          yaw_aim_stamp_ = sourceStamp(m->header.stamp);
          have_yaw_aim_ = true;
        });
      RCLCPP_WARN(get_logger(),
        "observe_yaw_mode=target：已启用 **oracle 指向**（真值方位 %s）——本臂须如实标注为 oracle 臂"
        "（ADR-0070(i)）；测量仍走真实感知通道", yaw_aim_gt_topic_.c_str());
    }
    reset_phase_service_ = declare_param_idempotent<std::string>(
      "reset_phase_service", "/agc/air/reset_phase");

    OffboardParams p;
    p.odom_topic = declare_param_idempotent<std::string>("odom_topic", "/agc/air/odom");
    p.status_topic = declare_param_idempotent<std::string>("status_topic", "/agc/air/status");
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
    // R-032：配套诊断（参考来源 / 冻结 / 安全层介入 / offset 应用），**非**命令者 / 非 ABI。
    track_status_pub_ = create_publisher<std_msgs::msg::String>(
      track_status_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    // R-035：控制许可 claim/ack（std_msgs/String JSON，transient_local + reliable；**非** ABI、**非**命令者）。
    // 判据 = 显式 claim（不同 uid 且 by==own_name）；**从不**凭发布者计数决定权威。交权顺序 =
    // takeoff 先 releaseSetpointPublishers() 让出唯一发布者，再 publishActive("trajectory_tracker")。
    permit_pub_ = create_publisher<std_msgs::msg::String>(
      control_permit_topic_, rclcpp::QoS(1).transient_local().reliable());
    permit_sub_ = create_subscription<std_msgs::msg::String>(
      control_permit_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::SharedPtr m) {onPermitClaim(*m);});

    // 相位复位触发（C5）：episode 起始把轨迹相位归零（spec R3；供 R2 起始时序调用）
    reset_phase_srv_ = create_service<std_srvs::srv::Trigger>(
      reset_phase_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {onResetPhase(res);});

    tracking_ref_sub_ =
      create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      tracking_ref_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr m) {
        tracking_ref_ = agc_air::fromMsg(*m);
        // R-032：以**消息自身 header.stamp** 计参考年龄（不再以接收 now 覆盖 → 旧包重放不会"变新鲜"）。
        tracking_ref_stamp_ = sourceStamp(m->header.stamp);
        have_tracking_ref_ = true;
      });

    target_sub_ = create_subscription<agc_msgs::msg::TargetTrackArray>(
      target_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](agc_msgs::msg::TargetTrackArray::SharedPtr m) {
        if (m->tracks.empty()) {have_target_ = false; return;}
        const auto & tr = m->tracks.front();
        const auto & p = tr.pose.pose.position;
        target_pos_ = agc_air::Vector3(p.x, p.y, p.z);
        target_confidence_ = tr.confidence;                 // R-032：目标门并检 confidence
        target_stamp_ = sourceStamp(tr.header.stamp);      // R-032：以源时间计目标年龄
        have_target_ = true;
      });

    active_sub_ = create_subscription<std_msgs::msg::String>(
      active_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::SharedPtr m) {commanded_controller_ = m->data;});

    rl_offset_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      rl_offset_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr m) {
        rl_offset_ = agc_air::Vector3(m->vector.x, m->vector.y, m->vector.z);
        // R-032：记录偏移**源时间**（寿命 / 生效窗口判据）。
        rl_offset_stamp_ = sourceStamp(m->header.stamp);
        have_rl_offset_ = true;
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
    track_status_pub_->on_activate();
    permit_pub_->on_activate();

    have_control_ = false;
    conflict_since_ = now();
    last_yaw_ = 0.0;
    // R-032：新 episode 起点重置"已获模式"与冻结状态。
    acquired_ = false;
    last_good_ref_ = agc_control::Reference();
    // R-035：**重启不残留旧权威**——重置授权 / 冲突 / 竞争者与 claim；随后由当前 active_controller 指定推导。
    resetAuthorityState();
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
    if (track_status_pub_) {track_status_pub_->on_deactivate();}
    if (permit_pub_) {permit_pub_->on_deactivate();}
    // R-035：停发即重置授权 / 冲突 / 竞争者与 claim（重启不残留旧权威，见 ADR-0050）。
    resetAuthorityState();
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
    track_status_pub_.reset();
    permit_pub_.reset();
    permit_sub_.reset();
    reset_phase_srv_.reset();
    trajectory_.reset();
    controller_.reset();
    resetAuthorityState();   // R-035：完整清理并重置授权 / 冲突 / 竞争者与 claim
    if (offboard_) {offboard_->cleanup();}
    offboard_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  /// /agc/air/odom 的 twist 为**机体系 FLU**（nav_msgs 约定，见 R-003）→ 还原世界 ENU 速度
  static agc_air::Vector3 worldVelFromOdom(const nav_msgs::msg::Odometry & o)
  {
    const agc_air::Quaternion q(o.pose.pose.orientation.w, o.pose.pose.orientation.x,
      o.pose.pose.orientation.y, o.pose.pose.orientation.z);
    return agc_air::bodyFluToEnu(q, agc_air::Vector3(
      o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z));
  }

  /// R-032：以**消息自身 header.stamp**（源时间）为准；未设源时间（全 0）则退回接收 now
  /// （兼容未置 stamp 的发布者，且不把接收时刻当作源时刻）。
  rclcpp::Time sourceStamp(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {return now();}
    return rclcpp::Time(stamp, RCL_ROS_TIME);
  }

  /// R-032：参考新鲜度以**源时间**计（旧 ref 重放 → 源 stamp 为过去 → 过期即拒）。
  bool trackingRefFresh() const
  {
    if (!have_tracking_ref_) {return false;}
    const double age = (now() - tracking_ref_stamp_).seconds();
    return age >= 0.0 && age < tracking_ref_timeout_s_;
  }

  /// R-032：目标新鲜度 = **源时间窗内** **且** `confidence >= confidence_min`（对齐契约 C10）。
  bool targetFresh() const
  {
    if (!have_target_) {return false;}
    const double age = (now() - target_stamp_).seconds();
    return age >= 0.0 && age < target_max_age_s_ && target_confidence_ >= confidence_min_;
  }

  /// W7-b（P1 甲）：**指向源**（真值方位）新鲜度——仅 `observe_yaw_mode=target` 时使用；
  /// 复用 `target_max_age_s_` 作年龄界（与契约 C10 目标窗口同量级），超龄即**不指向**（不凭陈旧方位转机头）。
  bool yawAimFresh() const
  {
    if (!have_yaw_aim_) {return false;}
    const double age = (now() - yaw_aim_stamp_).seconds();
    return age >= 0.0 && age < target_max_age_s_;
  }

  /// R-032：RL offset 的**寿命 / 生效窗口**（源时间 + 超时归零）。
  bool rlOffsetFresh() const
  {
    if (!have_rl_offset_) {return false;}
    const double age = (now() - rl_offset_stamp_).seconds();
    return age >= 0.0 && age < rl_offset_timeout_s_;
  }

  void tick()
  {
    if (!offboard_->odomValid()) {return;}

    const auto & odom = offboard_->odom();
    agc_control::State state;
    state.pos = agc_air::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y,
        odom.pose.pose.position.z);   // 世界 ENU（px4_adapter 已含 EKF 原点平移）
    state.vel = worldVelFromOdom(odom);   // 机体系 FLU → 世界 ENU（R-003）

    // ① 显式唯一授权（R-035）：仅由 active_controller 指定（交权路线：takeoff 先
    //    releaseSetpointPublishers() 让出唯一发布者，**再** publishActive("trajectory_tracker")）。
    //    **不**凭发布者计数判定权威（计数含静默 publisher / DDS 发现延迟）。
    is_authorized_ = (commanded_controller_ == own_name_);

    // ② 冲突裁决：显式竞争者 claim 持续 > conflict_persist_s_ → conflict_；消失 ≥ conflict_clear_s → 清。
    updateConflict();

    // ③ 控制许可 = 授权 **且** 无冲突。**先于任何 setpoint 决策**（check 静态核验顺序）。
    const bool permit = is_authorized_ && !conflict_;

    // ④ 发布者计数：**仅作告警**（含静默 publisher / DDS 发现延迟；**不**单独决定权威）。
    writers_warn_ = offboard_->countSetpointPublishers();

    // ⑤ 冲突 / 许可状态可观测（control_conflict）——冲突置位（若有）**先于**任何 setpoint 发布。
    publishConflict(conflict_);

    // ⑥ 仅在被授权且无冲突时接管（在有效接管点重置相位，C5 语义：接管即 t=0）。
    if (permit && !have_control_) {
      trajectory_->reset(state);
      controller_->reset();
      t0_ = now();   // R-035：新一集起点（接管 = 新许可生效点）。
      // 高度基准（ADR-0051，唯一世界高度语义 = 世界 ENU z）：状态已是世界 ENU（px4_adapter 含 EKF
      // 原点平移，R-003），此处 z_offset 是**层高基准**选择（轨迹 h=2.5 m 相对接管时刻机体高度）——
      // **非**世界原点代偿；对外 base_ref 与该 effective **同一基准**（发布带同一 z_offset_）。
      z_offset_ = state.pos.z() - trajectory_->sample(0.0).pos.z();
      have_control_ = true;
      // R-032：新一集起点——重置"已获模式" / 冻结 / 最后有效参考。
      frozen_ = false;
      acquired_ = false;
      last_good_ref_ = agc_control::Reference();
      last_yaw_ = agc_air::yawFromQuaternion(
        agc_air::Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
      RCLCPP_INFO(get_logger(), "接管控制权（唯一授权）：期望轨迹 %s", trajectory_->name().c_str());
    }

    const double t = have_control_ ? (now() - t0_).seconds() : 0.0;
    const agc_control::Reference base_ref = trajectory_->sample(t);

    // ⑦ 始终发布基础参考（对外话题，spec R8）——**非** setpoint（不构成命令者）。
    // R-036（ADR-0051）：对外 `base_ref` 与生效 `effective` 须**同一世界高度基准**——发布前对**副本**
    //   加 `z_offset_`（层高基准），使公开 base_ref 的 z 与 effective 一致（**不**静默不同）；
    //   **不**改动下游使用的 `base_ref`（下游 / effective 仍按世界 ENU 轨迹原高 + 同一 z_offset）。
    if (ref_pub_ && ref_pub_->is_activated()) {
      agc_control::Reference base_ref_out = base_ref;
      base_ref_out.pos.z() += z_offset_;
      ref_pub_->publish(agc_air::toMsg(base_ref_out, world_frame_, now()));
    }

    // ⑧ 无许可 → relinquish（**已定义安全状态**）：置位后**本拍不发** OCM / setpoint（复用 R-032 hold
    //    内部状态：冲突期已置 frozen_=true）→ PX4 自身 failsafe 保持/降落。
    if (!permit) {
      if (have_control_) {
        have_control_ = false;
        RCLCPP_WARN(get_logger(),
          "R-035：控制许可撤销（authorized=%d conflict=%d）→ relinquish（本拍停发 setpoint）",
          static_cast<int>(is_authorized_), static_cast<int>(conflict_));
      }
      publishClaim(permit);
      publishTrackStatus(permit, "no_permit", false, false, false, false, false, false, "", "");
      return;   // **本拍不发 OCM / setpoint**（不继续争夺）
    }

    // ⑨ 许可路径：参考 / 目标新鲜度（以**源 header.stamp**计，目标门并检 confidence，R-032）。
    const bool ref_fresh = trackingRefFresh();
    const bool tgt_fresh = targetFresh();

    // 参考来源裁定（spec R6 + R-032 冻结下游封闭）：
    //   tracking —— 参考与目标均新鲜 → 采用 tracking_ref（覆盖基础轨迹）；
    //   hold     —— **已获目标后**失参考 / 失目标 → 显式 hold（保持最后有效位置、前馈清零）；
    //   base     —— **从未获目标** → 正常基础八字。
    agc_control::Reference effective;
    const char * source = "base";
    if (ref_fresh && tgt_fresh) {
      acquired_ = true;
      frozen_ = false;
      last_good_ref_ = tracking_ref_;
      effective = tracking_ref_;
      source = "tracking";
    } else if (acquired_) {
      if (!frozen_) {
        frozen_ = true;
        RCLCPP_WARN(get_logger(),
          "已获目标后参考 / 目标失效（ref_fresh=%d tgt_fresh=%d）→ 显式 hold（不回落八字）",
          static_cast<int>(ref_fresh), static_cast<int>(tgt_fresh));
      }
      effective = last_good_ref_;
      effective.vel = agc_air::Vector3::Zero();
      effective.acc = agc_air::Vector3::Zero();
      source = "hold";
    } else {
      frozen_ = false;
      effective = base_ref;
      source = "base";
    }

    // RL 局部偏移：**仅正常跟踪模式**生效（hold / 基础模式不叠加）；寿命 / 生效窗口 + 能力限幅（R-032）。
    bool offset_applied = false;
    bool offset_clamped = false;
    if (std::string(source) == "tracking" && rlOffsetFresh()) {
      agc_air::Vector3 off = rl_offset_;
      const double n = off.norm();
      if (rl_offset_max_m_ > 0.0 && n > rl_offset_max_m_) {
        off *= rl_offset_max_m_ / n;
        offset_clamped = true;
      }
      effective.pos += off;
      offset_applied = true;
    }

    // 高度基准对齐（见接管处说明）
    effective.pos.z() += z_offset_;

    // 联合安全屏障（spec R7 + R-033）：联合约束复检（障碍 + 目标间距**同时**）+ 扫掠复检
    //      + **显式不可行路径**（无可行参考 → hold / 逃逸）；R-032 记录安全层是否介入。
    const agc_air::SafetyResult shield_res =
      shield_.apply(effective, state.pos, target_pos_, tgt_fresh);
    effective = shield_res.ref;
    const bool shield_active = shield_res.modified;
    const bool shield_infeasible = shield_res.infeasible;

    // LQR → 世界系加速度指令
    const agc_air::Vector3 acc = controller_->computeAcceleration(state, effective);

    // 偏航：默认**随速度方向**（限速率，零速时保持）；`observe_yaw_mode == "target"` 时
    // **对准目标** —— W7-b P1（甲方案）自变量 α 的落地：空地协同观测 = 令机体**固连前视相机**
    // 持续对准目标（反之机头随速度 → 目标常被甩出 ±50° 视场 → 观测间歇）。
    // 目标须过**指向源新鲜门** `yawAimFresh()`（真值方位，年龄 ≤ `target_max_age_s_`）；
    // 不过门则**退回随速度**（不凭陈旧方位转机头）。
    double yaw = last_yaw_;
    double desired_yaw = std::numeric_limits<double>::quiet_NaN();
    const bool aim_fresh = yawAimFresh();
    if (observe_yaw_mode_ == "target" && aim_fresh) {
      const double dx = yaw_aim_pos_.x() - state.pos.x();
      const double dy = yaw_aim_pos_.y() - state.pos.y();
      if (std::hypot(dx, dy) > 1e-3) {
        desired_yaw = std::atan2(dy, dx);
        if (!aim_used_logged_) {
          aim_used_logged_ = true;
          RCLCPP_WARN(get_logger(),
            "target 臂**指向生效**：desired_yaw=%.1f°（指向源到位，age 门通过）",
            desired_yaw * 180.0 / M_PI);
        }
      }
    } else if (observe_yaw_mode_ == "target" && !aim_skipped_logged_) {
      // 诊断（一次性）：指向分支**未生效**——区分「指向源未到 / 超龄」与「已到但几何退化」
      aim_skipped_logged_ = true;
      RCLCPP_WARN(get_logger(),
        "target 臂**指向未生效**：have_yaw_aim=%d aim_fresh=%d（源未到或超龄 → 退回随速度；"
        "本臂可见性将与 velocity 臂无差异）", static_cast<int>(have_yaw_aim_),
        static_cast<int>(aim_fresh));
    }
    if (!std::isfinite(desired_yaw)) {
      const double vh = std::hypot(effective.vel.x(), effective.vel.y());
      if (vh > 0.05) {desired_yaw = std::atan2(effective.vel.y(), effective.vel.x());}
    }
    if (std::isfinite(desired_yaw)) {
      const double max_step = max_yaw_rate_ / rate_hz_;
      double dyaw = desired_yaw - last_yaw_;
      while (dyaw > M_PI) {dyaw -= 2.0 * M_PI;}
      while (dyaw < -M_PI) {dyaw += 2.0 * M_PI;}
      yaw = last_yaw_ + std::clamp(dyaw, -max_step, max_step);
    }
    last_yaw_ = yaw;

    offboard_->publishControlMode();
    offboard_->publishSetpoint(effective.pos, effective.vel, acc, yaw);

    // R-032/R-035：配套诊断（std_msgs/String，非 ABI、非命令者）——许可 / 冲突 / 参考来源 /
    //   安全层介入 / offset / 实际写入者与频率（claim 另发 control_permit）。
    publishClaim(permit);
    publishTrackStatus(permit, source, ref_fresh, tgt_fresh, offset_applied, offset_clamped,
      shield_active, shield_infeasible, shield_res.reason, shield_res.action);
  }

  /// R-035：冲突裁决——判据为**显式竞争者 claim**（不同 uid 且 by==own_name），**非**发布者计数：
  ///   竞争者 claim 持续 > conflict_persist_s_ → conflict_=true（复用 R-032 hold：置 frozen_）；
  ///   竞争者 claim 停止刷新 ≥ conflict_clear_s_ → 视为消失 → 清 conflict_（若仍被授权则恢复）。
  void updateConflict()
  {
    const auto t = now();
    if (competing_ && (t - competing_last_).seconds() > conflict_clear_s_) {
      competing_ = false;   // 竞争者不再刷新 claim → 视为消失（**不**误报合法静默 publisher：其从不 claim）
      competitor_uid_.clear();
    }
    if (competing_) {
      if (!conflict_ && (t - competing_first_).seconds() > conflict_persist_s_) {
        conflict_ = true;
        frozen_ = true;   // 复用 R-032 hold 内部状态 = 已定义安全状态
        RCLCPP_WARN(get_logger(),
          "R-035：竞争者 claim（uid=%s）持续 > %.2fs → control_conflict（relinquish）",
          competitor_uid_.c_str(), conflict_persist_s_);
      }
    } else if (conflict_ && (t - competing_last_).seconds() > conflict_clear_s_) {
      conflict_ = false;
      frozen_ = false;
      RCLCPP_WARN(get_logger(), "R-035：竞争者消失 ≥ %.2fs → 清 control_conflict", conflict_clear_s_);
    }
  }

  /// R-035：许可 claim/ack 受理——**只有显式 claim**（`by==own_name` 且 `uid!=self_uid`）才算竞争者；
  /// 对**合法静默 publisher**（从不 claim）不误报。重复节点检出：同名重复实例（不同 uid）互相检出。
  void onPermitClaim(const std_msgs::msg::String & m)
  {
    const std::string & d = m.data;
    if (claimField(d, "ev") != "claim") {return;}
    if (claimField(d, "by") != own_name_) {return;}
    const std::string uid = claimField(d, "uid");
    if (uid.empty() || uid == self_uid_) {return;}
    const auto t = now();
    if (!competing_) {competing_first_ = t;}
    competing_ = true;
    competing_last_ = t;
    competitor_uid_ = uid;
  }

  /// R-035：发布控制许可 claim（std_msgs/String JSON；transient_local + reliable，**非** ABI、**非**命令者）。
  /// 状态变化或每 ~1s heartbeat 发一次（避免 50 Hz 刷屏）。
  void publishClaim(bool permit)
  {
    if (!permit_pub_ || !permit_pub_->is_activated()) {return;}
    const auto t = now();
    char buf[384];
    std::snprintf(buf, sizeof(buf),
      "{\"ev\":\"claim\",\"by\":\"%s\",\"uid\":\"%s\",\"epoch\":%u,\"permit\":%s,"
      "\"conflict\":%s,\"writers\":%zu,\"sp_hz\":%.1f,\"competitor_uid\":\"%s\"}",
      own_name_.c_str(), self_uid_.c_str(), claim_epoch_,
      permit ? "true" : "false", conflict_ ? "true" : "false", writers_warn_,
      offboard_ ? offboard_->setpointRateHz() : 0.0,
      competitor_uid_.empty() ? "-" : competitor_uid_.c_str());
    std_msgs::msg::String msg;
    msg.data = buf;
    const bool changed = (!last_claim_valid_ || msg.data != last_claim_);
    if (changed || (t - last_claim_time_).seconds() >= 1.0) {
      permit_pub_->publish(msg);
      last_claim_ = msg.data;
      last_claim_valid_ = true;
      last_claim_time_ = t;
      if (changed) {++claim_epoch_;}
    }
  }

  /// R-035：claim JSON 字段解析（`"key": "value"` 或 `"key": value`；无空格 / 有空格均容错）。
  static std::string claimField(const std::string & s, const std::string & key)
  {
    const auto kp = s.find("\"" + key + "\"");
    if (kp == std::string::npos) {return std::string();}
    const auto colon = s.find(':', kp + key.size() + 2);
    if (colon == std::string::npos) {return std::string();}
    auto vp = colon + 1;
    while (vp < s.size() && s[vp] == ' ') {++vp;}
    if (vp >= s.size()) {return std::string();}
    if (s[vp] == '"') {
      const auto e = s.find('"', vp + 1);
      if (e == std::string::npos) {return std::string();}
      return s.substr(vp + 1, e - vp - 1);
    }
    auto e = s.find_first_of(",}", vp);
    if (e == std::string::npos) {e = s.size();}
    return s.substr(vp, e - vp);
  }

  /// R-035：重置授权 / 冲突 / 竞争者与 claim 状态（on_activate / deactivate / cleanup / shutdown / error）。
  void resetAuthorityState()
  {
    is_authorized_ = false;
    conflict_ = false;
    competing_ = false;
    competitor_uid_.clear();
    frozen_ = false;   // 复用 R-032 hold 内部状态
    have_control_ = false;
    conflict_since_ = now();
    claim_epoch_ = 0;
    last_claim_.clear();
    last_claim_valid_ = false;
    last_conflict_pub_valid_ = false;
  }

  /// R-035：`control_conflict`（std_msgs/Bool，transient_local + reliable）——对消费方可观测。
  /// **仅状态变化时发布**（避免 50 Hz 刷屏；transient_local 锁存最近值供晚订阅者回放）。
  void publishConflict(bool value)
  {
    if (last_conflict_pub_valid_ && value == last_conflict_pub_) {return;}
    if (!conflict_pub_ || !conflict_pub_->is_activated()) {return;}
    std_msgs::msg::Bool msg;
    msg.data = value;
    conflict_pub_->publish(msg);
    last_conflict_pub_ = value;
    last_conflict_pub_valid_ = true;
  }

  /// R-032/R-033：跟踪诊断（std_msgs/String JSON）——`source` 区分参考来源（base / tracking / hold）、
  /// **安全层必要修正**（`shield_active`）vs **offset 叠加**（`offset_applied` / `offset_clamped`）；
  /// R-033 追加**显式不可行**诊断（`shield_infeasible` / `shield_reason` / `shield_action`）。
  /// R-035 追加**控制许可 / 冲突 / 写者记账**（`permit` / `conflict` / `writers` / `sp_hz` / `uid` /
  ///   `competitor_uid`）——既有字段**保持不变**（兼容 R-032/R-033 检查）。
  /// R-036 追加**高度基准**诊断（`z_offset` / `height_datum`）：显式标注对外 base_ref 与 effective 的
  ///   同一世界高度基准（唯一世界高度语义 = 世界 ENU z），使差异**可辨**（见 ADR-0051）。
  void publishTrackStatus(bool permit, const char * source, bool ref_fresh, bool tgt_fresh,
    bool offset_applied, bool offset_clamped, bool shield_active,
    bool shield_infeasible, const std::string & shield_reason,
    const std::string & shield_action)
  {
    if (!track_status_pub_ || !track_status_pub_->is_activated()) {return;}
    std_msgs::msg::String msg;
    char buf[896];
    std::snprintf(buf, sizeof(buf),
      "{\"acquired\": %s, \"frozen\": %s, \"source\": \"%s\", \"ref_fresh\": %s, "
      "\"target_fresh\": %s, \"offset_applied\": %s, \"offset_clamped\": %s, "
      "\"shield_active\": %s, \"shield_infeasible\": %s, \"shield_reason\": \"%s\", "
      "\"shield_action\": \"%s\", \"permit\": %s, \"conflict\": %s, \"writers\": %zu, "
      "\"sp_hz\": %.1f, \"uid\": \"%s\", \"competitor_uid\": \"%s\", "
      "\"z_offset\": %.4f, \"height_datum\": \"%s\"}",
      acquired_ ? "true" : "false", frozen_ ? "true" : "false", source,
      ref_fresh ? "true" : "false", tgt_fresh ? "true" : "false",
      offset_applied ? "true" : "false", offset_clamped ? "true" : "false",
      shield_active ? "true" : "false", shield_infeasible ? "true" : "false",
      shield_reason.c_str(), shield_action.c_str(),
      permit ? "true" : "false", conflict_ ? "true" : "false", writers_warn_,
      offboard_ ? offboard_->setpointRateHz() : 0.0, self_uid_.c_str(),
      competitor_uid_.empty() ? "-" : competitor_uid_.c_str(),
      z_offset_, "world_enu");
    msg.data = buf;
    track_status_pub_->publish(msg);
  }

  // C5：相位复位触发 —— 复用接管处同一套语义（trajectory_->reset(state); t0_=now()）。
  // 有有效 odom 时以当前机体状态重置轨迹与高度基准；无 odom 时以零状态重置相位。
  void onResetPhase(const std::shared_ptr<std_srvs::srv::Trigger::Response> & res)
  {
    if (!trajectory_) {
      res->success = false;
      res->message = "trajectory_tracker 未 configure";
      return;
    }
    agc_control::State state;
    if (offboard_ && offboard_->odomValid()) {
      const auto & odom = offboard_->odom();
      state.pos = agc_air::Vector3(
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
      state.vel = worldVelFromOdom(odom);   // 机体系 FLU → 世界 ENU（R-003）
      z_offset_ = state.pos.z() - trajectory_->sample(0.0).pos.z();
    }
    trajectory_->reset(state);
    if (controller_) {controller_->reset();}
    t0_ = now();  // 相位自 t=0 起算（C5）
    acquired_ = false;   // R-032：相位复位即新一集起点，清"已获模式" / 冻结 / 最后有效参考。
    frozen_ = false;
    last_good_ref_ = agc_control::Reference();
    res->success = true;
    res->message = "phase reset: t0=0";
    RCLCPP_INFO(get_logger(), "相位复位（/agc/air/reset_phase）：轨迹相位 t0=0");
  }

  std::string world_frame_, traj_plugin_, ctrl_plugin_, traj_ns_, ctrl_ns_;
  std::string ref_topic_, tracking_ref_topic_, target_topic_, active_topic_, conflict_topic_;
  std::string rl_offset_topic_, own_name_, reset_phase_service_, track_status_topic_;
  std::string control_permit_topic_;   // R-035：控制许可 claim/ack 话题（配套诊断，非 ABI）
  std::string self_uid_;               // R-035：进程实例唯一标识（区分同名重复节点）
  double rate_hz_{50.0};
  double tracking_ref_timeout_s_{0.5};
  double target_max_age_s_{0.5};       // R-032：目标源时间窗（对齐契约 C10 ref_max_age_s=0.5）
  double confidence_min_{0.5};         // R-032：目标 confidence 门槛（对齐契约 C10 confidence>=0.5）
  double rl_offset_timeout_s_{0.5};    // R-032：RL offset 寿命 / 生效窗口
  double rl_offset_max_m_{2.0};        // R-032：RL offset 能力限幅（幅值上限，m）
  double conflict_persist_s_{1.0};
  double conflict_clear_s_{2.0};       // R-035：竞争者 claim 消失清冲突阈值（s）
  double max_yaw_rate_{2.0};
  std::string observe_yaw_mode_{"velocity"};   // W7-b P1（甲）：velocity（机头随速度）| target（对准目标）
  std::string yaw_aim_gt_topic_{"/agc/gt/intruder_uav"};   // 仅 target 臂使用（oracle 指向源）
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr yaw_aim_sub_;
  agc_air::Vector3 yaw_aim_pos_{agc_air::Vector3::Zero()};
  rclcpp::Time yaw_aim_stamp_{0, 0, RCL_ROS_TIME};
  bool have_yaw_aim_{false};
  bool aim_used_logged_{false}, aim_skipped_logged_{false};   // 一次性诊断（W7-b P1 甲）

  pluginlib::ClassLoader<agc_control::TrajectoryGenerator> traj_loader_;
  pluginlib::ClassLoader<agc_control::Controller> ctrl_loader_;
  agc_control::TrajectoryGenerator::Ptr trajectory_;
  agc_control::Controller::Ptr controller_;
  SafetyShield shield_;

  std::unique_ptr<OffboardInterface> offboard_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr ref_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr conflict_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr active_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr track_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr permit_pub_;   // R-035
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_phase_srv_;

  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr tracking_ref_sub_;
  rclcpp::Subscription<agc_msgs::msg::TargetTrackArray>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr rl_offset_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr permit_sub_;   // R-035
  rclcpp::TimerBase::SharedPtr timer_;

  agc_control::Reference tracking_ref_;
  rclcpp::Time tracking_ref_stamp_{0, 0, RCL_ROS_TIME};
  bool have_tracking_ref_{false};
  agc_air::Vector3 target_pos_{agc_air::Vector3::Zero()};
  rclcpp::Time target_stamp_{0, 0, RCL_ROS_TIME};
  double target_confidence_{0.0};    // R-032：目标门 confidence
  bool have_target_{false};
  agc_air::Vector3 rl_offset_{agc_air::Vector3::Zero()};
  rclcpp::Time rl_offset_stamp_{0, 0, RCL_ROS_TIME};   // R-032：RL offset 源时间
  bool have_rl_offset_{false};
  std::string commanded_controller_;

  // R-032：已获模式 / 冻结 / 最后有效跟踪参考（供 hold 用）
  bool acquired_{false};
  bool frozen_{false};
  agc_control::Reference last_good_ref_;

  bool have_control_{false};
  double z_offset_{0.0};
  rclcpp::Time t0_{0, 0, RCL_ROS_TIME};
  double last_yaw_{0.0};
  // R-035：控制许可状态
  bool conflict_{false};
  rclcpp::Time conflict_since_{0, 0, RCL_ROS_TIME};
  bool is_authorized_{false};                 // 本机是否为被指定的授权命令者（active_controller==own_name）
  bool competing_{false};                     // 观察到显式竞争者 claim（不同 uid 且 by==own_name）
  std::string competitor_uid_;                // 竞争者 uid（无则空）
  rclcpp::Time competing_first_{0, 0, RCL_ROS_TIME};   // 首次观察竞争者的时刻
  rclcpp::Time competing_last_{0, 0, RCL_ROS_TIME};    // 最近一次竞争者 claim 刷新的时刻
  std::size_t writers_warn_{0};               // setpoint 发布者计数（**仅告警**）
  std::string last_claim_;                    // 最近 claim JSON（状态变化节流）
  bool last_claim_valid_{false};
  rclcpp::Time last_claim_time_{0, 0, RCL_ROS_TIME};
  unsigned claim_epoch_{0};                   // claim 状态变化序号
  bool last_conflict_pub_{false};             // 最近发布的 control_conflict 值
  bool last_conflict_pub_valid_{false};
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
