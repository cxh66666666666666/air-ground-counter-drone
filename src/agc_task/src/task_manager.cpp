// task_manager —— W6 完整任务状态机（spec R1 / R2 / R3）
//
// 态图（spec R1）：
//   IDLE ──(/agc/task/start：reset→reset_phase→门禁)──> AIR_SEARCH ──(目标新鲜且 confidence≥min)──>
//   TARGET_ACQUIRED ──(地面主控在追)──> GROUND_APPROACH
//     ├─(抵近持续达阈值, C8)──> TRACKING_SUCCESS
//     ├─(捕获 / 干扰命中, R11)──> INTERCEPT_CONFIRMED
//     ├─(wall-clock > T_max, C4)──> MISSION_FAILED(TIMEOUT)
//     └─(外部 abort)──> ABORTED
//   门禁不过 ──> INFRA_FAILURE
//
// 契约：先 reset 后校验 odom（#lessons 竞态修复）；C4 超时算失败；C5 相位复位（/agc/air/reset_phase，
//       仅 air_present=true 的整链模式下强制，见 R-001）；C8 终态"抵近并持续跟踪"且 SUCCESS 不得称"拦截完成"。
// R-013：/agc/task/start 为非阻塞异步事务（多线程执行器 + 独立服务回调组）；门禁数据须为**复位完成后
//       收到的新鲜样本**（epoch + 年龄上限），时钟门为**进展校验**；各门证据时刻随应答回传。
// R-021：episode 隔离——新 episode 清 capture_reason_ / jam_reason_；capture 结算改订配套事件
//       /agc/task/capture_event（解析 ep，仅本 episode 匹配才接受）；jam 仍订 /agc/task/jam_confirm
//       （R-018 值 JAM_*），以「start 后到达 + 到达时刻」为门槛；「先命中者胜」按事件到达时刻定序。
// R-022：命令幂等与终态锁存——start 仅在 IDLE / 任一终态接受，active 重入直接拒绝（不尝试 reset、
//       不 fail()）；abort 仅在 active 接受，终态锁存（latched_state_ + latched_epoch_）后迟到的 abort
//       不覆写；进入终态的各路径统一走 enterTerminal() 关联 episode。start 事务的调用结果分三类
//       （Ok / Failed / Indeterminate）：**确定性门禁失败**（收到 success=false）→ INFRA_FAILURE；
//       **非确定性超时/取消**（未收到应答）→ 仅回 success=false，**不改** state_（不误置）。新 episode
//       由显式 start 创建并发布 /agc/task/episode（std_msgs/Int32），供 R-021 配套事件关联。
// R-023：连续跟踪 dwell 逐拍判据——任何不满足「目标新鲜 + 同 target_id + 车地面 odom 有效未超龄 + 包络内」
//       的拍（含目标数组为空）一律中断清零，成功仍为「连续 t_dwell 有效」（C8 不降级）；新增
//       car_odom_max_age_s 寿命门与 /agc/task/dwell_event（std_msgs，非命令者）记录分段有效时长。
// R-041：目标失效策略 =「保留有期限的最后有效目标」(bounded hold-last-valid，ADR-0056)。goal_follow_target 下
//       目标失效时**不**反射固定点、**不**刷新时间戳；窗口内（goal_hold_valid_s）保持最后有效目标的**原位置 +
//       原源时刻**（age 自然增长）；超窗显式失效（valid=false）不发布带新时间戳的 goal 位姿。有效性/身份/时刻
//       经伴随诊断 /agc/ground/goal_info（std_msgs/String，transient_local，非命令者）传递；新 episode 清缓存。
// W7-b（步 1·甲-1）：空中控制权交接——整链（air_present=true）下，start 事务在门禁通过后**等「起飞完成
//       （悬停）」**再显式触发 `/agc/air/release_control`（takeoff_controller → trajectory_tracker）。
//       为何必须「等悬停」：takeoff_controller 是**唯一**发 ARM / Offboard-mode 的节点，提前 release 会停其
//       tick（[release_control]）→ 飞机永不 arm；实测 start 时尚未 arm、悬停发生在 start 后 ~10–14 s。

#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rmw/qos_profiles.h>
#include <agc_msgs/msg/agc_task_state.hpp>
#include <agc_msgs/msg/target_track_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

using Trigger = std_srvs::srv::Trigger;
using AgcTaskState = agc_msgs::msg::AgcTaskState;

class TaskManager : public rclcpp::Node
{
public:
  TaskManager()
  : Node("task_manager")
  {
    world_name_ = declare_parameter<std::string>("world_name", "agc_world");
    goal_x_ = declare_parameter<double>("goal_x", 4.0);
    goal_y_ = declare_parameter<double>("goal_y", 3.0);
    // 成功 / 失败判据（R3，可标定暂定值）
    d_track_ = declare_parameter<double>("d_track_m", 3.0);
    t_dwell_ = declare_parameter<double>("t_dwell_s", 3.0);
    t_max_ = declare_parameter<double>("t_max_s", 120.0);
    confidence_min_ = declare_parameter<double>("confidence_min", 0.5);
    target_max_age_s_ = declare_parameter<double>("target_max_age_s", 0.5);
    // R-013：复位后门禁数据新鲜度上限（odom_valid / 地面 odom 同族共用；cmd 沿用 0.5）
    odom_valid_max_age_s_ = declare_parameter<double>("odom_valid_max_age_s", 1.0);
    fsm_hz_ = declare_parameter<double>("fsm_hz", 10.0);
    // W7-b（ADR-0076）：tracking_milestone 是否等价为终态——缺省 true（既有行为不变）；
    //   capture 场景置 false 时，里程碑只记指标（tracking_milestone_met_ = true）并发布事件，
    //   **不**进入终态，让 `capture_adjudicator` 仍有机会判定 `INTERCEPT_CONFIRMED`。
    tracking_milestone_is_terminal_ = declare_parameter<bool>("tracking_milestone_is_terminal", true);
    // 目标发布口径：false=固定 goal（M0/W5 兼容）；true=跟随目标当前 XY（移动式拦截，ADR-0014）
    goal_follow_target_ = declare_parameter<bool>("goal_follow_target", false);
    // R-041：目标失效策略 =「保留有期限的最后有效目标」(bounded hold-last-valid)。
    //   目标不再新鲜时，在 goal_hold_valid_s_ 窗口内保持**最后一次有效目标的原位置与源时刻**
    //   （不刷新 stamp、不反射固定点）；超窗即显式失效（valid=false），由地面安全停车。
    goal_hold_valid_s_ = declare_parameter<double>("goal_hold_valid_s", 1.0);
    // R-022：reset / reset_phase 调用超时（s）；超时/取消视为 Indeterminate（非确定性），不误置状态
    reset_call_timeout_s_ = declare_parameter<double>("reset_call_timeout_s", 8.0);
    // R-023：车地面 odom 寿命门（s）——超出即判车位置失源，不得用旧位置累计 dwell
    car_odom_max_age_s_ = declare_parameter<double>("car_odom_max_age_s", 1.0);

    const auto odom_valid_topic =
      declare_parameter<std::string>("odom_valid_topic", "/agc/ground/odom_valid");
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/agc/ground/cmd_vel");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/agc/task/state");
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/agc/ground/goal");
    // R-041：目标有效性 / 身份 / 时刻的**伴随诊断**话题（std_msgs/String，transient_local，非命令者）。
    //   /agc/ground/goal 仍为 geometry_msgs/PoseStamped（**不**改冻结类型）；身份/有效性/时刻经此带外传递。
    const auto goal_info_topic =
      declare_parameter<std::string>("goal_info_topic", "/agc/ground/goal_info");
    const auto target_topic = declare_parameter<std::string>("target_topic", "/agc/targets");
    const auto ground_odom_topic =
      declare_parameter<std::string>("ground_odom_topic", "/agc/ground/odom");
    const auto air_active_topic =
      declare_parameter<std::string>("air_active_controller_topic", "/agc/air/active_controller");
    // R-018：干扰确认只认「已确认事件」（jammer 观测到真实 nav_state 持续达标后发布）；
    // 授权事件 /agc/task/jam_state（R-017）不再参与确认，故不订阅。
    const auto jam_confirm_topic =
      declare_parameter<std::string>("jam_confirm_topic", "/agc/task/jam_confirm");
    // R-021：结算事件改订**配套事件**（携带 episode / 时刻 / target），不再消费裸 capture_hit；
    // epoch_topic 学习当前 episode（新 episode 清原因 + 事件 ep 匹配）。
    const auto capture_event_topic =
      declare_parameter<std::string>("capture_event_topic", "/agc/task/capture_event");
    const auto epoch_topic =
      declare_parameter<std::string>("epoch_topic", "/agc/scenario/reset_epoch");
    // R-022：新 episode 发布标识（std_msgs/Int32，transient_local，非命令者），供 R-021 配套事件关联。
    const auto episode_topic =
      declare_parameter<std::string>("episode_topic", "/agc/task/episode");
    // R-023：dwell 分段有效时长事件话题（std_msgs/String，非命令者，供 harness / 证据关联）
    const auto dwell_event_topic =
      declare_parameter<std::string>("dwell_event_topic", "/agc/task/dwell_event");
    reset_phase_service_ = declare_parameter<std::string>("reset_phase_service", "/agc/air/reset_phase");
    // W7-b（步 1·甲-1）：空中控制权交接——本节点为唯一任务协调者，在**起飞完成（悬停）**后显式触发
    //   `/agc/air/release_control`（Trigger），把 setpoint 权威从 takeoff_controller 交给 trajectory_tracker。
    //   触发时机见文件头（提前 release 会让飞机永不 arm）。话题 / 门限均单一注入（契约 #params）。
    release_service_ = declare_parameter<std::string>("release_service", "/agc/air/release_control");
    air_odom_topic_ = declare_parameter<std::string>("air_odom_topic", "/agc/air/odom");
    // 悬停判据 = `/agc/air/odom` 世界 ENU z 相对**起飞前基准（本 run 最小 z，即地面高度）**的爬升量（m）。
    // 默认 2.3 与 `air.yaml` 的 takeoff_altitude_m=2.5 / hover_tolerance_m=0.25 对齐（2.5−0.25≈2.25）。
    air_hover_climb_m_ = declare_parameter<double>("air_hover_climb_m", 2.3);
    // 有界等待起飞完成（s）——超时即如实拒绝 start（INFRA_FAILURE），**不**静默假设已起飞。
    // 缺省值与真源 `w6_base.yaml`（45.0）保持一致，避免"节点缺省 ≠ 注入值"的漂移。
    air_hover_timeout_s_ = declare_parameter<double>("air_hover_timeout_s", 45.0);
    // 运行模式（R-001）：显式声明本次运行是否包含空中域，取代"服务缺席即推定无空中域"。
    // true = 整链（含空中域）：相位复位（C5）为硬门禁，服务未发现/超时/失败一律拒绝 start；
    // false = 纯地面运行（M0 / W5 / 轻量 W6）：无相位可复位，该项不适用。由 launch 单一设置。
    air_present_ = declare_parameter<bool>("air_present", true);
    ground_controller_name_ =
      declare_parameter<std::string>("ground_controller_name", "agc_rl");

    odom_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      odom_valid_topic, 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {
        odom_valid_ = m->data;
        last_odom_valid_rx_ = now();   // R-013：记录到达时刻（复位后新鲜度基准）
        have_odom_valid_ = true;
      });
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_topic, 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr) {last_cmd_rx_ = now(); have_cmd_ = true;});
    target_sub_ = create_subscription<agc_msgs::msg::TargetTrackArray>(
      target_topic, 10,
      [this](agc_msgs::msg::TargetTrackArray::SharedPtr m) {
        if (m->tracks.empty()) {have_target_ = false; target_id_.clear(); return;}
        const auto & t = m->tracks.front();
        have_target_ = true;
        target_id_ = t.target_id;
        target_x_ = t.pose.pose.position.x;
        target_y_ = t.pose.pose.position.y;
        target_conf_ = t.confidence;
        target_stamp_ = rclcpp::Time(t.header.stamp, RCL_ROS_TIME);
      });
    ground_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ground_odom_topic, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        car_x_ = m->pose.pose.position.x;
        car_y_ = m->pose.pose.position.y;
        last_car_odom_rx_ = now();   // R-013：记录到达时刻（复位后新鲜度基准）
        have_car_ = true;
      });
    air_active_sub_ = create_subscription<std_msgs::msg::String>(
      air_active_topic, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::SharedPtr m) {air_controller_ = m->data;});
    // R-035：**只读**订阅空中控制冲突（transient_local + reliable）——仅记录当前值 + 变化时日志。
    //   **严禁**改状态机 / 终态 / 语义（冲突非任务终态判据，仅为可观测性，见 ADR-0050）。
    const auto air_conflict_topic =
      declare_parameter<std::string>("air_control_conflict_topic", "/agc/air/control_conflict");
    air_conflict_sub_ = create_subscription<std_msgs::msg::Bool>(
      air_conflict_topic, rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::Bool::SharedPtr m) {
        if (m->data != air_control_conflict_) {
          air_control_conflict_ = m->data;
          RCLCPP_WARN(get_logger(),
            "R-035：空中控制冲突 control_conflict=%s（只读记录，不改任务状态机）",
            air_control_conflict_ ? "true" : "false");
        }
      });
    // W7-b（步 1·甲-1）：空中域里程计——**仅**用于「起飞完成（悬停）」判据（air_present=true 时生效）。
    //   取世界 ENU z（与 trajectoy_tracker / OffboardInterface 同一 odom 话题与口径）。
    air_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      air_odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        air_z_ = m->pose.pose.position.z;
        if (air_z_ < air_z_min_) {air_z_min_ = air_z_;}   // 本 run 最小 z = 起飞前基准（地面高度）
        have_air_odom_ = true;
      });
    jam_sub_ = create_subscription<std_msgs::msg::String>(
      jam_confirm_topic, 10,
      [this](std_msgs::msg::String::SharedPtr m) {
        // R-018：仅正向确认集合构成干扰终态；授权 / 发送 / ACK 单独不结算。
        // R-021：无 episode 字段的 jam 事件以「已存在本 episode（start 后）且在到达时刻记序」为门槛；
        // start 前到达（episode_epoch_ < 0）一律忽略，新 episode 亦在 onStart 清空。
        const std::string & d = m->data;
        if (d == "JAM_RTL" || d == "JAM_LAND" || d == "JAM_HOLD" || d == "JAM_LOST") {
          if (episode_epoch_ < 0) {return;}
          jam_reason_ = d;
          jam_rx_ = now();   // R-021：记录到达时刻（先命中者胜按时刻定序）
        }
      });
    // R-021：结算 capture 改订配套事件（ep / 时刻 / target）；ep 不匹配当前 episode → 拒收。
    capture_sub_ = create_subscription<std_msgs::msg::String>(
      capture_event_topic, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::SharedPtr m) {
        const std::string d = m->data;
        const int ep = parseEp(d);
        if (ep < 0 || episode_epoch_ < 0 || ep != episode_epoch_) {
          // ADR-0078：不再静默——episode 身份错位曾使已发生的 CAPTURE_NET 被整批丢弃（表现为 TIMEOUT）。
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "capture_event 拒收：ep=%d episode_epoch=%d（事件与当前 episode 身份不一致）",
            ep, episode_epoch_);
          return;   // 旧 / 异 episode 拒收
        }
        capture_reason_ = firstToken(d);
        capture_rx_ = now();
      });
    // R-021：学习当前 episode epoch（transient_local 对齐 scenario_reset 发布端 / R-015）。
    epoch_sub_ = create_subscription<std_msgs::msg::Int32>(
      epoch_topic, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Int32::SharedPtr m) {current_epoch_ = m->data;});

    state_pub_ = create_publisher<AgcTaskState>(state_topic, 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
    // R-041：goal 有效性 / 身份 / 时刻伴随诊断（transient_local，与 goal 同步发布；含失效判定）
    goal_info_pub_ = create_publisher<std_msgs::msg::String>(
      goal_info_topic, rclcpp::QoS(1).transient_local());
    // R-022：新 episode 标识发布（transient_local，对齐 epoch 话题 / R-015）。
    episode_pub_ = create_publisher<std_msgs::msg::Int32>(
      episode_topic, rclcpp::QoS(1).transient_local());
    // R-023：dwell 分段事件发布（std_msgs/String，depth 10，非命令者）
    dwell_pub_ = create_publisher<std_msgs::msg::String>(dwell_event_topic, 10);
    // 复位记账（C7 / R7）：重试次数与最终失败，供 run_logger 计算 reset_failure_rate
    const auto retries_topic = declare_parameter<std::string>("reset_retries_topic", "/agc/task/reset_retries");
    const auto failed_topic = declare_parameter<std::string>("reset_failed_topic", "/agc/task/reset_failed");
    reset_retries_pub_ = create_publisher<std_msgs::msg::Int32>(
      retries_topic, rclcpp::QoS(1).transient_local());
    reset_failed_pub_ = create_publisher<std_msgs::msg::Bool>(
      failed_topic, rclcpp::QoS(1).transient_local());

    reset_client_node_ = std::make_shared<rclcpp::Node>("task_manager_svc_client");
    reset_client_ = reset_client_node_->create_client<Trigger>("/agc/scenario/reset");
    reset_phase_client_ = reset_client_node_->create_client<Trigger>(reset_phase_service_);
    // W7-b（步 1·甲-1）：空中控制权交接客户端（takeoff_controller 提供 /agc/air/release_control）。
    release_client_ = reset_client_node_->create_client<Trigger>(release_service_);

    // R-013：start/abort 服务放入**独立互斥回调组**。服务回调会阻塞等待 reset/reset_phase
    // （spin_until_future_complete）；订阅留在默认组并交由多线程执行器在其它线程处理，
    // 故事务期间 odom_valid / 地面 odom / cmd_vel 仍持续更新（契约 #lessons）。
    srv_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    start_srv_ = create_service<Trigger>(
      "/agc/task/start",
      std::bind(&TaskManager::onStart, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, srv_cb_group_);
    abort_srv_ = create_service<Trigger>(
      "/agc/task/abort",
      std::bind(&TaskManager::onAbort, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, srv_cb_group_);

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&TaskManager::onStateTimer, this));
    goal_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&TaskManager::publishGoal, this));
    fsm_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / fsm_hz_), std::bind(&TaskManager::tickFsm, this));
  }

private:
  static const char * nameOf(uint8_t s)
  {
    switch (s) {
      case AgcTaskState::IDLE: return "IDLE";
      case AgcTaskState::AIR_SEARCH: return "AIR_SEARCH";
      case AgcTaskState::TARGET_ACQUIRED: return "TARGET_ACQUIRED";
      case AgcTaskState::GROUND_APPROACH: return "GROUND_APPROACH";
      case AgcTaskState::TRACKING_SUCCESS: return "TRACKING_SUCCESS";
      case AgcTaskState::MISSION_FAILED: return "MISSION_FAILED";
      case AgcTaskState::ABORTED: return "ABORTED";
      case AgcTaskState::INTERCEPT_CONFIRMED: return "INTERCEPT_CONFIRMED";
      case AgcTaskState::INFRA_FAILURE: return "INFRA_FAILURE";
      default: return "OTHER";
    }
  }

  static bool isActive(uint8_t s)
  {
    return s == AgcTaskState::AIR_SEARCH || s == AgcTaskState::TARGET_ACQUIRED ||
           s == AgcTaskState::GROUND_APPROACH;
  }

  static bool isTerminal(uint8_t s)
  {
    return s == AgcTaskState::TRACKING_SUCCESS || s == AgcTaskState::INTERCEPT_CONFIRMED ||
           s == AgcTaskState::MISSION_FAILED || s == AgcTaskState::ABORTED ||
           s == AgcTaskState::INFRA_FAILURE;
  }

  // R-021：配套事件串 `CAPTURE_NET ep=<epoch> t=<ros_sec> target=<id>` 的解析。
  static int parseEp(const std::string & s)
  {
    const auto pos = s.find("ep=");
    if (pos == std::string::npos) {return -1;}
    try {
      return std::stoi(s.substr(pos + 3));
    } catch (const std::exception &) {
      return -1;
    }
  }

  static std::string firstToken(const std::string & s)
  {
    const auto sp = s.find(' ');
    return (sp == std::string::npos) ? s : s.substr(0, sp);
  }

  // R-022：调用结果三分——Ok=收到应答且 success=true；Failed=收到应答但 success=false（**确定性**）；
  //        Indeterminate=服务未就绪（required 且 wait_for_service 超时）或 spin 超时/取消（**非确定性**）。
  enum class CallOutcome { Ok, Failed, Indeterminate };

  CallOutcome callTrigger(
    const rclcpp::Client<Trigger>::SharedPtr & cli, const char * what, bool required)
  {
    // optional（required=false）不阻塞等待服务出现。R-013 起本节点运行在多线程执行器上，
    // start/abort 服务回调位于独立回调组；订阅在默认组由其它线程处理，故事务期间
    // odom_valid / 地面 odom / cmd_vel 仍持续更新（契约 #lessons：阻塞服务回调不得冻结本地缓存）。
    if (!cli->service_is_ready()) {
      if (required) {
        if (!cli->wait_for_service(std::chrono::seconds(2))) {
          RCLCPP_WARN(get_logger(), "%s service unavailable（未就绪→Indeterminate）", what);
          return CallOutcome::Indeterminate;
        }
      } else {
        RCLCPP_WARN(get_logger(), "%s service unavailable（跳过，best-effort）", what);
        return CallOutcome::Ok;
      }
    }
    auto fri = cli->async_send_request(std::make_shared<Trigger::Request>());
    std::shared_future<Trigger::Response::SharedPtr> fut = fri.future.share();
    const auto rc = rclcpp::spin_until_future_complete(
      reset_client_node_, fut, std::chrono::duration<double>(reset_call_timeout_s_));
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      // R-022：超时/取消→清除悬挂的待决请求（否则 Client 内部状态泄漏），并判非确定性。
      cli->remove_pending_request(fri.request_id);
      RCLCPP_WARN(get_logger(), "%s call 超时/取消（Indeterminate）", what);
      return CallOutcome::Indeterminate;
    }
    return fut.get()->success ? CallOutcome::Ok : CallOutcome::Failed;
  }

  /// W7-b（步 1·甲-1）：有界等待「起飞完成（悬停）」——判据 = `/agc/air/odom` 世界 ENU z 相对**起飞前
  ///   基准**（本 run 最小 z，即地面高度）的爬升量 ≥ `air_hover_climb_m_`（飞机已在空中 ⇒ takeoff 已 arm
  ///   且进入 Offboard）。返回空串 = 就绪；非空 = 失败原因（供 fail()，如实拒绝 start，**不**静默假设已起飞）。
  std::string waitAirHover()
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(air_hover_timeout_s_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (have_air_odom_ && (air_z_ - air_z_min_) >= air_hover_climb_m_) {return std::string();}
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_WARN(get_logger(),
      "等待起飞完成超时（%.1fs 内未见爬升达 %.2f m：air_odom=%d z=%.2f base=%.2f）",
      air_hover_timeout_s_, air_hover_climb_m_, have_air_odom_ ? 1 : 0, air_z_, air_z_min_);
    return std::string("air_hover_timeout");
  }

  void onStart(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res)
  {
    // R-022：允许态检查——仅在 IDLE / 任一终态接受；active 重入**直接拒绝**（不尝试 reset、不 fail()）。
    // 在任何 reset 之前执行，避免 active 期间误发 reset 或把 active 打成 INFRA_FAILURE。
    if (isActive(state_)) {
      res->success = false;
      res->message = "start rejected: already_active state=" + std::string(nameOf(state_));
      RCLCPP_WARN(get_logger(), "start 拒绝：active 重入（state=%s），不尝试 reset", nameOf(state_));
      return;
    }
    // 1) 先 reset（契约 #lessons 竞态修复：reset 在前，不得先校验 odom，否则自伤 ODOM_INVALID）
    //    复位失败最多重试 3 次（C7 口径，ADR-0023）；重试不占 episode 预算（episode 计时自 start 成功起）
    //    R-022：reset 前记录 epoch（避免 reset 后 epoch 投递竞态）。
    const int ep_before_reset = current_epoch_;
    CallOutcome reset_outcome = CallOutcome::Failed;
    int attempts = 0;
    for (; attempts < 3; ++attempts) {
      reset_outcome = callTrigger(reset_client_, "scenario/reset", true);
      if (reset_outcome == CallOutcome::Ok) {break;}
      RCLCPP_WARN(get_logger(), "scenario/reset 第 %d 次失败，重试（≤3）", attempts + 1);
    }
    reset_retries_ = attempts;   // 0 = 首次即成功
    if (reset_outcome == CallOutcome::Failed) {
      // R-022：确定性门禁失败（确实收到 success=false 应答）→ 允许 INFRA_FAILURE。
      reset_failed_ = true;
      publishResetStats();
      fail(res, "reset_failed");
      return;
    }
    if (reset_outcome == CallOutcome::Indeterminate) {
      // R-022：非确定性超时/取消（未收到应答）→ 仅回 success=false，**不改** state_
      //（不误置 INFRA_FAILURE、也不误置 active）。
      reset_failed_ = true;
      publishResetStats();
      res->success = false;
      res->message = "start rejected: reset_indeterminate (slow/timeout; state unchanged)";
      RCLCPP_WARN(get_logger(), "start 事务超时/取消：状态保持 %s（不误置）", nameOf(state_));
      return;
    }
    reset_failed_ = false;
    publishResetStats();
    // 复位完成的 epoch：门禁新鲜度以"复位之后收到的数据"为基准（R-013；#lessons 禁读阻塞前缓存）
    reset_done_time_ = now();
    // R-022：episode 身份——有界等待 reset 使 epoch 递增（≤0.5s，每 5ms 探测；超时以 current_epoch_ 兜底），
    //        再记录本 episode epoch 并发布 /agc/task/episode（transient_local，供 R-021 配套事件关联）。
    {
      const auto ep_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
      while (current_epoch_ == ep_before_reset && std::chrono::steady_clock::now() < ep_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      episode_epoch_ = current_epoch_;
      std_msgs::msg::Int32 ep;
      ep.data = episode_epoch_;
      episode_pub_->publish(ep);
    }
    // 2) 相位复位（C5）：仅在显式声明"有空中域"的运行模式（air_present=true）下强制。
    //    整链必须收到成功应答；服务缺席、超时或应答失败一律拒绝 start（R-001）。
    //    air_present=false 才表示本运行声明无空中域，相位复位不适用（不再由服务缺席推定）。
    //    R-022：Failed→INFRA_FAILURE；Indeterminate→不改状态、只回失败。
    if (air_present_) {
      const CallOutcome phase_outcome = callTrigger(reset_phase_client_, "air/reset_phase", true);
      if (phase_outcome == CallOutcome::Failed) {
        fail(res, "phase_reset_failed");
        return;
      }
      if (phase_outcome == CallOutcome::Indeterminate) {
        res->success = false;
        res->message = "start rejected: phase_reset_indeterminate (slow/timeout; state unchanged)";
        RCLCPP_WARN(get_logger(), "phase 复位超时/取消：状态保持 %s（不误置）", nameOf(state_));
        return;
      }
    } else {
      RCLCPP_INFO(
        get_logger(), "air_present=false（声明无空中域）：跳过相位复位门禁（C5 不适用）");
    }

    // 3) 再校验门禁（reset 之后；= M0 五门 + 相位复位完成 + R-013 新鲜度/epoch + 时钟进展）——
    //    任一不过 → INFRA_FAILURE。各门数据须为**复位完成后**收到且未超龄（不用阻塞前缓存）。
    //    先**有界等待**复位后新鲜样本到达（订阅在其它线程持续接收）：复位完成瞬间适配器尚未
    //    发布下一帧，若不等待会把正常启动误判为 stale；超时仍无新样本 → 判 stale。
    auto freshNow = [this, &reset_done_time = reset_done_time_]() {
      const auto t = now();
      return have_odom_valid_ && last_odom_valid_rx_ >= reset_done_time &&
             (t - last_odom_valid_rx_).seconds() <= odom_valid_max_age_s_ &&
             have_car_ && last_car_odom_rx_ >= reset_done_time &&
             (t - last_car_odom_rx_).seconds() <= odom_valid_max_age_s_ &&
             have_cmd_ && last_cmd_rx_ >= reset_done_time &&
             (t - last_cmd_rx_).seconds() <= 0.5;
    };
    const auto wait_deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(odom_valid_max_age_s_);
    while (!freshNow() && std::chrono::steady_clock::now() < wait_deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const auto t_now = now();
    const double age_ov = (t_now - last_odom_valid_rx_).seconds();
    const double age_car = (t_now - last_car_odom_rx_).seconds();
    const double age_cmd = (t_now - last_cmd_rx_).seconds();
    const bool ov_fresh =
      have_odom_valid_ && last_odom_valid_rx_ >= reset_done_time_ && age_ov <= odom_valid_max_age_s_;
    const bool car_fresh =
      have_car_ && last_car_odom_rx_ >= reset_done_time_ && age_car <= odom_valid_max_age_s_;
    const bool cmd_fresh = have_cmd_ && last_cmd_rx_ >= reset_done_time_ && age_cmd <= 0.5;
    // 时钟门 = 进展校验：now>0 且相对上一采样严格推进（冻结时钟 → 相等 → 不过）
    const bool clock_synced = t_now.seconds() > 0.0;
    const bool clock_advancing = clock_synced && t_now > prev_clock_;

    std::string reason;
    if (!clock_synced) {reason = "clock_not_synced";}
    else if (!clock_advancing) {reason = "clock_not_progressing";}
    else if (world_name_ != "agc_world") {reason = "single_world_mismatch";}
    else if (!ov_fresh) {reason = "odom_valid_stale";}
    else if (!odom_valid_) {reason = "odom_invalid";}
    else if (!car_fresh) {reason = "ground_odom_stale";}
    else if (!cmd_fresh) {reason = "cmd_vel_loopback_missing";}

    const std::string evid = evidenceSummary(t_now, age_ov, age_car, age_cmd);
    if (!reason.empty()) {
      RCLCPP_INFO(get_logger(), "R-013 门禁证据(失败=%s): %s", reason.c_str(), evid.c_str());
      fail(res, reason, evid);
      return;
    }

    // 4) 空中控制权交接（W7-b 步 1·甲-1）：门禁通过后，整链模式**等「起飞完成（悬停）」**再显式触发
    //    release_control（takeoff_controller → trajectory_tracker）。轻量栈无空中域，整段跳过。
    //    R-022 三分：Failed→INFRA_FAILURE；Indeterminate→不改状态、只回失败（不误置）。
    if (air_present_) {
      const std::string hover_reason = waitAirHover();
      if (!hover_reason.empty()) {
        fail(res, hover_reason);
        return;
      }
      const CallOutcome rel_outcome = callTrigger(release_client_, "air/release_control", true);
      if (rel_outcome == CallOutcome::Failed) {
        fail(res, "release_control_failed");
        return;
      }
      if (rel_outcome == CallOutcome::Indeterminate) {
        res->success = false;
        res->message = "start rejected: release_control_indeterminate (slow/timeout; state unchanged)";
        RCLCPP_WARN(get_logger(), "release_control 超时/取消：状态保持 %s（不误置）", nameOf(state_));
        return;
      }
      RCLCPP_INFO(get_logger(),
        "空中控制权交接：已触发 %s（takeoff_controller → trajectory_tracker）", release_service_.c_str());
    }

    start_time_ = now();
    dwell_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    // R-023：新 episode 起点——dwell 全部清零（杜绝跨 episode 累计有效时长）。
    dwell_segment_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    dwell_target_id_.clear();
    dwell_epoch_ = -1;
    last_dwell_segment_s_ = 0.0;
    dwell_valid_total_s_ = 0.0;
    tracking_milestone_met_ = false;
    // R-041：新 episode 起点——清「最后有效目标」缓存（episode 间隔离，不得跨回合保持旧目标）。
    have_last_valid_target_ = false;
    last_valid_target_id_.clear();
    // R-022：新 episode 解锁终态锁存（latched_state_/epoch 复位）。
    latched_state_ = 0xFF;
    latched_epoch_ = -1;
    // R-021：新 episode 起点——清 capture / jam 原因与事件时刻（episode_epoch_ 已在 reset 后按 epoch 记录）。
    capture_reason_.clear();
    jam_reason_.clear();
    capture_rx_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    jam_rx_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    setState(AgcTaskState::AIR_SEARCH, "ok");
    res->success = true;
    res->message = "started: AIR_SEARCH " + evid;
    RCLCPP_INFO(get_logger(), "R-013 门禁证据(通过): %s", evid.c_str());
    RCLCPP_INFO(get_logger(), "start OK -> AIR_SEARCH");
  }

  void onAbort(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res)
  {
    // R-022：abort 仅在 active 接受；终态之后到达的 abort SHALL NOT 覆写已锁存终态（回显当前态）。
    if (!isActive(state_)) {
      res->success = false;
      res->message = "abort rejected: not_active state=" + std::string(nameOf(state_));
      RCLCPP_WARN(get_logger(), "abort 拒绝：非 active（state=%s，已锁存）", nameOf(state_));
      return;
    }
    // R-023：任务离开 active（abort / 进终态）即视为暂停——活跃 dwell 先中断记录。
    if (dwell_start_.nanoseconds() != 0) {interruptDwell(now(), "TASK_PAUSED");}
    enterTerminal(AgcTaskState::ABORTED, "abort");
    res->success = true;
    res->message = "aborted";
  }

  // R-022：进入终态的唯一入口——锁存「终态 + 所属 episode」并置状态（供迟到命令不覆写、结果可回溯）。
  void enterTerminal(uint8_t s, const std::string & reason)
  {
    latched_state_ = s;
    latched_epoch_ = episode_epoch_;
    setState(s, reason);
  }

  // R-023：dwell 事件发布——起始 / 中断（episode / 分段 / 累计时长），std_msgs 状态事件（非命令者）。
  void publishDwellEvent(const std::string & kind, const std::string & reason, double seg)
  {
    char buf[256];
    if (kind == "DWELL_START") {
      std::snprintf(
        buf, sizeof(buf), "DWELL_START ep=%d target=%s",
        episode_epoch_, dwell_target_id_.c_str());
    } else {
      std::snprintf(
        buf, sizeof(buf), "DWELL_INTERRUPT ep=%d reason=%s seg=%.3f total=%.3f",
        episode_epoch_, reason.c_str(), seg, dwell_valid_total_s_);
    }
    std_msgs::msg::String m;
    m.data = buf;
    dwell_pub_->publish(m);
  }

  // R-023：中断当前活跃 dwell——结算分段有效时长并入本 episode 累计，发布 DWELL_INTERRUPT 后清零。
  void interruptDwell(const rclcpp::Time & t, const std::string & reason)
  {
    if (dwell_start_.nanoseconds() == 0) {return;}
    const double seg = (t - dwell_segment_start_).seconds();
    last_dwell_segment_s_ = seg;
    dwell_valid_total_s_ += seg;
    publishDwellEvent("DWELL_INTERRUPT", reason, seg);
    dwell_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    dwell_segment_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    dwell_target_id_.clear();
    dwell_epoch_ = -1;
  }

  void fail(
    const std::shared_ptr<Trigger::Response> & res, const std::string & reason,
    const std::string & evid = "")
  {
    enterTerminal(AgcTaskState::INFRA_FAILURE, reason);
    res->success = false;
    res->message = "start rejected: " + reason + (evid.empty() ? std::string() : " " + evid);
    RCLCPP_WARN(get_logger(), "start rejected: %s", reason.c_str());
  }

  // R-013：门禁证据摘要（epoch / 各门到达时刻 / 时钟采样），随 Trigger 应答回传并可关联 run 记录。
  std::string evidenceSummary(
    const rclcpp::Time & t_now, double age_ov, double age_car, double age_cmd) const
  {
    char buf[512];
    std::snprintf(
      buf, sizeof(buf),
      "[evid] reset_done=%.3f now=%.3f prev_clock=%.3f world=%s "
      "odom_valid_rx=%.3f(age=%.3f,val=%d) ground_odom_rx=%.3f(age=%.3f) "
      "cmd_rx=%.3f(age=%.3f)",
      reset_done_time_.seconds(), t_now.seconds(), prev_clock_.seconds(), world_name_.c_str(),
      last_odom_valid_rx_.seconds(), age_ov, odom_valid_ ? 1 : 0,
      last_car_odom_rx_.seconds(), age_car,
      last_cmd_rx_.seconds(), age_cmd);
    return std::string(buf);
  }

  // 1 Hz 状态发布 + 时钟进展采样（默认回调组；服务阻塞期间仍持续采样，供时钟门比对）
  void onStateTimer()
  {
    publishState();
    prev_clock_ = now();
  }

  void publishResetStats()
  {
    std_msgs::msg::Int32 r;
    r.data = reset_retries_;
    reset_retries_pub_->publish(r);
    std_msgs::msg::Bool f;
    f.data = reset_failed_;
    reset_failed_pub_->publish(f);
  }

  void setState(uint8_t s, const std::string & reason)
  {
    state_ = s;
    reason_ = reason;
    publishState();
  }

  bool targetFresh(double & age_out) const
  {
    if (!have_target_) {return false;}
    age_out = (now() - target_stamp_).seconds();
    return age_out >= 0.0 && age_out <= target_max_age_s_ && target_conf_ >= confidence_min_;
  }

  void tickFsm()
  {
    if (!isActive(state_)) {return;}
    const auto t = now();
    AgeProbe();
    // C4：超时一律算失败
    if ((t - start_time_).seconds() > t_max_) {
      enterTerminal(AgcTaskState::MISSION_FAILED, "TIMEOUT");
      return;
    }
    double age = NAN;
    const bool fresh = targetFresh(age);
    switch (state_) {
      case AgcTaskState::AIR_SEARCH:
        if (fresh) {setState(AgcTaskState::TARGET_ACQUIRED, "target_acquired");}
        break;
      case AgcTaskState::TARGET_ACQUIRED:
        if (have_cmd_ && (t - last_cmd_rx_).seconds() < 0.5) {
          setState(AgcTaskState::GROUND_APPROACH, "ground_approach");
        }
        break;
      case AgcTaskState::GROUND_APPROACH: {
        // R11：捕获 / 干扰命中 → INTERCEPT_CONFIRMED。
        // R-018：jam_reason_ 仅由 jammer 的**已确认事件**（jam_confirm）写入 → 授权 / 发送不结算。
        // R-021：本 episode 内仅接受配套事件；先命中者胜**按事件到达时刻定序**（不固定 capture 优先）。
        const bool have_cap = !capture_reason_.empty();
        const bool have_jam = !jam_reason_.empty() && jam_reason_ != "JAM_NONE";
        if (have_cap && have_jam) {
          enterTerminal(
            AgcTaskState::INTERCEPT_CONFIRMED,
            (capture_rx_ <= jam_rx_) ? capture_reason_ : jam_reason_);
          break;
        }
        if (have_cap) {enterTerminal(AgcTaskState::INTERCEPT_CONFIRMED, capture_reason_); break;}
        if (have_jam) {enterTerminal(AgcTaskState::INTERCEPT_CONFIRMED, jam_reason_); break;}
        // R-023：dwell 逐拍显式判据——任何不满足「目标新鲜 + 同 target_id + 车地面 odom 有效未超龄 +
        //        包络内」的拍（含目标数组为空 / have_target_==false）一律中断清零；不得把失联段累计进
        //        dwell；成功仍为「连续 t_dwell 有效」（C8 不降级为累计接近时长）。
        // 本拍合法性
        const double car_age = (have_car_ && last_car_odom_rx_.nanoseconds() != 0)
                               ? (t - last_car_odom_rx_).seconds()
                               : std::numeric_limits<double>::infinity();
        const bool car_ok = have_car_ && car_age >= 0.0 && car_age <= car_odom_max_age_s_;
        const bool range_ok = have_target_ && car_ok &&
                              std::hypot(target_x_ - car_x_, target_y_ - car_y_) <= d_track_;
        // 1) 已有活跃 dwell：逐拍连续性检查，任一不满足即中断（含空数组 / 换 id / 车 odom 失效超龄 /
        //    目标失效 / 包络外 / epoch 变化）。
        if (dwell_start_.nanoseconds() != 0) {
          std::string ir;
          if (current_epoch_ != dwell_epoch_) {ir = "EPOCH_CHANGED";}
          else if (!have_target_) {ir = "TARGET_LOST";}
          else if (!fresh) {ir = (target_conf_ < confidence_min_) ? "TARGET_LOW_CONF" : "TARGET_STALE";}
          else if (!car_ok) {ir = have_car_ ? "CAR_ODOM_STALE" : "CAR_ODOM_MISSING";}
          else if (target_id_ != dwell_target_id_) {ir = "TARGET_ID_CHANGED";}
          else if (std::hypot(target_x_ - car_x_, target_y_ - car_y_) > d_track_) {ir = "RANGE_EXCEEDED";}
          if (!ir.empty()) {interruptDwell(t, ir);}
        }
        // 2) 无活跃 dwell 且本拍合法 → 起新 dwell（换 id / epoch 变化后在清零基础上从 0 重算，杜绝跨段累计）。
        if (dwell_start_.nanoseconds() == 0 && range_ok && fresh && have_target_) {
          dwell_start_ = t;
          dwell_segment_start_ = t;
          dwell_target_id_ = target_id_;
          dwell_epoch_ = current_epoch_;
          publishDwellEvent("DWELL_START", "", 0.0);
        }
        // 3) 连续达阈值才成功（保持 C8 不降级）
        if (dwell_start_.nanoseconds() != 0 && (t - dwell_start_).seconds() >= t_dwell_) {
          tracking_milestone_met_ = true;
          // ADR-0076：capture 场景（tracking_milestone_is_terminal=false）下**不**锁存 TRACKING_SUCCESS——
          //   否则 `capture_adjudicator` 的 GROUND_APPROACH 交火窗口会被提前关闭，捕获永远不发生。
          //   里程碑仍如实记入 tracking_milestone_met_（随 /agc/task/state 发布），供指标使用；
          //   结局交由捕获裁决（CAPTURE_NET）或 t_max TIMEOUT 决定。
          if (tracking_milestone_is_terminal_) {
            enterTerminal(AgcTaskState::TRACKING_SUCCESS, "tracking_milestone");
          }
        }
        break; }
      default: break;
    }
  }

  // 空实现占位（避免未用变量告警；保留扩展点）
  void AgeProbe() {}

  void publishState()
  {
    AgcTaskState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.state = state_;
    msg.state_name = nameOf(state_);
    msg.reason_code = reason_;
    msg.air_controller = air_controller_.empty() ? "none" : air_controller_;
    msg.ground_controller =
      (state_ == AgcTaskState::GROUND_APPROACH || state_ == AgcTaskState::TRACKING_SUCCESS) ?
      ground_controller_name_ : "none";
    msg.target_id = target_id_;
    double age = NAN;
    targetFresh(age);
    if (have_target_ && have_car_) {
      msg.target_xy_distance_m = static_cast<float>(std::hypot(target_x_ - car_x_, target_y_ - car_y_));
    } else {
      msg.target_xy_distance_m = std::nanf("");
    }
    msg.target_obs_age_s = std::isnan(age) ? std::nanf("") : static_cast<float>(age);
    msg.tracking_milestone_met = tracking_milestone_met_;
    state_pub_->publish(msg);
  }

  // R-041：目标失效策略 =「保留有期限的最后有效目标」（bounded hold-last-valid，见 ADR-0056）。
  //   目标失效时**不**反射固定点、**不**刷新时间戳（新 stamp 会掩盖失联）；在窗口内保持最后有效
  //   目标的**原位置与原源时刻**（age 自然增长，如实暴露陈旧）；超窗即显式失效（valid=false）。
  //   valid=false 时**不发布**带新时间戳的 goal 位姿——仅发伴随诊断，由地面安全停车（fail-safe stop）。
  //   有效性 / 身份 / 时刻经伴随话题 /agc/ground/goal_info（std_msgs/String，transient_local）带外传递。
  void publishGoal()
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "world";
    goal.pose.orientation.w = 1.0;

    bool valid = false;
    std::string reason;
    std::string tgt_id;
    double gx = goal_x_, gy = goal_y_;
    double src_stamp = -1.0;   // 源时刻（s）；<0 表示无源时刻
    double age = std::nan("");

    if (!goal_follow_target_) {
      // 固定任务目标本身恒有效；stamp = now（其有效性不依赖外部测量新鲜度）
      const rclcpp::Time tnow = now();
      goal.header.stamp = tnow;
      src_stamp = tnow.seconds();
      valid = true;
      reason = "FIXED_GOAL";
    } else if (targetFresh(age)) {
      // 目标新鲜：位置 = 目标 XY；stamp = **源时刻**（非 now）；关联 target_id；刷新「最后有效目标」缓存
      gx = target_x_; gy = target_y_;
      goal.header.stamp = target_stamp_;
      src_stamp = target_stamp_.seconds();
      tgt_id = target_id_;
      valid = true;
      reason = "TARGET_FRESH";
      have_last_valid_target_ = true;
      last_valid_target_x_ = target_x_;
      last_valid_target_y_ = target_y_;
      last_valid_target_stamp_ = target_stamp_;
      last_valid_target_rx_ = now();
      last_valid_target_id_ = target_id_;
    } else if (have_last_valid_target_ &&
      (now() - last_valid_target_rx_).seconds() <= goal_hold_valid_s_)
    {
      // 有期限保持：窗口内保持**最后有效目标**的原位置与原源时刻（age 自然增长，暴露陈旧、不掩盖）
      gx = last_valid_target_x_; gy = last_valid_target_y_;
      goal.header.stamp = last_valid_target_stamp_;
      src_stamp = last_valid_target_stamp_.seconds();
      tgt_id = last_valid_target_id_;
      age = (now() - last_valid_target_stamp_).seconds();
      valid = true;
      reason = "HOLD_LAST_VALID";
    } else {
      // 显式失效：超窗（有最后有效目标 → EXPIRED）或从未有效（TARGET_LOST）。
      // **不**发新时间戳固定点 goal 位姿；仅发伴随诊断 valid=false，由地面安全停车。
      valid = false;
      reason = have_last_valid_target_ ? "EXPIRED" : "TARGET_LOST";
      tgt_id = last_valid_target_id_;
      if (have_last_valid_target_) {
        src_stamp = last_valid_target_stamp_.seconds();
        age = (now() - last_valid_target_stamp_).seconds();
      }
    }

    if (valid) {
      goal.pose.position.x = gx;
      goal.pose.position.y = gy;
      goal_pub_->publish(goal);
    }

    // 伴随诊断：每次判定均发布（含失效）——下游据此判「此 goal 是否仍绑定有效目标」。
    std_msgs::msg::String info;
    char buf[256];
    std::snprintf(
      buf, sizeof(buf), "valid=%d;reason=%s;target_id=%s;src_stamp=%.6f;age=%.6f",
      valid ? 1 : 0, reason.c_str(), tgt_id.c_str(), src_stamp,
      std::isnan(age) ? -1.0 : age);
    info.data = buf;
    goal_info_pub_->publish(info);
  }

  std::string world_name_, reset_phase_service_, ground_controller_name_;
  // W7-b（步 1·甲-1）：空中控制权交接参数与观测（世界 ENU z 相对**起飞前基准**的爬升量判悬停）
  std::string release_service_, air_odom_topic_;
  double air_hover_climb_m_{2.3};
  double air_hover_timeout_s_{18.0};
  bool have_air_odom_{false};
  double air_z_{0.0};
  double air_z_min_{std::numeric_limits<double>::infinity()};
  double goal_x_{4.0}, goal_y_{3.0};
  double d_track_{3.0}, t_dwell_{3.0}, t_max_{120.0}, confidence_min_{0.5}, target_max_age_s_{0.5};
  double odom_valid_max_age_s_{1.0};   // R-013：复位后 odom_valid / 地面 odom 新鲜度上限
  double fsm_hz_{10.0};
  double reset_call_timeout_s_{8.0};   // R-022：reset / reset_phase 调用超时（s）
  bool goal_follow_target_{false};
  double goal_hold_valid_s_{1.0};   // R-041：最后有效目标保持窗口（s）；超窗显式失效
  // R-041：最后有效目标缓存（位置 / 源时刻 / 身份 / 最近有效到达时刻）；新 episode 清空
  bool have_last_valid_target_{false};
  double last_valid_target_x_{0.0}, last_valid_target_y_{0.0};
  rclcpp::Time last_valid_target_stamp_{0, 0, RCL_ROS_TIME};   // 最后有效目标的**源时刻**
  rclcpp::Time last_valid_target_rx_{0, 0, RCL_ROS_TIME};      // 最后有效目标的**到达时刻**（保持窗口基准）
  std::string last_valid_target_id_{};
  bool air_present_{true};   // 运行模式（R-001）：是否含空中域（决定相位复位门禁是否适用）

  uint8_t state_{AgcTaskState::IDLE};
  // R-022：终态锁存（进入终态时记录「锁存终态 + 所属 episode」；0xFF / -1 = 未锁存）
  uint8_t latched_state_{0xFF};
  int latched_epoch_{-1};
  std::string reason_;
  bool odom_valid_{false};
  bool have_odom_valid_{false};
  bool have_cmd_{false};
  rclcpp::Time last_odom_valid_rx_{0, 0, RCL_ROS_TIME};   // R-013：odom_valid 到达时刻
  rclcpp::Time last_car_odom_rx_{0, 0, RCL_ROS_TIME};     // R-013：地面 odom 到达时刻
  rclcpp::Time last_cmd_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reset_done_time_{0, 0, RCL_ROS_TIME};      // R-013：复位完成 epoch（新鲜度基准）
  rclcpp::Time prev_clock_{0, 0, RCL_ROS_TIME};           // R-013：时钟进展门的上一次采样
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time dwell_start_{0, 0, RCL_ROS_TIME};
  // R-023：dwell 逐拍判据状态
  rclcpp::Time dwell_segment_start_{0, 0, RCL_ROS_TIME};   // 当前有效段起点（与 dwell_start_ 同步）
  std::string dwell_target_id_{};                          // 当前 dwell 所属 target_id
  int dwell_epoch_{-1};                                    // dwell 起始时记录的 current_epoch_
  double last_dwell_segment_s_{0.0};                       // 刚被中断的那段有效时长（s）
  double dwell_valid_total_s_{0.0};                        // 本 episode 内累计有效时长（s）
  double car_odom_max_age_s_{1.0};                         // 车地面 odom 寿命门（s）

  bool have_target_{false};
  std::string target_id_{};
  double target_x_{0.0}, target_y_{0.0};
  float target_conf_{0.0f};
  rclcpp::Time target_stamp_{0, 0, RCL_ROS_TIME};
  bool have_car_{false};
  double car_x_{0.0}, car_y_{0.0};

  std::string air_controller_{};
  bool air_control_conflict_{false};   // R-035：只读记录的空中控制冲突（**不**入状态机判据）
  std::string jam_reason_{};
  std::string capture_reason_{};
  // R-021：episode 身份 + 事件到达时刻（先命中者胜按时刻定序）
  int current_epoch_{-1};     // 订阅 /agc/scenario/reset_epoch 学习的当前 epoch
  int episode_epoch_{-1};     // 本 episode epoch（start 成功后置为 current_epoch_；<0 = 尚无 episode）
  rclcpp::Time capture_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time jam_rx_{0, 0, RCL_ROS_TIME};
  bool tracking_milestone_met_{false};
  // ADR-0076：tracking_milestone 是否等价终态（capture 场景 false → 只记指标、不改终态）
  bool tracking_milestone_is_terminal_{true};
  int reset_retries_{0};
  bool reset_failed_{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odom_valid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<agc_msgs::msg::TargetTrackArray>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr air_active_sub_, jam_sub_, capture_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr air_conflict_sub_;   // R-035：只读 control_conflict
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr air_odom_sub_;   // W7-b：起飞完成（悬停）判据
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr epoch_sub_;   // R-021：episode epoch 学习
  rclcpp::Publisher<AgcTaskState>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_info_pub_;   // R-041：goal 有效性/身份/时刻伴随诊断
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr episode_pub_;   // R-022：新 episode 标识发布
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dwell_pub_;    // R-023：dwell 分段事件发布
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr reset_retries_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_failed_pub_;
  std::shared_ptr<rclcpp::Node> reset_client_node_;
  rclcpp::Client<Trigger>::SharedPtr reset_client_, reset_phase_client_, release_client_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_, abort_srv_;
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;   // R-013：start/abort 服务独立互斥回调组
  rclcpp::TimerBase::SharedPtr state_timer_, goal_timer_, fsm_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // R-013：多线程执行器。start/abort 服务回调（阻塞等待 reset/reset_phase 客户端）在独立
  // 回调组，节点自身订阅在默认组 → 另一个线程在事务期间持续更新门禁缓存（契约 #lessons）。
  auto node = std::make_shared<TaskManager>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
