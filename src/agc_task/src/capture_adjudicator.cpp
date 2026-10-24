// capture_adjudicator —— 发射式网捕获裁决器（W6 R9 / ADR-0022；W7-b T2/E2 升级为「最小动态火控」）
//
// 物理捕获 = **发射式网**。W7-b（T2 / E2，ADR-0070(c) / F-11）把 W6 的**闭式包络占位**
// 升级为**最小动态火控裁决**：
//   ① 交战门（相对高度与可及集）：h = target.z − car.z − launcher_h；h∈[applicable_min, applicable_max]
//      且该高度可达（vy0²−2gh ≥ 0）且水平距离 ≤ 该高度闭式射程 + d/2 → 可交战（envelope_in）。
//   ② 确定射击时刻：可交战 ∧ 冷却结束 ∧ 任务窗口开 ∧ 身份匹配 → 本拍发射（新 shot_id）。
//   ③ 目标预测：由目标 twist（缺则最近两次位姿差分）外推，取瞄准方位角。
//   ④ 展开-飞行：网逐帧弹道积分——水平 = v0·cosθ·drag·t（常速）、垂直 = v0·sinθ·t − ½g t²；
//      展开时间 net_t_exp_s 内有效半径线性增长至 d/2。
//   ⑤ 命中 / 失手：逐帧以 GZ 提供的目标真值位姿与网位置作球比对（半径 = d/2）→ 命中；
//      网落地 / 超 net_flight_timeout_s → 失手；随后进入冷却 fire_cooldown_s。
//   ⑥ 证据：shot_id（epoch-seq）、射击时刻、预测目标位、最近逼近距离、命中/失手、飞行时长。
//
// 生命周期规则（ADR-0091）：网一旦在飞，弹道时钟 / 落地 / 超时结算**独立于**发射资格——
//   车 / 目标位姿失效、置信度下降、目标越出适用高度 / 可达集 / 水平射程时仍**每拍**推进；
//   目标无效时不判命中（不用缓存位姿），但落地 / 超时照常结算。落地 / 超时一律以 ROS 仿真时钟
//   （use_sim_time=true）判定。每次 SHOT_FIRED 恰有一个结局：SHOT_HIT / SHOT_MISS，或复位 /
//   时钟回退时的显式 SHOT_ABORTED（reason=episode_reset|clock_regression）。
//
// 模型边界（如实，不得外推）：网的弹道在本节点内逐帧积分（非 GZ 物理刚体）；
//   「与 GZ 交叉核验」指逐帧与 GZ 目标/车真值位姿比对（目标运动来自 GZ 真值旁路），
//   非把网做成 GZ 实体。applicable_* 高度与网参数仍为可标定暂定值；载荷物理启用后须重标。
//   替换占位后任务难度先冻结（F-11）：命中结论一 episode 锁存；参数逐字沿用。
//
// 可插拔接口（R9）：本节点只依赖 CaptureAdjudicator 抽象；后续补真实弹道（含网体 GZ 实体）
//   仅换实现，调用方（task_manager，订 /agc/task/capture_event 配套事件）不改。
//
// I/O 契约：
//   输入：/agc/ground/odom（车世界位姿，含 z）、/agc/targets（目标世界位姿 + twist，含 z）
//         /agc/task/state（AgcTaskState，任务窗口门控）、/agc/task/episode（episode 身份；ADR-0078）
//   输出：/agc/task/capture_hit（std_msgs/String，命中值恰为 CAPTURE_NET，一次性 latched）
//         /agc/task/capture_event（std_msgs/String，配套事件，
//           CAPTURE_NET ep=<task episode id> t=<ros_sec> target=<id> shot=<id>；非命令者）
//         /agc/task/envelope_in（std_msgs/Bool，可交战：目标是否在火控交战门内）
//         /agc/task/envelope_jam（std_msgs/Bool，独立干扰作用包络，与网弹道不同源）
//         /agc/task/fire_control（std_msgs/String，火控事件：
//           SHOT_FIRED|SHOT_HIT|SHOT_MISS|SHOT_ABORTED shot=<id> …；中止事件附加 reason=<...> t=<sim>）
//         /agc/task/capture_evidence（std_msgs/String，标注：命中/失手/越界/不可达，限频跳变）
//   参数（launch/config 单一注入，节点不得覆写；契约 #params）：net_v0_mps / net_angle_deg / net_g /
//         net_diameter_m / net_drag / launcher_height_m / net_t_exp_s / fire_cooldown_s /
//         net_flight_timeout_s / model_id / applicable_min_alt_m / applicable_max_alt_m / jam_* + 话题名。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <agc_msgs/msg/agc_task_state.hpp>
#include <agc_msgs/msg/target_track_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "agc_task/dynamic_fire_control.hpp"

using AgcTaskState = agc_msgs::msg::AgcTaskState;
using agc_task::CaptureAdjudicator;
using agc_task::CaptureDecision;
using agc_task::CaptureStatus;
using agc_task::DynamicFireControlAdjudicator;
using agc_task::FireParams;
using agc_task::Pose2;

class CaptureAdjudicatorNode : public rclcpp::Node
{
public:
  CaptureAdjudicatorNode()
  : Node("capture_adjudicator")
  {
    FireParams p;
    p.v0 = declare_parameter<double>("net_v0_mps", p.v0);
    const double angle_deg = declare_parameter<double>("net_angle_deg", 75.0);
    p.angle_rad = angle_deg * M_PI / 180.0;
    p.g = declare_parameter<double>("net_g", p.g);
    p.diameter = declare_parameter<double>("net_diameter_m", p.diameter);
    p.drag = declare_parameter<double>("net_drag", p.drag);
    p.launcher_h = declare_parameter<double>("launcher_height_m", p.launcher_h);
    p.t_exp_s = declare_parameter<double>("net_t_exp_s", p.t_exp_s);
    p.cooldown_s = declare_parameter<double>("fire_cooldown_s", p.cooldown_s);
    p.flight_timeout_s = declare_parameter<double>("net_flight_timeout_s", p.flight_timeout_s);
    // ADR-0079（B2）：目标运动估计参数（单一注入；节点不得覆写）
    p.target_filter.fit_window_s =
      declare_parameter<double>("target_fit_window_s", p.target_filter.fit_window_s);
    p.target_filter.min_fit_s =
      declare_parameter<double>("target_fit_min_s", p.target_filter.min_fit_s);
    p.target_filter.min_samples = static_cast<size_t>(
      declare_parameter<int>("target_fit_min_samples", static_cast<int>(p.target_filter.min_samples)));
    p.target_filter.static_speed_eps =
      declare_parameter<double>("target_static_speed_eps_mps", p.target_filter.static_speed_eps);
    p.target_filter.gate_m = declare_parameter<double>("target_gate_m", p.target_filter.gate_m);
    p.target_filter.pos_alpha = declare_parameter<double>("target_pos_alpha", p.target_filter.pos_alpha);
    p.target_filter.max_lead_m = declare_parameter<double>("target_max_lead_m", p.target_filter.max_lead_m);
    model_id_ = declare_parameter<std::string>("model_id", "net_ballistic_frame_v2");
    p.applicable_min = declare_parameter<double>("applicable_min_alt_m", 0.0);
    p.applicable_max = declare_parameter<double>("applicable_max_alt_m", 2.009);
    applicableMin_ = p.applicable_min;
    applicableMax_ = p.applicable_max;
    jam_model_id_ = declare_parameter<std::string>("jam_model_id", "behavioral_range_gate");
    jam_min_ = declare_parameter<double>("jam_envelope_min_alt_m", 0.0);
    jam_max_ = declare_parameter<double>("jam_envelope_max_alt_m", 2.0);
    jam_max_range_ = declare_parameter<double>("jam_envelope_max_range_m", 1.0);

    car_max_age_s_ = declare_parameter<double>("car_max_age_s", 0.5);
    target_max_age_s_ = declare_parameter<double>("target_max_age_s", 0.5);
    confidence_min_ = declare_parameter<double>("confidence_min", 0.5);
    required_target_id_ = declare_parameter<std::string>("required_target_id", "");

    rate_hz_ = declare_parameter<double>("rate_hz", 20.0);
    // W7-b（T2/E2）：射击授权（任务决策，非自动）——缺省 false（保持跟踪类场景结局不变）。
    interception_authorized_ = declare_parameter<bool>("interception_authorized", false);
    const auto odom_topic = declare_parameter<std::string>("ground_odom_topic", "/agc/ground/odom");
    const auto target_topic = declare_parameter<std::string>("target_topic", "/agc/targets");
    const auto hit_topic = declare_parameter<std::string>("hit_topic", "/agc/task/capture_hit");
    const auto env_topic = declare_parameter<std::string>("envelope_topic", "/agc/task/envelope_in");
    const auto evidence_topic =
      declare_parameter<std::string>("capture_evidence_topic", "/agc/task/capture_evidence");
    const auto jam_env_topic =
      declare_parameter<std::string>("jam_envelope_topic", "/agc/task/envelope_jam");
    const auto task_state_topic =
      declare_parameter<std::string>("task_state_topic", "/agc/task/state");
    // ADR-0078：episode 身份**单一来源** = `task_manager` 发布的 `/agc/task/episode`（而非物理复位 epoch）。
    //   启动期 `scenario_reset` 的 initial gated reset 与 start 的 reset 存在竞态；若本节点沿用
    //   `/agc/scenario/reset_epoch`，事件 ep 会比 task_manager 锁存的 episode 身份大 1 → 全部被拒收。
    const auto episode_topic =
      declare_parameter<std::string>("episode_topic", "/agc/task/episode");
    const auto event_topic =
      declare_parameter<std::string>("capture_event_topic", "/agc/task/capture_event");
    const auto fire_ctl_topic =
      declare_parameter<std::string>("fire_control_topic", "/agc/task/fire_control");

    adjudicator_ = std::make_unique<DynamicFireControlAdjudicator>(p);
    hit_pub_ = create_publisher<std_msgs::msg::String>(
      hit_topic, rclcpp::QoS(1).transient_local());
    event_pub_ = create_publisher<std_msgs::msg::String>(
      event_topic, rclcpp::QoS(1).transient_local());
    env_pub_ = create_publisher<std_msgs::msg::Bool>(env_topic, 10);
    evidence_pub_ = create_publisher<std_msgs::msg::String>(evidence_topic, 10);
    jam_env_pub_ = create_publisher<std_msgs::msg::Bool>(jam_env_topic, 10);
    fire_ctl_pub_ = create_publisher<std_msgs::msg::String>(fire_ctl_topic, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        car_.x = m->pose.pose.position.x; car_.y = m->pose.pose.position.y;
        car_.z = m->pose.pose.position.z; car_.valid = true;
        car_rx_ = now();
      });
    target_sub_ = create_subscription<agc_msgs::msg::TargetTrackArray>(
      target_topic, 10, [this](agc_msgs::msg::TargetTrackArray::SharedPtr m) {
        if (m->tracks.empty()) {target_.valid = false; return;}
        const auto & t = m->tracks.front();
        const auto & pos = t.pose.pose.position;
        // ADR-0079：速度不再在回调里用逐帧差分（噪声被 1/dt 放大）；改由 TargetMotionFilter
        //   在控制拍上做时间窗回归，并按静止阈值置零。
        target_.x = pos.x; target_.y = pos.y; target_.z = pos.z;
        target_.valid = true;
        target_conf_ = t.confidence;
        target_id_ = t.target_id;
        target_rx_ = now();
        // W7-b T3：观测年龄 / 航迹选择一致性诊断（只读，不改选择规则）。
        target_stamp_s_ = rclcpp::Time(t.header.stamp).seconds();
        target_stamp_valid_ = true;
        n_tracks_ = m->tracks.size();
        size_t max_i = 0;
        for (size_t i = 1; i < m->tracks.size(); ++i) {
          if (m->tracks[i].confidence > m->tracks[max_i].confidence) {max_i = i;}
        }
        front_is_max_conf_ = (max_i == 0);
        max_conf_target_id_ = m->tracks[max_i].target_id;
      });
    task_state_sub_ = create_subscription<AgcTaskState>(
      task_state_topic, 10,
      [this](AgcTaskState::SharedPtr m) {task_state_ = m->state;});
    episode_sub_ = create_subscription<std_msgs::msg::Int32>(
      episode_topic, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Int32::SharedPtr m) {onEpisodeId(m->data);});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1e-3)),
      [this]() {tick();});
    RCLCPP_INFO(get_logger(),
      "capture_adjudicator（最小动态火控）就绪：model_id=%s v0=%.2f θ=%.1f° d=%.2f drag=%.2f "
      "适用高度=[%.3f, %.3f] m t_exp=%.2f s cooldown=%.2f s flight_timeout=%.2f s；"
      "独立干扰包络模型=%s 高度=[%.3f, %.3f] m 水平≤%.3f m",
      model_id_.c_str(), p.v0, angle_deg, p.diameter, p.drag,
      p.applicable_min, p.applicable_max, p.t_exp_s, p.cooldown_s, p.flight_timeout_s,
      jam_model_id_.c_str(), jam_min_, jam_max_, jam_max_range_);
  }

private:
  void tick()
  {
    const auto t = now();
    const double t_s = t.seconds();
    const bool car_fresh = car_.valid && (t - car_rx_).seconds() <= car_max_age_s_;
    const bool target_fresh =
      target_.valid && (t - target_rx_).seconds() <= target_max_age_s_ &&
      target_conf_ >= confidence_min_;

    // ADR-0091：新鲜度 / 置信度编码为位姿有效性，**不**提前 return —— 在飞网每拍都要推进。
    Pose2 car = car_;
    car.valid = car_fresh;
    Pose2 target = target_;
    target.valid = target_fresh;

    const bool window_open = (task_state_ == AgcTaskState::GROUND_APPROACH);
    const bool identity_ok =
      required_target_id_.empty() || (target_id_ == required_target_id_);
    const bool allow_fire =
      car_fresh && target_fresh && window_open && identity_ok && interception_authorized_ &&
      have_epoch_;

    const CaptureDecision d = adjudicator_->step(car, target, t_s, allow_fire);

    // 输入失效时不发布 envelope（保持既有 FCO 口径）；在飞推进 / 终局仍由下方 switch 处理。
    if (car_fresh && target_fresh) {
      std_msgs::msg::Bool eb;
      eb.data = d.engaged;
      env_pub_->publish(eb);

      const double hj = target.z - car.z;
      const double jdist = std::hypot(target.x - car.x, target.y - car.y);
      std_msgs::msg::Bool jb;
      jb.data = (hj >= jam_min_ && hj <= jam_max_ && jdist <= jam_max_range_);
      jam_env_pub_->publish(jb);
    }

    switch (d.status) {
      case CaptureStatus::kShotFired:
        publishFireControl("SHOT_FIRED", d);
        publishEvidence(evidenceString("SHOT_FIRED", d.shot_id, d));
        RCLCPP_INFO(get_logger(),
          "SHOT_FIRED shot=%s t=%.3f h=%.3f dist=%.3f pred=(%.2f,%.2f,%.2f) "
          "vel=(%.2f,%.2f,%.2f) est=(%.2f,%.2f,%.2f) t_est=%.3f target_id=%s conf=%.3f age=%.3f "
          "ntracks=%zu front_is_maxconf=%d maxconf_target_id=%s model_id=%s epoch=%d",
          d.shot_id.c_str(), d.t_fire, d.h, d.dist, d.predicted.x, d.predicted.y, d.predicted.z,
          d.target_vel.x, d.target_vel.y, d.target_vel.z,
          d.est.x, d.est.y, d.est.z, d.t_est,
          target_id_.c_str(), static_cast<double>(target_conf_), targetAgeS(t),
          n_tracks_, front_is_max_conf_ ? 1 : 0, max_conf_target_id_.c_str(),
          model_id_.c_str(), current_epoch_);
        break;
      case CaptureStatus::kHit:
        publishFireControl("SHOT_HIT", d);
        if (allow_fire && !fired_) {
          fired_ = true;
          std_msgs::msg::String hit;
          hit.data = "CAPTURE_NET";    // 契约 #terminal-semantics：值恰为 CAPTURE_NET
          hit_pub_->publish(hit);
          publishEvent(t, d.shot_id);
        }
        publishEvidence(evidenceString("CAPTURE_HIT", "CAPTURE_NET", d));
        RCLCPP_WARN(get_logger(),
          "CAPTURE（发射式网动态命中）：shot=%s model_id=%s h=%.3f dist=%.3f flight=%.3f s "
          "miss_dist=%.3f m epoch=%d",
          d.shot_id.c_str(), model_id_.c_str(), d.h, d.dist, d.flight_s, d.miss_dist, current_epoch_);
        break;
      case CaptureStatus::kMiss:
        publishFireControl("SHOT_MISS", d);
        publishEvidence(evidenceString("SHOT_MISS", d.shot_id, d));
        RCLCPP_WARN(get_logger(),
          "SHOT_MISS shot=%s model_id=%s h=%.3f dist=%.3f flight=%.3f s miss_dist=%.3f m epoch=%d",
          d.shot_id.c_str(), model_id_.c_str(), d.h, d.dist, d.flight_s, d.miss_dist, current_epoch_);
        break;
      case CaptureStatus::kAborted:
        // ADR-0091：显式中止（episode_reset / clock_regression）——不发布 capture_hit、不写 CAPTURE_NET。
        publishFireControl("SHOT_ABORTED", d);
        publishEvidence(evidenceString("SHOT_ABORTED", d.shot_id, d));
        RCLCPP_WARN(get_logger(),
          "SHOT_ABORTED shot=%s reason=%s t=%.3f flight=%.3f s miss_dist=%.3f m model_id=%s epoch=%d",
          d.shot_id.c_str(), d.reason.c_str(), d.t_event, d.flight_s, d.miss_dist,
          model_id_.c_str(), current_epoch_);
        break;
      default:
        break;
    }

    // 门标注只在两路输入均有效的拍产生（与既有 FCO 口径一致）。
    if (car_fresh && target_fresh) {
      const std::string label = skipLabel(d.status);
      if (!label.empty() && label != last_skip_label_) {
        last_skip_label_ = label;
        publishEvidence(evidenceString("CAPTURE_SKIP", label, d));
      } else if (label.empty()) {
        last_skip_label_.clear();
      }
    }
  }

  // episode 身份变化 = 新 episode → 复位火控（清冷却 / 在飞）与一次性命中标志。
  // ADR-0091：复位前若仍有在飞网，先发一条 SHOT_ABORTED reason=episode_reset（唯一结局）。
  void onEpisodeId(int e)
  {
    if (!have_epoch_ || e != current_epoch_) {
      if (adjudicator_->inFlight()) {
        const auto t = now();
        const CaptureDecision ab = adjudicator_->abortInFlight("episode_reset", t.seconds());
        publishFireControl("SHOT_ABORTED", ab);
        publishEvidence(evidenceString("SHOT_ABORTED", ab.shot_id, ab));
      }
      current_epoch_ = e;
      have_epoch_ = true;
      fired_ = false;
      last_skip_label_.clear();
      adjudicator_->reset(e);
    }
  }

  // 配套事件串 `CAPTURE_NET ep=<epoch> t=<ros_sec> target=<id> shot=<shot_id>`（可解析；非命令者）。
  // 末位追加 `shot=` 不破坏 task_manager 的 parseEp（stoi 止于非数字）与 firstToken。
  void publishEvent(const rclcpp::Time & t, const std::string & shot_id)
  {
    std::string text = "CAPTURE_NET ep=" + std::to_string(current_epoch_) +
                       " t=" + fmt(t.seconds()) + " target=" + target_id_ +
                       " shot=" + shot_id;
    std_msgs::msg::String m;
    m.data = text;
    event_pub_->publish(m);
  }

  void publishFireControl(const std::string & kind, const CaptureDecision & d)
  {
    std::string text = kind + " shot=" + d.shot_id +
                       " t_fire=" + fmt(d.t_fire) +
                       " flight_s=" + fmt(d.flight_s) +
                       " miss_dist_m=" + fmt(d.miss_dist) +
                       " h=" + fmt(d.h) + " dist=" + fmt(d.dist);
    // ADR-0091：中止事件附加 reason / 仿真时刻（追加 token，不破坏既有解析）。
    if (!d.reason.empty()) {
      text += " reason=" + d.reason + " t=" + fmt(d.t_event);
    }
    std_msgs::msg::String m;
    m.data = text;
    fire_ctl_pub_->publish(m);
  }

  std::string evidenceString(
    const char * kind, const std::string & value, const CaptureDecision & d) const
  {
    std::string text = kind + std::string(" ") + value +
                       " model_id=" + model_id_ +
                       " applicable_min_alt_m=" + fmt(applicableMin_) +
                       " applicable_max_alt_m=" + fmt(applicableMax_) +
                       " h=" + fmt(d.h) + " dist=" + fmt(d.dist) +
                       " shot=" + d.shot_id +
                       " flight_s=" + fmt(d.flight_s) +
                       " miss_dist_m=" + fmt(d.miss_dist);
    if (!d.reason.empty()) {
      text += " reason=" + d.reason;
    }
    return text;
  }

  void publishEvidence(const std::string & text)
  {
    std_msgs::msg::String m;
    m.data = text;
    evidence_pub_->publish(m);
  }

  std::string skipLabel(CaptureStatus s) const
  {
    switch (s) {
      case CaptureStatus::kOutOfApplicableRange: return "out_of_applicable_range";
      case CaptureStatus::kUnreachable: return "unreachable";
      default: return "";
    }
  }

  // W7-b T3：目标观测年龄 now − header.stamp（s；不可得时 -1）。
  double targetAgeS(const rclcpp::Time & t) const
  {
    return target_stamp_valid_ ? (t.seconds() - target_stamp_s_) : -1.0;
  }

  static std::string fmt(double v)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.4f", v);
    return std::string(buf);
  }

  double rate_hz_{20.0};
  bool fired_{false};
  bool interception_authorized_{false};
  std::string last_skip_label_;
  std::string model_id_{"net_ballistic_frame_v2"};
  std::string jam_model_id_{"behavioral_range_gate"};
  double applicableMin_{0.0}, applicableMax_{2.009};
  double jam_min_{0.0}, jam_max_{2.0}, jam_max_range_{1.0};
  double car_max_age_s_{0.5}, target_max_age_s_{0.5}, confidence_min_{0.5};
  std::string required_target_id_{};
  uint8_t task_state_{0};
  int current_epoch_{-1};
  bool have_epoch_{false};
  float target_conf_{0.0f};
  std::string target_id_{};
  // W7-b T3：航迹选择一致性 / 观测年龄（只读诊断，不参与发射判定）。
  double target_stamp_s_{0.0};
  bool target_stamp_valid_{false};
  size_t n_tracks_{0};
  bool front_is_max_conf_{true};
  std::string max_conf_target_id_{};
  Pose2 car_, target_;
  rclcpp::Time car_rx_{0, 0, RCL_ROS_TIME}, target_rx_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<CaptureAdjudicator> adjudicator_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hit_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr env_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr evidence_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr jam_env_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fire_ctl_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<agc_msgs::msg::TargetTrackArray>::SharedPtr target_sub_;
  rclcpp::Subscription<AgcTaskState>::SharedPtr task_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr episode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaptureAdjudicatorNode>());
  rclcpp::shutdown();
  return 0;
}
