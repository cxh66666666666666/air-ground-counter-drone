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
//         /agc/task/fire_control（std_msgs/String，火控事件：SHOT_FIRED|SHOT_HIT|SHOT_MISS shot=<id> …）
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

#include "agc_task/target_motion_filter.hpp"

using AgcTaskState = agc_msgs::msg::AgcTaskState;

namespace
{
struct Pose2
{
  double x{0.0}, y{0.0}, z{0.0};
  bool valid{false};
};

struct Vec3
{
  double x{0.0}, y{0.0}, z{0.0};
};

struct FireParams
{
  double v0{6.5};           // 发射初速 m/s
  double angle_rad{1.309};  // 抛射角 75°
  double g{9.81};           // 重力
  double diameter{1.0};     // 展开后有效直径 m
  double drag{0.9};         // 水平速度折算因子（对可及集射程）
  double launcher_h{0.0};   // 发射器相对车体基准高度 m
  double applicable_min{0.0};
  double applicable_max{2.009};
  double t_exp_s{0.2};           // 网展开时间 s（此期间有效半径线性增长）
  double cooldown_s{2.0};        // 射击后冷却 s
  double flight_timeout_s{3.0};  // 网飞行总时长上限 s（超时＝失手）
  // ADR-0079（B2）：目标运动估计参数（时间窗回归 + 静止阈值 + 新息门 + 提前量上限）
  agc_task::TargetMotionFilter::Params target_filter{};
};

// 裁决结果：命中 + 交战门 + 火控事件（W7-b T2 扩展）。
enum class CaptureStatus
{
  kHit,                    // 本拍判定命中
  kShotFired,              // 本拍发射（新 shot）
  kInFlight,               // 网在飞（未决）
  kMiss,                   // 网落地 / 超时 → 失手
  kCooldown,               // 冷却中 / 不可发射
  kOutOfApplicableRange,   // h 超出适用高度 → 不判命中并标注
  kUnreachable,            // 该高度不可达 → 不判命中并标注
  kOutOfEnvelope,          // 适用高度内但水平超出可交战范围（正常未入包络）
  kInvalid                 // 车 / 目标位姿缺失（不裁决、不标注）
};

struct CaptureDecision
{
  CaptureStatus status{CaptureStatus::kInvalid};
  bool hit{false};
  bool engaged{false};
  double dist{0.0};
  double h{0.0};
  double t_fire{0.0};
  double flight_s{0.0};       // 命中 / 失手时刻的飞行时长
  double miss_dist{0.0};      // 最近逼近距离
  Vec3 predicted{0.0, 0.0, 0.0};
  Vec3 target_vel{0.0, 0.0, 0.0};   // ADR-0079：发射时的滤波后目标速度（证据）
  Vec3 est{0.0, 0.0, 0.0};          // W7-b T3：发射时的滤波后目标位置（诊断）
  double t_est{0.0};                // W7-b T3：predictTarget 迭代后用于瞄准的到达时间（诊断）
  std::string shot_id;
};

// 可插拔裁决器接口（R9）：现为最小动态火控（逐帧弹道），后续可换真实弹道（含 GZ 网体）。
class CaptureAdjudicator
{
public:
  virtual ~CaptureAdjudicator() = default;
  virtual void reset(int epoch) = 0;
  virtual CaptureDecision step(
    const Pose2 & car, const Pose2 & target,
    double t_s, bool allow_fire) = 0;
};
}  // namespace

// 最小动态火控裁决器：交战门 + 射击时刻 + 目标预测 + 逐帧弹道 + 命中/失手 + 冷却。
class DynamicFireControlAdjudicator : public CaptureAdjudicator
{
public:
  explicit DynamicFireControlAdjudicator(const FireParams & p) : p_(p) {target_filter_.setParams(p_.target_filter);}

  void reset(int epoch) override
  {
    epoch_ = epoch;
    in_flight_ = false;
    have_shot_ = false;
    cooldown_until_ = -1.0;
    shot_seq_ = 0;
    target_filter_.reset();
  }

  CaptureDecision step(
    const Pose2 & car, const Pose2 & target,
    double t_s, bool allow_fire) override
  {
    CaptureDecision d;
    if (!car.valid || !target.valid) {return d;}   // kInvalid

    // ADR-0079：先更新目标运动估计，再用**滤波后**的位置做交战门 / 瞄准（不用单帧原始噪声）。
    target_filter_.update(t_s, agc_task::Vec3d{target.x, target.y, target.z});
    const agc_task::Vec3d tf = target_filter_.position();

    const double dx = tf.x - car.x, dy = tf.y - car.y;
    d.dist = std::hypot(dx, dy);
    // R-020：高度含车体——h = target.z − car.z − launcher_h。
    d.h = tf.z - car.z - p_.launcher_h;

    // ① 交战门：适用高度 → 可达 → 水平射程
    if (d.h < 0.0 || d.h < p_.applicable_min || d.h > p_.applicable_max) {
      d.status = CaptureStatus::kOutOfApplicableRange;
      return d;
    }
    const double vy0 = p_.v0 * std::sin(p_.angle_rad);
    const double disc = vy0 * vy0 - 2.0 * p_.g * d.h;
    if (disc < 0.0) {
      d.status = CaptureStatus::kUnreachable;
      return d;
    }
    const double t_desc = (vy0 + std::sqrt(disc)) / p_.g;   // 下降段到达该高度的时间
    const double reach = p_.v0 * std::cos(p_.angle_rad) * t_desc * p_.drag;
    if (d.dist > reach + 0.5 * p_.diameter) {
      d.status = CaptureStatus::kOutOfEnvelope;   // 正常未入包络（不标注）
      return d;
    }
    d.engaged = true;    // 可交战（/agc/task/envelope_in）

    // ④ 网在飞 → 逐帧推进 + 与 GZ 真值目标位姿作球比对
    if (in_flight_) {
      const Vec3 net = netPos(t_s);
      const double d3 = dist3(net, target);
      if (!have_miss_dist_ || d3 < miss_dist_) {miss_dist_ = d3; have_miss_dist_ = true;}
      d.shot_id = shot_id_;
      d.t_fire = t_fire_;
      d.miss_dist = miss_dist_;
      const double r = radius(t_s);
      if (d3 <= r) {
        in_flight_ = false;
        cooldown_until_ = t_s + p_.cooldown_s;
        d.hit = true;
        d.status = CaptureStatus::kHit;
        d.flight_s = t_s - t_fire_;
        return d;
      }
      const bool landed = net.z < (net_z0_ - 0.01);
      const bool timeout = (t_s - t_fire_) > p_.flight_timeout_s;
      if (landed || timeout) {
        in_flight_ = false;
        cooldown_until_ = t_s + p_.cooldown_s;
        d.status = CaptureStatus::kMiss;
        d.flight_s = t_s - t_fire_;
        return d;
      }
      d.status = CaptureStatus::kInFlight;
      return d;
    }

    // ⑤ 冷却 / 不可发射
    if (t_s < cooldown_until_) {
      d.status = CaptureStatus::kCooldown;
      return d;
    }
    if (!allow_fire || !target_filter_.settled()) {
      d.status = CaptureStatus::kCooldown;   // 冷却 / 未授权 / 运动估计未就绪（不发弹）
      return d;
    }

    // ②③ 发射：以目标预测方位角瞄准（ADR-0079：滤波后位置 + 有上限的提前量）
    double t_est = 0.0;
    const Vec3 pred = predictTarget(car, tf, t_est);
    const double pdx = pred.x - car.x, pdy = pred.y - car.y;
    const double pdist = std::hypot(pdx, pdy);
    ux_ = (pdist > 1e-6) ? (pdx / pdist) : 1.0;
    uy_ = (pdist > 1e-6) ? (pdy / pdist) : 0.0;

    ++shot_seq_;
    shot_id_ = std::to_string(epoch_) + "-" + std::to_string(shot_seq_);
    t_fire_ = t_s;
    net_x0_ = car.x;
    net_y0_ = car.y;
    net_z0_ = car.z + p_.launcher_h;
    in_flight_ = true;
    have_shot_ = true;
    miss_dist_ = std::numeric_limits<double>::infinity();
    have_miss_dist_ = false;

    d.status = CaptureStatus::kShotFired;
    d.shot_id = shot_id_;
    d.t_fire = t_s;
    d.predicted = pred;
    d.est = Vec3{tf.x, tf.y, tf.z};
    d.t_est = t_est;
    const agc_task::Vec3d fv = target_filter_.velocity();
    d.target_vel = Vec3{fv.x, fv.y, fv.z};
    d.miss_dist = miss_dist_;
    return d;
  }

  int shotSeq() const {return shot_seq_;}
  bool inFlight() const {return in_flight_;}

private:
  // 网逐帧弹道位置：水平 = v0·cosθ·drag·Δt（常速）；垂直 = v0·sinθ·Δt − ½gΔt²。
  Vec3 netPos(double t) const
  {
    const double dt = t - t_fire_;
    const double vh = p_.v0 * std::cos(p_.angle_rad) * p_.drag;
    Vec3 n;
    n.x = net_x0_ + vh * ux_ * dt;
    n.y = net_y0_ + vh * uy_ * dt;
    n.z = net_z0_ + p_.v0 * std::sin(p_.angle_rad) * dt - 0.5 * p_.g * dt * dt;
    return n;
  }

  // 有效半径：展开时间内线性增长至 d/2。
  double radius(double t) const
  {
    const double dt = t - t_fire_;
    const double r_full = 0.5 * p_.diameter;
    if (dt <= 0.0) {return 0.0;}
    if (dt < p_.t_exp_s) {return r_full * (dt / p_.t_exp_s);}
    return r_full;
  }

  // 目标预测：按预测到达时间（水平射程 / 网水平速度）迭代求交会点，再乘滤波后速度。
  //   ADR-0079：不使用逐帧位置差分；静止目标的提前量为 0，移动目标提前量有上限。
  Vec3 predictTarget(const Pose2 & car, const agc_task::Vec3d & tp, double & t_est_out) const
  {
    const double vh = std::max(p_.v0 * std::cos(p_.angle_rad) * p_.drag, 1e-3);
    double t_est = std::hypot(tp.x - car.x, tp.y - car.y) / vh;
    double t_used = t_est;
    Vec3 pred{tp.x, tp.y, tp.z};
    for (int i = 0; i < 2; ++i) {
      t_est = std::min(t_est, p_.flight_timeout_s);
      t_used = t_est;
      const agc_task::Vec3d q = target_filter_.predicted(t_est);
      pred = Vec3{q.x, q.y, q.z};
      t_est = std::hypot(q.x - car.x, q.y - car.y) / vh;
    }
    t_est_out = t_used;   // 实际用于瞄准的到达时间（诊断；不参与弹道）
    return pred;
  }

  static double dist3(const Vec3 & a, const Pose2 & b)
  {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
  }

  FireParams p_;
  agc_task::TargetMotionFilter target_filter_;
  int epoch_{-1};
  int shot_seq_{0};
  bool in_flight_{false};
  bool have_shot_{false};
  double t_fire_{0.0};
  double net_x0_{0.0}, net_y0_{0.0}, net_z0_{0.0}, ux_{1.0}, uy_{0.0};
  double cooldown_until_{-1.0};
  double miss_dist_{0.0};
  bool have_miss_dist_{false};
  std::string shot_id_;
};

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
    model_id_ = declare_parameter<std::string>("model_id", "net_ballistic_frame_v1");
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
    if (!car_fresh || !target_fresh) {return;}

    const bool window_open = (task_state_ == AgcTaskState::GROUND_APPROACH);
    const bool identity_ok =
      required_target_id_.empty() || (target_id_ == required_target_id_);
    const bool allow_fire = window_open && identity_ok && interception_authorized_ && have_epoch_;

    const CaptureDecision d = adjudicator_->step(car_, target_, t_s, allow_fire);

    std_msgs::msg::Bool eb;
    eb.data = d.engaged;
    env_pub_->publish(eb);

    const double hj = target_.z - car_.z;
    const double jdist = std::hypot(target_.x - car_.x, target_.y - car_.y);
    std_msgs::msg::Bool jb;
    jb.data = (hj >= jam_min_ && hj <= jam_max_ && jdist <= jam_max_range_);
    jam_env_pub_->publish(jb);

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
      default:
        break;
    }

    const std::string label = skipLabel(d.status);
    if (!label.empty() && label != last_skip_label_) {
      last_skip_label_ = label;
      publishEvidence(evidenceString("CAPTURE_SKIP", label, d));
    } else if (label.empty()) {
      last_skip_label_.clear();
    }
  }

  // episode 身份变化 = 新 episode → 复位火控（清冷却 / 在飞）与一次性命中标志。
  void onEpisodeId(int e)
  {
    if (!have_epoch_ || e != current_epoch_) {
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
    std_msgs::msg::String m;
    m.data = text;
    fire_ctl_pub_->publish(m);
  }

  std::string evidenceString(
    const char * kind, const std::string & value, const CaptureDecision & d) const
  {
    return kind + std::string(" ") + value +
           " model_id=" + model_id_ +
           " applicable_min_alt_m=" + fmt(applicableMin_) +
           " applicable_max_alt_m=" + fmt(applicableMax_) +
           " h=" + fmt(d.h) + " dist=" + fmt(d.dist) +
           " shot=" + d.shot_id +
           " flight_s=" + fmt(d.flight_s) +
           " miss_dist_m=" + fmt(d.miss_dist);
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
  std::string model_id_{"net_ballistic_frame_v1"};
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
