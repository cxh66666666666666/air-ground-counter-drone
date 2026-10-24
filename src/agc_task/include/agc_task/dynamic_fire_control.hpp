// dynamic_fire_control.hpp —— 最小动态火控状态机（W7-b T2/E2；ADR-0079；ADR-0091）
//
// 纯标准库实现（可单测；不依赖 ROS / GZ / PX4）。节点 capture_adjudicator.cpp 只做 I/O 与事件发布，
// 状态机与弹道在本头文件内，便于确定性 gtest 以显式 t_s 驱动。
//
// 生命周期（ADR-0091）：已发射网的在飞推进 / 落地 / 超时结算与发射资格解耦；
//   目标无效时不判命中但继续结算；复位 / 时钟回退以 SHOT_ABORTED 显式中止。

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "agc_task/target_motion_filter.hpp"

namespace agc_task
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
  TargetMotionFilter::Params target_filter{};
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
  kInvalid,                // 车 / 目标位姿缺失（不裁决、不标注）
  kAborted                 // 在飞网被显式中止（episode 复位 / 时钟回退）
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
  std::string reason;           // ADR-0091：中止原因（episode_reset | clock_regression）
  double t_event{0.0};          // ADR-0091：中止发生的仿真时刻
  bool miss_dist_valid{false};  // ADR-0091：在飞期间是否获得过有效目标观测
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
  // ADR-0091：显式中止在飞网（缺省无在飞状态 → 返回默认决策）。
  virtual CaptureDecision abortInFlight(const std::string & /*reason*/, double /*t_s*/)
  {
    return CaptureDecision{};
  }
  virtual bool inFlight() const {return false;}
};

// 最小动态火控裁决器：交战门 + 射击时刻 + 目标预测 + 逐帧弹道 + 命中/失手 + 冷却。
class DynamicFireControlAdjudicator : public CaptureAdjudicator
{
public:
  explicit DynamicFireControlAdjudicator(const FireParams & p) : p_(p)
  {
    target_filter_.setParams(p_.target_filter);
  }

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

    // ADR-0091：只有有效目标进入运动估计；失效拍不得用缓存位姿更新估计。
    if (target.valid) {
      target_filter_.update(t_s, agc_task::Vec3d{target.x, target.y, target.z});
    }

    // 交战门只用于 envelope_in 与新发射；**不得**阻断已发射网的推进 / 结算。
    const bool both_valid = car.valid && target.valid;
    if (both_valid) {
      const agc_task::Vec3d tf = target_filter_.position();
      const double dx = tf.x - car.x, dy = tf.y - car.y;
      d.dist = std::hypot(dx, dy);
      // R-020：高度含车体——h = target.z − car.z − launcher_h。
      d.h = tf.z - car.z - p_.launcher_h;
      gateStatus(d);   // ① 适用高度 → 可达 → 水平射程
    }

    // ④ 网在飞 → **每拍**推进弹道时钟并结算（先于一切发射资格检查）。
    if (in_flight_) {
      if (t_s < t_fire_ - 1e-6) {
        return abortInFlight("clock_regression", t_s);
      }
      const Vec3 net = netPos(t_s);
      d.shot_id = shot_id_;
      d.t_fire = t_fire_;
      d.miss_dist = have_miss_dist_ ? miss_dist_ : std::numeric_limits<double>::infinity();
      d.miss_dist_valid = have_miss_dist_;
      // 目标无效时**跳过命中判定**（不用缓存位姿），但落地 / 超时结算继续。
      if (target.valid) {
        const double d3 = dist3(net, target);
        if (!have_miss_dist_ || d3 < miss_dist_) {
          miss_dist_ = d3;
          have_miss_dist_ = true;
        }
        d.miss_dist = miss_dist_;
        d.miss_dist_valid = true;
        const double r = radius(t_s);
        if (d3 <= r) {
          in_flight_ = false;
          cooldown_until_ = t_s + p_.cooldown_s;
          d.hit = true;
          d.status = CaptureStatus::kHit;
          d.flight_s = t_s - t_fire_;
          return d;
        }
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

    // 发射资格：位姿有效性 → 交战门 → 冷却 → 授权 / 运动估计就绪。
    if (!both_valid) {return d;}   // kInvalid（不发射）
    if (d.status != CaptureStatus::kInvalid) {return d;}   // 门失败（不发射）
    // 此时 d.engaged == true（可交战，/agc/task/envelope_in）

    if (t_s < cooldown_until_) {
      d.status = CaptureStatus::kCooldown;
      return d;
    }
    if (!allow_fire || !target_filter_.settled()) {
      d.status = CaptureStatus::kCooldown;   // 冷却 / 未授权 / 运动估计未就绪（不发弹）
      return d;
    }

    // ②③ 发射：以目标预测方位角瞄准（ADR-0079：滤波后位置 + 有上限的提前量）
    const agc_task::Vec3d tf = target_filter_.position();
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

  // ADR-0091：显式中止在飞网（episode_reset / clock_regression）；非在飞时返回默认（kInvalid）。
  CaptureDecision abortInFlight(const std::string & reason, double t_s) override
  {
    CaptureDecision d;
    if (!in_flight_) {return d;}
    d.status = CaptureStatus::kAborted;
    d.shot_id = shot_id_;
    d.t_fire = t_fire_;
    d.flight_s = std::max(0.0, t_s - t_fire_);
    d.miss_dist = have_miss_dist_ ? miss_dist_ : std::numeric_limits<double>::infinity();
    d.miss_dist_valid = have_miss_dist_;
    d.reason = reason;
    d.t_event = t_s;
    in_flight_ = false;
    cooldown_until_ = t_s + p_.cooldown_s;
    return d;
  }

  int shotSeq() const {return shot_seq_;}
  bool inFlight() const override {return in_flight_;}
  const std::string & shotId() const {return shot_id_;}

private:
  // ① 交战门：适用高度 → 可达 → 水平射程（只决定 engaged / 门失败状态，不阻断在飞）。
  void gateStatus(CaptureDecision & d) const
  {
    if (d.h < 0.0 || d.h < p_.applicable_min || d.h > p_.applicable_max) {
      d.status = CaptureStatus::kOutOfApplicableRange;
      return;
    }
    const double vy0 = p_.v0 * std::sin(p_.angle_rad);
    const double disc = vy0 * vy0 - 2.0 * p_.g * d.h;
    if (disc < 0.0) {
      d.status = CaptureStatus::kUnreachable;
      return;
    }
    const double t_desc = (vy0 + std::sqrt(disc)) / p_.g;   // 下降段到达该高度的时间
    const double reach = p_.v0 * std::cos(p_.angle_rad) * t_desc * p_.drag;
    if (d.dist > reach + 0.5 * p_.diameter) {
      d.status = CaptureStatus::kOutOfEnvelope;   // 正常未入包络（不标注）
      return;
    }
    d.engaged = true;    // 可交战（/agc/task/envelope_in）
  }

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

}  // namespace agc_task
