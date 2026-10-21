// target_motion_filter.hpp —— 火控目标运动估计（B2 修复；参考 PX4 TargetEstimator / 社区 α-β / 滑动平均）
//
// 背景（ADR-0079）：
//   旧实现把 `twist≈0`（静止目标）当成“无速度”，退化为逐帧位置差分；20 Hz 测量噪声被 1/dt 放大成
//   1–2 m/s 的假速度 → 预测点偏离 1–2.5 m → 网（半径 0.5 m）连续打空。
//
// 本类只做**运动估计**，不参与命中判定：
//   * 位置：一阶低通（α），抑制单帧噪声；
//   * 速度：对最近 fit_window_s 的位置样本做最小二乘直线拟合（滑动平均 / α-β 的稳态形式），
//     低于 static_speed_eps 一律视为静止并置零（对齐 PX4 MINIMUM_SPEED_FOR_TARGET_MOVING=0.1）；
//   * 新息门：位置跳变 > gate_m 视为异常（换目标 / 复位 / 鬼影）→ 重置滤波，不外推；
//   * 预测：p + clamp(v·t_lead, max_lead_m)，提前量有上限，避免错误速度把瞄准点甩出网半径。
//
// 纯标准库实现（可单测；不依赖 ROS / Eigen）。

#pragma once

#include <cmath>
#include <deque>

namespace agc_task
{

struct Vec3d
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline double norm3(const Vec3d & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

class TargetMotionFilter
{
public:
  struct Params
  {
    double fit_window_s{1.5};        // 速度回归窗口（s）
    double min_fit_s{0.75};          // 允许“解算就绪”的最短时间跨度（s）
    size_t min_samples{8};           // 回归所需最少样本数
    double static_speed_eps{0.15};   // 低于此速度视为静止（m/s；PX4 先例 0.1）
    double gate_m{1.0};              // 位置新息门（m）
    double pos_alpha{0.5};           // 位置一阶低通系数
    double max_lead_m{1.5};          // 提前量上限（m）
    double max_speed_mps{10.0};      // 速度硬上限（m/s）
    double reset_gap_s{2.0};         // 测量间隔超此值 → 重新初始化
  };

  TargetMotionFilter() = default;
  explicit TargetMotionFilter(const Params & p) : p_(p) {}

  void setParams(const Params & p) {p_ = p;}
  const Params & params() const {return p_;}

  void reset()
  {
    buf_.clear();
    pos_ = Vec3d{};
    vel_ = Vec3d{};
    initialized_ = false;
    settled_ = false;
    have_last_ = false;
    last_t_ = 0.0;
  }

  // 输入一帧目标位置测量（world 系，m）。
  void update(double t_s, const Vec3d & meas)
  {
    if (!initialized_ || (have_last_ && (t_s - last_t_) > p_.reset_gap_s)) {
      buf_.clear();
      pos_ = meas;
      vel_ = Vec3d{};
      initialized_ = true;
      settled_ = false;
      have_last_ = true;
      last_t_ = t_s;
      buf_.push_back({t_s, meas});
      return;
    }

    // 新息门：与“上一状态按当前速度外推”比较；异常跳变直接重置，避免把鬼影融进速度。
    const double dt = std::max(1e-3, t_s - last_t_);
    const Vec3d pred{pos_.x + vel_.x * dt, pos_.y + vel_.y * dt, pos_.z + vel_.z * dt};
    const Vec3d innov{meas.x - pred.x, meas.y - pred.y, meas.z - pred.z};
    if (norm3(innov) > p_.gate_m) {
      reset();
      return update(t_s, meas);   // 以新位置重新起步（不再外推旧状态）
    }

    // 位置低通（速度估计仍用原始测量，避免低通额外相位滞后）
    pos_.x += p_.pos_alpha * (meas.x - pos_.x);
    pos_.y += p_.pos_alpha * (meas.y - pos_.y);
    pos_.z += p_.pos_alpha * (meas.z - pos_.z);

    buf_.push_back({t_s, meas});
    while (!buf_.empty() && (t_s - buf_.front().t) > p_.fit_window_s) {buf_.pop_front();}

    last_t_ = t_s;
    have_last_ = true;

    if (buf_.size() < p_.min_samples) {return;}
    const double span = buf_.back().t - buf_.front().t;
    if (span < p_.min_fit_s) {return;}

    // 最小二乘斜率（以窗口首样本时刻为原点，改善数值条件）。
    const double t0 = buf_.front().t;
    double st = 0.0, st2 = 0.0;
    double sx = 0.0, sy = 0.0, sz = 0.0;
    double stx = 0.0, sty = 0.0, stz = 0.0;
    for (const auto & s : buf_) {
      const double tt = s.t - t0;
      st += tt;
      st2 += tt * tt;
      sx += s.p.x;
      sy += s.p.y;
      sz += s.p.z;
      stx += tt * s.p.x;
      sty += tt * s.p.y;
      stz += tt * s.p.z;
    }
    const double n = static_cast<double>(buf_.size());
    const double den = n * st2 - st * st;
    if (std::fabs(den) < 1e-9) {return;}
    Vec3d v{
      (n * stx - st * sx) / den,
      (n * sty - st * sy) / den,
      (n * stz - st * sz) / den};

    double speed = norm3(v);
    if (speed > p_.max_speed_mps) {
      const double k = p_.max_speed_mps / speed;
      v.x *= k; v.y *= k; v.z *= k;
      speed = p_.max_speed_mps;
    }
    if (speed < p_.static_speed_eps) {v = Vec3d{};}   // 静止目标：不引入任何假速度

    vel_ = v;
    settled_ = true;
  }

  bool initialized() const {return initialized_;}
  bool settled() const {return settled_;}
  Vec3d position() const {return pos_;}
  Vec3d velocity() const {return vel_;}
  double lastUpdateS() const {return last_t_;}

  // 提前量有上限：即使速度估计短时异常，也不会把瞄准点甩出网半径。
  Vec3d predicted(double t_lead_s) const
  {
    Vec3d lead{vel_.x * t_lead_s, vel_.y * t_lead_s, vel_.z * t_lead_s};
    const double ln = norm3(lead);
    if (ln > p_.max_lead_m) {
      const double k = p_.max_lead_m / ln;
      lead.x *= k; lead.y *= k; lead.z *= k;
    }
    return Vec3d{pos_.x + lead.x, pos_.y + lead.y, pos_.z + lead.z};
  }

private:
  struct Sample
  {
    double t;
    Vec3d p;
  };

  Params p_{};
  std::deque<Sample> buf_{};
  Vec3d pos_{};
  Vec3d vel_{};
  bool initialized_{false};
  bool settled_{false};
  bool have_last_{false};
  double last_t_{0.0};
};

}  // namespace agc_task
