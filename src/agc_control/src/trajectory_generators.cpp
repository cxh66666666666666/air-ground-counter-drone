// agc_control/trajectory_generators.cpp —— 轨迹生成器插件实现
//
// 恒速保证：周期轨迹按弧长参数化——先建 s(θ) 表（梯形积分），再对给定 s 反解 θ，
// 使 |dp/dt| ≡ speed（C1：真八字全程恒速、无加减速）。

#include "agc_control/trajectory_generators.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>


namespace agc_control
{
namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

/// 最小 jerk 位置剖面 s(τ) 及其一、二阶导（τ ∈ [0,1]）
inline double minJerk(double tau)
{
  return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
}
inline double minJerkDot(double tau)
{
  return 30.0 * tau * tau * (1.0 + tau * (-2.0 + tau));
}
inline double minJerkDdot(double tau)
{
  return 60.0 * tau * (1.0 + tau * (-3.0 + 2.0 * tau));
}
}  // namespace

// ---------------------------------------------------------------- 参数曲线基类

void ParametricTrajectory::configure(ParamReader & p)
{
  speed_ = p.getDouble("speed", 2.0);
  height_ = p.getDouble("height", 2.5);
  center_.x() = p.getDouble("center_x", 0.0);
  center_.y() = p.getDouble("center_y", 0.0);
  center_.z() = p.getDouble("center_z", 0.0);
  samples_ = p.getInt("arc_samples", 4096);
  if (samples_ < 16) {samples_ = 16;}
  s_table_.clear();
  length_ = 0.0;
}

void ParametricTrajectory::reset(const State & /*initial*/, double /*yaw*/)
{
  // 周期轨迹固定在世界中（不随起点平移）：t=0 对应 θ=0（C5 相位复位）
  start_offset_.setZero();
}

double ParametricTrajectory::sigma(double theta) const
{
  return dp_dtheta(theta).norm();
}

double ParametricTrajectory::dsigma_dtheta(double theta) const
{
  const Eigen::Vector3d d1 = dp_dtheta(theta);
  const double s = d1.norm();
  if (s < 1e-9) {return 0.0;}
  return d1.dot(d2p_dtheta2(theta)) / s;
}

void ParametricTrajectory::buildArcTable() const
{
  const double dtheta = kTwoPi / static_cast<double>(samples_);
  s_table_.assign(samples_ + 1, 0.0);
  double acc = 0.0;
  double prev = sigma(0.0);
  for (int i = 0; i < samples_; ++i) {
    const double next = sigma((i + 1) * dtheta);
    acc += 0.5 * (prev + next) * dtheta;  // 梯形积分
    s_table_[i + 1] = acc;
    prev = next;
  }
  length_ = acc;
}

double ParametricTrajectory::thetaAt(double s) const
{
  if (s_table_.size() < 2) {buildArcTable();}
  // s_table_ 严格单调递增（σ>0），二分查找 + 线性内插
  const auto it = std::lower_bound(s_table_.begin(), s_table_.end(), s);
  int i = static_cast<int>(std::distance(s_table_.begin(), it));
  i = std::max(1, std::min(i, samples_));
  const double s0 = s_table_[i - 1];
  const double s1 = s_table_[i];
  const double dtheta = kTwoPi / static_cast<double>(samples_);
  const double frac = (s1 - s0) > 1e-12 ? (s - s0) / (s1 - s0) : 0.0;
  return (i - 1 + frac) * dtheta;
}

Reference ParametricTrajectory::sample(double t) const
{
  if (s_table_.size() < 2) {buildArcTable();}
  double s = std::fmod(speed_ * t, length_);
  if (s < 0.0) {s += length_;}

  const double theta = thetaAt(s);
  const double sg = sigma(theta);
  const double theta_dot = speed_ / sg;
  const double theta_ddot = -speed_ * speed_ * dsigma_dtheta(theta) / (sg * sg * sg);

  const Eigen::Vector3d d1 = dp_dtheta(theta);
  Reference r;
  r.pos = position(theta) + start_offset_;
  r.vel = d1 * theta_dot;
  r.acc = d2p_dtheta2(theta) * theta_dot * theta_dot + d1 * theta_ddot;
  return r;
}

// ---------------------------------------------------------------- 直线

void LinearTrajectory::configure(ParamReader & p)
{
  speed_ = p.getDouble("speed", 2.0);
  heading_deg_ = p.getDouble("heading_deg", 0.0);
  length_m_ = p.getDouble("length_m", 20.0);
  const double h = heading_deg_ * M_PI / 180.0;
  dir_ = Eigen::Vector3d(std::cos(h), std::sin(h), 0.0);
}

void LinearTrajectory::reset(const State & initial, double /*yaw*/)
{
  origin_ = initial.pos;
}

Reference LinearTrajectory::sample(double t) const
{
  const double d = std::clamp(speed_ * t, 0.0, length_m_);
  Reference r;
  r.pos = origin_ + dir_ * d;
  r.vel = (d < length_m_) ? Eigen::Vector3d(dir_ * speed_) : Eigen::Vector3d::Zero();
  r.acc.setZero();
  return r;
}

// ---------------------------------------------------------------- 圆周

void CircularTrajectory::configure(ParamReader & params)
{
  radius_ = params.getDouble("radius", 10.0);
  ParametricTrajectory::configure(params);
}

Eigen::Vector3d CircularTrajectory::position(double th) const
{
  return {center_.x() + radius_ * std::cos(th), center_.y() + radius_ * std::sin(th), height_};
}
Eigen::Vector3d CircularTrajectory::dp_dtheta(double th) const
{
  return {-radius_ * std::sin(th), radius_ * std::cos(th), 0.0};
}
Eigen::Vector3d CircularTrajectory::d2p_dtheta2(double th) const
{
  return {-radius_ * std::cos(th), -radius_ * std::sin(th), 0.0};
}

// ---------------------------------------------------------------- 螺旋

void HelicalTrajectory::configure(ParamReader & params)
{
  radius_ = params.getDouble("radius", 10.0);
  pitch_ = params.getDouble("pitch", 2.0);
  ParametricTrajectory::configure(params);
}

Eigen::Vector3d HelicalTrajectory::position(double th) const
{
  return {center_.x() + radius_ * std::cos(th), center_.y() + radius_ * std::sin(th),
    height_ + pitch_ * th / kTwoPi};
}
Eigen::Vector3d HelicalTrajectory::dp_dtheta(double th) const
{
  return {-radius_ * std::sin(th), radius_ * std::cos(th), pitch_ / kTwoPi};
}
Eigen::Vector3d HelicalTrajectory::d2p_dtheta2(double th) const
{
  return {-radius_ * std::cos(th), -radius_ * std::sin(th), 0.0};
}

// ---------------------------------------------------------------- 真八字（Gerono）

void Figure8Trajectory::configure(ParamReader & params)
{
  rx_ = params.getDouble("rx", 15.0);
  ry_ = params.getDouble("ry", 9.0);
  ParametricTrajectory::configure(params);
}

// x = cx + rx·sinθ；y = cy + ry·sinθ·cosθ（= (ry/2)·sin2θ）
Eigen::Vector3d Figure8Trajectory::position(double th) const
{
  return {center_.x() + rx_ * std::sin(th),
    center_.y() + ry_ * std::sin(th) * std::cos(th), height_};
}
Eigen::Vector3d Figure8Trajectory::dp_dtheta(double th) const
{
  return {rx_ * std::cos(th), ry_ * std::cos(2.0 * th), 0.0};
}
Eigen::Vector3d Figure8Trajectory::d2p_dtheta2(double th) const
{
  return {-rx_ * std::sin(th), -2.0 * ry_ * std::sin(2.0 * th), 0.0};
}

// ---------------------------------------------------------------- 悬停

void HoverTrajectory::configure(ParamReader & p)
{
  use_hold_z_ = p.getBool("use_hold_z", false);
  hold_z_ = p.getDouble("hold_z", 2.5);
}

void HoverTrajectory::reset(const State & initial, double /*yaw*/)
{
  hold_ = initial.pos;
  if (use_hold_z_) {hold_.z() = hold_z_;}
}

Reference HoverTrajectory::sample(double /*t*/) const
{
  Reference r;
  r.pos = hold_;
  return r;
}

// ---------------------------------------------------------------- 阶跃（最小 jerk）

void StepTrajectory::configure(ParamReader & p)
{
  step_x_ = p.getDouble("step_x", 5.0);
  step_y_ = p.getDouble("step_y", 0.0);
  step_z_ = p.getDouble("step_z", 0.0);
  duration_ = p.getDouble("duration", 5.0);
  if (duration_ < 1e-3) {duration_ = 1e-3;}
}

void StepTrajectory::reset(const State & initial, double /*yaw*/)
{
  origin_ = initial.pos;
}

Reference StepTrajectory::sample(double t) const
{
  const Eigen::Vector3d delta(step_x_, step_y_, step_z_);
  Reference r;
  if (t <= 0.0) {
    r.pos = origin_;
    return r;
  }
  if (t >= duration_) {
    r.pos = origin_ + delta;
    return r;
  }
  const double tau = t / duration_;
  r.pos = origin_ + delta * minJerk(tau);
  r.vel = delta * (minJerkDot(tau) / duration_);
  r.acc = delta * (minJerkDdot(tau) / (duration_ * duration_));
  return r;
}

}  // namespace agc_control

// pluginlib 导出（类名与 plugins.xml 的 type 一致）—— 同一实现供 air / ground 共用
PLUGINLIB_EXPORT_CLASS(agc_control::LinearTrajectory, agc_control::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(agc_control::CircularTrajectory, agc_control::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(agc_control::HelicalTrajectory, agc_control::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(agc_control::Figure8Trajectory, agc_control::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(agc_control::HoverTrajectory, agc_control::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(agc_control::StepTrajectory, agc_control::TrajectoryGenerator)
