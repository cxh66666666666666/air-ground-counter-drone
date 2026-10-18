// agc_air/safety_shield.cpp —— R-033 联合安全屏障（联合约束复检 + 扫掠复检 + 显式不可行路径）

#include "agc_air/safety_shield.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "agc_control/param_reader.hpp"

namespace agc_air
{

namespace
{
/// 联合投影迭代预算：足够收敛单盘 / 相容盘组；**不**收敛即判「不可行」（见 R-033 非交换投影）。
constexpr int kMaxProjectIters = 64;

/// R-033：联合约束（XY 圆盘）—— 静态障碍与目标间距**同时**生效的约束项。
struct Disk
{
  double x;
  double y;
  double r;
};

/// 点到线段 [a,b] 的最短距离（XY）—— 扫掠净空判据。
double pointSegDist(double px, double py, double ax, double ay, double bx, double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 1e-12) {
    t = ((px - ax) * dx + (py - ay) * dy) / len2;
    t = std::clamp(t, 0.0, 1.0);
  }
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

/// 端点可行性：点 (x,y) 是否同时位于**全部**约束圆盘之外；可回传最严重违约项下标。
bool feasibleXY(const std::vector<Disk> & ds, double x, double y, std::size_t * worst)
{
  double worst_viol = 0.0;
  std::size_t wi = ds.size();
  for (std::size_t i = 0; i < ds.size(); ++i) {
    const double d = std::hypot(x - ds[i].x, y - ds[i].y);
    if (d < ds[i].r - 1e-9) {
      const double v = ds[i].r - d;
      if (v > worst_viol) {worst_viol = v; wi = i;}
    }
  }
  if (worst) {*worst = wi;}
  return wi == ds.size();
}

/// 单约束径向推出：把点推到距圆心 ≥ r（退化时沿 +x）。
void pushOut(double & x, double & y, double cx, double cy, double r)
{
  const double dx = x - cx;
  const double dy = y - cy;
  const double d = std::hypot(dx, dy);
  if (d >= r) {return;}
  if (d < 1e-6) {x = cx + r; y = cy; return;}
  x = cx + dx / d * r;
  y = cy + dy / d * r;
}

/// 联合投影：反复推出**全部**违约约束，直至同时满足（收敛）或耗尽预算。
/// 与旧实现的差别：**不**做"逐障碍 + 目标"的**单遍**投影即返回——单遍后投影可重新落入前约束
/// （投影不可交换，R-033 病灶）；本函数**重复复检**，耗尽预算仍未同时满足即判不可行。
bool projectJointFeasible(const std::vector<Disk> & ds, double & x, double & y)
{
  for (int it = 0; it < kMaxProjectIters; ++it) {
    bool any_violation = false;
    for (const auto & d : ds) {
      if (std::hypot(x - d.x, y - d.y) < d.r - 1e-9) {
        pushOut(x, y, d.x, d.y, d.r);
        any_violation = true;
      }
    }
    if (!any_violation) {return true;}
  }
  return feasibleXY(ds, x, y, nullptr);
}

/// 扫掠复检：当前位置 → 参考端点 的直线段是否穿入任一**起点在外**的约束圆盘。
/// 起点已在圆盘内的约束跳过（那属于"自当前位置逃出"，由不可行路径处理，而非扫掠冲突）。
bool sweptClear(const std::vector<Disk> & ds,
  double ax, double ay, double bx, double by, std::size_t * hit)
{
  for (std::size_t i = 0; i < ds.size(); ++i) {
    const double da = std::hypot(ax - ds[i].x, ay - ds[i].y);
    if (da < ds[i].r) {continue;}
    if (pointSegDist(ds[i].x, ds[i].y, ax, ay, bx, by) < ds[i].r - 1e-9) {
      if (hit) {*hit = i;}
      return false;
    }
  }
  return true;
}
}  // namespace

void SafetyShield::configure_parameters(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params, const std::string & ns)
{
  agc_control::NodeParamReader p(std::move(params), ns);

  enabled_ = p.getBool("enabled", true);
  target_min_dist_ = p.getDouble("target_min_dist", 8.0);
  obstacle_margin_m_ = p.getDouble("obstacle_margin", 2.5);
  target_margin_m_ = p.getDouble("target_margin", 1.5);
  // R-033：跟踪误差 / 反应时间 / 可达性余量（配置项承载；绝对值为可标定量）
  reaction_time_s_ = p.getDouble("reaction_time_s", 0.2);
  max_speed_mps_ = p.getDouble("max_speed_mps", 2.0);
  reachability_margin_m_ = p.getDouble("reachability_margin", 0.3);

  const auto names = p.getStringArray("obstacle_names");
  const auto xs = p.getDoubleArray("obstacle_x");
  const auto ys = p.getDoubleArray("obstacle_y");
  const auto rs = p.getDoubleArray("obstacle_safety_radius");

  // R-038（见 ADR-0053）：障碍表为**平行数组**，四者长度**必须一致**。
  //   **删除**旧的 `n=min({...})` 静默按最短长度截断 —— 漏配一个数组会**静默漏障碍**；
  //   改为一律**拒绝**（抛错 → configure 失败 → 生命周期 error 路径接管；**不**静默运行）。
  if (names.size() != xs.size() || names.size() != ys.size() || names.size() != rs.size()) {
    std::ostringstream oss;
    oss << "安全层障碍平行数组长度不一致（拒绝启动）：obstacle_names=" << names.size()
        << " obstacle_x=" << xs.size() << " obstacle_y=" << ys.size()
        << " obstacle_safety_radius=" << rs.size() << "；四者必须相等（不静默按最短长度截断）。";
    throw std::runtime_error(oss.str());
  }

  obstacles_.clear();
  for (std::size_t i = 0; i < names.size(); ++i) {
    // R-038：几何合法性 —— 半径必须有限且 > 0，坐标必须有限；非法一律**拒绝**。
    if (!std::isfinite(rs[i]) || rs[i] <= 0.0) {
      std::ostringstream oss;
      oss << "安全层障碍几何非法（拒绝启动）：obstacle_safety_radius[" << i << "]=" << rs[i]
          << "（要求有限且 > 0）";
      throw std::runtime_error(oss.str());
    }
    if (!std::isfinite(xs[i]) || !std::isfinite(ys[i])) {
      std::ostringstream oss;
      oss << "安全层障碍几何非法（拒绝启动）：obstacle_x[" << i << "]=" << xs[i]
          << " obstacle_y[" << i << "]=" << ys[i] << "（要求有限）";
      throw std::runtime_error(oss.str());
    }
    obstacles_.push_back(StaticObstacle{names[i], xs[i], ys[i], rs[i]});
  }
}

SafetyResult SafetyShield::apply(const agc_control::Reference & desired,
  const agc_control::Vector3 & current_pos,
  const agc_control::Vector3 & target_pos, bool target_valid) const
{
  SafetyResult out;
  out.ref = desired;
  out.reason = "clear";
  if (!enabled_) {return out;}

  // 联合约束集（XY 圆盘）：静态障碍约束 **与** 目标间距约束 **同时** 生效。
  // ⚠ 二维静态限定模型（**限实验用途**）：仅含 XY 静态圆盘；**不含**车 / 动态障碍状态
  //   （动态障碍状态 = unsupported），**不**建模三维机体外形 / 载荷外伸。
  //   修正后边界 = 真实安全半径 + 跟踪误差 + 反应 + 可达性余量。
  const double extra = dynamicMargin();
  std::vector<Disk> disks;
  disks.reserve(obstacles_.size() + 1);
  for (const auto & ob : obstacles_) {
    disks.push_back(Disk{ob.x, ob.y, ob.safety_radius + obstacle_margin_m_ + extra});
  }
  if (target_valid) {
    disks.push_back(Disk{target_pos.x(), target_pos.y(), target_min_dist_ + target_margin_m_ + extra});
  }
  if (disks.empty()) {return out;}

  const double cx = current_pos.x();
  const double cy = current_pos.y();

  // ① 候选参考：期望端点；端点不可行时先做**联合投影**（重复复检，非单遍）
  double qx = desired.pos.x();
  double qy = desired.pos.y();
  const bool endpoint_ok = feasibleXY(disks, qx, qy, nullptr);
  if (!endpoint_ok) {projectJointFeasible(disks, qx, qy);}

  // ② 终检：候选端点**同时**满足全部约束 **且** 当前位置→候选端点 的直线段不穿约束
  std::size_t swept_hit = disks.size();
  const bool final_endpoint_ok = feasibleXY(disks, qx, qy, nullptr);
  const bool final_swept_ok = sweptClear(disks, cx, cy, qx, qy, &swept_hit);

  if (final_endpoint_ok && final_swept_ok) {
    if (std::hypot(qx - desired.pos.x(), qy - desired.pos.y()) > 1e-6) {
      out.modified = true;
      out.reason = "adjusted";
      out.ref.pos.x() = qx;
      out.ref.pos.y() = qy;
      // 去掉指向各约束中心的速度分量（不再朝里飞）
      for (const auto & d : disks) {
        const double dx = out.ref.pos.x() - d.x;
        const double dy = out.ref.pos.y() - d.y;
        const double dist = std::hypot(dx, dy);
        if (dist > 1e-6 && dist <= d.r + 1e-6) {
          const agc_control::Vector3 inward(-dx / dist, -dy / dist, 0.0);
          const double v_in = out.ref.vel.dot(inward);
          if (v_in > 0.0) {out.ref.vel -= inward * v_in;}
        }
      }
    }
    return out;
  }

  // ③ **显式不可行**：无可行参考 → simple hold / escape（**不**返回可能不可达参考）
  out.modified = true;
  out.infeasible = true;
  out.reason = endpoint_ok ? "infeasible_swept" : "infeasible_joint";

  double hx = cx;
  double hy = cy;
  if (feasibleXY(disks, hx, hy, nullptr)) {
    // hold：当前机体位置本身安全 → 保持当前位置
    out.ref.pos.x() = hx;
    out.ref.pos.y() = hy;
    out.action = "hold";
  } else {
    double ex = hx;
    double ey = hy;
    if (projectJointFeasible(disks, ex, ey)) {
      // escape：自当前机体位置逃到最近可行点（远离冲突障碍）
      out.ref.pos.x() = ex;
      out.ref.pos.y() = ey;
      out.action = "escape";
    } else {
      // 连联合投影亦不收敛：对**最严重违约**约束做单约束径向推出（保证该约束净空）
      std::size_t worst = disks.size();
      feasibleXY(disks, hx, hy, &worst);
      const Disk & d = disks[(worst == disks.size()) ? 0 : worst];
      pushOut(hx, hy, d.x, d.y, d.r);
      out.ref.pos.x() = hx;
      out.ref.pos.y() = hy;
      out.action = "escape";
    }
  }
  // 显式不可行路径：速度 / 加速度前馈清零（安全 hold / 逃逸，不给 LQR 追逐不可达参考）
  out.ref.vel = agc_control::Vector3::Zero();
  out.ref.acc = agc_control::Vector3::Zero();
  return out;
}

}  // namespace agc_air
