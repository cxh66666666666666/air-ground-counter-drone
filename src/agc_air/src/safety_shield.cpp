// agc_air/safety_shield.cpp

#include "agc_air/safety_shield.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "agc_control/param_reader.hpp"

namespace agc_air
{

namespace
{
/// 把参考点 `p` 从 `center` 沿径向推到距离 ≥ r（XY 平面）
agc_control::Vector3 pushOut(const agc_control::Vector3 & p, double cx, double cy, double r)
{
  const double dx = p.x() - cx;
  const double dy = p.y() - cy;
  const double d = std::hypot(dx, dy);
  agc_control::Vector3 out = p;
  if (d >= r) {return out;}
  if (d < 1e-6) {
    out.x() = cx + r;   // 退化情形：沿 +x 推出
    return out;
  }
  out.x() = cx + dx / d * r;
  out.y() = cy + dy / d * r;
  return out;
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

  const auto names = p.getStringArray("obstacle_names");
  const auto xs = p.getDoubleArray("obstacle_x");
  const auto ys = p.getDoubleArray("obstacle_y");
  const auto rs = p.getDoubleArray("obstacle_safety_radius");

  obstacles_.clear();
  const std::size_t n = std::min({names.size(), xs.size(), ys.size(), rs.size()});
  for (std::size_t i = 0; i < n; ++i) {
    obstacles_.push_back(StaticObstacle{names[i], xs[i], ys[i], rs[i]});
  }
}

agc_control::Reference SafetyShield::apply(const agc_control::Reference & desired,
  const agc_control::Vector3 & target_pos, bool target_valid) const
{
  if (!enabled_) {return desired;}
  agc_control::Reference ref = desired;

  // ① 静态障碍：参考点不侵入 (安全半径 + obstacle_margin)
  for (const auto & ob : obstacles_) {
    const double r_eff = ob.safety_radius + obstacle_margin_m_;
    const double dx = ref.pos.x() - ob.x;
    const double dy = ref.pos.y() - ob.y;
    const double d = std::hypot(dx, dy);
    if (d < r_eff) {
      ref.pos = pushOut(ref.pos, ob.x, ob.y, r_eff);
      // 去掉指向障碍的速度分量（不再朝里飞）
      if (d > 1e-6) {
        const agc_control::Vector3 inward(-dx / d, -dy / d, 0.0);
        const double v_in = ref.vel.dot(inward);
        if (v_in > 0.0) {ref.vel -= inward * v_in;}
      }
    }
  }

  // ② 对目标最小间距兜底：参考点不得比 (target_min_dist + target_margin) 更贴近目标
  if (target_valid) {
    const double r_eff = target_min_dist_ + target_margin_m_;
    const double dx = ref.pos.x() - target_pos.x();
    const double dy = ref.pos.y() - target_pos.y();
    const double d = std::hypot(dx, dy);
    if (d < r_eff) {
      ref.pos = pushOut(ref.pos, target_pos.x(), target_pos.y(), r_eff);
      if (d > 1e-6) {
        const agc_control::Vector3 inward(-dx / d, -dy / d, 0.0);
        const double v_in = ref.vel.dot(inward);
        if (v_in > 0.0) {ref.vel -= inward * v_in;}
      }
    }
  }

  return ref;
}

}  // namespace agc_air
