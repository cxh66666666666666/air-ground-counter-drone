// agc_control/lqr_controller.hpp —— LQR 控制器插件（pluginlib）
//
// 被控对象取位置环等效双积分器：ẍ = u（u 为加速度指令）。
// 误差状态 x = [e_p; e_v]（参考 − 实际），误差动态 ẋ = A x + B(ẍ_ref − u)。
// 增益由**离散代数 Riccati 方程（DARE）**迭代求解，u = ẍ_ref + K·x（ref.acc 即前馈）。
// 与执行器解耦：输出为世界系加速度，air / ground 各自转执行量（ADR-0018）。

#pragma once

#include <string>

#include <Eigen/Core>

#include "agc_control/controller.hpp"

namespace agc_control
{

class LqrController : public Controller
{
public:
  void configure(ParamReader & params) override;
  void reset() override {}
  Eigen::Vector3d computeAcceleration(const State & state, const Reference & ref) override;
  std::string name() const override {return "lqr";}

  /// 供单测 / 诊断：求得的 3×6 反馈增益（按 [e_p, e_v] 顺序）
  const Eigen::Matrix<double, 3, 6> & gain() const {return K_;}

private:
  void solveDiscreteLqr();

  double q_pos_{4.0};
  double q_vel_{4.0};
  double r_{1.0};
  double dt_{0.02};
  double acc_max_{4.0};   // 加速度指令限幅（m/s²），<=0 表示不限幅
  int max_iter_{200};
  double tol_{1e-10};

  Eigen::Matrix<double, 3, 6> K_{Eigen::Matrix<double, 3, 6>::Zero()};
};

}  // namespace agc_control
