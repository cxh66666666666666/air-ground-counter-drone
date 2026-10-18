// agc_control/lqr_controller.hpp —— LQR 控制器插件（pluginlib）
//
// 被控对象取位置环等效双积分器：ẍ = u（u 为加速度指令）。
// 误差状态 x = [e_p; e_v]（参考 − 实际），误差动态 ẋ = A x + B(ẍ_ref − u)。
// 增益由**离散代数 Riccati 方程（DARE）**迭代求解，u = ẍ_ref + K·x（ref.acc 即前馈）。
// 与执行器解耦：输出为世界系加速度，air / ground 各自转执行量（ADR-0018）。
//
// R-038（数值模型暴露与配置合法性启动校验，见 ADR-0053）：
//   - **精确离散化**：Ad = I + A·dt + 0.5·A²·dt²（本 A 幂零 → 高阶项为零 → 恰为精确）；
//     Bd = [[0.5·dt²·I],[dt·I]]（补上位置的 0.5·dt² 项，**精确**）；
//   - DARE 迭代记录**残差**与**收敛性**，未收敛 / 非有限增益 → **抛错**（配置失败，**不**静默使用）；
//   - `q_pos/q_vel/r/dt/tol/max_iter` **集中校验**（非正 / 非有限一律**拒绝**，**不**静默兜底）；
//   - 只读诊断（`converged()` / `residual()` / `iterations()` / `dt()` / `appliedFrequencyHz()`）**非** ABI。

#pragma once

#include <limits>
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

  // ---- R-038：数值健全只读诊断（**非** ABI / 非命令者）----
  /// DARE 迭代是否达残差阈值（未收敛 → configure 已抛错）
  bool converged() const {return converged_;}
  /// 末次迭代残差 max|P_next − P|（未收敛时 ≥ tol_）
  double residual() const {return residual_;}
  /// 实际迭代次数
  int iterations() const {return iters_;}
  /// 离散化步长 dt（s）
  double dt() const {return dt_;}
  /// 适用频率 1/dt（Hz）
  double appliedFrequencyHz() const {return dt_ > 0.0 ? 1.0 / dt_ : 0.0;}

private:
  void validateParameters();
  void solveDiscreteLqr();

  double q_pos_{4.0};
  double q_vel_{4.0};
  double r_{1.0};
  double dt_{0.02};
  double acc_max_{4.0};   // 加速度指令限幅（m/s²），<=0 表示不限幅
  // R-038：默认迭代预算 2000（实测：tol=1e-10 需约 559 次迭代收敛；旧默认 200 **未收敛**且被静默使用）。
  int max_iter_{2000};
  double tol_{1e-10};

  // R-038：DARE 收敛诊断（默认未收敛 → 未求解前不冒充可用）
  bool converged_{false};
  double residual_{std::numeric_limits<double>::infinity()};
  int iters_{0};

  Eigen::Matrix<double, 3, 6> K_{Eigen::Matrix<double, 3, 6>::Zero()};
};

}  // namespace agc_control
