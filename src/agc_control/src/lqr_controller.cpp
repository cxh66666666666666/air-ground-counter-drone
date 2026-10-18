// agc_control/lqr_controller.cpp —— LQR 控制器插件实现
//
// R-038（数值模型暴露与配置合法性启动校验；见 ADR-0053）：
//   ① 精确离散化（Bd 补上位置 0.5·dt² 项）；② DARE 收敛 / 残差 / 有限性检查，失败即抛错；
//   ③ 关键正数 / 有限性 / 权重可行域**集中校验**，非法配置**拒绝**（不静默兜底）。

#include "agc_control/lqr_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rclcpp/logging.hpp>

#include <Eigen/Dense>


namespace agc_control
{

namespace
{
/// 收集一处非法字段描述（非正 / 非有限）。
void noteBad(std::ostringstream & oss, const char * name, double value)
{
  oss << name << "=" << value << " ";
}
}  // namespace

void LqrController::validateParameters()
{
  // R-038：关键正数 / 有限性 / 权重可行域集中校验 —— 非法一律**拒绝**（**不**静默兜底）。
  // 可行域说明：Q = diag(q_pos·I₃, q_vel·I₃)、R = r·I₃，故 q_pos/q_vel>0、r>0 即 Q 半正定、R 正定。
  std::ostringstream bad;
  if (!std::isfinite(q_pos_) || q_pos_ <= 0.0) {noteBad(bad, "q_pos", q_pos_);}
  if (!std::isfinite(q_vel_) || q_vel_ <= 0.0) {noteBad(bad, "q_vel", q_vel_);}
  if (!std::isfinite(r_) || r_ <= 0.0) {noteBad(bad, "r", r_);}
  if (!std::isfinite(dt_) || dt_ <= 0.0) {noteBad(bad, "dt", dt_);}
  if (!std::isfinite(tol_) || tol_ <= 0.0) {noteBad(bad, "tol", tol_);}
  if (!std::isfinite(acc_max_)) {noteBad(bad, "acc_max", acc_max_);}
  if (max_iter_ <= 0) {bad << "max_iter=" << max_iter_ << " ";}
  if (!bad.str().empty()) {
    throw std::runtime_error(
      "LqrController 非法配置（拒绝启动，非静默兜底）：" + bad.str() +
      "；要求 q_pos/q_vel/r/dt/tol 有限且 > 0，max_iter > 0，acc_max 有限"
      "（Q=diag(q_pos·I₃,q_vel·I₃) 半正定、R=r·I₃ 正定）。");
  }
}

void LqrController::configure(ParamReader & p)
{
  q_pos_ = p.getDouble("q_pos", 4.0);
  q_vel_ = p.getDouble("q_vel", 4.0);
  r_ = p.getDouble("r", 1.0);
  dt_ = p.getDouble("dt", 0.02);
  acc_max_ = p.getDouble("acc_max", 4.0);
  max_iter_ = p.getInt("max_iter", 2000);   // R-038：预算需 ≥ ~559 方达 tol=1e-10（见 ADR-0053 取证）
  tol_ = p.getDouble("tol", 1e-10);

  validateParameters();   // R-038：非法 Q/R/dt/max_iter/tol → 明确拒绝（删除静默兜底 dt=0.02/r=1.0）
  solveDiscreteLqr();
}

void LqrController::solveDiscreteLqr()
{
  // 连续时间双积分器：A = [[0, I],[0, 0]]（**幂零**：A² = 0）、B = [[0],[I]]。
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
  A.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Zero();
  B.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();

  // 精确离散化：Ad = I + A·dt + 0.5·A²·dt²（本 A 幂零 → A²=0 → 高阶项为零 → **恰为精确**）。
  //   Bd = B·dt + 0.5·A·B·dt²（A·B = [[I],[0]]）→ Bd = [[0.5·dt²·I],[dt·I]]（含位置 0.5·dt² 项，**精确**）。
  const Eigen::Matrix<double, 6, 6> A2 = A * A;                       // = 0（幂零）
  const Eigen::Matrix<double, 6, 6> Ad =
    Eigen::Matrix<double, 6, 6>::Identity() + A * dt_ + 0.5 * A2 * dt_ * dt_;
  const Eigen::Matrix<double, 6, 3> Bd = B * dt_ + 0.5 * A * B * dt_ * dt_;

  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  Q.topLeftCorner<3, 3>() = q_pos_ * Eigen::Matrix3d::Identity();
  Q.bottomRightCorner<3, 3>() = q_vel_ * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R = r_ * Eigen::Matrix3d::Identity();

  // DARE 值迭代：P ← AᵀPA − AᵀPB(R+BᵀPB)⁻¹BᵀPA + Q；记录迭代次数与**残差** max|P_next−P|。
  Eigen::Matrix<double, 6, 6> P = Q;
  int it = 0;
  double residual = std::numeric_limits<double>::infinity();
  bool converged = false;
  for (; it < max_iter_; ++it) {
    const Eigen::Matrix<double, 6, 3> PB = P * Bd;
    const Eigen::Matrix3d S = R + Bd.transpose() * PB;
    const Eigen::Matrix<double, 6, 6> AtP = Ad.transpose() * P;
    const Eigen::Matrix<double, 6, 6> P_next =
      AtP * Ad - (AtP * Bd) * S.inverse() * (Bd.transpose() * P * Ad) + Q;
    residual = (P_next - P).cwiseAbs().maxCoeff();
    P = P_next;
    if (residual < tol_) {converged = true; ++it; break;}
  }
  iters_ = it;
  residual_ = residual;
  converged_ = converged;

  // R-038：数值健全诊断日志（ad/bd 是否精确式、dt、适用频率、DARE 收敛与残差）。
  RCLCPP_INFO(rclcpp::get_logger("agc_control.lqr"),
    "LQR 离散化：Ad=I+A·dt+0.5·A²·dt²（A 幂零 → 恰为**精确**）；"
    "Bd=[[0.5·dt²·I],[dt·I]]（**精确**，含位置 0.5·dt² 项）；dt=%.6f s（适用频率 %.1f Hz）；"
    "DARE 迭代 %d 次，残差=%.3e，%s（tol=%.1e）",
    dt_, appliedFrequencyHz(), iters_, residual_, converged ? "已收敛" : "未收敛", tol_);

  // 有限性：P / S 必须有限，否则**明确失败**（不静默使用）。
  if (!P.allFinite()) {
    throw std::runtime_error("LqrController DARE 迭代产生非有限 P（NaN/Inf）：拒绝使用（R-038）");
  }
  const Eigen::Matrix3d S = R + Bd.transpose() * P * Bd;
  if (!S.allFinite()) {
    throw std::runtime_error("LqrController DARE 中间量 S 非有限（NaN/Inf）：拒绝使用（R-038）");
  }
  K_ = S.inverse() * Bd.transpose() * P * Ad;  // 3×6
  if (!K_.allFinite()) {
    throw std::runtime_error("LqrController 反馈增益 K 非有限（NaN/Inf）：拒绝使用（R-038）");
  }
  // 未收敛（残差 ≥ tol）→ **明确失败**（不静默使用不可靠增益）。
  if (!converged) {
    std::ostringstream oss;
    oss << "LqrController DARE 达 max_iter=" << max_iter_ << " 未收敛：残差=" << residual
        << " ≥ tol=" << tol_ << "（dt=" << dt_ << " s）；拒绝使用不可靠增益（R-038）";
    throw std::runtime_error(oss.str());
  }
}

Eigen::Vector3d LqrController::computeAcceleration(const State & state, const Reference & ref)
{
  Eigen::Matrix<double, 6, 1> x;
  x.head<3>() = ref.pos - state.pos;
  x.tail<3>() = ref.vel - state.vel;

  Eigen::Vector3d u = K_ * x + ref.acc;

  if (acc_max_ > 0.0) {
    const double n = u.norm();
    if (n > acc_max_) {u *= acc_max_ / n;}
  }
  return u;
}

}  // namespace agc_control

PLUGINLIB_EXPORT_CLASS(agc_control::LqrController, agc_control::Controller)
