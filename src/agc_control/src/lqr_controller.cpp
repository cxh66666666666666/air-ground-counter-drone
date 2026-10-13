// agc_control/lqr_controller.cpp —— LQR 控制器插件实现

#include "agc_control/lqr_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>


namespace agc_control
{

void LqrController::configure(ParamReader & p)
{
  q_pos_ = p.getDouble("q_pos", 4.0);
  q_vel_ = p.getDouble("q_vel", 4.0);
  r_ = p.getDouble("r", 1.0);
  dt_ = p.getDouble("dt", 0.02);
  acc_max_ = p.getDouble("acc_max", 4.0);
  max_iter_ = p.getInt("max_iter", 200);
  tol_ = p.getDouble("tol", 1e-10);

  if (dt_ <= 0.0) {dt_ = 0.02;}
  if (r_ <= 0.0) {r_ = 1.0;}
  solveDiscreteLqr();
}

void LqrController::solveDiscreteLqr()
{
  // 连续时间双积分器
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
  A.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Zero();
  B.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();

  // 精确离散化（本 A 为幂零，仅留一阶项）
  Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Identity() + A * dt_;
  Eigen::Matrix<double, 6, 3> Bd = B * dt_;

  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  Q.topLeftCorner<3, 3>() = q_pos_ * Eigen::Matrix3d::Identity();
  Q.bottomRightCorner<3, 3>() = q_vel_ * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R = r_ * Eigen::Matrix3d::Identity();

  // DARE 值迭代：P ← AᵀPA − AᵀPB(R+BᵀPB)⁻¹BᵀPA + Q
  Eigen::Matrix<double, 6, 6> P = Q;
  for (int i = 0; i < max_iter_; ++i) {
    const Eigen::Matrix<double, 6, 3> PB = P * Bd;
    const Eigen::Matrix3d S = R + Bd.transpose() * PB;
    const Eigen::Matrix<double, 6, 6> AtP = Ad.transpose() * P;
    const Eigen::Matrix<double, 6, 6> P_next =
      AtP * Ad - (AtP * Bd) * S.inverse() * (Bd.transpose() * P * Ad) + Q;
    const double diff = (P_next - P).cwiseAbs().maxCoeff();
    P = P_next;
    if (diff < tol_) {break;}
  }

  const Eigen::Matrix3d S = R + Bd.transpose() * P * Bd;
  K_ = S.inverse() * Bd.transpose() * P * Ad;  // 3×6
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
