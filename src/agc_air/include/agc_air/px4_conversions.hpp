// agc_air/px4_conversions.hpp —— PX4 uORB 语义 → 契约口径 的映射（R2 / ADR-0005）
//
// 集中三件事，供 px4_adapter 与 setpoint 发布者共用：
//   ①时间：PX4 µs（自 boot）→ ROS 时间（sim 时钟，唯一时钟）
//   ②坐标：NED↔ENU、FRD↔FLU（位置 / 速度 / 姿态 / 角速度）
//   ③偏航：ENU 偏航 ↔ PX4(NED) 偏航
// 契约 [命名空间与坐标系]：世界系为 agc_world（ENU）；机体系 `drone/base_link`。

#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace agc_air
{

using Vector3 = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;

/// PX4 微秒时间戳 → 秒
inline double microsToSec(std::uint64_t micros)
{
  return static_cast<double>(micros) * 1e-6;
}

/// NED → ENU：enu = (E, N, U) = (ned.y, ned.x, −ned.z)
inline Vector3 nedToEnu(const Vector3 & ned)
{
  return Vector3(ned.y(), ned.x(), -ned.z());
}

/// FRD → FLU：机体系向量换基（forward 不变，right→left 取负，down→up 取负）
inline Vector3 frdToFlu(const Vector3 & frd)
{
  return Vector3(frd.x(), -frd.y(), -frd.z());
}

/// PX4 姿态四元数（FRD 机体系 → NED 世界系）→ ROS 约定（FLU → ENU）
/// R_enu←flu = R_enu←ned · R_ned←frd · R_frd←flu
inline Quaternion px4AttitudeToEnu(const std::array<float, 4> & q_xyzw)
{
  const Quaternion q_px4(static_cast<double>(q_xyzw[3]), static_cast<double>(q_xyzw[0]),
    static_cast<double>(q_xyzw[1]), static_cast<double>(q_xyzw[2]));
  // q(ENU←NED)：绕 (1,1,0)/√2 转 π
  const Quaternion q_enu_ned(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);
  // q(FLU←FRD)：绕 x 转 π
  const Quaternion q_flu_frd(0.0, 1.0, 0.0, 0.0);
  return (q_enu_ned * q_px4 * q_flu_frd).normalized();
}

/// ENU 偏航（自 +x 东逆时针）→ PX4 偏航（自北顺时针）：ψ_ned = π/2 − ψ_enu
inline double enuYawToPx4(double yaw_enu)
{
  double y = M_PI_2 - yaw_enu;
  while (y > M_PI) {y -= 2.0 * M_PI;}
  while (y < -M_PI) {y += 2.0 * M_PI;}
  return y;
}

/// PX4 偏航（NED）→ ENU 偏航
inline double px4YawToEnu(double yaw_ned)
{
  return enuYawToPx4(yaw_ned);
}

/// 由 ENU 姿态四元数取偏航角（自 +x 东逆时针）
inline double yawFromQuaternion(const Quaternion & q)
{
  const Eigen::Matrix3d R = q.toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

/// PX4 时间戳（自 boot 的 µs）→ ROS 时间（sim 时钟）的仿射映射。
///
/// PX4 内部时间与 gz `/clock`（sim 时间）不同源；用指数滑动平均估计二者的常数偏移，
/// 既保留 PX4 数据流内部的相对时序，又把绝对时间锚定到唯一时钟（契约 #namespace）。
class Px4TimeMapper
{
public:
  explicit Px4TimeMapper(double alpha = 0.05)
  : alpha_(alpha) {}

  /// 观察一对同刻时间（px4 µs 与其对应的 ROS 时刻 µs）
  void observe(std::uint64_t px4_us, std::int64_t ros_us)
  {
    const double offset = static_cast<double>(ros_us) - static_cast<double>(px4_us);
    if (!valid_) {
      offset_us_ = offset;
      valid_ = true;
    } else {
      offset_us_ = (1.0 - alpha_) * offset_us_ + alpha_ * offset;
    }
  }

  bool valid() const {return valid_;}
  double offsetUs() const {return offset_us_;}

  /// PX4 µs → ROS µs（未观测到时返回 0）
  std::int64_t toRosMicros(std::uint64_t px4_us) const
  {
    if (!valid_) {return 0;}
    return static_cast<std::int64_t>(static_cast<double>(px4_us) + offset_us_);
  }

private:
  double alpha_{0.05};
  double offset_us_{0.0};
  bool valid_{false};
};

}  // namespace agc_air
