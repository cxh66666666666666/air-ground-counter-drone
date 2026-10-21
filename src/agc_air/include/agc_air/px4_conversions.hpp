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
#include <sstream>
#include <string>

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
/// **字段序**：PX4 uORB / px4_msgs 的 `q` 为 Hamilton **[w,x,y,z]**（`VehicleOdometry.q[0]=w`，
///   ekf2 工具链即按 `q[0]→qw` 解析；见 `msg/VehicleOdometry.msg`「First value NaN if invalid」）。
///   **不得**按 ROS `geometry_msgs` 的 [x,y,z,w] 解析——否则得到与真值差约 60° 的错误姿态
///   （W7-b 取证：raw q=[0.877,0.004,0.002,0.480] 正确→ENU yaw 32.7°，按 [x,y,z,w]→89.5°）。
inline Quaternion px4AttitudeToEnu(const std::array<float, 4> & q_wxyz)
{
  const Quaternion q_px4(static_cast<double>(q_wxyz[0]), static_cast<double>(q_wxyz[1]),
    static_cast<double>(q_wxyz[2]), static_cast<double>(q_wxyz[3]));
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

/// 局部 NED 位置 → 世界 ENU（R-003）：world = origin_enu + R_enu←ned · p_ned。
/// PX4 EKF2 的 `VehicleOdometry.position` 是**相对 EKF 全局原点**的局部位置（源码确认），
/// 不是 gz 世界坐标；origin_enu 即该原点在世界 ENU 中的位置（SITL 下为 spawn 位）。
inline Vector3 localNedToWorldEnu(const Vector3 & pos_ned, const Vector3 & origin_enu)
{
  return origin_enu + nedToEnu(pos_ned);
}

/// 世界 ENU 速度 → 机体系 FLU：v_flu = R(q_enu)ᵀ · v_enu（q_enu 为 FLU→ENU 姿态）。
inline Vector3 enuToBodyFlu(const Quaternion & q_enu, const Vector3 & v_enu)
{
  return q_enu.normalized().conjugate() * v_enu;
}

/// 机体系 FLU 速度 → 世界 ENU：v_enu = R(q_enu) · v_flu。
/// `/agc/air/odom` 的 twist 依 nav_msgs 约定在 child frame（机体系），消费端据此还原世界速度。
inline Vector3 bodyFluToEnu(const Quaternion & q_enu, const Vector3 & v_flu)
{
  return q_enu.normalized() * v_flu;
}

/// 解析 spawn 位姿串（`"x,y,z[,roll,pitch,yaw]"`）的**前三个**数值 → 世界 ENU 位置。
/// 用作 `px4_adapter` 的局部原点：PX4 EKF 原点取**首次定位时刻的机体位置**，SITL 下即 spawn 位。
inline bool parseSpawnPositionXyz(const std::string & s, Vector3 & out)
{
  std::istringstream iss(s);
  std::string tok;
  double v[3] = {0.0, 0.0, 0.0};
  for (double & i : v) {
    if (!std::getline(iss, tok, ',')) {return false;}
    try {
      i = std::stod(tok);
    } catch (const std::exception &) {
      return false;
    }
  }
  out = Vector3(v[0], v[1], v[2]);
  return true;
}

/// PX4 时间戳（自 boot 的 µs）→ ROS 时间（sim 时钟）的仿射映射。
///
/// PX4 内部时间与 gz `/clock`（sim 时间）不同源；用指数滑动平均估计二者的常数偏移，
/// 既保留 PX4 数据流内部的相对时序，又把绝对时间锚定到唯一时钟（契约 #namespace）。
class Px4TimeMapper
{
public:
  /// 偏移跳变判定阈值（µs）：新观测的偏移与当前估计之差超过此值 → 判为**时间源尺度跳变**
  /// （典型 = uXRCE-DDS 桥时间同步把 PX4 时间戳推到墙上时钟尺度，而 ROS 侧是仿真时钟），
  /// **直接重锚**而非 EMA 混合——否则旧估计会把新尺度稀释成**离奇时间戳**（实测曾产出
  /// `sec≈−1.7e9` / `sec≈+8.5e7` 的脏 stamp，令下游年龄门失效）。
  static constexpr double kReanchorJumpUs = 100000.0;  // 0.1 s

  explicit Px4TimeMapper(double alpha = 0.05, double reanchor_jump_us = kReanchorJumpUs)
  : alpha_(alpha), reanchor_jump_us_(reanchor_jump_us) {}

  /// 丢弃当前锚定 → 下一帧观测重新锚定。用于 **EKF 复位** / **时钟跳变** / **不可信映射自愈**
  /// （实测：一次脏映射必须重锚，否则偏移被污染后会持续产出离奇 stamp）。
  void reset()
  {
    valid_ = false;
    offset_us_ = 0.0;
  }

  /// 观察一对同刻时间（px4 µs 与其对应的 ROS 时刻 µs）
  void observe(std::uint64_t px4_us, std::int64_t ros_us)
  {
    const double offset = static_cast<double>(ros_us) - static_cast<double>(px4_us);
    if (!valid_) {
      offset_us_ = offset;
      valid_ = true;
    } else if (std::fabs(offset - offset_us_) > reanchor_jump_us_) {
      offset_us_ = offset;      // 尺度跳变 → **重锚**（不 EMA 混合）
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

  /// PX4 µs → ROS **纳秒**（未观测到时返回 0）。
  /// 构造 `rclcpp::Time` 必须走这里：单整数构造器按**纳秒**解释，误传 µs 会让 Header
  /// 时间戳缩小 1000 倍（R-002）。两个单位接口成对存在，避免"数值对但单位错"。
  std::int64_t toRosNanoseconds(std::uint64_t px4_us) const
  {
    return toRosMicros(px4_us) * 1000;
  }

private:
  double alpha_{0.05};
  double reanchor_jump_us_{kReanchorJumpUs};
  double offset_us_{0.0};
  bool valid_{false};
};

}  // namespace agc_air
