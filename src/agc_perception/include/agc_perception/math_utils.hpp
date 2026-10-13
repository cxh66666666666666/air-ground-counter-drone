// agc_perception 内部数学工具（W4 共享）
//
// 仅服务于本包内节点（gt_bypass / ground_odom_adapter / gt_adapter / air_perception /
// air_fusion），不导出为公共接口。四元数 / 偏航 / 角度归一等纯函数。
#pragma once

#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>

namespace agc
{

inline double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

inline double normalizeAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

}  // namespace agc