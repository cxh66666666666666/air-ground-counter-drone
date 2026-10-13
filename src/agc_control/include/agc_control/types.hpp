// agc_control/types.hpp —— 通用控制层的输入 / 输出类型
//
// 世界系约定（契约 #namespace）：ENU / FLU（+x 东、+y 北、+z 上），单位 m、m/s、m/s²。
// 控制层**不含**机体系 / NED 语义——坐标换算属各消费者的适配层（air: px4_adapter）。

#pragma once

#include <Eigen/Core>

namespace agc_control
{

using Vector3 = Eigen::Vector3d;

/// 当前状态（世界系）：位置 / 速度
struct State
{
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
};

/// 参考轨迹（世界系）：位置 / 速度 / 加速度
struct Reference
{
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
};

}  // namespace agc_control
