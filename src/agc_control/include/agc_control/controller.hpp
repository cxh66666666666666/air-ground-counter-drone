// agc_control/controller.hpp —— 控制器插件基类（pluginlib）
//
// 形态对齐 Nav2 的 controller 插件抽象（ADR-0008 吸收点：pluginlib）。
// 通用接口与执行器解耦：入参 = state + ref，出参 = **加速度指令** acc_cmd（世界系 ENU/FLU）。
// 各消费者自行转执行量：air → PX4 位置/速度 setpoint；ground → cmd_vel（ADR-0018）。

#pragma once

#include <memory>
#include <string>

#include "agc_control/param_reader.hpp"
#include "agc_control/types.hpp"

namespace agc_control
{

class Controller
{
public:
  using Ptr = std::shared_ptr<Controller>;

  virtual ~Controller() = default;

  /// 读取插件参数（含 LQR 权重与离散化步长），并求解控制增益。
  virtual void configure(ParamReader & params) = 0;

  /// 清空内部状态（与轨迹 reset 同步调用）
  virtual void reset() = 0;

  /// 计算加速度指令（世界系）。ref.acc 为前馈项，实现**应当**叠加。
  virtual Eigen::Vector3d computeAcceleration(const State & state, const Reference & ref) = 0;

  virtual std::string name() const = 0;
};

}  // namespace agc_control
