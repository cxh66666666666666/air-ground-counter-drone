// agc_control/trajectory_generator.hpp —— 轨迹生成器插件基类（pluginlib）
//
// 形态对齐 Nav2 的 controller/planner 插件抽象（ADR-0008 吸收点：pluginlib）：
//   configure(节点, 插件实例名) 声明并读取自己的参数 → reset(起始态) → sample(t) 取参考。
// `t` 为**自 reset 起的相对时间**（s）；周期轨迹自动按周期取模（C5 相位复位即 reset）。

#pragma once

#include <memory>
#include <string>

#include "agc_control/param_reader.hpp"
#include "agc_control/types.hpp"

namespace agc_control
{

class TrajectoryGenerator
{
public:
  using Ptr = std::shared_ptr<TrajectoryGenerator>;

  virtual ~TrajectoryGenerator() = default;

  /// 读取插件参数（参数由 launch 单一设置，契约 #params）。
  virtual void configure(ParamReader & params) = 0;

  /// 相位复位：`t=0` 对应该起始态（C5）。后续 sample() 以本次 reset 为基准。
  virtual void reset(const State & initial, double yaw = 0.0) = 0;

  /// 取 `t` 时刻的参考（世界系）。
  virtual Reference sample(double t) const = 0;

  /// 是否周期轨迹；周期轨迹的 `period()` 为环绕一周的时长（s）。
  virtual bool is_periodic() const = 0;
  virtual double period() const = 0;

  virtual std::string name() const = 0;
};

}  // namespace agc_control
