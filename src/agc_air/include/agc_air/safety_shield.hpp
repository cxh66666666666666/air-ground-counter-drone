// agc_air/safety_shield.hpp —— 最小机载安全层（spec R7，C10 自保）
//
// 职责（不引入 Nav2 运行时进程；ADR-0003 空中域保留 LQR + 自研避障）：
//   ① 斥力式避让静态障碍（pole_*）：参考点不侵入障碍安全半径；
//   ② 对目标保持最小间距（安全间距的兜底，与 standoff 的距离带叠加）；
//   ③ 预留 RL 偏移接口（默认零偏，W7 填充；对齐旧 `/rl_planner/local_offset` 的"局部偏移"语义）。

#pragma once

#include <string>
#include <vector>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "agc_control/types.hpp"

namespace agc_air
{

struct StaticObstacle
{
  std::string name;
  double x{0.0};
  double y{0.0};
  double safety_radius{2.0};
};

class SafetyShield
{
public:
  /// 读参数（障碍表以**平行数组**给出）；由 launch 单一设置
  void configure_parameters(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params,
    const std::string & ns);

  /// 对期望参考施加安全修正：
  ///   - `desired`：期望参考（世界系 ENU）
  ///   - `target_pos` / `target_valid`：目标位置（用于最小间距兜底）
  ///
  /// 注：本层是**参考滤波器**，其输出由 LQR 跟踪，故机体实际位置相对参考存在跟踪滞后。
  /// `obstacle_margin` / `target_margin` 即为该滞后预留的余量：修正后的参考边界 =
  /// 安全半径 + margin，使**机体实际**不侵入真实安全半径（对齐契约 C6「安全滤波器参数
  /// 须用真实逃逸能力」的精神）。两者分开取值：障碍是**硬安全**判据，余量取足；
  /// 对目标的 `target_min_dist` 是带兜底，余量只需覆盖实测滞后。
  agc_control::Reference apply(const agc_control::Reference & desired,
    const agc_control::Vector3 & target_pos, bool target_valid) const;

  const std::vector<StaticObstacle> & obstacles() const {return obstacles_;}
  bool enabled() const {return enabled_;}
  double targetMinDist() const {return target_min_dist_;}
  double obstacleMargin() const {return obstacle_margin_m_;}
  double targetMargin() const {return target_margin_m_;}

private:
  std::vector<StaticObstacle> obstacles_;
  bool enabled_{true};
  double target_min_dist_{8.0};
  double obstacle_margin_m_{2.5};
  double target_margin_m_{1.5};
};

}  // namespace agc_air
