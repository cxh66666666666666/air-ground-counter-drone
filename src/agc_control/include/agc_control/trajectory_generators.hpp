// agc_control/trajectory_generators.hpp —— 轨迹生成器插件（pluginlib 具体实现）
//
// 插件清单（spec R3）：linear / circular / helical / figure8 / hover / step。
// 其中 **figure8 为真八字（Gerono）**：x = cx + rx·sinθ、y = cy + ry·sinθ·cosθ，
// rx=15 / ry=9 / h=2.5 m，**全程恒速 v=2.0 m/s（弧长参数化，无加减速）**（契约 #hard-constraints C1、ADR-0019）。
// 周期轨迹（circular/helical/figure8）统一走弧长参数化：θ(t) 由 s(t)=v·t 反解，保证 |dp/dt| ≡ v。

#pragma once

#include <vector>

#include <Eigen/Core>

#include "agc_control/trajectory_generator.hpp"

namespace agc_control
{

/// 参数曲线轨迹基类：子类只给几何，基类负责弧长参数化与恒速输出。
class ParametricTrajectory : public TrajectoryGenerator
{
public:
  void configure(ParamReader & params) override;
  void reset(const State & initial, double yaw = 0.0) override;
  Reference sample(double t) const override;

  bool is_periodic() const override {return true;}
  double period() const override
  {
    if (s_table_.size() < 2) {buildArcTable();}
    return length_ / speed_;
  }

protected:
  /// 形状（世界系，含 center / height）
  virtual Eigen::Vector3d position(double theta) const = 0;
  virtual Eigen::Vector3d dp_dtheta(double theta) const = 0;
  virtual Eigen::Vector3d d2p_dtheta2(double theta) const = 0;

  double speed_{2.0};                 // 恒速 (m/s)
  double height_{2.5};                // 巡航高度 (m)
  Eigen::Vector3d center_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d start_offset_{Eigen::Vector3d::Zero()};  // reset 时的起点修正（保持世界系连续）

private:
  void buildArcTable() const;
  double thetaAt(double s) const;
  double sigma(double theta) const;          // |dp/dθ|
  double dsigma_dtheta(double theta) const;  // d|dp/dθ|/dθ

  mutable std::vector<double> s_table_;  // 长度 N_+1，s_table_.back() == length_
  mutable double length_{0.0};
  int samples_{4096};
};

/// 直线（非周期）：自起点沿方向 `heading_deg` 直行 `length_m`，到点后保持。
class LinearTrajectory : public TrajectoryGenerator
{
public:
  void configure(ParamReader & params) override;
  void reset(const State & initial, double yaw = 0.0) override;
  Reference sample(double t) const override;
  bool is_periodic() const override {return false;}
  double period() const override {return 0.0;}
  std::string name() const override {return "linear";}

private:
  double speed_{2.0};
  double heading_deg_{0.0};
  double length_m_{20.0};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d dir_{Eigen::Vector3d::UnitX()};
};

/// 圆周（周期）：中心 `center`、半径 `radius`、恒速，高度 `height`。
class CircularTrajectory : public ParametricTrajectory
{
public:
  void configure(ParamReader & params) override;
  std::string name() const override {return "circular";}

protected:
  Eigen::Vector3d position(double theta) const override;
  Eigen::Vector3d dp_dtheta(double theta) const override;
  Eigen::Vector3d d2p_dtheta2(double theta) const override;

private:
  double radius_{10.0};
};

/// 螺旋（周期）：半径 `radius`、每圈爬升 `pitch`，恒速，高度 `height` 为起始高度。
class HelicalTrajectory : public ParametricTrajectory
{
public:
  void configure(ParamReader & params) override;
  std::string name() const override {return "helical";}

protected:
  Eigen::Vector3d position(double theta) const override;
  Eigen::Vector3d dp_dtheta(double theta) const override;
  Eigen::Vector3d d2p_dtheta2(double theta) const override;

private:
  double radius_{10.0};
  double pitch_{2.0};  // 每 2π 的 z 增量
};

/// 真八字 Gerono（周期）：x = cx + rx·sinθ、y = cy + ry·sinθ·cosθ（自交成 ∞）。C1 载体。
class Figure8Trajectory : public ParametricTrajectory
{
public:
  void configure(ParamReader & params) override;
  std::string name() const override {return "figure8";}

protected:
  Eigen::Vector3d position(double theta) const override;
  Eigen::Vector3d dp_dtheta(double theta) const override;
  Eigen::Vector3d d2p_dtheta2(double theta) const override;

private:
  double rx_{15.0};
  double ry_{9.0};
};

/// 悬停（非周期）：保持 reset 时刻的位置（可选 `hold_z` 指定绝对高度）。
class HoverTrajectory : public TrajectoryGenerator
{
public:
  void configure(ParamReader & params) override;
  void reset(const State & initial, double yaw = 0.0) override;
  Reference sample(double t) const override;
  bool is_periodic() const override {return false;}
  double period() const override {return 0.0;}
  std::string name() const override {return "hover";}

private:
  bool use_hold_z_{false};
  double hold_z_{2.5};
  Eigen::Vector3d hold_{Eigen::Vector3d::Zero()};
};

/// 阶跃（非周期）：自起点以**最小 jerk** 平滑过渡到 `step_x/y/z`（运动学可行，无速度跳变）。
class StepTrajectory : public TrajectoryGenerator
{
public:
  void configure(ParamReader & params) override;
  void reset(const State & initial, double yaw = 0.0) override;
  Reference sample(double t) const override;
  bool is_periodic() const override {return false;}
  double period() const override {return duration_;}
  std::string name() const override {return "step";}

private:
  double step_x_{0.0}, step_y_{0.0}, step_z_{0.0};
  double duration_{5.0};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
};

}  // namespace agc_control
