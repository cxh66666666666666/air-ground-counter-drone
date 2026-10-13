// test_trajectory_plugins.cpp —— 轨迹生成器插件测试
//
// 重点复核 C1：真八字（Gerono）全程恒速、无加减速，几何为自交 ∞ 形。

#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "agc_control/param_reader.hpp"

#include "agc_control/trajectory_generators.hpp"

using agc_control::CircularTrajectory;
using agc_control::Figure8Trajectory;
using agc_control::HoverTrajectory;
using agc_control::State;

class TrajectoryPluginsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owned_ = true;
    }
    node_ = std::make_shared<rclcpp::Node>("test_trajectory_plugins");
    reader_ = std::make_shared<agc_control::NodeParamReader>(
      node_->get_node_parameters_interface(), "test");
  }
  void TearDown() override
  {
    node_.reset();
    if (owned_ && rclcpp::ok()) {rclcpp::shutdown();}
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<agc_control::NodeParamReader> reader_;
  bool owned_{false};
};

TEST_F(TrajectoryPluginsTest, Figure8IsConstantSpeed)
{
  Figure8Trajectory traj;
  traj.configure(*reader_);
  State s;
  traj.reset(s);

  // 以 1 ms 步长扫过一个完整周期，校验 |v| ≡ 2.0
  const double period = traj.period();
  ASSERT_GT(period, 1.0);
  double max_dev = 0.0;
  double max_acc = 0.0;
  for (double t = 0.0; t < period; t += 0.001) {
    const auto r = traj.sample(t);
    max_dev = std::max(max_dev, std::abs(r.vel.norm() - 2.0));
    max_acc = std::max(max_acc, r.acc.norm());
  }
  EXPECT_LT(max_dev, 1e-3) << "|v| 应恒为 2.0 m/s（C1）";
  EXPECT_GT(max_acc, 0.0) << "八字轨迹存在法向加速度（曲率非零）";
}

TEST_F(TrajectoryPluginsTest, Figure8GeometrySpansAndSelfIntersects)
{
  Figure8Trajectory traj;
  traj.configure(*reader_);
  State s;
  traj.reset(s);

  const double period = traj.period();
  double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
  int at_origin = 0;
  for (double t = 0.0; t < period; t += 0.001) {
    const auto r = traj.sample(t);
    min_x = std::min(min_x, r.pos.x());
    max_x = std::max(max_x, r.pos.x());
    min_y = std::min(min_y, r.pos.y());
    max_y = std::max(max_y, r.pos.y());
    if (std::hypot(r.pos.x(), r.pos.y()) < 1e-3) {++at_origin;}
    EXPECT_NEAR(r.pos.z(), 2.5, 1e-9);
  }
  EXPECT_NEAR(min_x, -15.0, 1e-2);
  EXPECT_NEAR(max_x, 15.0, 1e-2);
  EXPECT_NEAR(min_y, -4.5, 1e-2);   // (ry/2)·sin2θ 幅值
  EXPECT_NEAR(max_y, 4.5, 1e-2);
  EXPECT_GE(at_origin, 3) << "自交点（θ=0、π、2π）应被多次经过";
}

TEST_F(TrajectoryPluginsTest, CircularIsConstantSpeed)
{
  CircularTrajectory traj;
  traj.configure(*reader_);
  State s;
  traj.reset(s);

  const double period = traj.period();
  EXPECT_NEAR(period, 2.0 * M_PI * 10.0 / 2.0, 1e-3);  // 2πr / v
  double max_dev = 0.0;
  for (double t = 0.0; t < period; t += 0.002) {
    max_dev = std::max(max_dev, std::abs(traj.sample(t).vel.norm() - 2.0));
  }
  EXPECT_LT(max_dev, 1e-3);
}

TEST_F(TrajectoryPluginsTest, HoverHoldsStart)
{
  HoverTrajectory traj;
  traj.configure(*reader_);
  State s;
  s.pos = Eigen::Vector3d(3.0, -4.0, 2.5);
  traj.reset(s);
  const auto r = traj.sample(123.0);
  EXPECT_TRUE(r.pos.isApprox(s.pos));
  EXPECT_TRUE(r.vel.isZero());
}
