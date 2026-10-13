// test_lqr_controller.cpp —— LQR 控制器插件测试
//
// 复核：①增益有限且正定方向正确；②无扰动下闭环收敛到参考（含前馈）。

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "agc_control/param_reader.hpp"

#include "agc_control/lqr_controller.hpp"
#include "agc_control/trajectory_generators.hpp"

using agc_control::LqrController;
using agc_control::Reference;
using agc_control::State;

class LqrControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owned_ = true;
    }
    node_ = std::make_shared<rclcpp::Node>("test_lqr_controller");
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

TEST_F(LqrControllerTest, GainsAreFiniteAndSane)
{
  LqrController c;
  c.configure(*reader_);
  const auto & K = c.gain();
  EXPECT_TRUE(K.allFinite());
  // Q / R 各轴同权 → 增益按轴解耦：对角（位置项 j、速度项 j+3）为正，跨轴耦合项为 0
  for (int i = 0; i < 3; ++i) {
    EXPECT_GT(K(i, i), 0.0) << "位置误差反馈应为正增益";
    EXPECT_GT(K(i, i + 3), 0.0) << "速度误差反馈应为正增益";
  }
  EXPECT_GT(K(0, 3), K(0, 0)) << "双积分器 LQR 的速度反馈强于位置反馈";
}

TEST_F(LqrControllerTest, HoverConvergesWithFeedforward)
{
  LqrController c;
  c.configure(*reader_);

  // 目标：保持 (0,0,2.5)
  Reference ref;
  ref.pos = Eigen::Vector3d(0.0, 0.0, 2.5);

  State s;
  s.pos = Eigen::Vector3d(3.0, -2.0, 4.0);  // 初始偏差

  const double dt = 0.02;
  for (int i = 0; i < 1500; ++i) {  // 30 s
    const Eigen::Vector3d a = c.computeAcceleration(s, ref);
    s.vel += a * dt;
    s.pos += s.vel * dt;
  }
  EXPECT_LT((s.pos - ref.pos).norm(), 0.05);
  EXPECT_LT(s.vel.norm(), 0.05);
}

TEST_F(LqrControllerTest, TracksFigure8WithSmallError)
{
  // 控制器 + 轨迹插件联调：仿真积分双积分器，评估跟踪误差 P95
  agc_control::Figure8Trajectory traj;
  traj.configure(*reader_);
  LqrController c;
  c.configure(*reader_);

  State s;
  traj.reset(s);
  s.pos = traj.sample(0.0).pos;

  const double dt = 0.02;
  std::vector<double> errs;
  for (double t = 0.0; t < 60.0; t += dt) {
    const auto ref = traj.sample(t);
    const Eigen::Vector3d a = c.computeAcceleration(s, ref);
    s.vel += a * dt;
    s.pos += s.vel * dt;
    errs.push_back((s.pos - ref.pos).norm());
  }
  std::sort(errs.begin(), errs.end());
  const double p95 = errs[static_cast<size_t>(0.95 * errs.size())];
  EXPECT_LT(p95, 0.15) << "真八字跟踪误差 P95 过大";
}
