// test_lqr_validation.cpp —— R-038 LQR 配置合法性 / DARE 收敛**失败型**验证
//
// 只验证**失败型行为**（非法配置被拒绝、未收敛被报告）与只读诊断暴露；
// **不**复制 DARE / 增益求解逻辑自证（无镜像实现）。

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "agc_control/lqr_controller.hpp"
#include "agc_control/param_reader.hpp"

using agc_control::LqrController;
using agc_control::NodeParamReader;

class LqrValidationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owned_ = true;
    }
    node_ = std::make_shared<rclcpp::Node>("test_lqr_validation");
    reader_ = std::make_shared<NodeParamReader>(node_->get_node_parameters_interface(), "v");
  }
  void TearDown() override
  {
    node_.reset();
    if (owned_ && rclcpp::ok()) {rclcpp::shutdown();}
  }
  void setD(const std::string & name, double v)
  {
    const std::string key = "v." + name;
    if (!node_->has_parameter(key)) {node_->declare_parameter(key, v);}
  }
  void setI(const std::string & name, int v)
  {
    const std::string key = "v." + name;
    if (!node_->has_parameter(key)) {node_->declare_parameter(key, static_cast<std::int64_t>(v));}
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<NodeParamReader> reader_;
  bool owned_{false};
};

// (a) 非法 Q/R/dt（≤0 / 非有限）**被拒绝**（非静默兜底 —— 旧实现会 dt<=0→0.02、r<=0→1.0）
TEST_F(LqrValidationTest, RejectsNonPositiveDtNotSilentlyReset)
{
  setD("dt", 0.0);            // 旧实现静默置回 0.02；现应拒绝
  LqrController c;
  EXPECT_THROW(c.configure(*reader_), std::runtime_error);
}

TEST_F(LqrValidationTest, RejectsNonPositiveRNotSilentlyReset)
{
  setD("r", -1.0);           // 旧实现静默置回 1.0；现应拒绝
  LqrController c;
  EXPECT_THROW(c.configure(*reader_), std::runtime_error);
}

TEST_F(LqrValidationTest, RejectsNonPositiveQPos)
{
  setD("q_pos", 0.0);
  LqrController c;
  EXPECT_THROW(c.configure(*reader_), std::runtime_error);
}

TEST_F(LqrValidationTest, RejectsNonPositiveQVel)
{
  setD("q_vel", -4.0);
  LqrController c;
  EXPECT_THROW(c.configure(*reader_), std::runtime_error);
}

TEST_F(LqrValidationTest, RejectsNonFiniteWeight)
{
  setD("q_pos", std::numeric_limits<double>::quiet_NaN());
  LqrController c;
  EXPECT_THROW(c.configure(*reader_), std::runtime_error);
}

TEST_F(LqrValidationTest, RejectsNonPositiveTol)
{
  setD("tol", 0.0);
  LqrController c;
  EXPECT_THROW(c.configure(*reader_), std::runtime_error);
}

TEST_F(LqrValidationTest, RejectsNonPositiveMaxIter)
{
  setI("max_iter", 0);
  LqrController c;
  EXPECT_THROW(c.configure(*reader_), std::runtime_error);
}

// (b) DARE 未收敛**被报告** → 明确失败（max_iter=1 远不足收敛 → 残差 ≥ tol）
TEST_F(LqrValidationTest, ReportsDareNonConvergenceAsExplicitFailure)
{
  setI("max_iter", 1);
  setD("tol", 1e-10);
  LqrController c;
  bool threw = false;
  try {
    c.configure(*reader_);
  } catch (const std::runtime_error & e) {
    threw = true;
    EXPECT_NE(std::string(e.what()).find("未收敛"), std::string::npos) << e.what();
  }
  EXPECT_TRUE(threw) << "DARE 未收敛应明确失败（不静默使用不可靠增益）";
}

// 合法配置：求解可行 + 只读诊断（收敛 / 残差 / 迭代 / dt / 适用频率）暴露
TEST_F(LqrValidationTest, ValidConfigConvergesAndExposesDiagnostics)
{
  setD("q_pos", 4.0);
  setD("q_vel", 4.0);
  setD("r", 1.0);
  setD("dt", 0.02);
  setD("tol", 1e-10);
  setI("max_iter", 2000);   // 实测 tol=1e-10 需约 559 次迭代（默认预算 2000）
  LqrController c;
  ASSERT_NO_THROW(c.configure(*reader_));
  EXPECT_TRUE(c.converged());
  EXPECT_LT(c.residual(), 1e-10);
  EXPECT_GT(c.iterations(), 0);
  EXPECT_DOUBLE_EQ(c.dt(), 0.02);
  EXPECT_NEAR(c.appliedFrequencyHz(), 50.0, 1e-9);
  EXPECT_TRUE(c.gain().allFinite());
}
