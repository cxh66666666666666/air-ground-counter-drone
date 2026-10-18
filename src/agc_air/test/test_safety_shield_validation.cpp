// test_safety_shield_validation.cpp —— R-038 安全层障碍配置**失败型**验证
//
// 只验证**失败型行为**：平行数组长度不一致 / 非法几何被**拒绝**（不静默按最短长度截断）+ 合法几何通过。
// **不**复制安全层投影 / 避障逻辑自证（无镜像实现）。

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "agc_air/safety_shield.hpp"

using agc_air::SafetyShield;

class SafetyShieldValidationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owned_ = true;
    }
    node_ = std::make_shared<rclcpp::Node>("test_safety_shield_validation");
  }
  void TearDown() override
  {
    node_.reset();
    if (owned_ && rclcpp::ok()) {rclcpp::shutdown();}
  }
  void setNames(const std::vector<std::string> & v)
  {
    node_->declare_parameter("safety.obstacle_names", v);
  }
  void setArr(const std::string & name, const std::vector<double> & v)
  {
    node_->declare_parameter("safety." + name, v);
  }
  SafetyShield shield()
  {
    SafetyShield s;
    s.configure_parameters(node_->get_node_parameters_interface(), "safety");
    return s;
  }
  rclcpp::Node::SharedPtr node_;
  bool owned_{false};
};

// (c) 平行数组长度不一致 → **检出并拒绝**（旧实现 n=min(...) 静默按最短长度截断 → 漏障碍）
TEST_F(SafetyShieldValidationTest, RejectsMismatchedParallelArrayLengths)
{
  setNames({"pole_1", "pole_2"});
  setArr("obstacle_x", {0.0});                 // 长度 1 ≠ 2
  setArr("obstacle_y", {-6.0, 6.0});
  setArr("obstacle_safety_radius", {2.0, 2.0});
  SafetyShield s;
  EXPECT_THROW(
    s.configure_parameters(node_->get_node_parameters_interface(), "safety"),
    std::runtime_error);
}

// 几何非法：半径 ≤ 0 → 拒绝
TEST_F(SafetyShieldValidationTest, RejectsNonPositiveRadius)
{
  setNames({"pole_1"});
  setArr("obstacle_x", {0.0});
  setArr("obstacle_y", {-6.0});
  setArr("obstacle_safety_radius", {0.0});
  SafetyShield s;
  EXPECT_THROW(
    s.configure_parameters(node_->get_node_parameters_interface(), "safety"),
    std::runtime_error);
}

// 几何非法：坐标非有限（NaN）→ 拒绝
TEST_F(SafetyShieldValidationTest, RejectsNonFiniteCoordinate)
{
  setNames({"pole_1"});
  setArr("obstacle_x", {std::numeric_limits<double>::quiet_NaN()});
  setArr("obstacle_y", {-6.0});
  setArr("obstacle_safety_radius", {2.0});
  SafetyShield s;
  EXPECT_THROW(
    s.configure_parameters(node_->get_node_parameters_interface(), "safety"),
    std::runtime_error);
}

// 合法几何：长度一致且半径 > 0、坐标有限 → 通过
TEST_F(SafetyShieldValidationTest, AcceptsValidGeometry)
{
  setNames({"pole_1", "pole_2"});
  setArr("obstacle_x", {0.0, 6.0});
  setArr("obstacle_y", {-6.0, 6.0});
  setArr("obstacle_safety_radius", {2.0, 2.0});
  SafetyShield s;
  ASSERT_NO_THROW(s.configure_parameters(node_->get_node_parameters_interface(), "safety"));
  EXPECT_EQ(s.obstacles().size(), 2u);
}
