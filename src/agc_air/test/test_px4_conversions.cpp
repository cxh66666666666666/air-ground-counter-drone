// test_px4_conversions.cpp —— uORB 语义映射单测（R2 判据）

#include <cmath>
#include <array>

#include <gtest/gtest.h>

#include "agc_air/px4_conversions.hpp"

using agc_air::Px4TimeMapper;
using agc_air::Vector3;

TEST(Px4Conversions, NedToEnu)
{
  // NED: 北 3、东 4、下 5 → ENU: 东 4、北 3、上 −5
  const Vector3 enu = agc_air::nedToEnu(Vector3(3.0, 4.0, 5.0));
  EXPECT_DOUBLE_EQ(enu.x(), 4.0);
  EXPECT_DOUBLE_EQ(enu.y(), 3.0);
  EXPECT_DOUBLE_EQ(enu.z(), -5.0);
}

TEST(Px4Conversions, FrdToFlu)
{
  const Vector3 flu = agc_air::frdToFlu(Vector3(1.0, 2.0, 3.0));
  EXPECT_DOUBLE_EQ(flu.x(), 1.0);
  EXPECT_DOUBLE_EQ(flu.y(), -2.0);
  EXPECT_DOUBLE_EQ(flu.z(), -3.0);
}

TEST(Px4Conversions, AttitudeIdentityIsLevelFacingNorth)
{
  // PX4 单位四元数 = 机体 FRD 水平、机头朝北；换到 FLU/ENU 后应仍水平，且机头指向 +y（北）
  // 字段序 = PX4 Hamilton **[w,x,y,z]**（单位阵 = w=1）
  const auto q = agc_air::px4AttitudeToEnu({1.0f, 0.0f, 0.0f, 0.0f});
  const Eigen::Matrix3d R = q.toRotationMatrix();
  // 机体系前向 (1,0,0) → 世界系
  const Eigen::Vector3d fwd = R * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(fwd.x(), 0.0, 1e-9);   // 东分量 0
  EXPECT_NEAR(fwd.y(), 1.0, 1e-9);   // 指向北
  EXPECT_NEAR(fwd.z(), 0.0, 1e-9);   // 无俯仰
  // 机体系上向 (0,0,1) → 世界系应为 +z（上）
  const Eigen::Vector3d up = R * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(up.z(), 1.0, 1e-9);
}

TEST(Px4Conversions, AttitudeFieldOrderIsPx4HamiltonWxyz)
{
  // W7-b 回归：PX4 q 为 Hamilton **[w,x,y,z]**。取 NED 偏航 58.1° 的纯偏航四元数
  // （≈ 实测 raw q=[0.877,0.004,0.002,0.480]）→ ENU 偏航应为 90°−58.1°=31.9°。
  // 若误按 ROS [x,y,z,w] 解析，会得到 ≈90°（正是复现的 ~58° 姿态错误）。
  const double y = 58.1 * M_PI / 180.0;
  const auto q = agc_air::px4AttitudeToEnu({static_cast<float>(std::cos(y / 2.0)), 0.0f, 0.0f,
      static_cast<float>(std::sin(y / 2.0))});
  // 输入为 float（与 PX4 一致），故容差按度取 1e-3（远小于“字段序错”时的 ~58° 偏差）
  EXPECT_NEAR(agc_air::yawFromQuaternion(q) * 180.0 / M_PI, 90.0 - 58.1, 1e-3);
}

TEST(Px4Conversions, YawRoundTrip)
{
  // ENU 偏航 0（指东）→ NED 偏航 π/2，再转回 0
  EXPECT_NEAR(agc_air::enuYawToPx4(0.0), M_PI_2, 1e-9);
  EXPECT_NEAR(agc_air::px4YawToEnu(M_PI_2), 0.0, 1e-9);
  EXPECT_NEAR(agc_air::px4YawToEnu(agc_air::enuYawToPx4(1.234)), 1.234, 1e-9);
}

TEST(Px4Conversions, YawFromQuaternionMatchesEnuConvention)
{
  // 绕 ENU +z 转 ψ → yawFromQuaternion 应还原 ψ
  const double psi = 0.7;
  const agc_air::Quaternion q(Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(agc_air::yawFromQuaternion(q), psi, 1e-9);
}

TEST(Px4Conversions, TimeMapperConvergesToOffset)
{
  Px4TimeMapper mapper(0.5);
  // 真实偏移 = +1_000_000 µs（ROS 比 PX4 快 1 s）
  for (int i = 0; i < 200; ++i) {
    const std::uint64_t px4 = 5'000'000 + i * 20'000;
    mapper.observe(px4, static_cast<std::int64_t>(px4) + 1'000'000);
  }
  ASSERT_TRUE(mapper.valid());
  EXPECT_NEAR(mapper.offsetUs(), 1'000'000.0, 1.0);
  EXPECT_EQ(mapper.toRosMicros(6'000'000), 7'000'000);
}

TEST(Px4Conversions, TimeMapperRosStampIsNanoseconds)
{
  // R-002 回归：适配层把映射结果交给 rclcpp::Time(单整数)，该构造器按**纳秒**解释。
  // 仅断言 toRosMicros（µs）不足以发现"数值对、单位错"——这里断言送进 Time 的数值是 ns：
  // 7 s 必须是 7e9；误用 µs 会得到 7e6（= 0.007 s，Header 缩小 1000 倍）。
  Px4TimeMapper mapper(0.5);
  mapper.observe(6'000'000, 7'000'000);   // PX4 6 s ↔ ROS 7 s（µs 对）
  ASSERT_TRUE(mapper.valid());
  EXPECT_EQ(mapper.toRosNanoseconds(6'000'000), 7'000'000'000LL);
  EXPECT_DOUBLE_EQ(
    static_cast<double>(mapper.toRosNanoseconds(6'000'000)) * 1e-9, 7.0);
}

TEST(Px4Conversions, LocalNedToWorldEnuAppliesOrigin)
{
  // R-003：非零出生点。局部 NED (2,3,-1) → ENU (3,2,1)，再叠加原点 (10,-5,0.2)
  const Vector3 world = agc_air::localNedToWorldEnu(Vector3(2.0, 3.0, -1.0),
    Vector3(10.0, -5.0, 0.2));
  EXPECT_DOUBLE_EQ(world.x(), 13.0);
  EXPECT_DOUBLE_EQ(world.y(), -3.0);
  EXPECT_DOUBLE_EQ(world.z(), 1.2);
  // 零原点退化为纯轴变换（与旧行为一致）
  const Vector3 flat = agc_air::localNedToWorldEnu(Vector3(2.0, 3.0, -1.0), Vector3::Zero());
  EXPECT_DOUBLE_EQ(flat.x(), 3.0);
  EXPECT_DOUBLE_EQ(flat.y(), 2.0);
  EXPECT_DOUBLE_EQ(flat.z(), 1.0);
}

TEST(Px4Conversions, VelocityFrameConversionAtNinetyDegreeHeading)
{
  // 90° 航向：NED 偏航 0（朝北）的单位四元数 → ENU 下机头即 +y（北）。字段序 [w,x,y,z]。
  const auto q = agc_air::px4AttitudeToEnu({1.0f, 0.0f, 0.0f, 0.0f});
  // 世界 ENU 向北 1 m/s → 机体系前向 x = 1（R-003：twist 在 child frame）
  const Vector3 north_body = agc_air::enuToBodyFlu(q, Vector3(0.0, 1.0, 0.0));
  EXPECT_NEAR(north_body.x(), 1.0, 1e-9);
  EXPECT_NEAR(north_body.y(), 0.0, 1e-9);
  EXPECT_NEAR(north_body.z(), 0.0, 1e-9);
  // 世界 ENU 向东 1 m/s → 机体系右向（-y）
  const Vector3 east_body = agc_air::enuToBodyFlu(q, Vector3(1.0, 0.0, 0.0));
  EXPECT_NEAR(east_body.x(), 0.0, 1e-9);
  EXPECT_NEAR(east_body.y(), -1.0, 1e-9);
  // 往返还原（消费端 bodyFluToEnu）
  const Vector3 back = agc_air::bodyFluToEnu(q, north_body);
  EXPECT_NEAR(back.x(), 0.0, 1e-9);
  EXPECT_NEAR(back.y(), 1.0, 1e-9);
}

TEST(Px4Conversions, ParseSpawnPositionXyz)
{
  Vector3 p;
  ASSERT_TRUE(agc_air::parseSpawnPositionXyz("10,-5,0.2,0,0,1.57", p));
  EXPECT_DOUBLE_EQ(p.x(), 10.0);
  EXPECT_DOUBLE_EQ(p.y(), -5.0);
  EXPECT_DOUBLE_EQ(p.z(), 0.2);
  ASSERT_TRUE(agc_air::parseSpawnPositionXyz("1,2,3", p));   // 仅位置三位亦接受
  EXPECT_DOUBLE_EQ(p.z(), 3.0);
  EXPECT_FALSE(agc_air::parseSpawnPositionXyz("1,2", p));    // 位数不足
  EXPECT_FALSE(agc_air::parseSpawnPositionXyz("a,b,c", p));  // 非数值
}
