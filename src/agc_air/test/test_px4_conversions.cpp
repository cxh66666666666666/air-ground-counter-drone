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
  const auto q = agc_air::px4AttitudeToEnu({0.0f, 0.0f, 0.0f, 1.0f});
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
