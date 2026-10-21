// test_target_motion_filter.cpp —— ADR-0079（B2）：目标运动估计单测
//
// 覆盖四类失效模式：
//   1. 静止目标 + 20 Hz 位置噪声 → 速度必须置零（不得把噪声放大成假速度）；
//   2. 匀速目标 → 速度估计收敛到真值；
//   3. 位置跳变（复位 / 鬼影）→ 新息门重置，不把跳变融进速度；
//   4. 提前量上限 → 异常速度不得把瞄准点甩出网半径。

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "agc_task/target_motion_filter.hpp"

using agc_task::TargetMotionFilter;
using agc_task::Vec3d;
using agc_task::norm3;

namespace
{
constexpr double kDt = 0.05;   // 20 Hz

double dist(const Vec3d & a, const Vec3d & b)
{
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                   (a.z - b.z) * (a.z - b.z));
}
}  // namespace

TEST(TargetMotionFilter, StaticNoisyTargetZeroVelocity)
{
  TargetMotionFilter f;
  std::mt19937 gen(42);
  std::normal_distribution<double> noise(0.0, 0.1);
  const Vec3d truth{4.5, 3.0, 2.0};

  for (int i = 0; i <= 80; ++i) {
    f.update(i * kDt, Vec3d{truth.x + noise(gen), truth.y + noise(gen), truth.z + noise(gen)});
  }

  ASSERT_TRUE(f.initialized());
  ASSERT_TRUE(f.settled());
  // 静止阈值（0.15 m/s）：噪声不得被放大成假速度。
  EXPECT_LT(norm3(f.velocity()), 0.15);
  // 1.3 s 飞行时间下，预测点仍应贴近静止目标。
  EXPECT_LT(dist(f.predicted(1.3), truth), 0.3);
}

TEST(TargetMotionFilter, LinearTargetVelocityConverges)
{
  TargetMotionFilter f;
  std::mt19937 gen(7);
  std::normal_distribution<double> noise(0.0, 0.05);
  const Vec3d start{0.0, 0.0, 2.0};
  const Vec3d vel{0.5, 0.2, 0.0};

  for (int i = 0; i <= 100; ++i) {
    const double t = i * kDt;
    f.update(t, Vec3d{
      start.x + vel.x * t + noise(gen),
      start.y + vel.y * t + noise(gen),
      start.z + noise(gen)});
  }

  ASSERT_TRUE(f.settled());
  EXPECT_NEAR(f.velocity().x, 0.5, 0.12);
  EXPECT_NEAR(f.velocity().y, 0.2, 0.12);
  EXPECT_LT(std::fabs(f.velocity().z), 0.12);
}

TEST(TargetMotionFilter, OutlierJumpResetsFilter)
{
  TargetMotionFilter f;
  const Vec3d a{4.5, 3.0, 2.0};
  for (int i = 0; i <= 40; ++i) {f.update(i * kDt, a);}
  ASSERT_TRUE(f.settled());

  // 单帧 5 m 跳变 → 新息门重置，不得把跳变融进速度。
  f.update(41 * kDt, Vec3d{a.x + 5.0, a.y, a.z});
  EXPECT_FALSE(f.settled());
  EXPECT_LT(norm3(f.velocity()), 1e-9);

  const Vec3d b{a.x + 5.0, a.y, a.z};
  for (int i = 42; i <= 90; ++i) {f.update(i * kDt, b);}
  EXPECT_TRUE(f.settled());
  EXPECT_LT(dist(f.position(), b), 0.3);
  EXPECT_LT(norm3(f.velocity()), 0.15);
}

TEST(TargetMotionFilter, LeadIsClamped)
{
  TargetMotionFilter f;
  const Vec3d start{0.0, 0.0, 2.0};
  const Vec3d vel{5.0, 0.0, 0.0};   // 快速目标：2 s 提前量会超过上限
  for (int i = 0; i <= 80; ++i) {
    const double t = i * kDt;
    f.update(t, Vec3d{start.x + vel.x * t, start.y, start.z});
  }
  ASSERT_TRUE(f.settled());
  const Vec3d pred = f.predicted(2.0);
  EXPECT_LE(norm3(Vec3d{pred.x - f.position().x, pred.y - f.position().y,
      pred.z - f.position().z}), 1.5 + 1e-9);
}
