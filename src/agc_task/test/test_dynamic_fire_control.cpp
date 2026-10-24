// test_dynamic_fire_control.cpp —— ADR-0091：火控在飞生命周期确定性单测
//
// 显式 `t_s`（20 Hz）驱动状态机；不依赖 ROS / GZ / PX4 / 墙钟随机性。
// 修复前（bug-compatible）的 step() 把位姿 / 交战门提前返回放在 `if (in_flight_)` 之前，
// 目标失效 / 越界后网既不推进也不结算 → 生命周期用例为**红**；修复后为**绿**。
//
// 覆盖（T2.1）：目标过期 / 低置信度、越出上下限高度、越出水平射程、车辆位姿缺失、
// 落地结算、超时结算、冷却、复位中止、时钟回退、唯一终局、正常命中。

#include <cmath>

#include <gtest/gtest.h>

#include "agc_task/dynamic_fire_control.hpp"

namespace
{
constexpr double kDt = 0.05;        // 20 Hz 控制拍
constexpr double kRateTol = 1.0 / 20.0;   // 一个控制周期
constexpr double kTimeout = 3.0;    // 默认 net_flight_timeout_s

agc_task::Pose2 makePose(double x, double y, double z, bool valid = true)
{
  agc_task::Pose2 p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.valid = valid;
  return p;
}

bool isTerminal(agc_task::CaptureStatus s)
{
  return s == agc_task::CaptureStatus::kHit ||
         s == agc_task::CaptureStatus::kMiss ||
         s == agc_task::CaptureStatus::kAborted;
}

struct FireOutcome
{
  bool fired{false};
  double t_fire{0.0};
  int fire_i{-1};
};

// 以 allow_fire=true 逐拍推进到首次 SHOT_FIRED（静止目标 0.75 s 后运动估计 settled）。
FireOutcome runToFire(
  agc_task::DynamicFireControlAdjudicator & adj,
  const agc_task::Pose2 & car, const agc_task::Pose2 & target, double t_end = 1.0)
{
  FireOutcome r;
  for (int i = 0; i * kDt <= t_end + 1e-9; ++i) {
    const auto d = adj.step(car, target, i * kDt, true);
    if (d.status == agc_task::CaptureStatus::kShotFired) {
      r.fired = true;
      r.t_fire = d.t_fire;
      r.fire_i = i;
      return r;
    }
  }
  return r;
}

struct TerminalScan
{
  int count{0};
  int shots{0};
  agc_task::CaptureStatus status{agc_task::CaptureStatus::kInvalid};
  agc_task::CaptureDecision decision;
};

// 从 i0 拍到 i1 拍推进，统计终局（kHit / kMiss / kAborted）与发射次数。
TerminalScan runFrom(
  agc_task::DynamicFireControlAdjudicator & adj,
  const agc_task::Pose2 & car, const agc_task::Pose2 & target,
  int i0, int i1, bool allow_fire)
{
  TerminalScan scan;
  for (int i = i0; i <= i1; ++i) {
    const auto d = adj.step(car, target, i * kDt, allow_fire);
    if (isTerminal(d.status)) {
      ++scan.count;
      scan.status = d.status;
      scan.decision = d;
    }
    if (d.status == agc_task::CaptureStatus::kShotFired) {++scan.shots;}
  }
  return scan;
}
}  // namespace

// 红测（ADR-0085(i-1) 最小复现）：发射后目标失效并保持 → 修复前无终局。
TEST(DynamicFireControlLifecycle, TargetInvalidAfterFireStillSettles)
{
  agc_task::FireParams p;   // 默认物理参数（v0=6.5, θ=75°, d=1.0, cooldown=2.0, timeout=3.0）
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  const auto car = makePose(0.0, 0.0, 0.0);
  auto target = makePose(2.0, 0.0, 1.0);

  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired) << "静止目标在 0.75 s 后 settled，1 s 内应可发射";
  ASSERT_TRUE(adj.inFlight());

  target.valid = false;   // 目标失效（与「置信度低于 confidence_min」在节点内同义）
  const auto scan = runFrom(adj, car, target, fire.fire_i + 1, 120, false);

  EXPECT_EQ(scan.count, 1) << "在飞网必须在仿真时间窗内恰好结算一次";
  EXPECT_EQ(scan.status, agc_task::CaptureStatus::kMiss);
  EXPECT_LE(scan.decision.flight_s, kTimeout + kRateTol + 1e-6);
  EXPECT_FALSE(scan.decision.hit);
  EXPECT_FALSE(scan.decision.miss_dist_valid) << "从未获得有效目标观测时不得伪造最近距离";
}

// 发射后置信度下降：节点把 confidence < confidence_min 编码为 target.valid=false。
TEST(DynamicFireControlLifecycle, LowConfidenceAfterFireStillSettles)
{
  agc_task::FireParams p;
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  const auto car = makePose(0.0, 0.0, 0.0);
  auto target = makePose(2.0, 0.0, 1.0);
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);

  target.valid = false;   // 低置信度 → 位姿无效
  const auto scan = runFrom(adj, car, target, fire.fire_i + 1, 120, false);
  EXPECT_EQ(scan.count, 1);
  EXPECT_EQ(scan.status, agc_task::CaptureStatus::kMiss);
  EXPECT_FALSE(scan.decision.hit);
}

// 发射后目标越出 applicable_max_alt_m：交战门失败不得冻结在飞网。
TEST(DynamicFireControlLifecycle, TargetAboveMaxHeightAfterFireStillSettles)
{
  agc_task::FireParams p;
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  const auto car = makePose(0.0, 0.0, 0.0);
  auto target = makePose(2.0, 0.0, 1.0);
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);

  target.z = 5.0;   // h=5.0 > applicable_max=2.009
  const auto scan = runFrom(adj, car, target, fire.fire_i + 1, 120, false);
  EXPECT_EQ(scan.count, 1);
  EXPECT_EQ(scan.status, agc_task::CaptureStatus::kMiss);
  EXPECT_LE(scan.decision.flight_s, kTimeout + kRateTol + 1e-6);
  EXPECT_TRUE(scan.decision.miss_dist_valid) << "目标有效时最近距离应可用";
}

// 发射后目标低于车体（h<0）：同为适用高度门失败。
TEST(DynamicFireControlLifecycle, TargetBelowMinHeightAfterFireStillSettles)
{
  agc_task::FireParams p;
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  const auto car = makePose(0.0, 0.0, 0.0);
  auto target = makePose(2.0, 0.0, 1.0);
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);

  target.z = -1.0;   // h=-1.0 < applicable_min=0
  const auto scan = runFrom(adj, car, target, fire.fire_i + 1, 120, false);
  EXPECT_EQ(scan.count, 1);
  EXPECT_EQ(scan.status, agc_task::CaptureStatus::kMiss);
  EXPECT_LE(scan.decision.flight_s, kTimeout + kRateTol + 1e-6);
}

// 发射后目标越出水平射程：kOutOfEnvelope 不得冻结在飞网。
TEST(DynamicFireControlLifecycle, TargetOutOfHorizontalRangeAfterFireStillSettles)
{
  agc_task::FireParams p;
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  const auto car = makePose(0.0, 0.0, 0.0);
  auto target = makePose(2.0, 0.0, 1.0);
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);

  target.x = 10.0;   // dist=10 m 远超该高度闭式射程 + d/2
  const auto scan = runFrom(adj, car, target, fire.fire_i + 1, 120, false);
  EXPECT_EQ(scan.count, 1);
  EXPECT_EQ(scan.status, agc_task::CaptureStatus::kMiss);
  EXPECT_LE(scan.decision.flight_s, kTimeout + kRateTol + 1e-6);
}

// 发射后车辆位姿缺失：弹道由发射原点锁定，落地结算不受影响。
TEST(DynamicFireControlLifecycle, CarInvalidAfterFireStillSettles)
{
  agc_task::FireParams p;
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  // 目标在包络内但不在弹道近旁（(1,0,0.2) 处网最近距离 ≈0.86 m > 0.5 m 半径）。
  auto car = makePose(0.0, 0.0, 0.0);
  const auto target = makePose(1.0, 0.0, 0.2);
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);

  car.valid = false;
  const auto scan = runFrom(adj, car, target, fire.fire_i + 1, 120, false);
  EXPECT_EQ(scan.count, 1);
  EXPECT_EQ(scan.status, agc_task::CaptureStatus::kMiss);
  EXPECT_FALSE(scan.decision.hit);
  EXPECT_LE(scan.decision.flight_s, kTimeout + kRateTol + 1e-6);
}

// 落地路径：默认参数下 flight_s 接近弹道落地时间（≈1.30 s），且在超时上界内。
TEST(DynamicFireControlLifecycle, LandingSettlesAtBallisticTime)
{
  agc_task::FireParams p;
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  const auto car = makePose(0.0, 0.0, 0.0);
  auto target = makePose(2.0, 0.0, 1.0);
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);

  target.valid = false;
  const auto scan = runFrom(adj, car, target, fire.fire_i + 1, 120, false);
  ASSERT_EQ(scan.count, 1);
  EXPECT_EQ(scan.status, agc_task::CaptureStatus::kMiss);
  EXPECT_NEAR(scan.decision.flight_s, 1.30, kDt + 1e-6);
  EXPECT_LE(scan.decision.flight_s, kTimeout + kRateTol + 1e-6);
}

// 超时路径：net_flight_timeout_s 小于弹道落地时间 → 在超时拍结算 SHOT_MISS。
TEST(DynamicFireControlLifecycle, TimeoutSettlesAtTimeout)
{
  agc_task::FireParams p;
  p.flight_timeout_s = 0.5;   // 小于落地时间（≈1.28 s）
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  const auto car = makePose(0.0, 0.0, 0.0);
  auto target = makePose(2.0, 0.0, 1.0);
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);

  target.valid = false;
  const auto scan = runFrom(adj, car, target, fire.fire_i + 1, 120, false);
  ASSERT_EQ(scan.count, 1);
  EXPECT_EQ(scan.status, agc_task::CaptureStatus::kMiss);
  // 超时判据 t_s - t_fire_ > 0.5：首个满足的拍为 0.55 s。
  EXPECT_NEAR(scan.decision.flight_s, 0.55, 1e-6);
  EXPECT_LE(scan.decision.flight_s, p.flight_timeout_s + kRateTol + 1e-6);
}

// 冷却：结算后 cooldown_until 之前不得发射；冷却结束后且目标 settled 才可再次发射。
TEST(DynamicFireControlLifecycle, CooldownBlocksThenAllows)
{
  agc_task::FireParams p;
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  const auto car = makePose(0.0, 0.0, 0.0);
  const auto target = makePose(1.0, 0.0, 0.2);   // 落地失手（最近 ≈0.86 m）
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);
  EXPECT_EQ(adj.shotSeq(), 1);

  // 失手于 t=2.05 s（flight=1.30），cooldown_until=4.05 s。
  const auto before = runFrom(adj, car, target, fire.fire_i + 1, 80, true);   // t ≤ 4.00
  ASSERT_EQ(before.count, 1) << "应恰好一次失手结算";
  EXPECT_EQ(before.status, agc_task::CaptureStatus::kMiss);
  EXPECT_EQ(before.shots, 0) << "冷却期内不得再次发射";
  EXPECT_EQ(adj.shotSeq(), 1);

  const auto after = runFrom(adj, car, target, 81, 90, true);   // 4.05 – 4.50
  EXPECT_EQ(after.shots, 1) << "冷却结束后应允许再次发射";
  EXPECT_EQ(adj.shotSeq(), 2);
}

// 复位中止：在飞时 abortInFlight → SHOT_ABORTED reason=episode_reset；复位后旧 shot 无终局。
TEST(DynamicFireControlLifecycle, ResetMidFlightAborts)
{
  agc_task::FireParams p;
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  const auto car = makePose(0.0, 0.0, 0.0);
  const auto target = makePose(2.0, 0.0, 1.0);
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);
  const std::string old_shot = adj.shotId();

  const auto flying = runFrom(adj, car, target, fire.fire_i + 1, 20, false);   // 推进到 t=1.0
  EXPECT_EQ(flying.count, 0);
  EXPECT_TRUE(adj.inFlight());

  const double t_abort = 1.0;
  const auto ab = adj.abortInFlight("episode_reset", t_abort);
  EXPECT_EQ(ab.status, agc_task::CaptureStatus::kAborted);
  EXPECT_EQ(ab.shot_id, old_shot);
  EXPECT_EQ(ab.reason, "episode_reset");
  EXPECT_DOUBLE_EQ(ab.t_event, t_abort);
  EXPECT_NEAR(ab.flight_s, t_abort - fire.t_fire, 1e-9);
  EXPECT_FALSE(ab.hit);
  EXPECT_FALSE(adj.inFlight());

  // 非在飞时再中止 → 默认决策（kInvalid），不产生额外事件。
  const auto no_abort = adj.abortInFlight("episode_reset", t_abort + 1.0);
  EXPECT_EQ(no_abort.status, agc_task::CaptureStatus::kInvalid);

  adj.reset(2);
  const auto post = runFrom(adj, car, target, 21, 60, false);   // 新 episode 1.05–3.00 s
  EXPECT_EQ(post.count, 0) << "旧 shot 不得在新 episode 内结算";
  EXPECT_FALSE(adj.inFlight());
}

// 时钟回退：t_s < t_fire_ → SHOT_ABORTED reason=clock_regression；无无限在飞 / 负飞行时长。
TEST(DynamicFireControlLifecycle, ClockRegressionAborts)
{
  agc_task::FireParams p;
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  const auto car = makePose(0.0, 0.0, 0.0);
  const auto target = makePose(2.0, 0.0, 1.0);
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);
  const std::string old_shot = adj.shotId();

  const double t_back = fire.t_fire - 0.25;   // 仿真时钟回退
  const auto d = adj.step(car, target, t_back, true);
  EXPECT_EQ(d.status, agc_task::CaptureStatus::kAborted);
  EXPECT_EQ(d.reason, "clock_regression");
  EXPECT_EQ(d.shot_id, old_shot);
  EXPECT_DOUBLE_EQ(d.t_event, t_back);
  EXPECT_DOUBLE_EQ(d.flight_s, 0.0);   // 负飞行时长被夹到 0
  EXPECT_FALSE(adj.inFlight());

  const auto post = runFrom(adj, car, target, static_cast<int>(t_back / kDt), 120, false);
  EXPECT_EQ(post.count, 0) << "中止后不得再为同一 shot 产生终局";
}

// 唯一终局：终局后继续 step 不得重复同一 shot 的 SHOT_HIT / SHOT_MISS / SHOT_ABORTED。
TEST(DynamicFireControlLifecycle, NoDuplicateTerminal)
{
  agc_task::FireParams p;
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  const auto car = makePose(0.0, 0.0, 0.0);
  auto target = makePose(2.0, 0.0, 1.0);
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);
  const std::string old_shot = adj.shotId();

  target.valid = false;
  const auto first = runFrom(adj, car, target, fire.fire_i + 1, 120, false);
  ASSERT_EQ(first.count, 1);
  EXPECT_EQ(first.decision.shot_id, old_shot);

  const auto second = runFrom(adj, car, target, 121, 200, false);
  EXPECT_EQ(second.count, 0) << "同一 shot 不得重复终局";
  EXPECT_EQ(second.shots, 0);
  EXPECT_EQ(adj.shotSeq(), 1);
}

// 正常命中：目标放在弹道后方（dt≈1.2 s 与网重合），首个进入半径的拍判 SHOT_HIT。
TEST(DynamicFireControlLifecycle, HitOnTrajectory)
{
  agc_task::FireParams p;
  agc_task::DynamicFireControlAdjudicator adj(p);
  adj.reset(1);

  // 网水平速度 vh = v0·cos75°·drag ≈1.5148 m/s；dt=1.2 s 处 x≈1.8177, z≈0.4710。
  const auto car = makePose(0.0, 0.0, 0.0);
  const auto target = makePose(1.81775, 0.0, 0.47102);
  const auto fire = runToFire(adj, car, target, 1.0);
  ASSERT_TRUE(fire.fired);

  const auto scan = runFrom(adj, car, target, fire.fire_i + 1, 60, true);
  ASSERT_EQ(scan.count, 1);
  EXPECT_EQ(scan.status, agc_task::CaptureStatus::kHit);
  EXPECT_TRUE(scan.decision.hit);
  EXPECT_EQ(scan.decision.shot_id, adj.shotId());
  EXPECT_NEAR(scan.decision.flight_s, 1.15, kDt + 1e-6);
  EXPECT_LE(scan.decision.flight_s, kTimeout + kRateTol + 1e-6);
  EXPECT_TRUE(scan.decision.miss_dist_valid);
  EXPECT_LE(scan.decision.miss_dist, 0.5 + 1e-9);
  EXPECT_FALSE(adj.inFlight());
}
