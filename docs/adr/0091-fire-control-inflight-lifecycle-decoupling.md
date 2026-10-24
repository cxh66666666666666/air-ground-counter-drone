# 火控在飞生命周期与发射资格解耦（模型身份 v2）

## Status

accepted

## Date

2026-10-24

## 背景

[ADR-0090](0090-w7b-post-audit-v7-gate-and-implementation-fixes.md)(c) 裁定「先修火控状态机，再决定 P2」。本 ADR 记录该修复的实现决策、边界与回归证据；**不产生新的 P1 / P2 verdict**，也**不改写**任何历史 `executed_*` 字段。

### 缺陷（已复现的实现问题，不是统计问题）

[`DynamicFireControlAdjudicator::step()`](../../src/agc_task/src/capture_adjudicator.cpp) 原实现把「车 / 目标位姿缺失」与「交战门（适用高度 / 可达 / 水平射程）」的提前返回放在 `if (in_flight_)` 分支**之前**；节点 [`tick()`](../../src/agc_task/src/capture_adjudicator.cpp) 也在车 / 目标新鲜度（含 `confidence_min`）失败时直接 `return`，**根本不调用 `step()`**。后果：网在飞期间目标估计短时越出高度 / 射程、或输入短暂失效时，**弹道时钟不推进、落地与超时都不结算**；门不再通过则一条 `SHOT_FIRED` **永久无结局**。

证据（既有日志，数值不改）：

| 来源 | 观测 |
|---|---|
| [ADR-0085](0085-w7b-p1-mechanism-activating-scenario-pilot.md)(i-1) | P1 协同 s15 seed5：`SHOT_FIRED shot=2-2 t=36.032` 后**约 114 s 无** `SHOT_HIT` / `SHOT_MISS`，至 episode 结束（sim≈150 s）；同一回合 `2-1` 先以 `flight=1.348 s` 正常失手 |
| [ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)(d-1) | P2 v3：seed3 `2-1` `flight=32.592 s`、seed5 `1-9` `flight=15.776 s`；`miss_dist` 最大 **5007.883 m**（4 发 >10 m） |
| 同 ADR (d-1) retry2 | 3 次 miss 的 `flight − t_est` 为 **+1.320 / +1.343 / +1.211 s**；网解析落地时间约 `2·v0·sinθ/g ≈ 1.28 s`，而两次 miss 的 `flight` 为 **2.132 / 1.452 s** ⇒ 落地判定被跳过 / 延迟，属**状态机时序** |
| [ADR-0090](0090-w7b-post-audit-v7-gate-and-implementation-fixes.md) | 把该缺陷列为独立修复项，修复前不注册新确证 run、不据诊断 run 重判 P2 |

## 决策

### (a) 在飞优先：推进 / 落地 / 超时结算与发射资格解耦

`step()` 现在**先**处理 `in_flight_`，再处理发射资格：

- 只要在飞，**每一控制拍**用调用方传入的仿真时刻推进弹道时钟；落地判据 `net.z < net_z0_ − 0.01`、超时判据 `t_s − t_fire_ > net_flight_timeout_s` 每拍检查。
- 发射资格（适用高度 / 可达 / 水平射程、冷却、授权、身份、任务窗口、`TargetMotionFilter::settled()`）**只约束新发射**。
- 节点 `tick()` 不再因新鲜度早退；把 `car_fresh` / `target_fresh`（含 `confidence_min`）编码为 `Pose2.valid` 后**始终**调用 `step()`。`envelope_in` / `envelope_jam` 只在两路输入均有效时发布，保持既有 FCO 口径。

### (b) 目标缺失不判命中，但照常结算

只有 `target.valid` 才更新 `TargetMotionFilter` 并做球比对；目标无效拍**跳过命中判定**（不使用缓存位姿），弹道推进与落地 / 超时结算继续。从未获得有效观测时 `miss_dist_m` 如实为不可用（`inf`，并以 `miss_dist_valid=false` 标注），**不**用过期位置伪造最近距离。

### (c) 唯一结局与显式中止

每次 `SHOT_FIRED` 恰有一个终局记录：`SHOT_HIT`、`SHOT_MISS`，或显式 `SHOT_ABORTED`。

- `/agc/task/fire_control` 新增 `SHOT_ABORTED shot=<id> reason=<episode_reset|clock_regression> t=<sim>`（追加 token，既有 `SHOT_FIRED|SHOT_HIT|SHOT_MISS` 解析不受影响）；`capture_evidence` 同步标注 `SHOT_ABORTED`。
- `SHOT_ABORTED` **不**发布 `/agc/task/capture_hit`、**不**写 `CAPTURE_NET`、**不**冒充命中 / 失手。
- episode 身份变化时，节点在 `adjudicator_->reset(e)` **之前**，若在飞则发 `SHOT_ABORTED reason=episode_reset`，随后复位在飞 / 冷却 / `shot_seq` / 一次性命中标志。
- 仿真时钟回退（`t_s < t_fire_ − 1e-6`）在 `step()` 内以 `SHOT_ABORTED reason=clock_regression` 结算，`flight_s` 夹到 `≥0`，不延长在飞。
- 结算后 `cooldown_until = t_settle + fire_cooldown_s`（仿真时钟）；同一 shot 不再产生第二个终局。

### (d) 仿真时钟与飞行上界

在飞推进、落地与超时结算以 ROS 仿真时钟（`use_sim_time=true`）为唯一时间基。正常 20 Hz 下 `flight_s` 应满足 `0 ≤ flight_s ≤ net_flight_timeout_s + 1/rate_hz + 数值容差`；不允许负飞行时长或 5007 m 级 `miss_dist`。

### (e) 模型身份升版 v2

`model_id` 由 `net_ballistic_frame_v1` 升为 `net_ballistic_frame_v2`（节点默认值与 [`w6_base.yaml`](../../src/agc_task/config/w6_base.yaml) 单一注入同步），用于显式区分「修复前 / 修复后」射击时序证据。**不**改任何物理参数：`net_v0_mps` / `net_angle_deg` / `net_g` / `net_diameter_m` / `net_drag` / `net_t_exp_s` / `fire_cooldown_s` / `net_flight_timeout_s` / `applicable_*` 与场景难度逐字沿用；**不**改 `t_est`（水平到达时间）的定义，**不**改弹道模型。

### (f) 接口与 ABI 边界

- `DynamicFireControlAdjudicator` 抽到可单测头文件 [`dynamic_fire_control.hpp`](../../src/agc_task/include/agc_task/dynamic_fire_control.hpp)（纯标准库 + `target_motion_filter.hpp`，无 ROS / GZ / PX4）。
- 冻结 9 消息 / 服务 ABI 未改；唯一执行口语义未改；未新增命令者；`CAPTURE_NET` 的终态语义与既有 `allow_fire` 门槛不变。
- 内部 `CaptureAdjudicator` 接口新增 `abortInFlight()` / `inFlight()`（非纯虚，缺省无在飞状态），不改变节点对可插拔实现的依赖方式。

## 备选

1. **在节点层缓存目标位姿继续判命中** —— 否决：会用过期位置伪造命中，违背「目标失效不判命中」与 [契约 #terminal-semantics](../../docs/ROS2迁移交接文档.md#terminal-semantics)。
2. **在飞期间单独维护第二套弹道时钟** —— 否决：与既有 `t_fire_` / 网位姿模型重复，增加状态与漂移风险；直接把在飞分支提前即可。
3. **用墙钟（`steady_clock`）做超时兜底** —— 否决：违反仿真时钟唯一时间基；GUI / headless 速率不同会改变结局。
4. **把中止记录并为 `SHOT_MISS`** —— 否决：复位 / 时钟回退不是物理失手，混用会污染命中率与时序统计。
5. **与移动目标地图触发（T4 / ADR-0092）合并提交** —— 否决：违背「每次只修一个原因」与 [ADR-0090](0090-w7b-post-audit-v7-gate-and-implementation-fixes.md)(d)。

## 后果

- **正面**：在飞网不再「悬挂」；每个 `SHOT_FIRED` 有唯一终局；`flight_s` 与 `miss_dist_m` 回到物理上界内；复位 / 时钟回退有显式、可审计的中止记录；状态机可直接以显式 `t_s` 做确定性回归。
- **代价 / 限制**：修复后 run 的射击 / 命中时序与修复前**不可直接混用**，须以 `model_id=v2` 区分；历史 run 保留为 pre-fix 证据，**不**回溯改写。下游脚本须容忍 `SHOT_ABORTED`（不当作命中 / 失手），该兼容性由独立任务覆盖。
- **边界**：输入失效拍不发布 `envelope_in` / `envelope_jam`；`CAPTURE_NET` 仍受既有 `allow_fire`（窗口 / 身份 / 授权 / 新鲜度）门控。命中判定只依赖目标位姿与已锁定的弹道原点 / 方向，车辆位姿缺失不改变弹道。
- **未决 / 后续**：结果来自修复前实现的 P1 v5 `inconclusive`、P2 v3 `falsified` / c2 维持原文；小批整链诊断（T3）与 W7-b 闭合（T5）另立记录。

## 回归证据

确定性单测 [`test_dynamic_fire_control.cpp`](../../src/agc_task/test/test_dynamic_fire_control.cpp)（显式 `t_s`，无 ROS / GZ / PX4）：

- 修复前（bug-compatible 头文件）首条用例 **红**：`scan.count = 0`、`scan.status = kInvalid` ⇒ 目标失效后在飞网无终局；
- 修复后 **绿**：13 个用例全部通过（目标过期 / 低置信度 / 越出上下限高度 / 越出水平射程 / 车辆位姿缺失 / 落地 / 超时 / 冷却 / 复位中止 / 时钟回退 / 唯一终局 / 正常命中）；
- `colcon test --packages-select agc_task --event-handlers console_direct+`：**3/3 通过**（含既有 `test_target_motion_filter` 与 `scene_config_invariants`）；
- `python3 tests/w6_r041_check.py`：**PASS**（a–h 全通过）。

## 参考

- 决策门：[ADR-0090](0090-w7b-post-audit-v7-gate-and-implementation-fixes.md)(c)(d)
- 缺陷最小复现：[ADR-0085](0085-w7b-p1-mechanism-activating-scenario-pilot.md)(i-1)
- 时序段证据：[ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)(d-1)
- 目标运动估计：[ADR-0079](0079-b2-fire-control-target-motion-estimation.md)
- 实现：[`dynamic_fire_control.hpp`](../../src/agc_task/include/agc_task/dynamic_fire_control.hpp)、[`capture_adjudicator.cpp`](../../src/agc_task/src/capture_adjudicator.cpp)、[`w6_base.yaml`](../../src/agc_task/config/w6_base.yaml)
- 契约：[硬约束](../../docs/ROS2迁移交接文档.md#hard-constraints)、[终态语义](../../docs/ROS2迁移交接文档.md#terminal-semantics)、[参数入口](../../docs/ROS2迁移交接文档.md#params)
- 规格：[`fix-fire-control-inflight-lifecycle-w7b-closure`](../../.trae/specs/fix-fire-control-inflight-lifecycle-w7b-closure/spec.md)

## 回归证据（2026-10-24）

- **红测（修复前）**：bug-compatible 头文件 + `colcon build --packages-select agc_task` + `./build/agc_task/test_dynamic_fire_control`。`DynamicFireControlLifecycle.TargetInvalidAfterFireStillSettles` 失败：目标失效后 `scan.count=0`（无终局）、`status=kInvalid`，即「有 `SHOT_FIRED` 无结局」。
- **绿测（修复后）**：同一命令 `13 tests from DynamicFireControlLifecycle` 全部 `PASSED`；覆盖目标过期 / 低置信度 / 越出上下限高度 / 越出水平射程 / 车辆位姿缺失 / 落地（`flight≈1.30 s`）/ 超时（`timeout=0.5 → 0.55 s`）/ 冷却 / 复位中止 / 时钟回退 / 唯一终局 / 正常命中。
- **包测试**：`colcon test --packages-select agc_task` 3/3 通过（`test_dynamic_fire_control` + `test_target_motion_filter` + `scene_config_invariants`），0 errors / 0 failures。
- **节点级**：`tests/w7b_fire_control_lifecycle_check.py` A–F 全过（7 fired / 7 terminals，无未闭合）；`tests/w6_r041_check.py` a–h 全 PASS。
- **lightweight E2**：s08 headless `20261024-w7-rl-b-145517-2702`、s08 GUI `20261024-w7-rl-b-152651-9675`、s15 seed5 headless `20261024-w7-rl-b-152800-3174`（4 发全部闭合）、s15 seed5 GUI `20261024-w7-rl-b-152850-0348`。
