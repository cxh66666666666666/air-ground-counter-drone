# 地图签名仅计 LETHAL_OBSTACLE（P2 重规划审计的地图伪触发修复）

## Status

accepted

## Date

2026-10-24

## 背景

W7-b 的 P2 臂 B（`p2_rl_high`）诊断（[ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)(d)）记录
每回合 **23.7** 次重规划，其中「地图变化」282 次（20 回合）、「目标身份变」190 次。R3 要求先审计这些触发
是**真实目标移动 / 真实地图变化 / 身份或签名伪触发**，再决定是否修复；且**每次只修一个原因**。

审计台架（`experiments/w7-rl-b/20261024-w7-rl-b-p2-replan-audit`，真实 `ground_baseline` 二进制 +
合成 map/odom/goal/target，非确证）得到：

- C1 固定目标 + 固定地图 + 固定 goal → 0 次「目标身份变」、0 次「地图变化」；C2 位移 1.2 m → 恰好 1 次
  「目标身份变」（`goal_delta_m=1.200`）；C3 微扰 0.3 m → 0 次；⇒ **目标触发本身行为正确**，日志标签
  「目标身份变」只是历史命名（实际判据是位移阈值）。
- C4 真实增一个 3×3 硬障碍 → 恰好 1 次「地图变化」（`hard_cells` 0 → 9）。
- **C5 自由 / 未知（0 ↔ -1）翻转 → 4 s 内 14 次「地图变化」**（每次 `hard_cells` 0 ↔ 9，静态层 0 ↔ 9）。
  未知格在 `track_unknown_space=true` 下由 static 层写成 `NO_INFORMATION=255`，而
  `ground_baseline::mapSignature()` 的判据是 `cost >= 253` ⇒ **255 被当作硬障碍**。
- 同一判据还把 `INSCRIBED_INFLATED_OBSTACLE=253`（inflation 内切圆环）计为硬障碍：C4 中
  `hard_cells=21 = static 9 + inflation 12`；即「未增障、仅膨胀环变化」也会触发地图重规划。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)、
[#execution-port](../ROS2迁移交接文档.md#execution-port)、
[#params](../ROS2迁移交接文档.md#params)；前序：[ADR-0055](0055-map-content-identity-and-monotonic-update-sequence.md)
（地图内容身份）、[ADR-0056](0056-goal-invalidation-policy-and-displacement-triggered-replan.md)
（位移阈值化重规划）、[ADR-0027](0027-baseline-path-binding-and-replan-failstop.md)（路径绑定 / fail-stop）。

分类：**重规划触发判据的实现缺陷（签名口径）**。本 ADR 只改一个原因：**地图变化的签名口径**；
不改目标位移阈值、周期 / 时限、`kReplanGuardS`、规划器、控制律或任何 ABI。

## 取证（先学后做）

同版本（ROS 2 Jazzy）Nav2 的代价常量与层语义（本机 `/opt/ros/jazzy/include/nav2_costmap_2d/...` 实读）：

- `cost_values.hpp`：`NO_INFORMATION = 255`、`LETHAL_OBSTACLE = 254`、
  `INSCRIBED_INFLATED_OBSTACLE = 253`。三者语义**互不相同**：未知 ≠ 致命占据 ≠ 内切膨胀。
- `StaticLayer`（`static_layer.hpp`，`nav2_costmap_2d::StaticLayer`）把 `/agc/map` 的 `100`（occupied）
  写成 `LETHAL_OBSTACLE`，把 `-1`（unknown，`track_unknown_space=true`）写成 `NO_INFORMATION`。
- `InflationLayer`（`inflation_layer.hpp`）对致命占据的内切圆环写 `INSCRIBED_INFLATED_OBSTACLE`。
- `Costmap2D` 几何访问器 `getSizeInCellsX/Y` / `getOriginX/Y` / `getCharMap`（`costmap_2d.hpp`）是只读
  栅格入口；`Costmap2DPublisher` 以 full / delta 分离内容与更新（[ADR-0055](0055-map-content-identity-and-monotonic-update-sequence.md)
  已取证）——「地图是否变化」应按**真实占据内容**判断，而非把未知 / 膨胀环混入。

⇒ 本项目原判据 `cost >= 253` 与上述语义不一致；「硬障碍」应为 **`LETHAL_OBSTACLE == 254`**。

## 决策

把 `ground_baseline::mapSignature()` 的参与判据由 `cost >= 253` 改为 **`cost == 254`
（`LETHAL_OBSTACLE`）**；分层诊断计数同口径。签名仍为 FNV-1a over
`sx / sy / origin_x / origin_y / 致命占据格索引`，仍与规划成功时刻的 `last_plan_map_sig_` 比较；
`mapChangedSincePlan()` 的调用点、`needReplan` 的返回逻辑、阈值、限流、规划行为**不变**。

不采用（本次不做）：

1. **仅把阈值调为 `>= 254`**——仍包含 `NO_INFORMATION=255`，C5 仍会伪触发；
2. **改用 `/agc/map` 内容身份 / `update_seq`**——属于另一原因（内容身份 vs 派生 costmap 粒度），需独立
   变更；本批不改消息消费路径；
3. **分层签名只计 static + 已确认动态障碍**——与 1 同类但更大改动，留待若仍观察到伪触发时再议；
4. **对 obstacle / voxel 层瞬态清理加去抖**——本批证据未显示该层是主因（C5 的触发来自 static 层未知格）；
   若后续全链诊断确认，再独立修复。

## 后果

- **正面**：未知格翻转与 inflation 内切环变化**不再**触发地图重规划；真实增删占据格仍触发（C4 回归保留）。
  对臂 B，若 v3 中的地图触发含此类伪触发，其频次会被高估；修复后可用新 run 复核（不改 v3 verdict）。
- **代价 / 限制**：以 `== 254` 为判据时，某些仅由「内切膨胀」定义的近障区域不再计入——但这本来就不是
  栅格占据变化；路径规划仍消费完整 costmap，控制安全不受影响。签名仍是**派生 costmap**口径，不是
  `/agc/map` 内容身份；后者若需要，另立 ADR。
- **适用范围**：本修复消除「未知 255 / inflation 253」两类伪触发。若 B 的地图触发主要由 obstacle/voxel 层
  对**移动目标**的 `254` marking 迁移造成（「动态目标被当作静态地图障碍」），本修复**不会**消除该部分；
  需全链新 run 按原因复核后另行决策（本次不合并修改）。
- **回归**：`scripts/w7b_p2_replan_bench.py` C1–C6；修复后 C5 由 14 → 0 次，C4 保持 1 次；
  既有 R-041 契约测试 `tests/w6_r041_check.py` 仍全 PASS（见证据）。
- **边界**：不改冻结 9 消息 / 服务 ABI；不改唯一执行口语义；不新增第二 ROS 命令者；不改规划器 / 控制律 /
  安全层 / 终态语义；不进入在线 RL。

## 证据

- 审计 run：`experiments/w7-rl-b/20261024-w7-rl-b-p2-replan-audit/metrics/p2_replan_audit.{json,md}`；
  台架：`metrics/p2_replan_bench.{json,md}`（修复前）与 `metrics/p2_replan_bench-postfix.{json,md}`（修复后）。
- 修复前 C5：14 次地图触发、`hard_cells` 0↔9、`layers=[static_layer:9,...]`；修复后 C5：0 次。
- 修复后 C4：1 次地图触发（真实增障），签名差异仍被记录。

## 参考

- 审计来源：[spec](../../.trae/specs/verify-w7b-p1-p2-before-v7/spec.md)、[ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)
- 前序：[ADR-0055](0055-map-content-identity-and-monotonic-update-sequence.md)、[ADR-0056](0056-goal-invalidation-policy-and-displacement-triggered-replan.md)、[ADR-0027](0027-baseline-path-binding-and-replan-failstop.md)
- 实现：[`ground_baseline.cpp`](../../src/agc_ground/src/ground_baseline.cpp)（`mapSignature` / `mapSigInfo`）
- 台架：[`w7b_p2_replan_bench.py`](../../scripts/w7b_p2_replan_bench.py)、[`w6_r041_check.py`](../../tests/w6_r041_check.py)
