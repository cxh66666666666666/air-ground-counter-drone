# 移动目标 costmap marking 与重规划触发策略（地图签名仅静态层）

## Status

accepted

## Date

2026-10-24

## 背景

[ADR-0089](0089-map-signature-lethal-only-for-replan-trigger.md) 把 `ground_baseline::mapSignature()` 的参与判据由 `cost >= 253` 收窄为 `cost == 254`，消除了「`NO_INFORMATION=255` / `INSCRIBED_INFLATED_OBSTACLE=253`」两类伪触发（C5 由 14 → 0，C4 真实增障仍触发 1 次）。但 ADR-0089 明确**不消除**「移动目标在 obstacle / voxel 层被标记为 lethal」造成的剩余触发；[ADR-0090](0090-w7b-post-audit-v7-gate-and-implementation-fixes.md)(d) 要求该原因**独立 ADR、独立提交、独立回归**，不与火控状态机修复合并。

本 ADR 只处理**地图变化的签名口径**；不改目标位移阈值、`replan_period_s_`、`kReplanGuardS`、max path age、规划器、控制律、安全层或任何 ABI。

### 审计证据（诊断，非确证）

承载 run：`experiments/w7-rl-b/20261024-w7-rl-b-p2-predchain-retry2`（P2 臂 B，`s08_static_none_nominal_capture`，seeds 0–2，full 链）。审计产物：[`20261024-w7-rl-b-p2-map-trigger-audit`](../../experiments/w7-rl-b/20261024-w7-rl-b-p2-map-trigger-audit/metrics/p2_map_trigger_audit.md)（[`p2_map_trigger_audit.json`](../../experiments/w7-rl-b/20261024-w7-rl-b-p2-map-trigger-audit/metrics/p2_map_trigger_audit.json)）。

- 30 次「地图变化」触发全部带分层明细；`hard_cells` **7–14**；`static_layer=0` 全部。
- 连续触发间分层计数变化：**static 0 / obstacle 2 / voxel 26**。即剩余触发几乎全部由 **voxel 层**的 lethal 计数迁移驱动；obstacle 层只在少数时刻随 voxel 一起变化（4↔5），static 层完全不动。
- 典型序列（seed2，t=25.9–28.1 s）：`voxel_layer` 11 → 12 → 13 → 14 → 13，`obstacle_layer` 恒为 5，`static_layer` 恒为 0；同段 `HL_DIAG` 目标估计在 `(4.5–4.7, 3.0–3.2)`（距离触发 goal 的 1–2 m 内）。场景中唯一持续移动的对象是靶机；voxel 层的 `min_obstacle_height=0.30` / `max_obstacle_height=2.5` 覆盖靶机高度（≈2 m），点云源为 `/agc/sim/car_points`。
- **路径受影响程度（代理量）**：触发前后最近一次「规划成功：N 点」的路径点数变化比例 **86.67%**（30 次中 26 次变化），绝对变化中位 1 点，最大变化 7 点（e.g. 24→31、29→24、7→3）。现有日志不记录路径几何（长度 / 偏离），故本审计用**点数变化**作为路径受影响的代理；几何偏离量**未量化**，已如实登记。

⇒ 剩余触发的根因是**派生 costmap 的传感器层（voxel）对移动目标的 lethal marking 迁移**，不是 `/agc/map` 静态内容变化。当前 `mapSignature()` 读取的是 master `Costmap2D`（所有层合并），因此把动态层变化误当作「地图变化」。

### 同版本 Nav2 取证（本机 Jazzy）

- `nav2_costmap_2d::VoxelLayer`（`voxel_layer.hpp`，继承 `ObstacleLayer` → `CostmapLayer`）在 `updateBounds` / `updateCosts` 中按观测源做 marking / clearing；`min_obstacle_height` / `max_obstacle_height` / `mark_threshold` 决定哪些 3D 点写入 lethal。移动目标在高度范围内会随其位置改变 lethal 格。
- `nav2_costmap_2d::ObstacleLayer`（`obstacle_layer.hpp`）同理：`marking` / `clearing` 由观测源驱动，2D scan 的 marking 也是动态内容。
- `nav2_planner::PlannerServer`（`planner_server.hpp`）在**规划请求 / 目标更新 / 规划频率**上创建路径；其 costmap 通过 `Costmap2DROS` 更新，但 **costmap 更新本身不直接触发一次新的规划**。Nav2 bringup 参数以 `expected_planner_frequency` 控制规划节律（`nav2_params.yaml`，本机 20.0）。
- ⇒ Nav2 的语义是：**全局规划在请求 / 周期上消费当前 costmap**，而不是「每次 costmap 内容变化都立即重规划」。动态障碍的即时安全由控制器 / `collision_monitor` 等层承担。

## 决策

`ground_baseline::mapSignature()` **只使用 `static_layer` 自身的栅格内容**（尺寸 / 原点 / `LETHAL_OBSTACLE=254` 格索引，FNV-1a 64 位）；**obstacle / voxel 层的动态 marking 不再参与「地图变化」签名**，由既有 `replan_period_s_` 周期重规划在下一拍消费完整 costmap（规划器仍读取 master costmap，因此动态障碍仍影响路径与安全层）。

具体：

1. 在 `Costmap2DROS::getLayeredCostmap()->getPlugins()` 中定位 `static_layer`，`dynamic_cast<nav2_costmap_2d::CostmapLayer*>` 后读取其 `getCharMap()` / `getSizeInCellsX/Y()` / `getOriginX/Y()`，只对 `254` 格做 FNV-1a。
2. 若 `static_layer` 尚不可用（地图未到 / 尺寸为 0），回退到 master `Costmap2D` 并在日志标注 `signature_from_static=false`；一旦 static 层可用，签名自动切回静态层。
3. `mapSigInfo()` 增加 `sig_hard_cells`（参与签名的格数）与 `signature_from_static`；保留既有 `hard_cells`（master 全部层，诊断）与 `layers`（分层计数）。`logMapTrigger` 同时打印两者，便于后续审计。
4. `needReplan()` 的返回逻辑、位移阈值、周期、guard、max age、规划器 / 控制律 / ABI **逐字不变**。

**不采用**：

1. **直接过滤目标点云 / 删除 `/agc/sim/car_points` 中的靶机点** —— 否决：会掩盖真实障碍，违背「不得直接过滤目标点云」。
2. **对 obstacle / voxel 层的瞬态清理做去抖** —— 否决：移动目标会持续迁移，去抖只能降低频率、不能消除根因；且与同版本 Nav2 的规划节律不一致。
3. **按目标位置排除 lethal 格** —— 否决：需要把火控 / 目标身份耦合进地图签名，可能隐藏目标附近的真实障碍；属另一原因。
4. **接受当前行为** —— 否决：30 次触发 / 3 episodes、86.67% 触发改变路径点数，重规划预算与路径抖动成本明显；且与 Nav2 节律不符。
5. **改用 `/agc/map` 内容身份 / `update_seq`** —— 仍属另一原因（内容身份 vs 派生 costmap 粒度），本 ADR 不处理。

## 后果

- **正面**：移动目标在 voxel / obstacle 层的 marking 迁移**不再**触发「地图变化」；真实 `/agc/map` 静态增删占据格仍触发；未知 / inflation 伪触发维持 ADR-0089 的消除状态。
- **代价 / 限制**：动态障碍（voxel / scan）不再造成**立即**重规划；它们仍被规划器在**下一次周期规划**时消费，并在控制 / 安全层即时生效。响应延迟受 `replan_period_s_` 约束（既有频率，不改）。若未来要求「动态障碍即时重规划」，须另立 ADR，定义持续性判据与安全层协同。
- **可比性**：修复后 P2 重规划频次与 pre-fix run（含 retry2 / v3）**不可直接比较**；历史 run 保留原样，P2 v3 `falsified` / c2 不回溯改写。
- **回归（实测，2026-10-24）**：C1–C6 保持；新增 C7 / C8：
  - C7「合成移动目标样式 scan / pointcloud marking 迁移」：修复前 **4 次**地图触发 → 修复后 **0 次**；
  - C8「真实静态增障 + 动态 marking 同时存在」：修复前 **4 次** → 修复后**恰好 1 次**（只由静态层增障触发）；
  - C1–C6 逐项与修复前一致（C4 真实增障仍 1 次、C5 未知翻转 0 次）；
  - R-041 契约测试继续 **PASS**。
  台架产物：`experiments/w7-rl-b/20261024-w7-rl-b-p2-map-trigger-bench/metrics/p2_replan_bench-before-c7.{json,md}` 与 `p2_replan_bench-postfix.{json,md}`。
- **边界**：不改冻结 9 消息 / 服务 ABI；不改唯一执行口语义；不新增第二命令者；不改规划器 / 控制律 / 安全层 / 终态语义；不进入在线 RL。

## 证据

- 审计 run：`experiments/w7-rl-b/20261024-w7-rl-b-p2-map-trigger-audit/metrics/p2_map_trigger_audit.{json,md}`（源 `20261024-w7-rl-b-p2-predchain-retry2`）。
- 回归台架：`experiments/w7-rl-b/20261024-w7-rl-b-p2-map-trigger-bench/metrics/p2_replan_bench-{before-c7,postfix}.{json,md}`。
- 分层触发序列：同 run `log/ep_p2_rl_high.r1_s08_static_none_nominal_capture_2.log`（t=25.9–28.1 s）。
- 前序：[ADR-0089](0089-map-signature-lethal-only-for-replan-trigger.md) 的 C4 / C5 回归。
- 实现：[`ground_baseline.cpp`](../../src/agc_ground/src/ground_baseline.cpp)（`mapSigInfo` / `mapSignature` / `logMapTrigger`）。
- 台架：[`w7b_p2_replan_bench.py`](../../scripts/w7b_p2_replan_bench.py)（C1–C8）、[`w6_r041_check.py`](../../tests/w6_r041_check.py)。

## 参考

- 上游：[ADR-0090](0090-w7b-post-audit-v7-gate-and-implementation-fixes.md)(d)、[ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)(d-1)。
- 前序：[ADR-0055](0055-map-content-identity-and-monotonic-update-sequence.md)、[ADR-0056](0056-goal-invalidation-policy-and-displacement-triggered-replan.md)、[ADR-0027](0027-baseline-path-binding-and-replan-failstop.md)、[ADR-0089](0089-map-signature-lethal-only-for-replan-trigger.md)。
- 同版本 Nav2：`/opt/ros/jazzy/include/nav2_costmap_2d/nav2_costmap_2d/{voxel_layer,obstacle_layer,static_layer,costmap_layer}.hpp`、`/opt/ros/jazzy/include/nav2_planner/planner_server.hpp`、`/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`。
