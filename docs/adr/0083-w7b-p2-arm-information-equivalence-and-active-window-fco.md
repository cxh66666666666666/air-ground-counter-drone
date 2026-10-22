# W7-b P2 修正：臂信息等价 + active-window FCO 口径

## Status

accepted

## Date

2026-10-22

## 背景

W7-b P2 v2 批次（`w7-rl-b/20261022-w7-rl-b-112833-1155`，20 对 / 40 episodes）按预注册 v2 判定为
“未达优越性判据”。判定后审计发现两处**未考虑到的设计 / 口径缺陷**，该批次**不能**作为
“职责分层 vs c2”的公平证据：

1. **臂信息不等价（致命）**
   - 臂 A（`p2_c2_low_level`）的 c2 策略只订阅 `/agc/ground/goal`（[inference_node.py](../../src/agc_rl/agc_rl/inference_node.py)），
     **不订阅** `/agc/targets`。
   - 场景 `s08_static_none_nominal_capture` 的 `task_manager.goal_follow_target=false`，固定目标
     `(4.5, 3.0)`（[scenarios.yaml](../../src/agc_task/config/scenarios.yaml)），而 `scenario_reset` 的目标真值
     也是 `(4.5, 3.0)`；`publishGoal` 在固定模式下 `valid=true, reason=FIXED_GOAL`，**与感知无关**
     （[task_manager.cpp](../../src/agc_task/src/task_manager.cpp)）。
   - 臂 B（`p2_rl_high`）的高层只订阅 `/agc/targets`，用**感知目标**出拦截点
     （[high_level_node.py](../../src/agc_rl/agc_rl/high_level_node.py)）。
   - 因此 A 是**oracle 目标臂**，B 是**感知驱动臂**；“两臂只差 IV = 命令源”不成立；预注册的
     `target_source: air_fusion` 对 A 并未生效（A 不消费 target）。

2. **primary DV 被终态后停泊污染（严重）**
   - `run_logger` 的固定观测窗（`window_fixed_v1`）自首个 active 拍起 60 s，**不随终态截断**
     （[run_logger.cpp](../../src/agc_task/src/run_logger.cpp)）；`capture_adjudicator` 的
     `/agc/task/envelope_in` 在终态后仍按几何**每拍发布**
     （[capture_adjudicator.cpp](../../src/agc_task/src/capture_adjudicator.cpp)）。
   - v2 实测：primary `fire_control_opportunity_rate` 中，**约 97%（A）/ 100%（B）的机会拍发生在终态之后**；
     它实际测的是“任务结束后车停在包络里多久”，并被终态时刻驱动
     （`corr(Δfco, Δterminal_ticks) = -0.51`）。
   - 用 `episode_window_metrics` 的 active-window FCO（`GROUND_APPROACH → 终态`）重算：
     A 0.109 vs B 0.042，Δ = −0.067，n=19，p = 0.029。方向仍为 B 不优，但量级 / 显著性已不同。

**P1 跨命题风险（登记，不在本 ADR 改判）**：同一固定窗口也用于 P1。P1 现行 run 的 fixed-window
Δ = +0.299（p = 0.0005），active-window Δ = +0.056（p = 0.098）；约 92%/86% 的 FCO 来自终态后。
P1 的“成立”对窗口口径高度敏感，须在 W8 冻结前复核；本 ADR 只登记风险，不改写 P1 v1 结论。

## 决策

### (a) v2 批次不作为 P2 判定证据

`w7-rl-b/20261022-w7-rl-b-112833-1155` 标记为 **confounded**，保留为**历史 / 探索证据**；
其“P2 证伪”记录撤销。预注册 v2 的 `status: executed_falsified` 改为 v3 的 `pre_registered`，
并把该 run 登记在 `decision.p2.invalidated_runs`。

### (b) 两臂信息等价（P2 v3）

P2 两臂统一 `goal_follow_target=true`（launch 参数 `goal_follow_target` → `task_manager.goal_follow_target`）：
- 臂 A：任务目标 `/agc/ground/goal` 由 `task_manager` 按 `/agc/targets` 生成（与 B 同源感知目标）；
- 臂 B：高层继续直接消费 `/agc/targets`；
- 两臂都经同一安全层 / 唯一执行口；A 不再获得目标真值坐标。

残余差异（如实登记）：`task_manager.goal_hold_valid_s=1.0 s` vs 高层
`target_max_age_s=0.5 s`；差异方向不偏向 B（A 的目标保持更久），不解释 A 的优势，但 v3 报告须登记。
- 残余差异 2（如实登记）：`rl_high` 模式下 `ground_baseline` 仍消费 `task_manager` 的
  `/agc/ground/goal_info`；`goal_follow_target=true` 后该诊断随 `/agc/targets` 波动，B 的低层可能在
  高层仍有目标时被全局 `valid=0` 停车。方向不解释 A 的优势，但 v3 报告须登记；如需完全解耦，
  可在 `rl_high` 分支改接独立 goal_info（后续）。

### (c) primary DV 改为 active-window FCO

P2 v3 的 primary = `fire_control_opportunity_rate_active`
（`episode_window_metrics.ticks_fire_control_opportunity / vis_ticks_total`）：
- 分母 / 分子都只覆盖 `GROUND_APPROACH → 终态`，不含终态后停泊；
- 失败 / infra / 未达终态 → 0 计入分母（沿用 `keep_all_failures_and_infra`）；
- 原 `fire_control_opportunity_rate`（`window_fixed_v1`）降为**描述 / legacy**，不参与 v3 判定；
- 这不是“事后改主量”：v3 是**新预注册版本**，v2 的已执行口径不回溯改写；active-window 在
  ADR-0074 中已作为 `episode_window_metrics` 保留，v3 只是把它提升为 P2 primary。

### (d) 预注册 v3

- `n_paired_episodes_per_arm = 20`、`alpha = 0.05`、`power = 0.80`；
- `min_detectable_effect = 0.10`（absolute rate）；规划依据：v2 active-window 配对差 SD=0.1224
  （n=19，剔除 infra seed 8；仅作规划估计，不入判定）；N=20 时实际 power≈0.97；
- 阈值 / 样本量在 v3 run **之前**写定，沿用 [F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13) 纪律。

### (e) 重跑与判定

用 v3 口径重跑 seeds 0–19（两臂 / 40 episodes），由
[`scripts/w7b_p2_adjudicate.py`](../../scripts/w7b_p2_adjudicate.py) 按 `statistics.p2` 判定；
**结果（2026-10-22）**：run `w7-rl-b/20261022-w7-rl-b-v3-n20r` 已完成（40 episodes / 墙钟 2.39 h / 终态 infra 0/40）；primary active FCO **Δ = −0.0561**（95% CI [−0.0971, −0.0151]）、p = 0.010、n=20 → **未达 MDE 0.10 且方向为负 → P2 证伪、保留 c2**（详见 [ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(j)）。

### (f) P1 不变

P1 v1 的判定规则 / 阈值 / 已执行结论**不改写**；本 ADR 只登记窗口口径敏感性，W8 前另行复核。

### (g) 高层目标抖动修复（smoke 发现，2026-10-22）

- **现象**：P2 v3 冒烟（2 seeds，**高层滤波修复前**）中，B 单回合出现 **175 次** `重规划触发：目标身份变`（约每 0.25 s 一次），而 A 无此现象；B 的 active 窗口约 1326 ticks vs A 约 361 ticks，B 明显变慢。
- **根因**：`air_fusion` 的 twist 是融合位置差分（[air_fusion.cpp](../../src/agc_perception/src/air_fusion.cpp)），单帧位置尖峰经 `lead_s=1.0 s` 外推后使目标点跳动 > `replan_displacement_m=1.0 m`，低层每帧触发重规划，路径跟踪退化。
- **修法（不改 IV）**：高层取**置信度最高**航迹（不再 `front()`）、目标位置 / 速度一阶低通（`target_tau_s=0.5`、断流 `target_gap_reset_s=1.0` 重置）、前向位移限幅 `max_lead_m=1.0 m`；仍为规则高层，不引入学习。
- **验证**：修复后同 2 seeds 冒烟复验 —— B seed0 `重规划触发` **175 → 11**（其余见 run 产物）；两臂终态正常。

## 备选

1. **保留 v2，仅补 active-window 敏感性** —— 否决：信息不等价是臂构造缺陷，不能用统计敏感性补救；
   且“只差命令源”的前提已不成立。
2. **让 B 也消费 `/agc/ground/goal`（固定真值目标）** —— 否决：高层退化为转发，P2 的
   “规则高层出拦截点”自变量消失。
3. **只修 DV、不改臂信息** —— 否决：A 仍有 oracle 目标优势，结论只能限定为
   “oracle 目标 c2 vs 感知驱动分层”，不是预注册的职责分层命题。
4. **改 P1 的窗口口径并重判** —— 本 ADR 不做：P1 已执行且口径版本化；先以敏感性登记，避免
   跨命题连锁改判。

## 后果

- **正面**：P2 的 IV 恢复为“命令源”单一变量；primary 不再被终态后停泊污染；v2 的“证伪”不再被
  当作结论；v3 给出了可采信的判定（证伪、保留 c2）；P1 的窗口风险被显式登记。
- **代价 / 限制**：v3 40 episodes 已完成（约 2.39 h）；A 改为感知目标后成功率 / 时长变化，v3 结果
  不可与 v2 逐值对照；残余的 1.0 s vs 0.5 s 目标保持差异与 goal_info 耦合仍未完全消除。
- **未决 / 后续**：
  - v3 已执行并判定（[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(j)）；P2 消融（协同观测 / 目标预测）未执行；
  - P1 fixed vs active 窗口敏感性复核（W8 前）；
  - 若 v3 仍不成立，是否以“任务成功率 / 捕获时长”为新决策终点——须**新预注册**，不得事后改
    primary（[ADR-0082](0082-w7b-p2-arm-b-rule-high-level-first.md)(d) 的触发条件仍然有效）。

## 参考

- P2 臂定义：[ADR-0082](0082-w7b-p2-arm-b-rule-high-level-first.md)
- 判定记录：[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(i)
- 固定观测窗来源：[ADR-0074](0074-visibility-metric-fixed-observation-window.md)
- 预注册：[`w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)
- 契约：[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics) / [#execution-port](../ROS2迁移交接文档.md#execution-port)
