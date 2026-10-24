# W7-b P1 窗口口径复核：active-window FCO 为 primary（预注册 v4）

## Status

accepted

## Date

2026-10-23

## 背景

[ADR-0083](0083-w7b-p2-arm-information-equivalence-and-active-window-fco.md) 在修正 P2 时登记了**跨命题风险**：P2 v2 的 primary 被 `window_fixed_v1` 的终态后停泊污染（约 97%/100% 的机会拍在终态之后），而 **P1 使用同一固定窗口**。P1 现行 run `w7-rl-b/20261021-w7-rl-b-175313-3256` 的口径对比为：

| 口径 | 均值配对差（协同 − 非协同） | 95% CI | 配对 t p | 判定 |
|---|---|---|---|---|
| `fire_control_opportunity_rate`（`window_fixed_v1`，60 s） | **+0.2988** | [+0.1490, +0.4485] | **5.1e-04** | 成立（ADR-0071(h) 历史结论） |
| `fire_control_opportunity_rate_active`（`GROUND_APPROACH → 终态`） | **+0.0561** | [−0.0070, +0.1192] | **0.097** | 未达显著；且低于预注册 MDE 0.10 |

同一 run 中约 **92%（协同）/ 86%（非协同）** 的机会拍发生在终态之后（[ADR-0083](0083-w7b-p2-arm-information-equivalence-and-active-window-fco.md) 背景节）。因此 P1 的"成立"目前只在一个**已知被终态后停泊污染**的 DV 上成立；而 P2 的"证伪"已改用 active-window DV。若不在 W8 冻结前统一，主线性结论与副线性结论将建立在**不同口径**上，且主线性结论可能只是"任务结束后在包络里停了多久"的代理。

本 ADR 只做 **P1 的 DV 口径复核裁定**，不改 P1 的臂定义，也不改写 v1 已执行结论。

## 决策

### (a) 新预注册 v4：P1 primary 改为 active-window FCO

[`w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml) 升 `version: 4`、`proposition_versions.p1: 4`：

- **primary** = `fire_control_opportunity_rate_active`（`episode_window_metrics.ticks_fire_control_opportunity / vis_ticks_total`；`GROUND_APPROACH → 终态`，不含终态后停泊）；
- **secondary** = `min_clearance_m`（只报告，不参与判定）；
- **legacy** = `fire_control_opportunity_rate`（`window_fixed_v1`）降为描述量，**不**参与 v4 判定；
- **样本 / 显著性不变**：N=20 配对 / α=0.05 / power=0.80 / **MDE=0.10**（不因观测到更小效应而下调）；
- **臂定义不变**：`p1_coop`（`target` + `air_tracking_ref=on`）vs `p1_noncoop`（`velocity` + `air_tracking_ref=off`），同场景 / 同 seed / 同安全层 / 同终态。

预注册在本 ADR 与配置文件写定**之后**才启动新 run；判定由 [`scripts/w7b_p1_adjudicate.py`](../../scripts/w7b_p1_adjudicate.py) 按 `decision.p1` / `statistics.p1` 执行，脚本不硬编码阈值。

### (b) v1 结论保留为历史，不回溯改写

[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(h) 的 fixed-window 判定继续作为**历史记录**存在，不删除、不改数；v4 结果**另行**记录为 P1 的现行口径结论。两套口径并列报告，避免"跨版本回写"。

### (c) 失败判据与后续

- v4 判定 = 双侧配对 t `p < α` **且** 均值配对差 ≥ MDE 0.10 ⇒ P1 成立；否则**证伪**。
- 若 v4 证伪：主线性结论改为"**P1 在 clean DV（active-window）下未成立**"，不得再以 fixed-window 的 +0.299 作为主结论。
- 若需改以**任务成功率 / 捕获时长 / 首次火控机会时间**为决策终点，须**新预注册版本（v5）**并重新匹配样本量 / MDE；**禁止**事后改 primary（[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)(h)）。
- 若 v4 成立：P1 的适用边界仍为**显式 oracle 指向臂** + 已验证通道族（[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)(i)），不扩展为真实检测 / sim-to-real 结论。

### (d) W8 门禁

W8 的对照与冻结 **SHALL NOT** 在 P1 v4 结果落盘前冻结 P1 结论。P1 的 fixed / active 双口径数值、v4 判定与证据 `run_id` 必须一并进入 W8 的对照报告。

### (e) 执行结果（2026-10-23）：**证伪**

承载 run `w7-rl-b/20261023-w7-rl-b-p1v4-n20`（`backend:=full` / `s08_static_none_nominal_capture` / seeds 0–19 / CRN 同 seed 配对 / 40 episodes / 墙钟 1.73 h）；判定产物同 run `metrics/p1_adjudication.json`（`prereg_version: 4`）。

| DV | 角色 | n 配对 | 剔除 | 均值配对差（协同 − 非协同） | 95% CI | 配对 t p | Wilcoxon p | 判定 |
|---|---|---|---|---|---|---|---|---|
| `fire_control_opportunity_rate_active` | **primary** | 20 | 0 | **+0.0278** | [−0.0113, +0.0669] | **0.153** | 0.498 | **未达 MDE=0.10 且不显著 → 证伪** |
| `min_clearance_m` | secondary | 20 | 0 | +0.1911 | [−0.5000, +0.8822] | 0.570 | 0.729 | 不显著（仅报告） |
| `fire_control_opportunity_rate`（`window_fixed_v1`） | legacy | 20 | 0 | +0.2812 | [+0.1543, +0.4082] | 1.8e-04 | 5.9e-04 | 仅描述，不参与 v4 判定 |

- **口径纪律**：primary 的失败 / infra / 缺值按 0 计入分母（`primary_honored=true`、剔除 0 对、零填 3 = 协同 seeds 4/5 + 非协同 seed 13）；N=20 与预注册一致；阈值 / 样本量 / 臂定义**未改**。
- **infra**：终态 infra **0/40**；**9** 个 `(arm, seed)` 触发 infra 重试（协同 3 / 非协同 6），均恢复；属机器特性，不计作臂效应（[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)）。
- **各臂画像**：协同 **18/20 `INTERCEPT_CONFIRMED`**（seeds 4/5 `MISSION_FAILED`）；非协同 **19/20**（seed 13 失败）。`visibility_rate` 均值协同 0.946 / 非协同 0.389（该量仍为 `window_fixed_v1` 口径，含终态后），不作为判定。
- **结论（现行 P1 结论）**：在 **clean DV（active-window FCO）** 下，**协同观测相对非协同观测的收益未成立**；v1 的 fixed-window"成立"（+0.2988）被确认为**终态后停泊污染的产物**（本 run 同向复现 +0.2812 vs active +0.0278）。适用边界仍为**显式 oracle 指向臂** + 已验证通道族（不涉及真实检测 / sim-to-real）。
- **不得表述**：不得称"协同策略被证明无效"（本命题的自变量是**观测指向与跟踪参考**的组合，非已训练的协同策略）；不得以 legacy 口径数值作为现行结论。
- **三态读法（[ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md)，2026-10-23 补记）**：本节 primary 的 CI95 **[−0.0113, +0.0669]** **完全低于** MME 0.10 ⇒ 按 v6 三态判据为 **`falsified`（可排除"收益达到门槛"）**，与原文"证伪"**结论一致**且判据更严谨；原文 verdict 字段不改写。

### (f) 机制边界（**场景不足以激活 α**；2026-10-23 补记）

上表"证伪"是**按预注册执行的判定**，但它**不等于**"协同无收益"。同一 run 的机制级证据显示，s08 的固定条件使 α 的因果链几乎被**旁路**：

| 量（active 窗口） | 协同臂 | 非协同臂 |
|---|---|---|
| 可见率（`vis_ticks_visible / vis_ticks_total`） | 0.932 | **0.869** |
| 有效火控机会率（primary） | 0.136 | 0.106 |
| 终态 | 18/20 `INTERCEPT_CONFIRMED` | 19/20 |

- **FCO 门 = 包络内 ∧ 目标可见**（[`run_logger.cpp`](../../src/agc_task/src/run_logger.cpp) 的 `evaluateTick`；净空为独立指标、不入此门）。**包络占用由地面车路径决定**——s08 为 `goal_follow_target=false` 且 goal = 目标真值 `(4.5, 3.0)`，**地面路径与感知无关**，故包络占用对两臂**同源**；唯一受 α 影响的因子只剩"可见"，而它只差 6 个百分点。
- 可见之所以接近饱和：目标 **static**、**无遮挡**（`obstacle=none`）、**标称感知**（`perception=nominal`，FOV 100°/70° 与 70°/50°，见 [`perception_sensors.yaml`](../../src/agc_perception/config/perception_sensors.yaml)）→ 即使非协同臂机头随速度扫动，目标仍在视场内约 87% 的时间，**指向的边际价值被压到 ≈6 pt**。
- **结论的读法**：v4 的"证伪"**适用于本场景**（静态 / 无障碍 / 标称感知 / 固定目标）；不得据此声称"协同观测在一般情况下无收益"。机制激活需要场景具备：目标**运动**（跟随才有意义）、**遮挡 / 丢包 / FOV 收窄**（指向才决定可观测性）、或**任务目标跟随感知**（地面路径才依赖观测）。这些条件在 s08 全部缺席。
- **不回溯**：本节只补解释边界，**不**改 (e) 的判定与数值；是否在机制激活场景下另立命题由**新预注册（v5）**承载。

## 备选

1. **不重跑，直接用 v1 run 重算 active-window** —— 否决：新 primary 在数据已被观测后才写定，预注册强度不足；v1 run 仅用于规划（SD 估计）与敏感性登记，不作 v4 判定证据。
2. **保留 v1 结论，只把敏感性写进限制** —— 否决：与 P2 v2 的处置不一致（同一缺陷在副线被作废、在主线被保留），W8 / 论文会冻结一个已知被污染的主结论。
3. **下调 MDE 至 0.05 让 v4 更易通过** —— 否决：事后改阈值，违反 [F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13) 与预注册纪律。
4. **改臂定义（例如两臂都关 `tracking_ref`，只留指向差异）** —— 本 ADR 不做：臂定义变更属于**新命题 / 新预注册**，不得混入口径复核；留待 P1 消融。

## 后果

- **正面**：P1 与 P2 使用同一套 clean DV，跨命题比较恢复一致；主线性结论要么在 clean DV 下成立，要么被诚实地判为不成立；预注册 v4 在 run 前写定，证据链可追溯。
- **代价 / 限制**：需一次 N=20 P1 重跑（40 episodes，full 链，约 1.6–2.4 h）；若 v4 证伪，项目主线结论将发生实质性变化，后续 W8 / 论文需按新结论组织。
- **可能结果**：v4 的效应量若仍 ≈0.056，则即使显著也低于 MDE 0.10 → 按预注册判为证伪；这是**设计内的诚实结果**，不得解释为"接近成立"。
- **未决 / 后续**：P2 消融（协同观测 / 目标预测）仍未执行；W7-c 多 worker 未开始；W8 待 P1 v4 + 消融结论稳定后进入。

## 参考

- 口径风险登记：[ADR-0083](0083-w7b-p2-arm-information-equivalence-and-active-window-fco.md)
- P1 历史判定：[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(h)（fixed-window）
- 固定观测窗来源：[ADR-0074](0074-visibility-metric-fixed-observation-window.md)
- 命题与判定框架：[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)
- 预注册：[`w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)
- 规格：[`.trae/specs/w7-rl-b/`](../../.trae/specs/w7-rl-b/spec.md)
- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)、[#execution-port](../ROS2迁移交接文档.md#execution-port)、[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)
