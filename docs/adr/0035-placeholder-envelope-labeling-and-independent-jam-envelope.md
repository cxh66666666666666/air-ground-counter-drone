# 占位包络标注与独立干扰作用包络（R-020：高度含车体 / 模型身份与适用范围 / 捕获≠干扰）

## Status

accepted

## 背景

评审风险 **R-020（占位包络物理含义）**（[10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-020)）指出
`capture_adjudicator`（[源码](../../src/agc_task/src/capture_adjudicator.cpp)）现式存在三处失真：

1. **高度未计车体**：旧式 `h = target.z − launcher_h`（**未减 `car.z`**）——改变地形 / 车体高度即造成
   错误裁决；`car.z` 实际来自 `/agc/ground/odom` 的 `pose.pose.position.z`。
2. **固定 `v0/θ` 点弹道被扩成整圆盘且无范围标签**：由固定发射初速 / 角推出的某高度弹道**交点**，
   不意味着该高度下**所有** `dist ≤ R` 都可及；默认 `6.5 m/s`、`75°` 的**点弹道解析顶点仅约 `2.009 m`**
   （未计展开时长 `t_exp` 与网几何），而常见目标高度为 `2.0 m` → 真实目标高度可能逼近 / 超过可达高度；
   旧实现**无模型身份与本模型适用范围**标签，读者无从判断裁决是否落在模型有效域。
3. **捕获与干扰共源**：`capture` 与 `jam` 共用同一 `/agc/task/envelope_in` 包络（[ADR-0024](0024-capture-adjudication-fire-control-window.md)
   (b) 已定「分离干扰作用包络」方向），令两类**不同作用模型**混淆。

分类：**设计风险**。[ADR-0024](0024-capture-adjudication-fire-control-window.md) 已主动承认现规则为**闭式占位**；
本项**不**把缺失完整火控误报为未声明违约，要求贯彻该限制。

**先学后做（取证依据）**：[ADR-0022](0022-launched-net-capture-mechanism.md)(b) 的网发射物理模型与
[ADR-0024](0024-capture-adjudication-fire-control-window.md)(b) 的「后续火控窗口 / 瞄准提前量 / 分离干扰包络」
方向；同版本已落地做法（可插拔 `CaptureAdjudicator` 抽象、`transient_local` latched `capture_hit`、单一注入参数；
前序 R-018/R-019 harness 的桩环境结构）——本变更在既有结构内落地，**不**新增 ABI。

## 决策

### (a) 高度**含车体**
`h = target.z − car.z − launcher_h`（R-020；`car.z` 取 `/agc/ground/odom`）。保留 `h ≥ 0` 与可达判据
（`vy² ≥ 2gh`，即 `disc = vy² − 2gh ≥ 0`）。

### (b) 模型身份与适用范围标签
`capture_adjudicator` 增参（`launch/config` **单一注入**，节点不得覆写；契约 [#params](../ROS2迁移交接文档.md#params)）：
`model_id`（默认 `"ballistic_closed_form"`）、`applicable_min_alt_m`（默认 `0.0`）、
`applicable_max_alt_m`（默认 `2.009` ≈ 当前默认点弹道解析顶点 `(v0·sinθ)²/(2g)`，占位；未计 `t_exp` / 网几何）。
- **命中**：`/agc/task/capture_hit` 取值仍**恰为** `CAPTURE_NET`（契约 [#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)，
  `task_manager` 精确匹配，**不得**追加标签）；同时在新证据话题 `/agc/task/capture_evidence`（`std_msgs/String`）
  发 `CAPTURE_HIT CAPTURE_NET model_id=.. applicable_min_alt_m=.. applicable_max_alt_m=.. h=.. dist=..`。
- **越界 / 不可达**：目标高度超出 `[applicable_min_alt_m, applicable_max_alt_m]` **或**不可达（`disc<0`）→
  **不判命中**，证据发 `CAPTURE_SKIP <label> h=.. dist=.. model_id=..`（`out_of_applicable_range` / `unreachable`），
  `envelope_in` 相应为 `false`；**限频**（同一状态跳变只发一次），避免每拍刷屏。

### (c) **独立干扰作用包络**（捕获 ≠ 干扰）
新增**独立**干扰包络模型（与网弹道模型**不同源**）与话题 `/agc/task/envelope_jam`（`std_msgs/Bool`，可经
`jam_envelope_topic` 覆盖），参数组 `jam_envelope_*`：`jam_model_id`（默认 `"behavioral_range_gate"`）、
`jam_envelope_min_alt_m`（0.0）、`jam_envelope_max_alt_m`（2.0）、`jam_envelope_max_range_m`（1.0）。
判据：`hj = target.z − car.z`，当 `jam_envelope_min_alt_m ≤ hj ≤ jam_envelope_max_alt_m` **且**水平距离
≤ `jam_envelope_max_range_m` 为真。**捕获作用包络保持在 `/agc/task/envelope_in`**，仍用原网弹道（闭式占位）模型。
`jammer` 门控话题（`envelope_topic`）改指 `/agc/task/envelope_jam`，`envelope_max_age_s` 新鲜度门控沿用（[ADR-0032](0032-jam-single-authorization-and-validated-event.md) / R-017）。

### (d) 可达性核验（**不**倒调参数）
以**真实目标高度记录**（场景默认 `scenario_reset.target_z`）核验可达性：`r020.json` 写
`reachability_check` 块（`real_target_alt_m` / `reachable_max_alt_m` / `reachable` / `note`）。
若真实目标高度超过该点弹道可达高度，**如实报告**（模型当前不可达），**不**以结果期望反推参数制造命中。

### (e) 不改 ADR-0024 的后续火控口径 / 不改 ABI
本次**仅**：标注占位 + 修正高度 + 分离干扰包络。**真实火控 / 弹道 / 网展开（含 `t_exp`、瞄准提前量、失手 /
冷却）仍归后续 workstream**（[ADR-0024](0024-capture-adjudication-fire-control-window.md)(b)）。
新增 / 变更通道均为既有 `std_msgs`（证据 / 包络），**未**改契约冻结 9 消息 / 服务 ABI，**未**新增第二 ROS
命令者，**未**改唯一执行口（[#execution-port](../ROS2迁移交接文档.md#execution-port)）语义。可插拔裁决器结构
（`CaptureAdjudicator` 抽象 / `EnvelopeAdjudicator`）与 `/agc/task/capture_hit` 的 `transient_local` latched
发布者保持不变。

## Considered Options

- **维持旧式（高度未计车体 + 无范围标签 + 捕获干扰共源）** —— 被否：即 R-020 缺口（物理含义失真 / 不可复核 / 作用模型混淆）。
- **本轮直接实现真实弹道 / 火控窗口** —— 被否：受工期约束，且会改动已验收 R9/R11 口径（[ADR-0024](0024-capture-adjudication-fire-control-window.md)）。
- **把 `applicable_max_alt_m` 倒调至覆盖期望目标高度以制造命中** —— 被否：违反「不以结果期望倒调参数」，属数据造假。
- **(a)–(e) 组合（标注占位 + 修正高度 + 分离干扰包络，火控留后续）** —— **采纳**。

## Consequences

- **如实限制**：本模型为**占位**闭式点弹道，默认可达顶点约 **`2.009 m`**（未计 `t_exp` / 网几何 / 瞄准提前量）；
  **不得**依据本占位结果在报告 / 论文中声称「移动式主动拦截」（承接 [ADR-0024](0024-capture-adjudication-fire-control-window.md)(a) 表述纪律）。
  由于 `applicable_max_alt_m` 取顶点，`[0, apex]` 内恒可达（`disc ≥ 0`），故 `unreachable` 标注实际为**防御性**
  分支；超出顶点即命中 `out_of_applicable_range` 标注。
- 高度含车体后，地形 / 车体高度变化不再造成错误裁决；目标在捕获包络内但不在独立干扰包络内时，干扰
  **不**生效（两作用模型不再共源），消解「捕获必先触发 → `先命中者胜` 退化」的一半病灶。
- **风险标注 / 置信度**：以自包含 harness（假 odom / 目标源 + 回读标注话题）验证，见 [Evidence](#evidence)；
  真实火控 / 弹道整链对拍留待后续（与本仓既有 harness 同口径）。
- 只读旧树边界：本次未读 `reference/`；未新增契约 / ABI 变更。

## Evidence

命令化验收 `scripts/run_w6_r020.sh headless 20261016-w6-r020-0`（自包含 harness，无 gz / launch），
结果 `experiments/w6-task-eval/20261016-w6-r020-0/metrics/r020.json`（`overall_pass: true`）：

| 用例 | 注入 | 观测 |
|---|---|---|
| a 车体高度计入 | `car.z=1.0`、`launcher_height_m=0.0`、目标 `z=1.0` 近距 | 命中 `capture_hit=CAPTURE_NET`；证据 `h≈0.0`（旧式会得 `1.0`）；带 `model_id` / 适用高度 |
| b 超适用范围 | 目标 `z=3.0`（>`applicable_max_alt_m`），`car.z=0` | **无** `capture_hit`；证据含 `out_of_applicable_range`；`envelope_in=false` |
| c 捕获 ≠ 干扰 | 目标在捕获包络内（水平 ≈1.2 m、`h≈0`）但 > 干扰包络水平门 | `envelope_in==true` 且 `envelope_jam==false`；发 `RTL` 后 jammer **无**任何 `JAM_*` 授权，日志表明因包络不生效被忽略 |
| d 可达性核验 | 读 config 解析顶点 + 场景默认目标高度 | `r020.json` 含 `reachability_check` 块；ADR-0035 存在且含占位 / 限制表述 |
