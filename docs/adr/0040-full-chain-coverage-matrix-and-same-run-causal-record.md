# 整链覆盖矩阵与同一 run 因果记录（R-025：backend/mode 如实标注 + pass 命题化）

## Status

accepted

> 注（S-05 / [ADR-0061](./0061-w6-runtime-topology-backend-identity-and-single-parameterized-entry.md)）：W6 顶层入口已统一为 [`w6.launch.py`](../../src/agc_task/launch/w6.launch.py)（`backend` 参数 ∈ lightweight | full），原 `w6_task_eval.launch.py` / `w6_full.launch.py` 退役；本 ADR 历史结论正文不改，下文的旧入口名按现入口读作同一拓扑。

## 背景

评审风险 **R-025（验收整链范围）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-025)）指出：批量入口
[`episode_runner.py`](../../scripts/episode_runner.py) 运行的是**轻量栈**（
[`w6_task_eval.launch.py`](../../src/agc_task/launch/w6_task_eval.launch.py) = `m0_skeleton` + `capture_adjudicator` +
`jammer`，**无** PX4 / 空中链路），却把「期望终态匹配」直接呈现为整链完成；既有整链产物（s02/s03
的 [`full_linear.json`](../../experiments/w6-task-eval/20261014-w6-full-s02/metrics/full_linear.json) /
`full_maneuver.json`）只含运动分类 / armed / nav_state（`jam=null`），**缺** `start OK` / `release_control` /
`CAPTURE` / 真实终态。后果：**「轻量 PASS」可能被误读为「双 PX4 整链 / C3 GUI 验收」**；基准成功率可能来自
更容易的静态真值目标。

相关挑战：**X-02**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-02)，挑战 C3「每份实证都 GUI」的范围）；
**X-04**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-04)，挑战冻结接口禁止任何语义演进）。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C3**（实证 / 验收非无头）、
[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)、[#messages](../ROS2迁移交接文档.md#messages)（9 消息 ABI 不得变更）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一执行口语义不变）、[#params](../ROS2迁移交接文档.md#params)。

**先学后做（取证依据）**：核验本机同版本既有实现——① 两个 composition 的拓扑差异（整链含双 PX4 实例 + 2 个
uXRCE-DDS Agent + 空中节点 + `takeoff_controller` 的 `/agc/air/release_control` 服务端；轻量仅任务层 + 裁决器）；
② 既有整链产物字段缺口；③ 全仓**无**节点自动调用 `release_control`（须由证据收集器显式调用）。
结论：应**如实标注** backend / mode，并以**同一 run 因果记录**补足整链证据。

分类：**设计风险与未决事项**。本 ADR 给出「如何补整链验收证据」的明确决定，**不**回退任何已验收组件。

## 决策

### (a) 覆盖矩阵（backend / mode × 各轴，逐轴标注）

以 `lightweight / full` × `GUI(headless / gui)` × `task_start` × `own_takeover(release_control)` ×
`target_motion` × `real_terminal` 构造覆盖矩阵；每一格标 **covered / uncovered / not_applicable** +
**evidence 出处**（产物路径 / 源文件事实）+ 未覆盖的**如实原因**。矩阵由
[`w6_r025_check.py`](../../tests/w6_r025_check.py) 静态核验源文件（后端能力列）+ 读产物生成，
落 `<run_id>/coverage_matrix.md`。

- **backend 能力列**（源文件事实）：`lightweight` = 0 PX4 实例 / 0 Agent / 无空中节点 / 无 takeover 服务端；
  `full` = 2 PX4 实例 / 2 Agent / 4 空中节点（`px4_adapter` / `takeoff_controller` / `trajectory_tracker` /
  `standoff_follower`）/ 有 takeover 服务端。
- **未执行轴**（如本轮未跑的 `gui` 模式）**一律标 uncovered 并写明原因**；`own_takeover` 在轻量为
  **not_applicable**（源事实：轻量栈不含空中链路），**不**以命名推定其已覆盖。

### (b) 同一 run 因果记录（探针，非节点代码）

[`w6_r025_full_probe.py`](../../scripts/archive/w6_r025_full_probe.py) 在**已运行**的 `w6_full.launch.py` 上执行，
在**同一 run（同一 run_id / 同一进程时间线）**回答全部 5 问：

1. 等 `/agc/air/active_controller` 出现（`takeoff_controller`）→ 再额外等（己方起飞到悬停）；
2. 调 `/agc/air/release_control`（Trigger）→ 等 `active_controller` 翻转为 `trajectory_tracker`（**己方接管证据**）；
3. 调 `/agc/task/start`（Trigger）→ 记 `success` + `message`（`started: AIR_SEARCH` = **start OK**）；
4. **墙钟**看门狗等**任一终态**（超时则如实 `terminal_reached=false`）；
5. 全程记时间线（各事件墙钟时刻 + 仿真时刻 `/clock`）。

产物 `<run_id>/same_run.json`：`causal_record.start_ok` / `own_takeover`（`active_controller_before/after` +
`takeover_ok`）/ `target_motion`（`classified` + `stats`）/ `capture` / `jam`（原始事件串）/
`terminal`（`state_name` + `reason_code` + `reached`），外加 `equipment`（双 PX4 配置 / 2 Agent /
入侵者 armed + nav_state）与 `full_chain_executed`。**所有 bool 均取自真实观测**（值可为 true/false，
**不**伪造）。

### (c) pass 命题化（pass ↔ 明确验收命题强绑定）

- [`episode_runner.py`](../../scripts/episode_runner.py)：每 episode 与 campaign report 加
  `backend="lightweight"` / `launch="w6_task_eval.launch.py"` 标注；每 episode 加 `proposition`
  （由 `expected_terminal` / `expected_reason` / `backend` 生成，如「轻量栈（m0_skeleton+capture_adjudicator+jammer，
  headless）在 s08 达 `INTERCEPT_CONFIRMED(CAPTURE_NET)`（**非整链 / 非 C3 GUI 验收**）」）
  与 `pass`（= `expected_match` **且** backend 已标注）；report 加 `coverage` 汇总块（逐轴 value + evidence）。
- [`w6_r025_full_probe.py`](../../scripts/archive/w6_r025_full_probe.py) 的 `same_run.json` 亦给 `proposition` 与 `pass`。
- **不**把「期望字符串匹配」自动等价为整链 / C3 GUI 验收。

### (d) X-02 边界：只如实标注 backend / mode，**不改**契约 C3 文字

本修复**只**在 run 产物（`report.json` / `same_run.json` / `coverage_matrix.md`）如实标注后端与模式，
**不改**契约 C3 文字、**不**改 [#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) 中「实证 / 验收非无头」的约束。
`gui` 轴本轮未执行的模式如实标 uncovered；headless 产物**只作辅助证据**，C3 仍需 GUI 材料（X-02 的裁定未在本 ADR 消解）。

### (e) X-04 边界：R-021 配套事件不改 ABI

（承接 [ADR-0036](0036-capture-event-episode-isolation-and-event-identity.md)）本修复新增的仅为**非 ROS 接口**的 run
产物与**只读**探针；`/agc/task/capture_event` / `/agc/task/jam_confirm` 仍是 `std_msgs/String` **配套事件**
（**非**命令者）；`/agc/task/capture_hit` 仍**恰为** `CAPTURE_NET`；**未**新增第二 ROS 命令者；**未**改
[#execution-port](../ROS2迁移交接文档.md#execution-port)；冻结 **9 消息**（`src/agc_msgs/`）字段 / 顺序**未改**。

### (f) Runbook `#status` 保留为进度事实

[Runbook](../ROS2迁移步骤.md#status) 的完成状态**保留**为**进度事实**，本修复**不改**该状态表；覆盖矩阵与
整链因果记录**不**据此自动推定已覆盖。

## Considered Options

- **维持现状（轻量 PASS 视同整链完成、无 backend/mode 标注）** —— 被否：即 R-025 缺口，轻量 PASS 冒充整链 / C3 GUI 验收。
- **改契约 C3 文字以允许 headless 验收** —— 被否：越界（属 X-02 契约裁定，非本修复），且违反真源分层。
- **把整链整段纳入批量 runner** —— 被否：整链需双 PX4（重），批量逐次整链成本高，且超本修复范围；改为**同一 run 探针**补证。
- **在报告里直接把「期望匹配」标为 pass 完成** —— 被否：pass 必须与**明确命题**绑定，且与 backend 强关联。
- **(a)–(f) 组合（覆盖矩阵 + 同一 run 因果记录 + pass 命题化 + X-02/X-04 边界 + Runbook 状态保留）** —— **采纳**。

## Consequences

- 「轻量 PASS」与「整链 / C3 GUI 验收」**显式区分**；未执行 / 不适用轴**如实标注**，无「无证据却称已覆盖」。
- 整链证据不再只有运动分类：同一 run 记 `start OK` + 己方接管（`release_control`）+ 目标运动 + 捕获 / 干扰观测 +
  真实终态，且与 `equipment`（双 PX4 / 2 Agent / 入侵者 armed+nav_state）同源。
- **证据驱动、非臆断**：整链 run 的 `pass` 由其命题决定；即使某问为 `false` 亦如实记录（见 Evidence）。
- **ABI / 契约不变**：冻结 9 消息 / 唯一执行口语义 / 契约文字均**未改**；新增仅落 run 产物。
- **只读旧树边界**：本次未读 `reference/`；未新增契约 / ABI 变更。

## Evidence

命令化验收 `bash scripts/run_w6_r025.sh headless 20261016-w6-r025-0`（Phase A 轻量 → Phase B 整链 → Phase C 核查）。

**Phase A（轻量 campaign，s01+s08 seed=0 headless）** → `<run_id>/lightweight/report.json`：
`backend="lightweight"`、`launch="w6_task_eval.launch.py"`；s01 → `TRACKING_SUCCESS`（`expected_match=true`）、
s08 → `INTERCEPT_CONFIRMED(CAPTURE_NET)`（`expected_match=true`）；每 episode 均有 `proposition`，`pass` 与之绑定；
report 含 `coverage`（`backend/gui/task_start/own_takeover(not_applicable)/target_motion/real_terminal`）。

**Phase B（整链 `w6_full.launch.py` 双 PX4，s08 seed=0 headless）** → `<run_id>/same_run.json`（`full_chain_executed=true`）：

| 5 问 | 真实观测 |
|---|---|
| start OK | `start_ok=true`，`message="started: AIR_SEARCH [evid] reset_done=… world=agc_world …"` |
| 己方接管（release_control） | `active_controller_before="takeoff_controller"` → `after="trajectory_tracker"`，`takeover_ok=true`（耗时 ≈0.02 s） |
| 目标运动 | `classified="maneuver"`（`dx≈1.99` / `dy≈2.05` / `n=764`；来自 `/agc/gt/intruder_uav`） |
| 捕获 / 干扰 | `capture.observed=false`、`jam.observed=false`（**未观测**） |
| 真实终态 | `state_name="TRACKING_SUCCESS"` / `reason_code="tracking_milestone"`，`reached=true` |
| equipment | `dual_px4_configured=true` / `px4_agents=2` / `intruder_armed=true` / `intruder_nav_state=14`（OFFBOARD） |

该 run 的 `proposition`（s08 达 `INTERCEPT_CONFIRMED(CAPTURE_NET)` + 捕获可观测）**未被满足** → `same_run.pass=false`。
**如实原因**：整链日志
（`experiments/w6-task-eval/20261016-w6-r025-0/log/launch-attempt-1.log`）显示
`capture_adjudicator: CAPTURE SKIP：CAPTURE_SKIP out_of_applicable_range … h=2.04 > applicable_max_alt_m=2.009`
—— PX4 驱动的入侵者在整链中悬停高度略高于网捕获适用高度上限，故 `CAPTURE_NET` 未触发；任务先以
`tracking_milestone` 结算。**不**伪造捕获证据（此即「轻量 PASS ≠ 整链验收」的具体体现）。

**Phase C（`w6_r025_check.py`，自包含，不起 launch）** → `<run_id>/metrics/r025.json`（`overall_pass=true`）：

| 用例 | 判据 | 结果 |
|---|---|---|
| (a) 轻量标注 | `lightweight` 标注存在；每 `expected_match=true` episode 有 `proposition` | ✅ |
| (b)/(b′) 整链同一 run | `full_chain_executed=true` 且 5 问字段齐全、取自真实观测（值可为 true/false） | ✅ |
| (c) pass ↔ 命题 | 每个 `pass` 均有命题 | ✅ |
| (d) 未覆盖如实标注 | 无「无证据却称已覆盖」；uncovered / N.A. 均有如实原因 | ✅ |
| (e) 冻结 ABI | `hit.data = "CAPTURE_NET";` 存在；`git status --porcelain src/agc_msgs` 为空 | ✅ |

`overall_pass` = (a) ∧ (c) ∧ (d) ∧ (b′) ∧（并入红线回归）(e)。矩阵落 `<run_id>/coverage_matrix.md`；
**未执行轴**（`lightweight/gui`、`full/gui`）如实标 uncovered（本轮未跑 GUI 渲染）。
