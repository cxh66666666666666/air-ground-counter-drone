# C10 冻结下游封闭：已获模式、源时间与 offset 寿命（R-032）

## Status

accepted

## Date

2026-10-17

## 背景

评审风险 **R-032（C10 冻结边界）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-032)）指出：
[`standoff_follower`](../../src/agc_air/src/standoff_follower.cpp) 在目标过期时**正确清零 hold 前馈**，
但其**下游** [`trajectory_tracker`](../../src/agc_air/src/trajectory_tracker.cpp) 的失效保护**不封闭**：

- **参考 / 目标年龄都重置为接收 `now`**（原 `tracking_ref_stamp_ = now()` / `target_stamp_ = now()`）→
  旧包重放（`header.stamp` 为过去）会被"变新鲜"，源时间语义丢失。
- **目标门仅 `1 s` 且不检 `confidence`**（原 `(now()-target_stamp_)<1.0`）→ 与契约 C10
  （`ref_max_age_s=0.5`、`confidence≥0.5`）不一致；低 confidence 持续消息仍被当作有效目标。
- **参考超时回落 `base_ref`**（原 `trackingRefFresh() ? tracking_ref_ : base_ref`）→
  **未区分「从未获目标」与「获后故障」**：获目标后 follower 死亡 / 参考超时 → 静默恢复八字。
- **RL `rl_offset` 无寿命 / 无能力限幅、永久叠加**（原 `effective.pos += rl_offset_`）。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C10**（跟踪常开 / 安全间距 /
`ref_max_age_s=0.5` + `confidence≥0.5` 过期即冻结 / 自保）、[#execution-port](../ROS2迁移交接文档.md#execution-port)
（唯一命令者语义不变）、[#messages](../ROS2迁移交接文档.md#messages)（9 消息 ABI 不得变更）；
产物纪律 [ADR-0007](0007-experiment-artifacts-convention.md)。

分类：**设计风险**。本 ADR 只对**空中跟踪的失效保护闭合**做决定，**不**回退已验收组件，
**不**改冻结 ABI，**不**新增第二 ROS 命令者。评审亦明确：**正常 drop-out 的冻结**与
**全链故障保证**应**分列验收**，不得混为一项。

## 取证（先学后做）

按 [AGENTS.md](../../AGENTS.md) 铁律「先学后做」，**先**对齐同版本 Nav2 的成功用法：

- **恢复（recovery）/ hold 语义**：Nav2 `nav2_behaviors/plugins/wait.cpp`（`Wait` 恢复行为）在
  **无有效进展**时进入**显式等待 / 保持**动作，而非静默沿用旧目标继续运动。→ 本批把
  **已获目标后失参考 / 失目标**判为**显式 hold**（保持最后有效位置、前馈清零），
  **不**回落基础八字。
- **传感器新鲜度以源时间 + 有效窗口判据**：Nav2 `nav2_collision_monitor` 的 `source_timeout`
  参数定义为「**Maximum time interval in which data is considered valid**」，且
  `process(..., const std_msgs::msg::Header & header)` 以**数据自身时间基准**判有效；
  `ActionType` 为 `{DO_NOTHING, STOP, SLOWDOWN, APPROACH, LIMIT}`——**显式安全动作**而非静默放行。
  → 本批以**消息自身 `header.stamp`** 计参考 / 目标年龄（**不**以接收 `now` 覆盖），
  并对目标门**叠加 `confidence` 判据**。
- **看门狗 / 就绪协议**：`nav2_lifecycle_manager` 的 bond（同 [ADR-0046](0046-runner-three-layer-acceptance-and-integrity-gate.md)）。
  → 本批的「已获模式 + 显式 hold」即节点自身的**失效看门狗**（失参考/失目标即进入已定义安全状态）。

**本项目现状差距**（读取时工作区）：`trajectory_tracker.cpp` 的参考 / 目标年龄用接收 `now`
（约 `:113` / `:124`）；参考超时回落 `base_ref`（约 `:243`）；`rl_offset_` 永久叠加、无寿命 / 限幅
（约 `:246`）；目标门仅 `1 s` 且不检 confidence（约 `:252`）。`standoff_follower.cpp` 的
`targetFresh()`（`:112-117`，以 `tr.header.stamp` + confidence）与 hold 清前馈（`:134-137`）**已**
符合「先学」结论，故**本批不改**其冻结语义（见决策 (f)）。

## 决策

### (a) 已获模式（`acquired_`）+ 显式 hold，**不**静默回落 `base_ref`

`trajectory_tracker` 记录**是否已获目标**（首次成功采用 `tracking_ref` 时置 `acquired_=true`，并
记 `last_good_ref_`）。参考来源裁定三分：

- `tracking`：参考与目标**均新鲜** → 采用 `tracking_ref`（覆盖基础轨迹）。
- `hold`：**已获目标后**失参考 / 失目标 → 显式 hold（保持 `last_good_ref_` 的位置，**速度 / 加速度
  前馈清零**），**不**回落 `base_ref`（R-032 病灶）。
- `base`：**从未获目标** → 正常基础八字（不占用跟踪参考）。

新 episode（activate / 接管 / 相位复位）重置 `acquired_` / `frozen_` / `last_good_ref_`。

### (b) 参考 / 目标年龄以**源 `header.stamp`** 计（**不**以接收 `now` 覆盖）

`sourceStamp(header.stamp)`：未设源时间（全 0）退回接收 `now`（兼容未置 stamp 的发布者），
否则取**消息自身时刻**。旧包重放（`header.stamp` 为过去）→ 按源时间判年龄 → **过期即拒**，
**不**因接收而"变新鲜"。

### (c) 目标门 = 源时间窗 **且** `confidence ≥ confidence_min`

`targetFresh()` = `age ∈ [0, target_max_age_s_)` **且** `target_confidence_ ≥ confidence_min_`，
默认 `target_max_age_s_ = 0.5` / `confidence_min_ = 0.5`（对齐契约 C10）。**低 confidence 持续消息**
→ 目标视为过期 / 冻结（**不**仅凭"消息新鲜"继续跟踪）。

### (d) `rl_offset` 寿命 / 生效窗口 + 能力限幅

`rlOffsetFresh()`：以**源时间**判寿命（`rl_offset_timeout_s_`，默认 0.5 s，超时归零）。
应用时**幅值限幅** `rl_offset_max_m_`（默认 2.0 m）。**仅正常跟踪模式（`tracking`）生效**——
hold / base 模式**不**叠加 offset，从而**区分**「安全层必要修正」（`shield_active`）与
「追逐旧目标」（hold 源 + offset 不生效表达）。

### (e) 配套诊断（**非** ABI / **非**命令者）

新增 `std_msgs/String` 诊断话题 `/agc/air/tracking_status`（参数 `tracking_status_topic`），每条 tick
发布 JSON：`{acquired, frozen, source(base|tracking|hold), ref_fresh, target_fresh, offset_applied,
offset_clamped, shield_active}`。**不**改冻结 9 消息 ABI；**不**新增第二 ROS 命令者；仅供验收 / 观测。

### (f) 与 `standoff_follower` 的边界

`standoff_follower` 的 `targetFresh()`（源时间 + confidence）与 hold 清前馈**已**符合 (b)/(c)/(a)
的"先学"结论，**本批不改**其冻结语义；R-032 的缺口在**其下游**（tracker 的参考裁定与 offset），
故修复集中于 `trajectory_tracker.cpp`。

### (g) 验收（实跑逻辑台架 + 自包含核查）

新增 [`run_w6_r032.sh`](../../scripts/run_w6_r032.sh) + [`w6_r032_bench.py`](../../scripts/w6_r032_bench.py)
（Phase A **实跑**真实 `trajectory_tracker`，合成驱动发布 odom / active_controller / tracking_ref /
targets / rl_offset，订阅 `/fmu/in/trajectory_setpoint` 重建**有效参考** + `/agc/air/tracking_status`）+
[`w6_r032_check.py`](../../scripts/w6_r032_check.py)（读台架证据 + 静态核查）。四场景：

- **a 已获后失参考**：停发 `tracking_ref`（目标仍新鲜）→ hold（位置保持、**不**回落八字）。
- **b 旧 ref 重放**：`header.stamp` 为 3 s 前 → 按源时间拒 → 保持基础八字（**未**被旧包推移）。
- **c 低 confidence 持续**：目标消息持续但 `confidence < 门槛`，且 `tracking_ref` 持续**变化** →
  冻结（**不**跟随变化的 tracking_ref）。
- **d offset 寿命 / 限幅**：超幅（~17.3 m）→ 限幅到 `rl_offset_max_m`（2.0 m）；源停止（超寿命）→ 归零。

产物落 `experiments/w6-task-eval/<run_id>/{r032/bench.json,metrics/r032.json,log/}`。
**范围标注**：本台架为**逻辑台架**（隔离安全层，`safety.enabled=false`），聚焦 tracker 的参考裁定；
**非**整链 / 非 GUI / 非 C3 验收——**未**把「常开冻结（正常 drop-out）」与「全链故障保证」混为一项。

## 备选

1. **继续用接收 `now`**：最简单、与旧行为一致；但旧包重放会"变新鲜"、源时间语义丢失——否决（(b)）。
2. **目标门继续 1 s 且不检 confidence**：免新增参数；但与契约 C10（0.5 s + confidence）不一致，
   低 confidence 持续消息仍被跟踪——否决（(c)）。
3. **获后失参考仍回落 `base_ref`**：可保持"飞机继续飞"；但正是 R-032 病灶（获后故障退回八字）——否决（(a)）。
4. **offset 永久叠加**：实现最简；但会掩盖真实安全约束、追逐旧目标——否决（(d)）。
5. **引入 QP / CBF 联合安全**：更优但**超出本批**（属 R-033 后续），且非 R-032 病灶——不采纳。

## 后果

- **正面**：获后失参考 / 失目标进入**显式 hold**（**不**静默回落八字）；参考 / 目标年龄以**源时间**计
  （旧包过期即拒）；目标门**叠加 confidence**；`rl_offset` 有**寿命 + 限幅**且**仅**正常跟踪生效；
  区分「安全层必要修正」与「追逐旧目标」的**诊断**可见。
- **代价 / 限制**：新增参数 `target_max_age_s` / `confidence_min` / `rl_offset_timeout_s` /
  `rl_offset_max_m`（承载者登记于 `air.yaml`）；诊断话题 `/agc/air/tracking_status` 为新 `std_msgs/String`
  （**未**改冻结 ABI）。目标门由 `1 s` **收紧为 0.5 s**（对齐契约 C10）。
- **未决 / 后续**：`/agc/air/tracking_status` 仅作诊断；整链（含 PX4 / GUI）的 R-032 行为属 P2 后续；
  台架隔离安全层，安全层联合约束与自保证据属 **R-033**。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-032)
- 规格：[fix-runner-green-freeze-uav-safety-r31-r33 spec](../../.trae/specs/fix-runner-green-freeze-uav-safety-r31-r33/spec.md)（R-032）
- Nav2 参照：`nav2_behaviors/plugins/wait.cpp`（recovery / hold）；`nav2_collision_monitor`
  （`source_timeout` + `ActionType`）；`nav2_lifecycle_manager`（bond 看门狗）
- 前序：[ADR-0046](0046-runner-three-layer-acceptance-and-integrity-gate.md)（runner 三层验收 / 完整性硬门）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 实现：`trajectory_tracker.cpp` / `air.yaml` / `run_w6_r032.sh` / `w6_r032_bench.py` / `w6_r032_check.py`
