# 干扰真实行为确认（证据链拆分 / ACK ≠ 效果 / 负例分类）

## Status

accepted

## 背景

评审风险 **R-018（虚假确认，致命）**：`jammer` 发布 `VehicleCommand` 后**立即**发布 `JAM_*`
（`/agc/task/jam_state`），无任何 status / ack 回读；`task_manager` 只要收到「非 `JAM_NONE`」的
字符串即置 `INTERCEPT_CONFIRMED`。证据（`src/agc_task/scripts/jammer.py`）：`_on_cmd` 在包络通过后
`_send_nav_state(nav_state)` 紧接 `pub_state_.publish(String(data=reason))`；`task_manager.cpp` 的
`jam_sub_` 回调 `jam_reason_ = m->data`，FSM 分支 `jam_reason_ != "JAM_NONE" → INTERCEPT_CONFIRMED`。

后果：**PX4 缺席、命令被拒、`target_system` 错误、模式短暂进入后退出**均会报告「干扰成功」→
干扰成功率与核心论文结论失真（评审 Top-1：能直接产生与实际飞控效果无关的拦截成功）。

契约约束（[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) /
[#execution-port](../ROS2迁移交接文档.md#execution-port) / [#params](../ROS2迁移交接文档.md#params) /
[#messages](../ROS2迁移交接文档.md#messages) / [#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)）：
不新增命令者、不改冻结 9 消息 ABI、参数须经 `config/w6_base.yaml` **单一注入**；`INTERCEPT_CONFIRMED`
的 `reason_code` 仍为 `JAM_*`。

同版本取证（[ADR-0012](./0012-px4-body-as-toolchain-dependency.md) pin v1.15.4）：本机
`src/px4_msgs/msg/VehicleStatus.msg` 的 `nav_state` 枚举 `AUTO_LOITER=4` / `AUTO_RTL=5` /
`OFFBOARD=14` / `AUTO_LAND=18`；`VehicleStatus` 由 commander 发布（BEST_EFFORT）。对齐 ROS 2 action
「**goal accepted ≠ result succeeded**」语义（[ADR-0008](./0008-nav2-paradigm-absorption-as-code-principle.md)
吸收矩阵的「结果与接受分离」）。

## 决策

### (a) 证据链拆分：意图 → 授权 → 发送 → ACK → 真实 nav 状态 → 持续行为
`jammer` 仍是干扰**授权 / 确认的唯一裁决者**（[ADR-0032](./0032-jam-single-authorization-and-validated-event.md)）。
在既有 R-017 **授权事件**（`/agc/task/jam_state` 的 `JAM_*`，**语义保留**以驱动 `intruder_driver`）之外，
新增两条 `std_msgs/String` 通道（**非**冻结 9 消息，非命令话题）：
- **`/agc/task/jam_confirm`**（`confirm_topic`）：**仅**在满足确认真实条件时发布一次
  `JAM_RTL|JAM_LAND|JAM_HOLD|JAM_LOST`；
- **`/agc/task/jam_evidence`**（`evidence_topic`）：全量证据 / 负例（可审计）。
授权 / 发送 / ACK 单独**不**构成确认。

### (b) 确认窗口 + 持续行为
`jammer` 订阅入侵者 PX4 `VehicleStatus`（`vehicle_status_topic`，默认 `/{px4_ns}/fmu/out/vehicle_status`，
BEST_EFFORT depth 5）。发送后开**确认窗口**（`jam_confirm_window_s`，默认 `3.0 s`）；**仅当**实际
`nav_state`（`last_status_rx` 在新鲜度阈值内）**等于所命令模式**且**持续 ≥ `jam_sustain_s`**（默认
`1.0 s`）才 `pub_confirm_`；`ACK` = 窗口内首次收到状态流（**仅**证据，**不**确认）。模式中途退出会
**重置**持续计时起点。

### (c) 负例分类（不确认）
窗口到期未确认时按**实际观测**分类（证据落 `evidence_topic`）：
- `JAM_FAIL_TARGET_ABSENT`：从未见过目标 `VehicleStatus`（飞控缺席 / sysid 不符）；
- `JAM_FAIL_NO_FEEDBACK`：曾见过状态但**窗口内无**状态（数据链断续）；
- `JAM_FAIL_MODE_UNREACHED`：窗口内有状态但**未达**目标模式（命令被拒 / `target_system` 不匹配）；
- `JAM_FAIL_NOT_SUSTAINED`：曾达目标模式但**未持续** `jam_sustain_s`（短暂进入后退出）。

### (d) `task_manager` 只认已确认事件
`task_manager` 增参数 `jam_confirm_topic`（默认 `/agc/task/jam_confirm`）；干扰确认订阅改指该话题，
且回调**仅**对正向集合 `{JAM_RTL, JAM_LAND, JAM_HOLD, JAM_LOST}` 置 `jam_reason_`。FSM 分支不变
（`jam_reason_` 置位 → `INTERCEPT_CONFIRMED`，`reason_code = JAM_*`）。**授权事件 `/agc/task/jam_state`
不再被订阅**，故**单独不足以**确认。

### (e) 释放 / 参数 / 不改 ABI
`''/NONE/CLEAR` 释放：取消待确认、`state_topic` 照旧发 `JAM_NONE`、`evidence_topic` 发 `JAM_RELEASE`。
新增参数（`jammer.vehicle_status_topic / jam_sustain_s / jam_confirm_window_s / confirm_topic /
evidence_topic`；`task_manager.jam_confirm_topic`）经 `config/w6_base.yaml` **单一注入**（launch 透传），
节点不覆写。不改冻结 9 消息 / 服务 ABI（新通道均为 `std_msgs/String`）；不新增第二 ROS 命令者。

## Considered Options

- **保持现状（发送即 `JAM_*` 即确认）** —— 被否：即 R-018 缺口（无 status / ack 回读即成功）。
- **仅凭 ACK 确认（收到任一 `VehicleStatus` 即成功）** —— 被否：ACK ≠ 效果；PX4 可能拒命令 / 模式瞬进瞬出。
- **以「达到模式」为唯一判据、不要求持续** —— 被否：模式短暂进入后退出仍会误判（R-018 明列）。
- **改冻结 9 消息或新增命令话题** —— 被否：违反 #messages / #execution-port；改用 `std_msgs/String` 状态 / 事件。
- **让 `task_manager` 自行订 `VehicleStatus` 判定** —— 被否：重开双裁决，违背「`jammer` 单一授权 / 确认者」。
- **(a)–(e) 组合** —— **采纳**。

## Consequences

- 干扰**确认**与**授权**分离：`task_manager` 的 `INTERCEPT_CONFIRMED` 只由真实 `nav_state` 持续达标驱动；
  授权 / 发送 / ACK 单独不再结算。
- PX4 缺席 / 无反馈 / `target_system` 不匹配 / 命令被拒 / 模式短暂进入 均产出**负例事件**、**不**确认
  （干扰成功率不再被虚假确认污染）。
- `/agc/task/jam_state`（R-017 授权）语义**保留**（`intruder_driver` 停发 Offboard 链路不变）；新增两条
  证据通道为 `std_msgs/String`，未改冻结 ABI；未新增命令者。
- 新增参数全部经 `config/w6_base.yaml` 单一注入（场景覆盖透传），节点不覆写。
- **风险标注 / 置信度**：确认闭环以**假 PX4 的回读状态流**验证（[#evidence](#evidence)），未含真实
  PX4 Offboard 动力学与真实电磁场；真实 SITL 整链对拍留待后续（与本仓既有 harness 同口径）。
  R-019（failsafe 因子分离）与 R-020（占位包络标注）为**另项**，不在本决策范围。

## Evidence

命令化验收 `scripts/run_w6_r018.sh headless 20261016-w6-r018-0`（自包含 harness，无 gz / launch），
结果 `experiments/w6-task-eval/20261016-w6-r018-0/metrics/r018.json`（`overall_pass: true`）：

| 用例 | 注入 | 观测 |
|---|---|---|
| a 无状态 | 合法 + 包络新鲜 | 授权 `JAM_RTL`；负例 `JAM_FAIL_TARGET_ABSENT`；**无** `jam_confirm` |
| b1 `target_system` 不匹配 | jammer 发 sysid=99，假 PX4 仅认 2 | `JAM_ACK nav_state=14` → `JAM_FAIL_MODE_UNREACHED`；不确认 |
| b2 无状态流 | jammer 订无发布者话题 | 负例 `JAM_FAIL_TARGET_ABSENT`；不确认 |
| c 未达模式 | 有状态（恒 14） | `JAM_FAIL_MODE_UNREACHED`；不确认 |
| d 短暂进入退出 | 达标 0.4 s < `jam_sustain_s`(1.0) | `JAM_FAIL_NOT_SUSTAINED`；不确认 |
| e 持续 ≥ `jam_sustain_s` | 达标并保持 | `jam_confirm` 出 `JAM_RTL`；`task_manager → INTERCEPT_CONFIRMED`（`reason_code=JAM_RTL`） |
| f 仅授权事件 | 仅发 `/agc/task/jam_state` | `task_manager` **不**进入 `INTERCEPT_CONFIRMED` |
