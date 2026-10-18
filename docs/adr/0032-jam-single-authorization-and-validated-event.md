# 干扰单一授权与已校验事件（jammer 唯一裁决 / 包络新鲜度 / 复用 jam_state）

## Status

accepted

## 背景

评审风险 **R-017（干扰单一授权与已校验事件）**：干扰授权存在**双路径**——`jammer` 作为注入执行者
按包络裁决，但 `intruder_driver` **直订原始命令话题** `/agc/task/jam_cmd`（`std_msgs/String`）自行判定
`jammed_` 并停发 Offboard 流。证据（`src/agc_task/scripts/intruder_driver.py`）：参数 `jam_cmd_topic` +
回调 `_on_jam`，将原始字符串 `key not in ('', 'NONE', 'CLEAR')` 直接当作「已授权」信号。

后果：
- **授权语义不单一**：`jammer` 的包络裁决（合法 + 包络内）被 `intruder_driver` **旁路**——原始意图
  （即使未知 / 包络外）也能切断 `intruder_driver` 的 Offboard 流，与「干扰仅在作用包络内生效」矛盾。
- **无包络新鲜度**：`jammer` 的 `envelope_in_`（Bool）**无时间戳**——一旦为真即长期有效，包络过期
  （如目标已飞离、`capture_adjudicator` 停发）仍会授权注入。
- **两处各自解析原始命令**：未知命令的判定（`jammer` warn vs `intruder_driver` 视为 jam）不一致。

契约约束（[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) /
[#execution-port](../ROS2迁移交接文档.md#execution-port) / [#params](../ROS2迁移交接文档.md#params)）：
不新增命令者、不新增命令话题、不改冻结 ABI；参数须经 `config/w6_base.yaml` **单一注入**。

已有可复用机制：`jammer` 已在产出 `/agc/task/jam_state`（`std_msgs/String`，如 `JAM_RTL`），
`task_manager` 亦消费该话题（[task_manager.cpp](../../src/agc_task/src/task_manager.cpp)）。**复用**该话题
作为「已校验授权事件」即无需新增话题 / 命令者。

## 决策

### (a) `jammer` 为唯一授权裁决
`jammer` 是干扰授权的**唯一裁决者**：仅当「命令合法」（`key in JAM_MAP`）**且**「作用包络**新鲜**」
才产出**已校验授权事件** `JAM_*`（发布到 `state_topic`，默认 `/agc/task/jam_state`）并 `_send_nav_state`。
未知命令 / 包络外 / 包络**过期（未就绪）** 一律**不产出事件**。

### (b) 包络新鲜度 `envelope_max_age_s`
新增参数 `envelope_max_age_s`（double，默认 `0.5`，[#params](../ROS2迁移交接文档.md#params) 单一注入）。
`_on_env` 除置 `envelope_in_` 外，记 `last_env_rx_ = time.monotonic()`。授权判据 `_envelope_ok()`：
`envelope_in_ and last_env_rx_ is not None and (time.monotonic() - last_env_rx_) <= envelope_max_age_s`。
即包络**过期视同包络外**——与「capture_adjudicator 以 20 Hz 发布、停发即为失效」的语义一致；日志区分
「包络外」与「包络过期 / 未就绪」两种措辞。

### (c) 复用 `/agc/task/jam_state` 作已校验事件；`JAM_NONE` 为释放事件
不新增命令者 / 不新增命令话题。`jam_state` 即**已校验授权事件**：`JAM_RTL / JAM_LAND / JAM_HOLD /
JAM_LOST` = 授权；`JAM_NONE` = **释放**（非注入）。`''/NONE/CLEAR` 输入 → **始终发出** `JAM_NONE`
（释放事件无条件发出，从不受包络 / 新鲜度门控）。`task_manager` 对 `jam_state` 的既有消费
（`jam_reason_ != 'JAM_NONE' → INTERCEPT_CONFIRMED`）**不改**——其虚假确认问题属 **R-018**，另项。

### (d) `intruder_driver` 只消费已校验事件，移除原始直订
- **删除** 参数 `jam_cmd_topic` 与其原始命令订阅（原直订路径移除）。
- **新增** 参数 `jam_state_topic`（默认 `/agc/task/jam_state`），订阅 `std_msgs/String` depth 10 →
  回调 `_on_jam_state`：`key = m.data.strip().upper()`；`jammed = key not in ('', 'JAM_NONE')`；
  其后转移逻辑与 R-016 **不变**（False→True 记 `jam_on_us_`；True→False 且 `jam_on_us_` / `t0_` 均在则
  `t0_ += (now - jam_on_us_)`，日志 `jam CLEAR (rebaseline pause=...)`）。`jammed_`→停发 Offboard 流
  （模拟链路被切断）语义不变（[ADR-0031](./0031-intruder-episode-start-protocol.md)）。

### (e) 不改冻结 ABI
9 个 `agc_msgs` 消息 / 服务 ABI **不动**（沿用 `std_msgs/String`）；不新增地面 / 己方命令者。

## Considered Options

- **保持双路径（driver 直订原始命令）** —— 被否：即 R-017 缺口（授权语义不单一、无新鲜度）。
- **新增专用「已校验事件」话题** —— 被否：违反「不新增命令话题」；复用既有 `/agc/task/jam_state` 更简且
  `task_manager` 已在消费。
- **包络无新鲜度（仅布尔）** —— 被否：包络过期仍授权注入（目标已飞离仍被干扰）。
- **让 `intruder_driver` 自行做包络 / 新鲜度裁决** —— 被否：重开双裁决（两处各自解析），违背「单一授权」；
  裁决归 `jammer`。
- **(a)–(e) 组合** —— **采纳**。

## Consequences

- 干扰授权**单一化**：原始意图（未知 / 包络外 / 过期）不再能切断 `intruder_driver` 流；仅 `jammer` 的
  已校验 `JAM_*` 事件生效（`len(setpoint)` 持续增长、无授权事件）。
- 包络**过期视同包络外**：`capture_adjudicator` 停发后 `envelope_max_age_s` 内即失效。
- `JAM_NONE` 无条件释放：`CLEAR` 后参考从干扰发生时刻连续续上（`|Δ| < 0.3 m`，重基准语义不变）。
- **边界标注 [R-018] 另项**：`jam_state` 的 `JAM_*` 仍会被 `task_manager` 当作
  `INTERCEPT_CONFIRMED`（虚假确认）——**不在本次范围**，交 R-018。
- 未新增 / 修改冻结接口 ABI；未新增命令者 / 命令话题。
- 新增参数（`jammer.envelope_max_age_s` / `intruder_driver.jam_state_topic`）经 `config/w6_base.yaml`
  单一注入（场景覆盖透传），节点不覆写。
