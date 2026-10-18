# 入侵者 episode 起点协议（预热 / epoch / 起始位姿 / 切入与边界 / CLEAR 重基准）

## Status

accepted

> 注（S-05 / [ADR-0061](./0061-w6-runtime-topology-backend-identity-and-single-parameterized-entry.md)）：W6 顶层入口已统一为 [`w6.launch.py`](../../src/agc_task/launch/w6.launch.py)（`backend` 参数 ∈ lightweight | full），原 `w6_task_eval.launch.py` / `w6_full.launch.py` 退役；本 ADR 历史结论正文不改，下文的旧入口名按现入口读作同一拓扑。

## 背景

评审风险 **R-016（入侵者起点）**：`intruder_driver` 按**第一帧 odom** 起算运动，episode reset
**不复位其原点、时间和干扰状态**。证据（评审定位到 `src/agc_task/scripts/intruder_driver.py`）：
`origins_` / `t0_` 只在首帧 odom 初始化一次、**无 task/reset 订阅**；**预热/解锁时间被算进轨迹参数 t**；
`maneuver` 的 t=0 setpoint 直接从原点跳到圆周点 `(ox+radius, oy)`；`linear` **无运行区边界**；
`CLEAR` 继续已推进时间导致 setpoint 跳跃。

后果：相同 scene/seed 的**目标起点依赖启动快慢**，同进程多回合偏差累积；恢复干扰可能产生 setpoint
跳跃；登记的运动轴不是 episode 内统一轨迹；`maneuver` 切入不连续。

契约约束（[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) / [#execution-port](../ROS2迁移交接文档.md#execution-port)）：
`/agc/ground/cmd_vel` 是**唯一命令车体运动**的通道，空中控制权归己方 LQR / 入侵者自身；
`intruder_driver` 只命令**入侵者** PX4，**不新增命令者**。参数须经
[#params](../ROS2迁移交接文档.md#params) **单一注入**（`src/agc_task/config/w6_base.yaml` 为基参、
`config/scenarios.yaml` 为场景覆盖，节点不得覆写）。

已有可复用机制：`scenario_reset` 每次**成功复位**后发布 `/agc/scenario/reset_epoch`
（`std_msgs/Int32`，`rclcpp::QoS(1).transient_local()` → RELIABLE + TRANSIENT_LOCAL，单调递增），
见 [ADR-0030](./0030-scenario-reset-controlled-transaction.md)。该 epoch 即天然的多回合起始信号。

## 决策

### (a) 预热独立于 episode 计时
节点启动即记 `warmup_t0_`（`_now_us()`）；`tw = now - warmup_t0_`。`tw > 1.0 s` 才请求 ARM；`armed_`
且 `nav_state_ != OFFBOARD(14)` 时请求 `VEHICLE_CMD_SET_NAV_STATE = OFFBOARD`（保留 `arm_retry_s` 重试语义）。
**预热时间不进入轨迹参数 t**——轨迹推进只由 episode `t0_` 驱动。依据：PX4 Offboard 要求切 OFFBOARD 前
持续流 > 1 s，这段预热是**链路就绪**而非任务推进（对齐 [ADR-0019](./0019-uav-trajectory-and-tracking-parameter-calibration.md)
的参考轨迹语义）。

### (b) episode epoch 订阅
新增参数 `epoch_topic`（默认 `/agc/scenario/reset_epoch`）。订阅 `std_msgs/Int32`，QoS = depth 1 +
RELIABLE + `DurabilityPolicy.TRANSIENT_LOCAL`（与发布端一致）。收到 **epoch 值变化**
（`last_epoch_ is None or data != last_epoch_`）视为**新 episode**：`last_epoch_ = data`；`origins_ = None`
（在**下一条 odom** 重新锚定原点 = 进入 episode 时的位姿）；`t0_ = None`；`episode_armed_ = True`；
`stable_since_ = None`；`jammed_ = False`；`jam_on_us_ = None`；`terminated_ = False`；日志
`episode armed (epoch=N)`。**重置 intent 不新增命令者**——epoch 只是既有复位事件的消费。

### (c) 起点协议（预热 → 进入起始位姿 → 确认稳定 → episode t=0）
- `_start_pose()` = `_target(0.0)`（见 (d)，并经受同一夹持）。
- **未进入 episode（`t0_ is None`）**：setpoint **恒定** = `_start_pose()`（悬停保持，**不推进**）。
- `t0_ is None` 且 `episode_armed_` 且未 `terminated_` 时做**稳定判定**：`dist(pos_, _start_pose()) <= start_tol_m`
  且 `speed <= start_vel_eps_mps` → 记 `stable_since_`；连续满足 `start_dwell_s` → `t0_ = ts`，日志
  `episode t=0 (epoch=N)`。
- episode 内：`t = (ts - t0_) * 1e-6`，setpoint = `_target(t)`。切换瞬间 `t ≈ 0`，与 `_start_pose()`
  **连续**（无跳跃）。

### (d) 轨迹 / 切入 / 运行区边界
`_target(t)` 基于 `origins_`（NED）：
- `static` → `(ox, oy, z)`；
- `linear` → `(ox, oy + v*t, z)`；
- `maneuver` → `(ox + r*cos(θ), oy + r*sin(θ), z)`，`θ = (v/r)*t`；
- `z = oz - alt_m_`。
即 `maneuver` 的**切入 = t=0 落在圆周定义点 `(ox+r, oy)`**（不再从原点起跳），与起始位姿一致。

**运行区边界**：新参数 `run_region_radius_m`（默认 15.0，`<= 0` 关闭）。setpoint **水平**距 `origins_`
超过 `R` 时，把目标**夹持**到半径 `R` 的边界点，置 `terminated_ = True`，仅日志一次
`[boundary] run region reached R=...`；此后 `t` 继续但 setpoint **停在边界**（越界终止，_不外推_）。
`_start_pose()` 亦经受同一夹持函数，保证起始位姿与推进轨迹一致。

### (e) CLEAR 重基准（参考连续）
干扰输入沿用现有 `jam_cmd_topic`（默认 `/agc/task/jam_cmd`，`std_msgs/String`）——**本决策不改其来源**
（原始意图 → 已校验授权事件的收敛属 **R-017**，另项处理）。`jammed_ True→False`（`CLEAR` / `NONE` / 空）
且 `t0_ is not None` 时：`t0_ += (now - jam_on_us_)`——把暂停时长**加回** `t0_`，使恢复后的参考 `t` 从
**干扰发生时刻**连续续上（**不跳变**）；日志 `jam CLEAR (rebaseline pause=...)`。`jammed_` 守卫使 tick
提前 `return`（停发 Offboard 流 → PX4 failsafe）**保持不变**。

### (f) 其余保持现状
OCM / setpoint 发布、`_send_cmd` 的 `target_system`、停止 Offboard 的 failsafe 语义**均不改**。

## 参数（[#params](../ROS2迁移交接文档.md#params) 单一注入，节点不得覆写）

`src/agc_task/config/w6_base.yaml` 新增 `intruder_driver` 段：
`px4_ns: "px4_1"` / `target_system: 2` / `motion: "static"` / `alt_m: 2.0` / `speed_mps: 1.0` /
`radius_m: 6.0` / `rate_hz: 20.0` / `arm_retry_s: 2.0` / `jam_cmd_topic: "/agc/task/jam_cmd"` /
`epoch_topic: "/agc/scenario/reset_epoch"` / `start_tol_m: 0.4` / `start_vel_eps_mps: 0.35` /
`start_dwell_s: 0.5` / `run_region_radius_m: 15.0`。场景覆盖（`motion` / `speed_mps` / `radius_m`）由
`config/scenarios.yaml` 合并后透传（`w6_full.launch.py` / `w6_task_eval.launch.py` 的 `parameters=common`）。

## Considered Options

- **保持现状（首帧 odom 起算、无复位）** —— 被否：即 R-016 所指缺口（起点依赖启动快慢、多回合累积）。
- **用 `/agc/task/state` 的相位变化当 episode 信号** —— 被否：状态机相位非「复位完成」的权威时刻，且
  start 门禁的 epoch 语义已由 [ADR-0030](./0030-scenario-reset-controlled-transaction.md) 统一到
  `/agc/scenario/reset_epoch`；consume 既有复位事件更简且不新增命令者。
- **让 `maneuver` 从原点起跳、渐进到圆周** —— 被否：使 t=0 定义与场景登记（圆心 + 半径）不符，且与
  「起点 = 定义运动 t=0」语义矛盾；改把 t=0 定义为**圆周定义点**（切入连续）。
- **越界后继续外推** —— 被否：运行区即实验口径边界；改为**夹持 + 终止**（不外推）。
- **CLEAR 后保持 t 连续推进（不复位时间）** —— 被否：会产生 setpoint 跳跃；改为**把暂停时长加回 `t0_`**
  （重基准，参考连续）。
- **(a)–(f) 组合** —— **采纳**。

## Consequences

- `intruder_driver` 起点、时间、干扰态按 episode epoch 复位：相同 scene/seed 的起点**不依赖启动快慢**，
  同进程多回合起点一致（`|S1-S2| < 0.05 m`）。
- 预热/解锁时间**不再**进入轨迹参数 `t`；轨迹推进严格由起点稳定后的 `t0_` 起算。
- `maneuver` 切入落在圆周定义点；`linear` / `maneuver` 均受运行区边界约束（越界终止、不外推）。
- `CLEAR` 后参考从干扰发生时刻连续续上（`|x_after - x_before| < 0.3 m`，未重基准将差 ≈ 暂停时长 × 速度）。
- 未新增/修改冻结接口 ABI；未新增命令者；`jam_cmd` **来源**未动（R-017 另行收敛为已校验授权事件）。
- 新增参数全部经 `config/w6_base.yaml` 单一注入（场景覆盖透传），节点不覆写。
- **风险标注**：本决策的闭环跃迁程度以**已观测 setpoint 流**验证（假 PX4 无真实机体动力学）；真实 PX4
  Offboard 下从起点稳定到 OFFBOARD 接管的行为需整链对拍（同 R-016 原文置信度声明）。
