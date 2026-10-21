# 空中 lifecycle 启动健壮性：服务预热 + bond 仅在 activate 建立

## Status

accepted

## Date

2026-10-21

## 背景

B1 诊断（[ADR-0079](0079-b2-fire-control-target-motion-estimation.md)(d)）在 `air_perception` 增加了目标门控拒绝原因计数。coop 诊断批次（`w7-rl-b/20261021-w7-rl-b-195638-1456`）取得零可见回合的直接证据：

- `target_present=1.00`，但 `no_pose=306/268/242/228/224`（每 5 s 窗口）→ `/agc/air/odom` 不新鲜；
- `px4_adapter` 在 configure 后日志出现 `与 lifecycle_manager 的 bond 断开 → 失效停机`，随后停止发布 `/agc/air/odom`；
- `lifecycle_manager_air` 停在 `Configuring takeoff_controller` 后再无推进，episode 结束时 SIGINT 5 s 未响应、被升级 SIGTERM；
- `fov_az/fov_el` 仅少量瞬时拒绝（10/58 次），不是主因。

**根因（两层）**：

1. **Nav2 lifecycle manager 的阻塞式服务调用**：`change_state` 走 `nav2_util::ServiceClient`，默认**无限超时**；本机 DDS 发现存在"首个服务调用超时"的已知特性。manager 在节点 lifecycle 服务尚未完成发现时发起 configure，会**永久阻塞**，后续 activate 永不发生。
2. **节点侧 bond 时序过早**：`AgcLifecycleNode` 在 **configure** 阶段创建 bond 并开始 4 s 心跳超时；而 Nav2 manager 只在 **activate** 时才建立其侧 bond。若 configure→activate 间隔 > 4 s（manager 卡住时必然如此），bondcpp 触发 broken → `on_bond_broken()` 失效停机 → 即使 manager 后续恢复也无法产出数据。

## 决策

### (a) manager 启动前预热 lifecycle 服务连接

新增 [`air_lifecycle_prewarm.py`](../../src/agc_air/scripts/air_lifecycle_prewarm.py)：

- 对 `px4_adapter / takeoff_controller / trajectory_tracker / standoff_follower` 各调用一次
  `/<node>/get_state`（3 次重试，容忍首次发现超时）；
- **只预热、不改任何节点状态**；完成后 launch 的 `OnProcessExit` 才启动 Nav2 `lifecycle_manager`；
- 预热失败会打印 `[prewarm] ... unavailable/timed out`，便于直接定位。

### (b) node 侧 bond 改在 **activate** 建立

`AgcLifecycleNode`：

- `on_configure` **不再** `createBond()`；
- `on_activate` 成功后才 `createBond()` + `startBondWatchdog()`，与 manager 的 activate 侧 bond **同拍**；
- bond 失效安全语义**不变**（[ADR-0049](0049-lifecycle-repeatable-resource-state-machine-and-liveness-detection.md)）：已建立的 bond 断开仍触发失效停机。

### (c) 契约边界（**未**改动）

- 不改 Nav2 `lifecycle_manager` 本体，不引入自定义 manager；
- 不改 `bond_timeout`（manager 5 s）/ `bond_heartbeat_period`（节点 0.1 s）；
- 不改空中节点控制逻辑、感知门限、任务难度；
- 唯一新增运行期动作是启动前的 `get_state` 调用（只读）。

## 备选

1. **固定延时启动 manager** —— 否决：R-034 已明确废弃固定延时；延时无法覆盖发现慢的偶发情况。
2. **自写 lifecycle 管理器（带超时/重试）** —— 否决：偏离 ADR-0008 的 Nav2 复用；且需自行实现 bond 监控。
3. **关闭 bond（`bond_timeout=0` / `bond_heartbeat_period=0`）** —— 否决：丢失 ADR-0049 的 manager 失联保护。
4. **提高 bond 超时到 > manager 卡顿时长** —— 不充分：manager 卡死时 activate 永不发生，延长超时只掩盖问题。

## 验证

- 启动日志应出现 `[prewarm] <node>: ok`，随后 manager 依次 `Configuring` 四个节点并进入 `Activating`；
- `px4_adapter` 日志应出现 `已与 lifecycle_manager 建立 bond（bond formed）`（activate 之后），且**不再**出现 configure 阶段的 `bond 断开`；
- `air_perception` 的 `B1 诊断` 中 `no_pose` 应保持 0；
- 行为红-绿：原先出现 bond 断开的 `p1_coop/seed1` 等种子应正常观测并达 `INTERCEPT_CONFIRMED`。
- 承载 run：`w7-rl-b/20261021-w7-rl-b-204755-7986`（`p1_coop` seeds 1/11/15，`backend:=full`）：
  - prewarm 日志 `[prewarm] <node>: ok`（部分节点首次 `get_state` 超时、重试成功——印证 DDS 发现预热假设）；
  - `lifecycle_manager_air` 依次完成四个节点的 **Configuring + Activating**，四节点均 `bond formed`；
  - **0 条 `bond 断开`**；三个种子均 `INTERCEPT_CONFIRMED`（`vis=0.992/0.992/0.996`）；
  - `seed1`/`seed15` 各触发 1–2 次 infra 重试（`air_hover_timeout` / `ACTIVE`），重试后成功；属既有 uXRCE 启动噪声，与 bond 无关；
  - 其余 `no_pose` 窗口均出现在 **px4_adapter 激活前**或 PX4 首帧 odom 到达前，属启动瞬态；最终观测可见率 ~0.99。

## 后果

- **正面**：启动期 lifecycle 卡死不再导致 `px4_adapter` 失效；零可见的链路主因被消除；启动失败有显式 prewarm 日志。
- **代价 / 限制**：启动多了 4 次只读 `get_state`（毫秒级）；manager 若在**已发现**服务上仍丢失应答，理论上仍可能阻塞（Nav2 无限超时），但概率显著降低；本 ADR 不解决该 Nav2 固有限制。
- **未决 / 后续（B1 次因，已裁定保留）**：感知量程 `range_max_m=12.0` 与空中站距带 `[8, 15] m` 不一致 → 飞在最外侧时目标必然超出量程（诊断中 `rmax` 持续数百拍）。**裁定（2026-10-21）：不修改任一参数**——零可见主因已由本 ADR 修复，`rmax` 对两臂**对称**且已被可见率指标如实计入；改参数会改变任务难度 / 观测能力，需新版本重跑，代价与收益不匹配。处理方式：把该几何盲区固化为**环境边界**，并由批次产物 [`metrics/perception_health.json`]（`w7b_arms_runner.py`）自动报告 `rmax/no_pose/fov` 占比。若未来需要更干净的观测条件，另立 ADR 做 v2（站距上限 ≈11.5 m 或量程 16 m，二选一）并两臂重跑。
- **回归防线**：prewarm 日志 + `no_pose` 计数可在批次日志中直接检查；启动期 manager 进度（是否到 Activating）可作为批次健康检查。

## 参考

- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)、[#execution-port](../ROS2迁移交接文档.md#execution-port)
- 前序 ADR：[ADR-0008](0008-nav2-paradigm-absorption-as-code-principle.md)（Nav2 复用）、[ADR-0049](0049-lifecycle-repeatable-resource-state-machine-and-liveness-detection.md)（bond 失效停机）、[ADR-0079](0079-b2-fire-control-target-motion-estimation.md)（B1 诊断计数）
- 代码：[`agc_lifecycle_node.hpp`](../../src/agc_air/include/agc_air/agc_lifecycle_node.hpp)、[`air_bringup.launch.py`](../../src/agc_air/launch/air_bringup.launch.py)、[`air_lifecycle_prewarm.py`](../../src/agc_air/scripts/air_lifecycle_prewarm.py)
