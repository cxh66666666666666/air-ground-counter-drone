# 生命周期可重复资源状态机与失联检测（R-034）

## Status

accepted

## Date

2026-10-17

> 注（S-05 / [ADR-0061](./0061-w6-runtime-topology-backend-identity-and-single-parameterized-entry.md)）：W6 顶层入口已统一为 [`w6.launch.py`](../../src/agc_task/launch/w6.launch.py)（`backend` 参数 ∈ lightweight | full），原 `w6_task_eval.launch.py` / `w6_full.launch.py` 退役；本 ADR 历史结论正文不改，下文的旧入口名按现入口读作同一拓扑。

## 背景

评审风险 **R-034（生命周期恢复）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-034)）指出，
W3 空中链路的生命周期只验证了**首次 autostart**，跨 episode 复用 / 故障恢复的可靠性不足：

- [`agc_lifecycle_node.hpp`](../../src/agc_air/include/agc_air/agc_lifecycle_node.hpp) 的 `AgcLifecycleNode`
  是**只转发四回调**（`on_configure / on_activate / on_deactivate / on_cleanup`）的薄基类，**无**
  `on_shutdown` / `on_error` 清理路径；
- [`trajectory_tracker.cpp`](../../src/agc_air/src/trajectory_tracker.cpp) 在 `configure` 反复
  `declare_parameter`（`cleanup` 不 `undeclare`）→ `cleanup→configure` 二轮可能抛**重复参数异常**；
- [`w6_full.launch.py`](../../src/agc_task/launch/w6_full.launch.py) 与
  [`air_bringup.launch.py`](../../src/agc_air/launch/air_bringup.launch.py) 的 `lifecycle_manager_air`
  **`bond_timeout=0.0`** → **关闭节点失联检测**（且注释自认节点未创建 bond）；
- 部分 timer 为 `wall timer`，退出时 Offboard setpoint 如何**失效停机**未明确。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C10**（无人机对目标**常开**跟踪且
**负责自身飞行安全**）、[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一空中控制权语义**不变**）、
[#messages](../ROS2迁移交接文档.md#messages)（冻结 9 消息 ABI **不得变更**）；产物纪律
[ADR-0007](0007-experiment-artifacts-convention.md)。

分类：**设计风险（恢复可靠性 / 失效安全）**。本 ADR 决定**可重复生命周期资源状态机、shutdown·error 完整清理、
bond 失联检测与失效停机**；**不**改冻结 9 消息 ABI，**不**变唯一执行口语义，**不**新增第二 ROS 命令者。

## 取证（先学后做）

按 [AGENTS.md](../../AGENTS.md) 铁律「先学后做」，**先**对齐**同版本** Nav2（`nav2_util` / `nav2_lifecycle_manager`）
与 `bondcpp` 的成功用法，再改自身实现。

**Nav2 `nav2_util::LifecycleNode`（[`lifecycle_node.hpp`](/opt/ros/jazzy/include/nav2_util/nav2_util/lifecycle_node.hpp)
/ 同版本 `src/lifecycle_node.cpp`）用法结论：**

- 构造期声明 `bond_heartbeat_period`（默认 0.1）并**将 `bond_disable_heartbeat_timeout` 置 true**
  （"server side never times out from lifecycle manager"）；
- `createBond()`：`bond::Bond("bond", get_name(), shared_from_this())` → `setHeartbeatPeriod(period)` +
  `setHeartbeatTimeout(4.0)` + `start()`（**topic=`bond`、id=节点名**）；由派生节点在 `on_configure` 调用、
  `on_cleanup` 调 `destroyBond()`；
- 转发/清理：`on_rcl_preshutdown()` 调 `runCleanups()`（按当前状态 deactivate→cleanup）+ `destroyBond()`；
  `on_error` 默认仅记 FATAL 并返回 SUCCESS（**未**真正实现 error 态）。

**Nav2 `nav2_lifecycle_manager`（同版本 `src/lifecycle_manager.cpp`）用法结论：**

- 参数 `node_names` / `autostart` / **`bond_timeout`（默认 4.0）** / `bond_respawn_max_duration` /
  `attempt_respawn_reconnection`；
- `changeStateForNode(node, TRANSITION_ACTIVATE)` 后调 `createBondConnection(node)`：`bond::Bond("bond", node,
  manager)` + `setHeartbeatTimeout(bond_timeout)` + `setHeartbeatPeriod(0.10)` + `waitUntilFormed(bond_timeout/2)`；
  未形成则报 "Server ... was unable to be reached ... may be misconfigured"；
- `bond_timer_`（200 ms）→ `checkBondConnections()`：若 `isBroken()` → **CRITICAL FAILURE: SERVER ... IS DOWN
  after not receiving a heartbeat for ... ms. Shutting down related nodes** → `reset(true)`（硬复位）+ 可选 respawn。

**`bondcpp` 语义（同版本 `src/bond.cpp`）：** `Bond(topic, id, node, on_broken, on_formed)`；未显式声明时
`bond_disable_heartbeat_timeout` 默认 **false** → 对端心跳超时（收到 `Status` 触发 `Heartbeat()` 重置超时定时器；
超时 `Death()` 触发 `on_broken`）；`isBroken()` 反映状态机 `Dead`。

**本项目现状差距（读取时工作区）：** `AgcLifecycleNode` 仅 `final` 转发四回调，**无** `on_shutdown` / `on_error`；
`trajectory_tracker.cpp` 配置段（约 `:53`）用裸 `declare_parameter`，`on_cleanup_impl`（约 `:196`）只 reset 资源、
**不 undeclare**；两个 launch 均 `bond_timeout=0.0`。另实测确认：rclcpp **静态类型参数不可 `undeclare`**
（`cannot undeclare a statically typed parameter`）——故「注销」只能以**释放资源**（订阅 / 发布器 / 定时器 / 服务）
+ **幂等声明**（`has_parameter` 先判，对齐 `nav2_util::declare_parameter_if_not_declared`）承载。

**本批取证实测（真实二进制 + 真实 `nav2_lifecycle_manager`，`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`）：**

- 被管节点 bas 生成 bond 后，manager 打印 `Server trajectory_tracker connected with bond.` → **bond 兼容成立**；
- 杀被管节点后约 6 s，manager 打印 `SERVER trajectory_tracker IS DOWN after not receiving a heartbeat for 5000 ms.`
  → **失联可判**；
- 杀 manager 后约 4 s，节点侧看门狗打印 `与 lifecycle_manager 的 bond 断开 → 失效停机`，`ros2 lifecycle get`
  回 **unconfigured**，setpoint 停发 → **被管节点进入安全语义**。

## 决策

### (a) 可重复资源状态机（configure 幂等）

基类提供 `declare_param_idempotent<T>(name, default)`：`has_parameter` 先判、已声明则直接读取；被管空中节点
（`px4_adapter / takeoff_controller / trajectory_tracker / standoff_follower`）与 `OffboardInterface` 一律改用该助手。
`cleanup` **释放全部资源**（订阅 / 发布器 / 定时器 / 服务 reset），使 `configure→activate→deactivate→cleanup→configure`
可**重复**（第二轮不抛重复参数异常）。**如实标注**静态类型参数不可 `undeclare`，幂等由 `has_parameter` 守卫承载。

### (b) 转发 `on_shutdown` / `on_error` 做完整清理

基类新增虚函数 `on_shutdown_impl` / `on_error_impl`（**默认复用 cleanup 语义**），并 `final` 转发 `on_shutdown` /
`on_error`：停看门狗 → 运行清理 → `destroyBond`。`on_error` 返回 SUCCESS → 状态回 **UNCONFIGURED**
（对齐 rcl 默认状态机 errorprocessing→unconfigured）。被管节点在 `deactivate / cleanup / shutdown / error`
时**停止产出控制数据**（停定时器、发布器 on_deactivate/reset），**不**残留有效 setpoint。

### (c) bond 失联检测（对齐 Nav2，启用兼容检测）

基类实现 `createBond()` / `destroyBond()`（对齐 `nav2_util`）：`bond::Bond("bond", get_name(), shared_from_this())`、
`setHeartbeatPeriod(bond_heartbeat_period)`、`setHeartbeatTimeout(4.0)`、`start()`；在 `on_configure` 建、`on_cleanup`
销毁。**与 `nav2_util` 默认不同**：节点侧**不**将 `bond_disable_heartbeat_timeout` 置 true，**保留节点侧心跳超时**
——使被管节点可**检测 manager 失联**（安全优先；此为对 Nav2 默认的有意偏离，见备选 2）。

### (d) 失效停机（节点侧看门狗）

`on_activate` 建 250 ms 看门狗轮询 `bond_->isBroken()`；**仅当 bond 曾建立过（`on_formed` 置位）后断开**才触发
`on_bond_broken()`（默认：真实 `deactivate→cleanup` 转移，状态回 UNCONFIGURED，setpoint 停发）。**未**建立过
（无 manager 的独立运行，如既有验收台架）**不**误停。

### (e) launch 配置

`w6_full.launch.py` 与 `air_bringup.launch.py` 的 `bond_timeout` 由 `0.0` 改为 **5.0**（启用与已选节点**兼容**的
bond 失联检测；manager 检测被管节点死亡 → 硬复位）。**不**机械复制参数，注释写明依据本 ADR。

### (f) 验收（实跑台架 + 自包含核查）

新增 [`run_w6_r034.sh`](../../scripts/run_w6_r034.sh) + [`w6_r034_bench.py`](../../scripts/w6_r034_bench.py)
（Phase A **实跑**真实节点 + 真实 `lifecycle_manager`）+ [`w6_r034_check.py`](../../scripts/w6_r034_check.py)
（Phase B 自包含核查）。覆盖：① `configure/activate/deactivate/cleanup` **循环 ≥2 轮**；② **direct shutdown**
（ACTIVE→FINALIZED）；③ **error 注入**（错误类型参数 → configure 回调异常 → 生命周期 error → 完整清理）；
④ **manager / 被管节点死亡**（bond 失联可判 + 失效停机安全语义）。

### (g) 边界（不改契约）

**不**改冻结 9 消息 / 服务 ABI，**不**变唯一执行口语义，**不**新增第二 ROS 命令者；bond 为 lifecycle 配套
（非 ABI、非命令者）。

## 备选

1. **保持薄基类、`bond_timeout=0`**：最省事；但失联不可判、退出清理不明——否决 (a)(c)。
2. **完全照抄 `nav2_util`（节点侧 `bond_disable_heartbeat_timeout=true`）**：manager 死亡时节点**静默继续**，
   与 C10「失效停机 / 不静默继续上游命令」冲突——不采纳；改保留节点侧检测（(c)(d)）。
3. **cleanup 用 `undeclare_parameter`**：实测对静态类型参数抛 `cannot undeclare a statically typed parameter`
   ——不可行；改「释放资源 + 幂等声明」（(a)）。
4. **只做「替代看门狗」不实现 bond**：`bond_timeout` 仍须为 0（manager 会因 bond 不成形而误判），
   且无法复用 Nav2 manager 语义——否决；改实现**兼容 bond**。
5. **`on_error` 返回 FAILURE（默认）**：状态趋 FINALIZED（**不**可再配置）；本批选 SUCCESS→UNCONFIGURED，
   保留可恢复性。

## 后果

- **正面**：`cleanup→configure` 可重复（无重复参数异常）；`shutdown / error` 走**完整清理**（资源全释放、
  setpoint 停发）；被管节点死亡 **manager bond 可判**（"IS DOWN" 硬复位）；manager 死亡 **节点失效停机**回
  UNCONFIGURED（不静默继续上游命令）；`bond_timeout` 不再无条件置 0。
- **代价 / 限制**：新增依赖 `bond` / `bondcpp`（`agc_air`）；节点侧保留心跳超时（偏离 `nav2_util` 默认）
  → manager 重启若超过 heartbeat timeout 会使节点先失效停机（**安全保守**，manager 复位后可重新 configure）；
  静态类型参数**不可注销**（「注销」以释放资源为准）。
- **未决 / 后续**：R-035（单一控制权威）依赖本项；`bond_respawn` 重连策略未在本批启用
  （manager `attempt_respawn_reconnection` 默认 true，本批只验证失联可判，未验证 respawn 恢复）；
  实机独立急停不在 R-034 范围。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-034)
- 规格：[fix-lifecycle-control-height-numeric-r34-r38 spec](../../.trae/specs/fix-lifecycle-control-height-numeric-r34-r38/spec.md)（R-034）
- Nav2 参照：`nav2_util::LifecycleNode`（`createBond` / `destroyBond` / `on_rcl_preshutdown`）/
  `nav2_lifecycle_manager`（`createBondConnection` / `checkBondConnections` / `bond_timeout`）/ `bondcpp`（`Bond` /
  `isBroken` / 心跳超时）
- 前序：[ADR-0048](0048-uav-joint-safety-barrier-and-explicit-infeasible-path.md)（空中安全层）、
  [ADR-0008](0008-nav2-paradigm-absorption-as-code-principle.md)（Nav2 吸收）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 实现：`agc_lifecycle_node.hpp` / `offboard_interface.{hpp,cpp}` / `trajectory_tracker.cpp` /
  `takeoff_controller.cpp` / `standoff_follower.cpp` / `px4_adapter.cpp` / `w6_full.launch.py` /
  `air_bringup.launch.py` / `run_w6_r034.sh` / `w6_r034_bench.py` / `w6_r034_check.py`
