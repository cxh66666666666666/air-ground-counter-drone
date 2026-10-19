# 空中控制唯一权威与控制许可（R-035）

## Status

accepted

## Date

2026-10-17

## 背景

评审风险 **R-035（空中控制唯一性）**（[10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-035)）指出，
W3 空中链路把「**拓扑数量**」当成「**实际控制权**」的判据，不能证明唯一实际命令者：

- [`trajectory_tracker.cpp`](../../src/agc_air/src/trajectory_tracker.cpp) 的 `tick()` **先**发
  `publishControlMode()` + `publishSetpoint(...)`，**再**用 `countSetpointPublishers() > 1` 置
  `control_conflict` 标志 → **本拍仍发有效 setpoint**（冲突未先于发包）；
- `countSetpointPublishers()`（[`offboard_interface.cpp`](../../src/agc_air/src/offboard_interface.cpp)）
  以图内省 `count_publishers()` 计数，**含静默 publisher**、且 **DDS 发现延迟不保证事件顺序**；
- [`task_manager.cpp`](../../src/agc_task/src/task_manager.cpp) **不订阅** `control_conflict`。

后果：异常重复 writer 仍可能继续争夺，或对**合法静默 publisher 误报**；验收不能只看 `conflict=false`。

契约真源：[#execution-port](../ROS2迁移交接文档.md#execution-port)（**空中控制权**：起飞阶段归起飞控制器；
`release_control` 后由 LQR 轨迹跟踪节点接管）、[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)
**C10**（常开跟踪 + 自身飞行安全）、[#messages](../ROS2迁移交接文档.md#messages)（冻结 9 消息 ABI **不得变更**）；
产物纪律 [ADR-0007](0007-experiment-artifacts-convention.md)；前序 [ADR-0049](0049-lifecycle-repeatable-resource-state-machine-and-liveness-detection.md)（R-034）。

分类：**设计风险（多命令者 / 权威判定）**。本 ADR 决定**显式控制许可（lease）+ claim/ack 顺序 + 冲突已定义安全状态 +
实际写入者记账**；**不**改冻结 9 消息 ABI，**不**变唯一执行口语义，**不**新增第二 ROS 命令者。

## 取证（先学后做）

按 [AGENTS.md](../../AGENTS.md) 铁律「先学后做」，**先**对齐**同版本** Nav2 的成功用法，再改自身实现。

**Nav2 同版本（`/opt/ros/jazzy/include/...`）用法结论：**

- `nav2_controller/controller_server.hpp`：**单权威 = `current_controller_`**（单一**已激活**控制器名，`:261`）+
  **单一输出所有者** `vel_publisher_`（`std::unique_ptr<nav2_util::TwistPublisher>`，`:233`），经
  `publishVelocity` / `publishZeroVelocity`（`:174` / `:175`）唯一出口 → 「**单一输出所有者**」范式。
- `nav2_util/twist_publisher.hpp`：`nav2_util::TwistPublisher` 封装**唯一** `cmd_vel` 发布器，并提供
  `on_activate` / `on_deactivate` **生命周期门控** + `is_activated()` → 「**许可 + 生命周期**」范式
  （与本项目 `OffboardInterface` 的 `sp_pub_` 门控一致）。
- `nav2_util/simple_action_server.hpp`：目标 **accept / reject**（`handle->is_active()` 门控，`:174`）+ 前任
  `terminate(...)`（`:361` / `:486`）→ 「**命令 accept/reject + ack 顺序**」范式（对齐本 ADR 的 claim/ack 顺序）。
- **`twist_mux` 本机未安装**（`/opt/ros/jazzy/share/twist_mux` 不存在）→ 多源优先级 + 超时单权威多路复用在本机无从参照，
  故**以 `controller_server`「单一输出所有者」范式为参照**（如实记录，不臆造）。

**本项目现状差距（读取时工作区）：** `trajectory_tracker.cpp` 冲突置位在 `publishSetpoint` **之后**（病灶）、
计数含静默节点；`countSetpointPublishers()` 无写入记账；`task_manager.cpp` 不订阅 `control_conflict`。

## 决策

### (a) 单一权威 = 显式控制许可（lease），判据「本机是否唯一授权命令者」

**授权（grant）** = 交权路线中的**显式指定** `active_controller == own_name`（话题
`/agc/air/active_controller`，`std_msgs/String`，transient_local；只由 `takeoff_controller` /
`trajectory_tracker` 发布）。**交权顺序**（保持现状，见 (e)）：先 `takeoff_controller` 调
`OffboardInterface::releaseSetpointPublishers()`（**销毁** setpoint / OCM 发布器，真正让出唯一发布者），**再**
`publishActive("trajectory_tracker")`。**禁用**「仅凭发布者计数」决定权威。

### (b) ack（许可声明）= 新话题 `/agc/air/control_permit`

`trajectory_tracker` 订阅并发布**新话题** `/agc/air/control_permit`（`std_msgs/String`，transient_local +
reliable；参数 `control_permit_topic`）。每条 claim 为 JSON（`std_msgs/String.data`）：

```
{"ev":"claim","by":"<commander>","uid":"<self_uid>","epoch":<n>,"permit":<bool>,
 "conflict":<bool>,"writers":<n>,"sp_hz":<r>,"competitor_uid":"<uid|->"}
```

`self_uid` 在节点**构造期**生成（`getpid()` + 随机数）→ 进程实例唯一、可区分**同名重复节点**。

- **对合法静默 publisher 不误报**：只有**显式 claim**（不同 `uid` 且 `by==own_name`）才算竞争者；
  `writers` 计数**仅作告警**，**永不**单独吊销权威。
- **重复节点检出**：观察到 `uid != self_uid` 且 `by==own_name` 的 claim → `competing_=true`（记 `competitor_uid_`）。

### (c) 冲突 → 已定义安全状态（先置位，后决定发包）

`tick()` 重排为：**先**由 `active_controller` 推 `is_authorized_` → `updateConflict()` → `permit = is_authorized_ && !conflict_`
→ 发布 `control_conflict`（**先于**任何 setpoint）→ **无许可则 publishClaim + publishTrackStatus 后 `return`（本拍不发 OCM / setpoint）**。
竞争者 claim 持续 > `conflict_persist_s_` → `conflict_=true`；**冲突期停发** `publishControlMode()` + `publishSetpoint()`
（= **relinquish**，让出 offboard 命令 → PX4 自身 failsafe 保持/降落，即「已定义安全状态」），并复用 R-032 hold 内部状态
（置 `frozen_=true`）。竞争者消失 ≥ `conflict_clear_s_`（新参数，默认 2.0）→ 清冲突、恢复（若仍被授权）。

### (d) 重启不残留旧权威

`on_activate` / `on_deactivate` / `on_cleanup`（及基类转发的 `on_shutdown` / `on_error`）**重置**
`is_authorized_` / `conflict_` / `competing_` / `competitor_uid_` / claim；`on_activate` 后**重新**从当前
`active_controller` 指定推导（新订阅以 transient_local 取得锁存值）。

### (e) 保持交权路线（不新增第二命令者）

`takeoff_controller::onRelease()` 顺序**保持**（先 relinquish 再指定，锚到本 ADR）；`/fmu/in/trajectory_setpoint`
的唯一 ROS 发布器仍只在 `OffboardInterface`。

### (f) 实际 setpoint 写入者 / 频率记账（诊断，非 ABI）

`OffboardInterface` 记账 setpoint 写入次数（`setpoint_write_count_`）与首次 / 末次写入时刻，
暴露 `setpointWriteCount()` / `setpointRateHz()` / `hasSetpointPublisher()`；`cleanup()` 清零。写入
`control_permit` claim / `tracking_status` 诊断 → 供 run 记录回答「谁在写、频率、持续多久」。

### (g) `control_conflict` 对消费方可观测

保持 `std_msgs/Bool` transient_local + reliable；`trajectory_tracker` **仅状态变化时**发布。`task_manager.cpp`
增加**只读**订阅（transient_local）记录该值 + 变化时 `RCLCPP_WARN`；**严禁**改 task_manager 状态机 / 终态语义。

### (h) 验收（实跑台架 + 自包含核查）

新增 [`run_w6_r035.sh`](../../scripts/archive/run_w6_r035.sh) + [`w6_r035_bench.py`](../../scripts/archive/w6_r035_bench.py)
（Phase A **实跑**真节点 + 真 `lifecycle_manager`）+ [`w6_r035_check.py`](../../tests/w6_r035_check.py)
（Phase B 自包含核查）。覆盖：① 迟到 `release_control`；② 重复节点；③ manager 重启；④ 静默 publisher（**不**误报）。

### (i) 边界（不改契约）

**不**改冻结 9 消息 / 服务 ABI，**不**变唯一执行口语义，**不**新增第二 ROS 命令者；`/agc/air/control_permit`
与 track status / run config 字段均为**配套诊断 / 许可**（非 ABI、非命令者）。

## 备选

1. **保持「仅计数 >1 即告警」**：最省事；但计数含静默节点、本拍仍发包、无法背书写者身份 —— 否决 (a)–(c)。
2. **以发布者计数（含计数超时）作为唯一权威**：对静默 publisher / DDS 发现延迟误报，且不能区分合法重复 —— 否决 (a)。
3. **新增一个独立「许可仲裁节点」发布授权**：会引入**第二 ROS 命令者 / 第二权威源**，与
   [#execution-port](../ROS2迁移交接文档.md#execution-port) 冲突 —— 否决 (a)（许可由既有 `active_controller` 承载）。
4. **冲突时只置标志、仍本拍发 setpoint**：即病灶本身（争夺未停）—— 否决 (c)。
5. **要求离线 / 静态唯一（禁止重复节点）**：运行时无法保住；改为**运行时检出 + 已定义安全状态** —— 采纳 (b)(c)。

## 后果

- **正面**：唯一权威由**显式许可**判定（非计数）；迟到 release / 重复节点 / manager 重启均按许可与顺序处理；
  冲突**先置位、后停发**（relinquish → 已定义安全状态）；合法静默 publisher **不误报**；实际写者 / 频率可复核；
  `control_conflict` 对 task_manager 可观测（只读）。
- **代价 / 限制**：新增 `std_msgs/String` 话题与若干诊断字段（**非** ABI、**非**命令者）；claim/ack 依赖 DDS
  发现（本地测试网内成立）；冲突恢复阈值为可标定量（`conflict_clear_s`，默认 2.0 s）。
- **未决 / 后续**：冲突恢复后是否需重新 `reset` 参考相位由 R-036（C1 高度语义）一并裁定；`twist_mux`
  本机未安装，多源优先级多路复用未在本批引入。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-035)
- 规格：[fix-lifecycle-control-height-numeric-r34-r38 spec](../../.trae/specs/fix-lifecycle-control-height-numeric-r34-r38/spec.md)（R-035）
- 契约：[#execution-port](../ROS2迁移交接文档.md#execution-port)、[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C10）、
  [#messages](../ROS2迁移交接文档.md#messages)
- Nav2 参照：`nav2_controller/controller_server.hpp`（`current_controller_` / `publishVelocity` / `TwistPublisher`）、
  `nav2_util/twist_publisher.hpp`（唯一发布器 + 生命周期门控）、`nav2_util/simple_action_server.hpp`（accept/reject + ack）
- 前序：[ADR-0049](0049-lifecycle-repeatable-resource-state-machine-and-liveness-detection.md)（R-034 可重复生命周期 / 失效停机）、
  [ADR-0048](0048-uav-joint-safety-barrier-and-explicit-infeasible-path.md)（空中安全层）、
  [ADR-0008](0008-nav2-paradigm-absorption-as-code-principle.md)（Nav2 吸收）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 实现：`trajectory_tracker.cpp` / `offboard_interface.{hpp,cpp}` / `takeoff_controller.cpp` / `task_manager.cpp` /
  `air.yaml` / `run_w6_r035.sh` / `w6_r035_bench.py` / `w6_r035_check.py`
