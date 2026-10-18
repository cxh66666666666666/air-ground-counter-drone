# FSM 命令幂等与终态锁存（R-022：start/abort 允许态 / 终态锁存 + episode 关联 / 非确定性不误置）

## Status

accepted

## 背景

评审风险 **R-022（FSM 重入与终态）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-022)）指出
`task_manager`（[源码](../../src/agc_task/src/task_manager.cpp)）的命令入口**非幂等**：

1. **start 无允许态检查**：`onStart` 顶部无状态判别，立即尝试 `scenario/reset`；**active** 期间重入
   （网络重试 / 重复点击）先 reset；reset 被拒后走 `fail()` → **无条件**置 `INFRA_FAILURE`，覆盖正在进行的运行回合。
2. **abort 无允许态检查**：`onAbort` **无条件** `setState(ABORTED)`；终态之后到达的迟到 abort 会**覆写**已落定终态。
3. **终态无锁存 / 无 episode 关联**：`setState` 唯写 `state_`，终态不记录「归属 episode」——实验记录里
   最后状态会替代真正结果（训练与评估不一致）。
4. **超时与结果不区分**：`callTrigger` 返回 `bool`，把「服务未就绪 / `spin_until_future_complete` 超时」
   与「收到 `success=false` 应答」混为一谈 → 慢应答 / 取消会**误置** `INFRA_FAILURE`。

**先学后做（取证依据）**：ROS 2 服务的「**接受 ≠ 结果**」语义与 `rclcpp::Client` 待决请求清理
（`spin_until_future_complete` 超时后须 `remove_pending_request`，否则 Client 内部状态泄漏，参考本机
`/opt/ros/jazzy/include/rclcpp/rclcpp/client.hpp`）；Nav2 lifecycle 的「命令仅在允许态被接受、否则拒绝并回显」
模式（[ADR-0003](./0003-nav2-pattern-complete-absorption.md) / [ADR-0008](./0008-nav2-paradigm-absorption-as-code-principle.md)）；
契约 [#decisions](../ROS2迁移交接文档.md#decisions)（`task_manager` 架构 = **自研确定性 C++ 状态机**，
**不**引入 `bt_navigator`）、[#params](../ROS2迁移交接文档.md#params)（参数单一注入 / 节点不得覆写）、
[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)、[#execution-port](../ROS2迁移交接文档.md#execution-port)
（唯一执行口 / 不新增第二命令者）；R-021（[ADR-0036](./0036-capture-event-episode-isolation-and-event-identity.md)）
已引入 episode 身份（`episode_epoch_` / `/agc/scenario/reset_epoch`）与配套事件——本 ADR 复用该基座。

分类：**设计风险**。

## 决策

### (a) 允许态矩阵
| 命令 | 接受态 | 拒绝态（回显当前态；不 reset / 不 fail / 不覆写） |
|---|---|---|
| `/agc/task/start` | `IDLE` + 任一终态（`TRACKING_SUCCESS` / `INTERCEPT_CONFIRMED` / `MISSION_FAILED` / `ABORTED` / `INFRA_FAILURE`） | `AIR_SEARCH` / `TARGET_ACQUIRED` / `GROUND_APPROACH`（active） |
| `/agc/task/abort` | active（`AIR_SEARCH` / `TARGET_ACQUIRED` / `GROUND_APPROACH`） | `IDLE` + 任一终态 |

`onStart` 顶部（**任何** reset 之前）判 `isActive(state_)` → 拒绝并从应答回显当前态；
`onAbort` 判 `!isActive(state_)` → 拒绝并回显**已锁存**状态。两者均**不再**触发侧效应。

### (b) 调用结果三分（确定性 vs 非确定性）
`callTrigger` 返回从 `bool` 改为 `enum class CallOutcome { Ok, Failed, Indeterminate }`：
- `Ok` = 收到应答且 `success=true`；
- `Failed` = 收到应答但 `success=false`（**确定性**门禁失败）；
- `Indeterminate` = 服务未就绪（`required` 且 `wait_for_service` 超时）或 `spin_until_future_complete`
  超时 / 取消（**非确定性**）。超时时对 Client 调 `remove_pending_request()` 清理待决请求。

新增参数 `reset_call_timeout_s`（默认 `8.0`，单一注入）替换硬编码超时，reset 与 reset_phase 调用共用；
best-effort（`required=false`）可选调用语义不变。

**关键区分**：`Failed` → 允许置 `INFRA_FAILURE`（确定性门禁失败）；`Indeterminate` → **仅回** `success=false`
应答，**不改** `state_`（不误置 `INFRA_FAILURE`、也不误置 active）。相位复位（C5）同规则。

### (c) 终态锁存 + episode 关联
新增 `latched_state_{0xFF}` / `latched_epoch_{-1}` 与唯一入口 `enterTerminal(s, reason)`：
记录「锁存的终态 + 所属 `episode_epoch_`」后再 `setState`。所有进终态路径统一走 `enterTerminal`——
`tickFsm` 的 `MISSION_FAILED` / `TRACKING_SUCCESS` / `INTERCEPT_CONFIRMED`、`onAbort` 的 `ABORTED`、
`fail()` 的 `INFRA_FAILURE`；非终态转换（`AIR_SEARCH` / `TARGET_ACQUIRED` / `GROUND_APPROACH`）仍走 `setState`。
迟到命令据此**拒绝并回显**，不覆写锁存结果。

### (d) 新 episode 由显式 start 创建
`start` 成功路径清除锁存（`latched_state_ = 0xFF` / `latched_epoch_ = -1`）；`reset` 前记录
`ep_before`，`reset` `Ok` 后**有界等待**（≤0.5s）`current_epoch_ != ep_before`（规避 reset 后 epoch 投递竞态，
超时以 `current_epoch_` 兜底），再置 `episode_epoch_` 并发布 `/agc/task/episode`（`std_msgs/Int32`，
`transient_local`，**非命令者**），供 R-021 配套事件关联；`fail()` **不再**在 active 重入路径被调用。

### (e) ABI / 命令者边界
新增话题仅 `std_msgs` 状态话题 `/agc/task/episode`；**未改**契约冻结 9 消息 / 服务字段 / 顺序
（[#messages](../ROS2迁移交接文档.md#messages)）；`/agc/task/capture_hit` 取值仍恰为 `CAPTURE_NET`
（[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)）；**未新增**第二 ROS 命令者，**未改**
[#execution-port](../ROS2迁移交接文档.md#execution-port) 唯一执行口语义；**不引入** `bt_navigator`（确定性 FSM 仍成立）。

## Considered Options

- **维持旧式（start 无检查 / abort 无条件覆写 / bool 混淆超时与失败）** —— 被否：即 R-022 缺口。
- **引入 `bt_navigator` / 行为树运行时承载命令门控** —— 被否：违反 [#decisions](../ROS2迁移交接文档.md#decisions)
  「`task_manager` = 自研确定性 C++ 状态机，不引入 `bt_navigator`」；本缺口可在既有 FSM 内闭合。
- **超时/取消也一律置 `INFRA_FAILURE`** —— 被否：把**非确定性**当**确定性**失败，慢应答会污染状态
  （正是 R-022 所指「取消与 start 服务响应关联」的反面）。
- **新增带 episode 字段的冻结 9 消息 / 新命令者** —— 被否：违反 [#messages](../ROS2迁移交接文档.md#messages)、
  [#execution-port](../ROS2迁移交接文档.md#execution-port)；episode 标识以新增 `std_msgs` 话题表达。
- **(a)–(e) 组合（允许态矩阵 + 三分结果 + 终态锁存与 episode 关联 + start 创建新 episode + ABI 边界）** —— **采纳**。

## Consequences

- active 期间重入 start 被**拒绝**且**不**再调用 `scenario/reset`；终态之后到达的 abort **不**覆写锁存结果；
  active 内重复 abort **幂等**；慢应答 / 取消（`Indeterminate`）**不**误置状态（`state_` 保持不变）。
- 每个终态携带「锁存终态 + 所属 episode」；新 episode 由显式 start 创建并**发布** `/agc/task/episode`，
  与 R-021 配套事件可关联。
- **ABI 不变**：新增话题均 `std_msgs` 状态；唯一执行口未改；未新增命令者；`capture_hit` 仍恰为 `CAPTURE_NET`。
- **风险标注 / 置信度**：以自包含 harness（假源 + 假 reset 服务，含**慢应答**桩）验证，见 [Evidence](#evidence)；
  命令入口与状态赋值的确定性由契约证据保证（[R-022](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-022) 置信度：高）。
- 只读旧树边界：本次未读 `reference/`；未新增契约 / ABI 变更。

## Evidence

命令化验收 `bash scripts/run_w6_r022.sh headless 20261016-w6-r022-0`（自包含 harness，无 gz / launch），
结果 `experiments/w6-task-eval/20261016-w6-r022-0/metrics/r022.json`（`overall_pass: true`）：

| 用例 | 注入 | 观测（关键判据） |
|---|---|---|
| a active 重入 | start 成功入 `AIR_SEARCH` 后再 start | `success=false`、态仍 `AIR_SEARCH`（非 `INFRA_FAILURE`）、假 reset 调用计数**未增** |
| b 终态后新 episode | `t_max_s=2` 自然到 `MISSION_FAILED` 后 start | `success=true`、态回 `AIR_SEARCH`、`/agc/task/episode` `1→2` **递增** |
| c 迟到 abort | 到 `MISSION_FAILED` 后 abort | abort `success=false`、态**仍** `MISSION_FAILED`（未覆写） |
| d 慢应答 + 超时 | `reset_call_timeout_s=1.5`，假 reset 延迟 3s | `success=false`、`message` 含 `indeterminate`、态**仍** `IDLE`（非 `INFRA_FAILURE`、非 active） |
| e 重复 abort | start→abort→再 abort | 首次 `ABORTED`；二次被拒且态**仍** `ABORTED`（幂等） |
| f 冻结值 / ABI | `grep` 源码 + `git` 工作树 | `hit.data = "CAPTURE_NET";` 存在；`src/agc_msgs` 未改 |

回归：`bash scripts/run_w6_r021.sh headless 20261016-w6-r021-regress` **PASS**
（`experiments/w6-task-eval/20261016-w6-r021-regress/metrics/r021.json` `overall_pass: true`）——episode 隔离 /
配套事件消费语义未回退。
