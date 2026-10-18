# 场景复位受控事务与失效安全（R-015；含 X-03 门控裁定）

## Status
accepted

## 背景

评审风险 **R-015（复位事务）**：`scenario_reset` 的复位只有**两次非原子 `set_pose`**——车、目标依次
写入，**目标写入失败会留下「车已移动、目标未复位」的中间态**；无写入后**读回**核验；
`isResettable` **漏 `INTERCEPT_CONFIRMED`**（成功拦截后同进程下一回合不可复位）；启动 4 s 定时器
以 `doReset(true)` **bypass 门控**，早启动时可被初始复位覆盖（契约 [#lessons](../../docs/ROS2迁移交接文档.md#lessons)
要求「保留门控」）。同时评审 **X-03** 指出契约 [#task-gating](../../docs/ROS2迁移交接文档.md#task-gating)
「非运行期不得写位姿」与实现允许 IDLE/终态复位、拒绝 active 的**方向相反**，文字歧义须显式裁定。

本 ADR 记录三项决策：**(a) 复位受控事务设计**、**(b) X-03 门控窗口裁定（措辞澄清，非语义变更）**、
**(c) Nav2 lifecycle 资源可重建范式的吸收落点**。

---

## (a) 复位受控事务设计

`/agc/scenario/reset` 改为**受控事务**（对齐 Nav2 lifecycle 的「显式状态窗口 + 资源可重建」语义）：

### (a-1) 门控窗口（撤销运动许可）
`isResettable` **补齐为全部「非 active」态**：`IDLE + TRACKING_SUCCESS + INTERCEPT_CONFIRMED(7)
+ MISSION_FAILED + ABORTED + INFRA_FAILURE`。active（`AIR_SEARCH / TARGET_ACQUIRED / GROUND_APPROACH`）
**一律拒**（理由码 `not_resettable`）。门控通过即**撤销运动许可**——非 active 期间无授权命令者
（RL 仅在 `GROUND_APPROACH` 驱动，见 `src/agc_rl/agc_rl/inference_node.py`；其余命令源同样受
任务门控），故「停止执行」在策略层面已成立。

### (a-2) 停止执行 = 门控 + 车静止证据（**不新增命令者**）
按契约 [#execution-port](../../docs/ROS2迁移交接文档.md#execution-port)，`/agc/ground/cmd_vel` 是
**唯一命令车体运动**的通道，落地者为 `ground_drive_relay`；唯一例外是**场景复位的位姿写入**。
因此 `scenario_reset` **禁止发布任何 cmd_vel**（**SHALL NOT** 成为第二命令者）。「停止执行」以
**门控（非 active）** + **车静止证据**实现：复位前订阅 `/agc/ground/odom`（`nav_msgs/Odometry`），
取 twist 线速度模长，做**有界等待**（上限 `stop_exec_max_s`，默认 0.5 s），速度 < `quiescence_eps`
（0.05 m/s）视为静止并记录证据。该等待**非致命**：残余为滑行，且随后复位 teleport 会中和；非静止
仅记 `car_not_quiescent` 证据（速度 / 等待时长），不阻断复位。

### (a-3) 读回前置 + 原子事务 + 回滚
- **读回前置**：订阅真值旁路 gt（`car_gt_topic` / `target_gt_topic`，world 系 `PoseStamped`），
  缓存**最新位姿**作为「写入前状态」与回滚目标；要求**写入前该缓存新鲜**（年龄 ≤ `readback_max_age_s`），
  否则判 `gt_stale` 拒绝。
- **事务顺序**：写车 → **读回**（对 gt 最新位姿，XY 欧氏距 ≤ `readback_tolerance_m`；有界等待读回更新，
  上限 `readback_timeout_s`）→ 写目标 → 读回校验。
- **失败处理**：任一步失败 → **回滚**已写入实体到「写入前」位姿；**回滚再失败** → 置**持久**标记
  `env_needs_rebuild_` 且 reason 后缀 `env_rebuild_needed`。**绝不留「车已移动、目标未复位」中间态**。
- 失败 reason 码简洁可判：`not_resettable` / `no_candidate`（沿用 R-014）/ `gt_stale` /
  `car_set_pose_failed` / `target_set_pose_failed` / `readback_mismatch_car` /
  `readback_mismatch_target` / `rollback_failed`（+ `env_rebuild_needed`）。

### (a-4) epoch 与 car_start 仅成功发布
`car_start`（`/agc/scenario/car_start`）**仅在完整成功后**发布；每次成功复位 `reset_epoch_++` 并发布
`/agc/scenario/reset_epoch`（`std_msgs/Int32`，transient_local depth 1），`car_start.header.stamp = now()`，
服务应答含 `epoch=N`（失败保持 `reset rejected: <reason>`）。

### (a-5) 启动初始复位走门控
启动 4 s 定时器改 `doReset()`（经**同一门控**，**SHALL NOT** bypass），保留延迟等 gz / 桥就绪。
早启动时若任务仍为 active，初始复位被拒（不再覆盖）。

### (a-6) 执行器
`main()` 由 `rclcpp::spin` 改 **`MultiThreadedExecutor`**；复位服务 + 启动定时器置**独立互斥回调组**，
订阅留默认组——事务回调内的有界阻塞（静止观察 / 读回等待）期间，gt / odom 缓存仍由其它线程持续刷新
（对齐 [ADR-0028](./0028-task-start-transaction-and-freshness-gate.md) 执行器范式）。

---

## (b) X-03 门控窗口裁定（**措辞澄清**，非语义变更）

**契约文字**：[#task-gating](../../docs/ROS2迁移交接文档.md#task-gating) 「`scenario_reset` 受**运行期门控**
（非运行期不得写位姿）」。**歧义**：「运行期」未定义——可指 (i) launch **会话**存在，或 (ii) **任务已获
运动许可（active）**期间。现实现（IDLE / 终态可复位、拒 active）与字面 (i) **方向相反**，且 [#lessons](../../docs/ROS2迁移交接文档.md#lessons)
要求「保留门控」、[#hard-constraints](../../docs/ROS2迁移交接文档.md#hard-constraints) C7 又要求 start
前复位（计时前）——遵守一句可能违反另一句。

**裁定**：把契约 [#task-gating](../../docs/ROS2迁移交接文档.md#task-gating) 的「运行期」明确为
**(ii) 任务已获运动许可（active）期间**。于是：

> **复位允许窗口 = 非 active（`IDLE` + 全部终态，含 `INTERCEPT_CONFIRMED`）+ 受控事务。**
> **禁止窗口 = active（`AIR_SEARCH / TARGET_ACQUIRED / GROUND_APPROACH`）**——批评的正是「active 飞行中
> 任意写位姿」。

**理由**：
1. 门控的**意图**是防「非运行期写位姿覆盖 `car_start`」（[#lessons](../../docs/ROS2迁移交接文档.md#lessons)
   原话），而 `car_start` 仅在复位成功后发布——意指「active 期间不得被复位覆盖」。故「运行期」语义应
   锚定 **active（运动许可）**，而非 launch 会话。
2. C7 要求 start 前复位（episode 起始把轨迹相位 / 位姿复位到 t=0），此刻任务尚未 active——若「运行期」
   取 (i)，则 C7 与门控**直接冲突**、复位无从发生。
3. 复位在非 active 下执行**撤销运动许可 + 受控事务 + 读回回滚**，安全语义反而**强于**「非运行期不得写」
   的字面禁令；(b) 窗口明确后，实现方向与契约意图一致。

**性质声明**：本裁定为**措辞澄清**（明确未定义词「运行期」的所指），**非语义变更**——不改变「非运行期
不得写位姿」的安全目标，只消除「运行期」在会话/active 之间的歧义；实现方向（非 active 可复位）在新
口径下即为契约意图。**本任务不静默改契约文字**：仅以本 ADR 记录裁定并显式报告（供后续单独裁定契约
文字是否需回填 `#task-gating`）。裁定前，`#task-gating` 原文与实现的冲突以本 ADR 为准。

---

## (c) 先学后做（取证）

### (c-1) Nav2 lifecycle cleanup / configure「资源可重建」语义
核验本机 `nav2_util` / `nav2_lifecycle_manager`（Jazzy 头文件）：
- `nav2_util::LifecycleNode` / `LifecycleServiceClient`（`change_state` + `get_state`，`nav2_util/service_client.hpp`）。
- `nav2_lifecycle_manager`：`reset(hard_reset)` → 归一至 **UNCONFIGURED**（已知安全休止态）；`configure()`
  → 重新创建资源；`startup_lifecycle_nodes` / `reset_lifecycle_nodes` 对每次状态迁移**带超时重试**
  （`retries` 默认 3，因服务调用可能挂起）。
衰减口径：**复位 = 可重复的受控事务**——非 active 为「已知安全窗口」，进入后做**原子写入 + 读回**，
失败则**回滚到写入前状态**（或标记环境需重建）；成功后发 epoch + `car_start` 作为「已重新就绪」信号。

### (c-2) GZ `set_pose` 语义（实测）
`ros_gz_interfaces/srv/SetEntityPose`：`req.entity.name` / `req.entity.type`（`Entity.msg`：
`MODEL=2`）/ `req.pose` → `bool success`。**实测结论**（起 gz 世界 + set_pose 桥，`ros2 service call`）：

| 实体名 | 类型 | 返回 |
|---|---|---|
| `no_such_uav`（不存在） | MODEL | `success=True` |
| `pole_1`（存在） | MODEL | `success=True` |
| `''`（空名） | MODEL | `success=True` |

即 **GZ `set_pose` 对不存在的实体名仍返回 `success=True`**——`success` 标志**不**能判定写入是否真正
作用到目标实体。故**故障注入改以「读回」判失败**：预约 `target_entity_name=no_such_uav`，写后读回
`/agc/gt/intruder_uav` 未到新位姿 → 读回超时 → `readback_mismatch_target` → 回滚。

> 这条实测同时约束：**任何「写入成功」的判据都不得只看 `set_pose.success`**，必须以真值读回为准——这正是
> (a-3) 读回校验的直接依据。

---

## Nav2 吸收点

| 吸收点 | 来源 | 落点 |
|---|---|---|
| **复位用显式状态窗口**（进入 RESETTING 前撤销许可），非「服务可达即写入」 | `对齐 Nav2 lifecycle`（`nav2_lifecycle_manager` 显式迁移 + `get_state`） | (a-1) 门控窗口；(a-2) 撤销许可 + 静止证据 |
| **资源可重建**（cleanup 后可重复 configure；异常清理） | `对齐 Nav2 lifecycle cleanup/configure` 语义 | (a-3) 原子写入 / 读回 / 回滚；(a-4) epoch 重开回合 |
| **重试**（状态迁移带超时重试） | `对齐 nav2_util::startup/reset_lifecycle_nodes`（`retries`） | (a-3) 读回有界等待；R-014 候选重采样 |
| 单点权威（不并行命令者） | `对齐 ROS 2 唯一权威控制`（契约 [#execution-port](../../docs/ROS2迁移交接文档.md#execution-port)） | (a-2) **不新增**第二命令者、不发布 cmd_vel |
| 执行器 / 回调组隔离（阻塞回调不冻结本地缓存） | `对齐 rclcpp 多线程执行器` | (a-6) `MultiThreadedExecutor` + 事务回调组 |
| `set_pose` 写入作用判定 | `全新重写`（GZ 语义无对应 lifecycle 语义） | (c-2) 真值读回为准 |

**未采纳**：不引入 `bt_navigator` / 行为树；**不新增**第二 ROS 命令者；**不改**冻结 9 消息 / 服务 ABI
（沿用 `std_srvs/Trigger`）。

## 参数（`src/agc_task/config/w6_base.yaml` 单一注入，节点不得覆写）

`car_entity_name` / `target_entity_name` / `car_gt_topic` / `target_gt_topic` / `reset_epoch_topic`
/ `readback_tolerance_m: 0.25` / `readback_timeout_s: 1.0` / `readback_max_age_s: 0.5` /
`stop_exec_max_s: 0.5` / `quiescence_eps: 0.05`（`ground_odom_topic` 亦显式化）。

## Considered Options

- **保持两次非原子 `set_pose`（目标失败即留中间态）** —— 被否：即 R-015 所指缺口。
- **复位前由 `scenario_reset` 下「零速」cmd_vel 作为「停止执行」** —— 被否：违反契约
  [#execution-port](../../docs/ROS2迁移交接文档.md#execution-port)（唯一例外仅「位姿写入」），会新增第二命令者。
  改以门控 + 静止证据（(a-2)）。
- **只用 `set_pose.success` 判写入成功** —— 被否：(c-2) 实测证明其对不存在实体仍为 true；改以真值读回。
- **非 active（IDLE+终态）+ 受控事务** —— **采纳**。

## Consequences

- `scenario_reset` 服务应答：成功 `reset ok epoch=N`；失败 `reset rejected: <reason>`（reason 见 (a-3)）。
- 启动初始复位不再 bypass 门控；早启动若任务 active 则被拒。
- 新增 `/agc/scenario/reset_epoch`（`Int32`，transient_local）供多回合 / 起始新鲜门关联；`car_start`
  仅在完整成功后发布（配套 [ADR-0028](./0028-task-start-transaction-and-freshness-gate.md) 的新鲜门）。
- 门控窗口裁定以本 ADR 为准，**契约 [#task-gating](../../docs/ROS2迁移交接文档.md#task-gating) 文字未改**，
  供后续单独裁定回填。
- 不新增 / 不修改冻结接口 ABI；不新增命令者。
