# 任务起始异步事务与新鲜门禁（R-013）

## Status
accepted

## 背景与决策

评审风险 **R-013（起始门禁竞态）**：`/agc/task/start` 在**主单线程回调内阻塞**（`spin_until_future_complete`
于另建客户端节点），`scenario/reset` + `air/reset_phase` 等待期间任务自身的 `odom_valid` / 地面 odom /
`cmd_vel` 订阅**停止更新**；故「reset 后再校验」实际仍读**阻塞前缓存**。且 `odom_valid` **无年龄**、
`world` 仅比较**参数字符串**、时钟仅 `now>0`。[契约 #lessons](../../docs/ROS2迁移交接文档.md#lessons)
要求「先 reset 后校验」，本次将其落为**可判定**语义。

本 ADR 记录三项决策：**(a) 起始事务不阻塞节点（多线程执行器 + 服务独立回调组）**、
**(b) 门禁绑定"复位后新鲜数据"（epoch + 年龄）**、**(c) 时钟门改为进展校验、世界名保留为配置校验**。

### (a) 起始事务不阻塞节点：多线程执行器 + 服务独立回调组

**先学后做（取证）**：本机 Jazzy 头文件已核验 `rclcpp::executors::MultiThreadedExecutor`
（`multi_threaded_executor.hpp`；`executor.hpp` `add_node`）、`rclcpp::CallbackGroupType::{MutuallyExclusive,Reentrant}`
（`callback_group.hpp`）、`create_service(name, cb, const rmw_qos_profile_t&, CallbackGroup::SharedPtr)`
4 参重载（`node.hpp`）、`rmw_qos_profile_services_default`（`rmw/qos_profiles.h`）。

**决策**：`main()` 由 `rclcpp::spin` 改为 `MultiThreadedExecutor`；`/agc/task/start`、`/agc/task/abort`
放入**专用互斥回调组**（`srv_cb_group_`）。服务回调可在**本线程**阻塞等待 `reset` / `reset_phase`
（`callTrigger` 保留 `spin_until_future_complete(reset_client_node_, ...)`），而订阅留在**默认组**、
由执行器**其它线程**持续处理——事务期间 `odom_valid` / 地面 odom / `cmd_vel` **不再冻结**。start 与 abort
同组互斥，避免并发重入。**未**引入第二 ROS 命令者，**未**改冻结 9 消息 ABI。

### (b) 门禁绑定"复位后新鲜数据"（epoch + 年龄）

**决策**：`scenario/reset` 成功后记 `reset_done_time_`（复位完成 epoch）。门禁数据须同时满足
**到达时刻 ≥ `reset_done_time_`**（复位后收到）**且 年龄 ≤ 上限**：

- `odom_valid` / 地面 odom：年龄 ≤ `odom_valid_max_age_s`（config，默认 1.0 s）。
- `cmd_vel` 回环：年龄 ≤ 0.5 s（沿用既有判据）。

复位完成瞬间 `ground_odom_adapter` 尚未发布下一帧，若**不等待**会把正常启动误判为 stale；故先做
**有界等待**（上限 = `odom_valid_max_age_s`）待新样本到达，超时仍无 → 判 stale（理由码
`odom_valid_stale` / `ground_odom_stale`）。`odom_valid` 由 `ground_odom_adapter` **周期发布**（已核验），
复位后 ~1 周期内必到。

### (c) 时钟门 = 进展校验；世界名保留为配置校验

**决策**：1 Hz 状态计时器（默认回调组，服务阻塞期间**持续采样**）维护 `prev_clock_`。时钟门通过条件为
`now().seconds() > 0` **且** `now() > prev_clock_`（**严格推进**）——仿真时钟冻结（`/clock` 停发）时
三时刻相等 → 不过，理由码 `clock_not_progressing`。`world_name_` 比较**保留**为**配置校验**
（`single_world_mismatch`）。各门判定输出 `evidenceSummary()`（epoch / 各门到达时刻 / 时钟采样 / 世界名），
随 Trigger 应答回传并记入日志，供 run 记录关联。

## 证据（实跑，轻量 W6 栈）

`scripts/run_w6_r013.sh headless 20261016-w6-r013-0` → attempt 1 PASS，
`experiments/w6-task-eval/20261016-w6-r013-0/metrics/r013.json`（`overall_pass: true`）：

| 子测 | 判据 | 结果 |
|---|---|---|
| a 正常通过 | 复位后有界等待 → 新数据门 + 时钟进展通过，携带 `[evid]` | PASS（`odom_valid_rx=16.532(age=0.012)`） |
| b 复位后无新数据 | SIGSTOP `ground_odom_adapter` → `odom_valid_stale` | PASS（`age=1.512` → 拒） |
| d 时钟冻结 | 停 `/clock` 桥接（`parameter_bridge --ros-args`）→ `clock_not_progressing` | PASS（`now=prev_clock` → 拒） |
| c 错误世界名 | `world_name=wrong_world` → `single_world_mismatch` | PASS（拒） |

## Nav2 吸收点

| 吸收点 | 来源 | 落点 |
|---|---|---|
| 起始用**显式就绪**（ready 前不执行），非「服务可达即成功」 | `对齐 Nav2 lifecycle`（`nav2_lifecycle_manager` bond + 状态查询语义） | 起始事务 + 新鲜门 |
| 阻塞回调不冻结本地缓存（执行器/回调组隔离） | `对齐 rclcpp 多线程执行器`（本机头文件已核验） | `MultiThreadedExecutor` + `srv_cb_group_` |

**未采纳**：不引入 `bt_navigator` / 行为树；**不新增第二 ROS 命令者**。

## 参数（`src/agc_task/config/w6_base.yaml` 单一注入）

- 新增 `odom_valid_max_age_s`（复位后 `odom_valid` / 地面 odom 新鲜度上限；`cmd_vel` 沿用 0.5 s 常量）。
