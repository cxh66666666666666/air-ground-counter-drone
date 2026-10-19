# 连续跟踪 dwell 逐拍判据中断（R-023：空数组不清 dwell / 换目标清零 / 车 odom 寿命门 / 分段有效时长）

## Status

accepted

## 背景

评审风险 **R-023（连续跟踪判据）**（[10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-023)）指出
`task_manager`（[源码](../../src/agc_task/src/task_manager.cpp)）的 dwell 判据**不是逐拍连续的**：

1. **空目标数组不清 dwell**：`target_sub_` 收到空数组仅置 `have_target_=false`；`tickFsm` 的
   `GROUND_APPROACH` dwell 分支被 `if (have_target_ && have_car_)` 整体**跳过**——旧 `dwell_start_`
   **保留**，重获同目标后 `(now − dwell_start_)` 已跨过失联段，被计入「持续」。
2. **换 `target_id` 不清 dwell**：身份变化无判据，不同目标的近距观测可拼成一段 dwell。
3. **车 odom 无寿命**：仅看 `have_car_` 布尔，`ground_odom_sub_` 停发后旧车位置无限期有效，可用旧位置
   与当前目标累计。
4. **无分段有效时长记录**：中断归零即丢，无法区分「连续 t_dwell」与「累计接近」。

后果：C8 的「抵近并挡续跟踪」成功可能把**失联段**或**不同目标**拼起来，使 SUCCESS 语义失真。契约
[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C8** 要求终态为「抵近并持续跟踪」，
[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics) 的 `TRACKING_SUCCESS` 由
`reason_code=tracking_milestone` 触发。

**先学后做（取证依据）**：对齐 R-013 已引入的**到达时刻**基座（`last_car_odom_rx_` /
`last_odom_valid_rx_`，[ADR-0028](./0028-task-start-transaction-and-freshness-gate.md)）；契约
[#params](../ROS2迁移交接文档.md#params)（参数单一注入 / 节点不得覆写）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一执行口语义不变）、
[#decisions](../ROS2迁移交接文档.md#decisions)（`task_manager` = 自研确定性 C++ 状态机，**不**引入
`bt_navigator`）；已冻结 9 消息（[#messages](../ROS2迁移交接文档.md#messages)）字段/顺序**不改**。

分类：**事实错误与设计风险**。保持成功语义，**不**降级成累计接近时长。

## 决策

### (a) dwell 提升为逐拍显式判据
`GROUND_APPROACH` 分支内（capture / jam 结算之后）改为：
- 本拍合法性：`car_age`（由 `last_car_odom_rx_` 到达时刻算）→ `car_ok`（`have_car_ && 0 ≤ age ≤
  car_odom_max_age_s_`）；`range_ok = have_target_ && car_ok && 水平距 ≤ d_track_`。
- **已有活跃 dwell** 逐拍连续性检查：`epoch` 变化 / `have_target_==false`（**含空数组**）/ 目标失效
  （`TARGET_STALE` 或 `TARGET_LOW_CONF`）/ 车 odom 失效超龄（`CAR_ODOM_STALE` / `CAR_ODOM_MISSING`）/
  换 `target_id`（`TARGET_ID_CHANGED`）/ 包络外（`RANGE_EXCEEDED`）——**任一不满足即中断清零**。
- **无活跃 dwell 且本拍合法** → 起新 dwell（换 id / epoch 变化后**从 0 重算**，杜绝跨段累计）。

### (b) 换目标 / 车 odom 失效 / 冻结与 epoch / 任务暂停分别定义
| 触发 | 中断原因 | 定义 |
|---|---|---|
| 换 `target_id` | `TARGET_ID_CHANGED` | 身份变化即清零（不作跨目标拼接） |
| 车 odom 停发 / 超龄 | `CAR_ODOM_MISSING` / `CAR_ODOM_STALE` | 不用旧车位置累计 |
| 目标冻结 / 失效 / 低 conf | `TARGET_STALE` / `TARGET_LOW_CONF` / `TARGET_LOST` | 目标失新鲜或低于 `confidence_min` 即断 |
| clock epoch 变化 | `EPOCH_CHANGED` | 本 episode 身份改变即断（不复用跨 episode 计时） |
| 任务暂停（abort / 进终态） | `TASK_PAUSED` | 离开 active 即视为暂停；`onAbort` 内先中断活跃 dwell；其余终态路径由新 episode 清零 |

### (c) 分段有效时长记录
新增 `dwell_segment_start_`（当前有效段起点，与 `dwell_start_` 同步）、`last_dwell_segment_s_`（刚被中断的
那段时长）、`dwell_valid_total_s_`（本 episode 累计有效时长）。`interruptDwell(t, reason)` 结算
`seg = (t − dwell_segment_start_).seconds()`，并入累计后发布中断事件再清零。

### (d) 成功语义不降级
成功仍为「**连续** `t_dwell` 有效」（`dwell_start_` 未被中断即 `(t − dwell_start_) ≥ t_dwell_s`）→
`tracking_milestone_met_=true` + `enterTerminal(TRACKING_SUCCESS, "tracking_milestone")`。**不**改为累计
接近时长，**不**降低阈值。

### (e) 参数单一注入
新增 `car_odom_max_age_s`（默认 `1.0`）——车地面 odom 寿命门；`dwell_event_topic`（默认
`/agc/task/dwell_event`）——配套事件话题。均经 `config/w6_base.yaml` 单一注入，节点不得覆写。

### (f) 事件协议 / ABI 边界
新增话题 `/agc/task/dwell_event`（`std_msgs/String`，depth 10，**非命令者**），事件串：
- 起始：`DWELL_START ep=<int> target=<id>`
- 中断：`DWELL_INTERRUPT ep=<int> reason=<REASON> seg=<%.3f> total=<%.3f>`

`ep`=`episode_epoch_`，`seg`=刚被中断的那段有效时长（s），`total`=本 episode 累计有效时长（s）。
**未改**契约冻结 9 消息 / 服务字段 / 顺序；`capture_hit` 仍恰为 `CAPTURE_NET`；**未新增**第二 ROS 命令者；
**未改** [#execution-port](../ROS2迁移交接文档.md#execution-port) 唯一执行口语义；**不引入** `bt_navigator`。

## Considered Options

- **维持旧式（只在 `have_target_&&have_car_` 内清零 / 空数组跳过 / 车 odom 无寿命）** —— 被否：即 R-023 缺口，
  失联段与不同目标可拼成「持续」。
- **把 dwell 改成累计接近时长（允许中断后累加）** —— 被否：违反 C8 成功语义，降低判据。
- **新增带 dwell 字段的冻结 9 消息 / 新命令者** —— 被否：违反 [#messages](../ROS2迁移交接文档.md#messages)、
  [#execution-port](../ROS2迁移交接文档.md#execution-port)；分段时长以新增 `std_msgs` 状态事件表达。
- **引入 `bt_navigator` 承载连续判据** —— 被否：违反 [#decisions](../ROS2迁移交接文档.md#decisions)；
  缺口可在既有确定性 FSM 内闭合。
- **(a)–(f) 组合（逐拍判据 + 分类中断 + 分段时长记录 + 成功不降级 + 单一注入 + ABI 边界）** —— **采纳**。

## Consequences

- 空目标数组 / 换 `target_id` / 车 odom 停发超龄 / epoch 变化 / 任务暂停，均**中断清零**；dwell **不**跨段、
  **不**跨目标、**不**跨 episode 累计。
- 成功仍为「连续 `t_dwell` 有效」（C8 不降级为累计接近时长）。
- `last_dwell_segment_s_` / `dwell_valid_total_s_` 记录分段有效时长，供 harness 与 run 证据关联。
- **ABI 不变**：新增话题均 `std_msgs` 状态；唯一执行口未改；未新增命令者；`capture_hit` 仍恰为
  `CAPTURE_NET`；契约文档未改。
- **风险标注 / 置信度**：以自包含 harness（假源 + 假 reset 服务，含**空数组**与**停发 odom** 桩）
  验证，见 [Evidence](#evidence)；判据分支的确定性由契约证据保证
  （[R-023](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-023) 置信度：高）。
- 只读旧树边界：本次未读 `reference/`；未新增契约 / ABI 变更。

## Evidence

命令化验收 `bash scripts/run_w6_r023.sh headless 20261016-w6-r023-0`（自包含 harness，无 gz / launch），
结果 `experiments/w6-task-eval/20261016-w6-r023-0/metrics/r023.json`（`overall_pass: true`，6/6）：

| 用例 | 注入 | 观测（关键判据） |
|---|---|---|
| a 空数组跨段 | 抵近→空数组 `run(2.5)`（> `t_dwell_s=2.0`）→重获 | 中断 `TARGET_LOST seg=1.000 total=1.000`；重获后 1.0s **仍非** `TRACKING_SUCCESS`（`no_early_success=True`），再续 1.5s → `TRACKING_SUCCESS`（旧逻辑会跨段直接成功） |
| b 换 target_id | `tgt_id` `intruder_uav`→`other_uav` | 中断 `TARGET_ID_CHANGED seg=1.000`；不早成功，续时后 `TRACKING_SUCCESS` |
| c 车 odom 失效 | 停发 `/agc/ground/odom`（`car_odom_max_age_s=0.5`） | 中断 `CAR_ODOM_STALE seg=1.450`；不早成功，续时后 `TRACKING_SUCCESS` |
| d epoch 变化 | `epoch += 7`（发布 `/agc/scenario/reset_epoch`） | 中断 `EPOCH_CHANGED seg=1.000`；不早成功，续时后 `TRACKING_SUCCESS` |
| e 分段有效时长 | 有效段 `run(1.2)` 后置空数组 | `DWELL_INTERRUPT seg=1.200 total=1.200`（`seg∈(0.6,1.5)`、`total>0`） |
| f 冻结值 / ABI | `grep` 源码 + `git` 工作树 | `hit.data = "CAPTURE_NET";` 存在；`src/agc_msgs` 未改 |

回归：`bash scripts/run_w6_r022.sh headless 20261016-w6-r022-regress` **PASS**
（`experiments/w6-task-eval/20261016-w6-r022-regress/metrics/r022.json` `overall_pass: true`）——
命令幂等 / 终态锁存 / episode 身份语义未回退。
