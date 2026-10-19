# 目标失效策略与位移阈值化重规划（R-041）

## Status

accepted

## Date

2026-10-18

## 背景

评审风险 **R-041（目标失效与重规划抖动）**（[10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-041)）指出两处病灶：

- [`task_manager.cpp`](../../src/agc_task/src/task_manager.cpp) 的 `publishGoal()`：`goal_follow_target` 为真且
  `targetFresh()` 为假时，**仍发送新时间戳**的固定 `goal_x_/goal_y_`——**不是**最后有效目标、**也不**表达失效；
  新 `stamp = now()` **掩盖失联**（消费者按 `Header` 判年龄会以为本时刻仍有有效目标）。
- [`ground_baseline.cpp`](../../src/agc_ground/src/ground_baseline.cpp) 的 goal 回调：对 **`>1e-6` 任意浮点变化**
  即 `force_replan_ = true`，**先于** `needReplan` 的位移 / 地图 / 周期判断——融合小噪声可**每条 goal 消息**触发
  重规划，`replan_period_s` 不再代表所述作用，路径噪声 / 规划 CPU 抖动影响 RTF 与控制。

契约真源：[#namespace](../ROS2迁移交接文档.md#namespace)（`world` / `car/base_link`）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（地面唯一命令通道）、
[#params](../ROS2迁移交接文档.md#params)（launch 单一设置）、
[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C10 目标新鲜与冻结）、
[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)、[#messages](../ROS2迁移交接文档.md#messages)（冻结 9 消息）。

分类：**设计风险（目标语义 / 重规划触发口径）**。本 ADR 决定**目标失效策略 =「保留有期限的最后有效目标」、
目标身份 / 有效性 / 时刻经伴随诊断传递、重规划位移阈值化**；**不**改冻结 9 消息 / 服务 ABI，**不**变唯一执行口
语义，**不**新增第二 ROS 命令者，**不**假定地面失效动作与 C10 空中 `freeze` 相同。

## 取证（先学后做）

按 [AGENTS.md](../../AGENTS.md) 铁律「先学后做」，**先**对齐**同版本**（ROS 2 Jazzy，本机 `/opt/ros/jazzy`）
Nav2 的**目标变化触发重规划**与**位姿 / 位移阈值容差**成功用法，**再**改自身实现。

**同版本 Nav2 取证（本机 `/opt/ros/jazzy/include/...` 实读的源文件与符号）：**

- **目标变化触发重规划 = 离散「goal updated」事件，而非逐消息浮点变化**：
  `nav2_behavior_tree::GoalUpdatedController`
  （`include/nav2_behavior_tree/plugins/decorator/goal_updated_controller.hpp` :34-83）为装饰器，
  **仅当 goal 被更新时才 tick 其子节点**（`goal_was_updated_` 成员 :80）；`GoalUpdatedCondition`
  （`include/nav2_behavior_tree/plugins/condition/goal_updated_condition.hpp` :30-80）**在 goal 于黑板被更新时返回
  SUCCESS**。二者均以**离散的目标更新信号**（换目标 / 重发布目标）为判据，**不**以「连续 goal 流里任一浮点
  变化」触发——**逐消息 `>1e-6` 即重规划**不符合 Nav2 惯例。
- **位姿 / 位移阈值（L2 距离 + 容差），而非任意浮点相等**：
  `nav2_controller::SimpleGoalChecker`
  （`include/nav2_controller/plugins/simple_goal_checker.hpp` :50-90）「**only checks the position difference**」，
  维护 `xy_goal_tolerance_` 与其**缓存平方** `xy_goal_tolerance_sq_`（:75-79）；`PositionGoalChecker`
  （`include/nav2_controller/plugins/position_goal_checker.hpp` :29-62）同以 `xy_goal_tolerance_` /
  `xy_goal_tolerance_sq_` 判 XY；`StoppedGoalChecker`
  （`include/nav2_controller/plugins/stopped_goal_checker.hpp` :49-71）在位置上再加**速度容差**
  （`rot_stopped_velocity_` / `trans_stopped_velocity_`）。位移度量统一为 `nav2_util::geometry_utils::euclidean_distance`
  （`include/nav2_util/nav2_util/geometry_utils.hpp` :52-104，`std::hypot(dx, dy)`）——**阈值化位移**是本版本惯例。
- **路径时效 / 有效性另行命中重规划**：`PathExpiringTimerCondition`
  （`include/nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.hpp` :41-80，端口 `seconds`）与
  `IsPathValidCondition`（`include/nav2_behavior_tree/plugins/condition/is_path_valid_condition.hpp`）表达按
  **时限 / 有效性**触发重规划；`nav2_controller::ControllerServer` 另有 `updateGlobalPath()` / `isGoalReached()`
  （`include/nav2_controller/controller_server.hpp` :166-183）承接新路径。
- **结论（本版本惯例）**：① 目标变化以**离散事件**触发重规划；② 位移比较用**阈值化 L2 距离容差**，**不**用
  任意浮点相等；③ 时限 / 有效性**另行**参与重规划判据。

**本项目现状差距（读取时工作区）：** `publishGoal` 目标不新鲜仍发新时间戳固定点（掩盖失联、无身份 / 有效性）；
`ground_baseline` goal 回调 `>1e-6` 即 `force_replan_ = true`（先于 `needReplan`，逐消息抖动重规划）。

**本批取证实测（本机真实二进制 / 实跑）：** 见
`experiments/w6-task-eval/20261018-w6-r041-0/r041/bench.json` —— 修复后：新鲜期 goal 位置=目标、`stamp`≈目标
**源时刻**（≠ `now`）；停喂后窗口内 `HOLD_LAST_VALID` 保持最后有效位置 + 原源时刻、超窗 `valid=0 reason=EXPIRED`
且**无**新时间戳固定点 goal、**从不**反射固定点；微噪声（抖幅 0.3 < 1.0）窗内**无**「目标身份变」触发、重发次数
≈ 周期（0.6 Hz vs 期望 ≤4 次）；真实位移 2.0 m 触发重规划（延迟 ~0.02 s）且新路径端点 ≈ 新目标；`goal_info valid=0`
→ `cmd_vel_raw≈0`。

## 决策

### (a) 目标失效策略裁定：「保留有期限的最后有效目标」（bounded hold-last-valid）

**三选一（安全停车 / 保留有期限的最后有效目标 / 受限搜索）** 中选**保留有期限的最后有效目标**：

- **候选一「立即安全停车」否决**：`targetFresh` 依赖 `target_max_age_s=0.5`，融合 / 通道**瞬时丢帧**即会触发；
  在目标跟踪的常态抖动下过早停车，任务在「目标仍在、仅本帧未达」时被误判失联，与实际不符。
- **候选三「受限搜索」否决**：本项目**无搜索策略**（无 `bt_navigator` / 恢复搜索行为，不引入），且搜索范围受
  作业区与安全约束限制；以「搜索」为名会产生无依据的额外运动，超出本批范围。
- **候选二「保留有期限的最后有效目标」采纳**：它**如实区分**「短暂未达」与「实质失联」——窗口内保持**最后一次
  有效目标的原位置与原源时刻**（`age` 自然增长、**暴露**陈旧而非掩盖），**不**反射固定点、**不**刷新 `stamp`；
  超窗后**显式失效**并触发地面**安全停车**。窗口长度由 `goal_hold_valid_s` 标定（default 1.0 s）。

### (b) 目标身份 / 有效性 / 时刻的传递（带外伴随诊断）

- `goal_follow_target=false`：固定任务目标本身恒有效 → 位置 `goal_x_/goal_y_`、`stamp=now`、
  `valid=true reason=FIXED_GOAL`。
- `goal_follow_target=true` 且 `targetFresh()`：位置=目标 XY、`stamp=`**目标源时刻**（**非** `now`）、关联
  `target_id`、`valid=true reason=TARGET_FRESH`，并刷新「最后有效目标」缓存（位置 / 源时刻 / 身份 / 到达时刻）。
- 否则若在 `goal_hold_valid_s` 窗口内：位置=最后有效位置、`stamp=`**最后源时刻**（`age` 自然增长）、`valid=true
  reason=HOLD_LAST_VALID`。
- 否则**显式失效**：`valid=false reason=EXPIRED`（有最后有效）或 `TARGET_LOST`（从未有效）；**不发布**带新时间戳
  的 goal 位姿。
- 有效性 / 身份 / 时刻经**伴随诊断话题** `/agc/ground/goal_info`（`std_msgs/String`，**transient_local**，非命令者）
  传递：`valid=<0|1>;reason=<...>;target_id=<id>;src_stamp=<double 秒>;age=<double 秒>`。**不改** `/agc/ground/goal`
  的 `geometry_msgs/PoseStamped` 类型（冻结接口 / 类型不变）。新参数 `goal_hold_valid_s`、`goal_info_topic` 由
  launch 单一注入（[#params](../ROS2迁移交接文档.md#params)）。`/agc/task/start` 新 episode **清空**「最后有效目标」
  缓存（与既有 episode 隔离一致）。

### (c) 重规划改为位移阈值化（移除任意浮点强制重规划）

- **移除** `ground_baseline` goal 回调里的 `hypot(...) > 1e-6 → force_replan_ = true`。改为：**始终**更新
  `goal_x_/goal_y_`；**仅首次**获得 goal（`!have_goal_`，合法实质绑定）时置 `force_replan_ = true`（首次立即规划）。
- 后续重规划**完全**交给 `needReplan` 的既有判据：**位移阈值** `replan_displacement_m`（既有，1.0 m）→ **地图变化**
  （costmap 硬障碍签名，R-012）→ **时限** `max_path_age_s` → **周期** `replan_period_s`，并保持 `kReplanGuardS=0.2 s`
  限流（对齐 Nav2 GoalUpdated 离散事件 + 阈值 / 时限 / 周期惯例）。
- 目标身份关联沿用「规划时刻 goal」`last_plan_goal_`（R-012 路径绑定）；本批**不**改 R-012 的 `last_plan_goal_` /
  `last_plan_map_sig_` / `path_valid_` 与 fail-stop 语义。

### (d) 地面失效动作与 C10 空中 `freeze` 的分野（分别定义）

- **地面失效动作 = 安全停车（fail-safe stop）**：`ground_baseline` 消费 `/agc/ground/goal_info` 的 `valid=`；
  `valid=false` → `tick()` 顶部（`!active_` 检查后）`publish(0,0); return;`（先于路径跟随 / 重规划）。默认
  `goal_valid_=true`（无 `/agc/ground/goal_info` 的旧运行保持既有行为）。
- **空中 C10 `freeze`**（[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) 第 10 条）：无人机对目标的
  standoff 跟踪在目标失效（`ref_max_age_s=0.5` / 低 `confidence`）时**冻结位姿 / 悬停**（保持参考、不追逐旧目标）。
- **二者分别定义**：地面 = 「有期限保持最后有效目标」后**安全停车**；空中 = **冻结位姿 / 悬停**。**不**假定地面
  失效动作与空中 `freeze` 相同（评审 R-041 明确要求分别定义）。

### (e) 验收

- `scripts/run_w6_r041.sh` + `scripts/w6_r041_bench.py`（真实 `task_manager` / `ground_baseline` 二进制 + 合成
  target / odom / map / goal 流；配 `static_transform_publisher`（`world→car/base_link`）+ 台架合成 `/agc/map`
  （latched）自持拉起，对齐 R-012 基线台架做法）+ `scripts/w6_r041_check.py`（自包含核查 (a)…(h)，输出
  `overall_pass`）。

### (f) 边界（不改契约）

**不**改冻结 9 消息 / 服务 ABI（`git status src/agc_msgs` 空），**不**变唯一执行口语义，**不**新增第二 ROS 命令者；
新增均为**诊断 / run config**（`/agc/ground/goal_info` 为 `std_msgs/String`；`goal_hold_valid_s` / `goal_info_topic`
为 run config）；`/agc/ground/goal` 仍为 `geometry_msgs/PoseStamped`，语义（`frame_id=world`、`orientation.w=1.0`）
不变；`/agc/ground/cmd_vel_raw` 仍为基线输出（下游经安全层到唯一执行口），本批**未**改命令链。

## 备选

1. **目标失效即「立即安全停车」**——**否决**：`target_max_age_s=0.5` 使瞬时丢帧即停，过早停车、误判常态抖动。
2. **「受限搜索」**——**否决**：本项目无搜索策略（不引入 `bt_navigator` / 恢复搜索行为），且搜索范围受作业区 /
   安全约束限制，会引入无依据运动。
3. **保留 `>1e-6` 强制重规划、仅加注释**——**否决**：正是 R-041 要消除的「逐消息抖动重规划」；改为位移阈值化
   （对齐 Nav2 `xy_goal_tolerance` / `euclidean_distance` 阈值惯例）。
4. **改 `/agc/ground/goal` 消息类型以携带身份 / 有效性**——**否决**：会动冻结接口语义；改用**带外伴随诊断**
   （`/agc/ground/goal_info`，`std_msgs/String`）。

## 后果

- **正面**：目标失联**不再被新时间戳固定点掩盖**——窗口内保持最后有效目标（原位置 + 原源时刻，`age` 暴露陈旧），
  超窗**显式失效** → 地面**安全停车**；`goal` 关联目标**身份与时刻**，下游可判「此 goal 是否仍绑定有效目标」；
  微噪声 goal **不**触发重规划（重规划由位移阈值 / 地图 / 时限 / 周期触发，频率可复核）；地面失效动作与 C10 空中
  `freeze` **分别定义**。
- **代价 / 限制**：`goal_hold_valid_s` 为**可标定量**（default 1.0 s），窗口内地面仍朝最后有效目标前进（**故意**，
  以「不丢目标」为先；超窗即停）；`/agc/ground/goal_info` 与 `/agc/ground/goal` 需**同源**（由 `task_manager` 单一
  发布）方可保证有效性一致。
- **未决**：`goal_hold_valid_s` 的**最终标定值**待与目标运动 / 通道特性联合标定（本批用 default 1.0 s 及验收台架
  0.5 s / 2.0 s 窗口）；`/agc/ground/goal_info` 的**消费方**（RL 主控 `agc_rl`）失效停车是否同样接线属 W5 / W7 范围。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 证据

- `experiments/w6-task-eval/20261018-w6-r041-0/r041/bench.json`（真实二进制节点行为台架，`overall_pass=true`）：
  `r041.json` / `r041/check.json`（核查 (a)…(h) 全 PASS）。

## Nav2 吸收点

| 吸收点 | 来源 | 落点 |
|---|---|---|
| 目标变化以**离散事件**触发重规划（非逐消息浮点变化） | `对齐 Nav2`（`GoalUpdatedController` / `GoalUpdatedCondition`） | 移除 `>1e-6` 强制重规划，改位移阈值化 |
| 位移 / 容差用**阈值化 L2 距离**（`xy_goal_tolerance` / `euclidean_distance`） | `对齐 Nav2`（`SimpleGoalChecker` / `PositionGoalChecker` / `nav2_util::geometry_utils`） | `needReplan` 沿用 `replan_displacement_m` |
| 路径**时限 / 有效性**另行参与重规划 | `对齐 Nav2`（`PathExpiringTimerCondition` / `IsPathValidCondition`） | `max_path_age_s` / `replan_period_s` / 地图签名（承接 R-012） |
| 目标失效**显式失效 + 位置保持 / 停车**（非静默回退固定点） | `全新重写` | bounded hold-last-valid + 地面安全停车 + `/agc/ground/goal_info` |
| 伴随诊断以 `std_msgs`（带外）承载身份 / 有效性 / 时刻 | `全新重写` | `/agc/ground/goal_info`（不改冻结 `PoseStamped`） |

**未采纳**：不引入 `bt_navigator` / 行为树；**不新增第二 ROS 命令者**（契约 [#execution-port](../ROS2迁移交接文档.md#execution-port)）；
不改 Nav2 planner 插件本体。

## 参数（config 单一注入）

- `src/agc_task/config/w6_base.yaml`（`task_manager` 段）：新增 `goal_hold_valid_s`（default 1.0）、
  `goal_info_topic`（default `/agc/ground/goal_info`）。
- `src/agc_ground/config/baseline.yaml`（`ground_baseline` 段）：新增 `goal_info_topic`（default
  `/agc/ground/goal_info`）；既有 `replan_displacement_m` / `max_path_age_s` / `replan_period_s` / `stop_on_invalid`
  语义保持。

## 参考

- 评审原文：[10-风险#r-041](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-041)。
- 契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)、[#execution-port](../ROS2迁移交接文档.md#execution-port)、
  [#params](../ROS2迁移交接文档.md#params)、[#messages](../ROS2迁移交接文档.md#messages)。
- 前序：[ADR-0027](./0027-baseline-path-binding-and-replan-failstop.md)（路径绑定与 fail-stop）、
  本批 [ADR-0054](./0054-ground-odom-source-time-and-reset-epoch-velocity.md) /
  [ADR-0055](./0055-map-content-identity-and-monotonic-update-sequence.md)。
