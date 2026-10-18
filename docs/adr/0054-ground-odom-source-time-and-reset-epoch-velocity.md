# 地面 odom 源时刻分离与复位 epoch 速度历史（R-039）

## Status

accepted

## Date

2026-10-18

## 背景

评审风险 **R-039（地面里程计有效性）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-039)）指出，
[`ground_odom_adapter.cpp`](../../src/agc_perception/src/ground_odom_adapter.cpp) 把**发布时刻**当成**测量时刻**：

- `onGt()` 把**源样本时刻** `t = rclcpp::Time(msg.header.stamp).seconds()` 存入 `history_` 并设 `last_rx_sim_ = t`；
- `age()` `return std::max(0.0, now_s - last_rx_sim_)` —— **负年龄被钳为 0**（future stamp / 时钟回拨仍算新鲜）；
- `tick()` 用 `create_wall_timer`（30 Hz）以 `now()` 作为 odom / TF 的 `header.stamp`，而位姿取**最近一次 GT 样本**的
  `cur_pos_/cur_quat_` → **发布时刻冒充测量时刻**，消费者按 `Header` 判年龄会以为**缓存位置是本时刻测量**；
- `velocity()` 用 `history_` 短窗差分求 world 速度，但 `history_` **不与 `/agc/scenario/reset` 的 teleport 关联**
  → 复位跳变位移被差分成**非物理速度尖峰**，进入基线控制、门禁与 TTC。

契约真源：[#namespace](../ROS2迁移交接文档.md#namespace)（`world` / `car/base_link`、速度帧语义）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（地面唯一命令通道，本项只改感知端 odom 发布与诊断）、
[#task-gating](../ROS2迁移交接文档.md#task-gating)（`/agc/scenario/reset` 受运行期门控）、
[#params](../ROS2迁移交接文档.md#params)（launch 单一设置）、
[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C10 目标新鲜与冻结 / C9 指标口径）、
[#lessons](../ROS2迁移交接文档.md#lessons)（reset 后须校验新数据 / 不得冻结本地缓存）。

分类：**设计风险（时间口径）**。本 ADR 决定 **源采样时刻与发布时刻显式分离、不静默外推、负年龄 / 时钟回拨显式无效、
复位 epoch 清空速度历史**；**不**改冻结 9 消息 / 服务 ABI，**不**变唯一执行口语义，**不**新增第二 ROS 命令者。

## 取证（先学后做）

按 [AGENTS.md](../../AGENTS.md) 铁律「先学后做」，**先**对齐**同版本**（ROS 2 Jazzy）Nav2 / tf2 / robot_localization 的
**采样时刻 vs 接收时刻 / 外推惯例 / 负年龄处置**成功用法，再改自身实现。

**同版本 Nav2 / tf2 / robot_localization 取证（本机 `/opt/ros/jazzy/include/...`）：**

- **tf2 以消息 `stamp` 查变换，而非接收时刻**：`tf2_ros::MessageFilter`
  （`include/tf2_ros/tf2_ros/message_filter.hpp:370` `rclcpp::Time stamp = mt::TimeStamp<M>::value(*message);`
  随后用 `stamp` 调 `buffer_.canTransform(...)`）——**测量时刻即消息 `Header.stamp`**，`nav2_amcl`
  （`include/nav2_amcl/amcl_node.hpp:178` `tf2_ros::MessageFilter<LaserScan>`）据此把激光按**其自身 stamp** 对齐。
- **无静默外推，外推越界即显式异常**：`tf2` 定义 `ExtrapolationException` /
  `ForwardExtrapolationException` / `NoDataForExtrapolationException`
  （`include/tf2/tf2/exceptions.hpp:145,171,184`）；`tf2::TimePointZero`
  （`include/tf2/tf2/time.hpp:45`）代表「取缓存最新」，**不是**「拿旧值假装当前测量」。`nav2_amcl` 另用
  `transform_tolerance_`（`amcl_node.hpp:386`）作为**显式**的发布 `tf` 时间前移量——**外推需显式模型 + 可诊断**。
- **robot_localization 对「未来 / 过去 dating」与旧包有显式处置**：
  `include/robot_localization/robot_localization/ros_filter.hpp:770` `tf_time_offset_` 注释
  「**For future (or past) dating the world_frame->base_link_frame transform**」——外推是**显式偏移参数**；
  `ros_filter.hpp:752` `last_set_pose_time_` 注释明确要处理「**比 setPose 还旧**的新消息」；滤波另有 `reset` 服务
  与 `measurement_history_`（`ros_filter.hpp:725`）——**复位会重置状态 / 历史**。
- **结论（本版本惯例）**：① 发布 / 消费的时间以**源采样 `Header.stamp`** 为准，接收时刻仅用于诊断；
  ② 外推**必须**显式建模（参数 / 专门时刻），**不得**无模型静默外推；③ 负年龄 / 时钟跳变 / 旧包按**显式**分支处理，
  **不得**静默钳为 0；④ 复位 / setPose 会**重置滤波状态与历史**。

**本项目现状差距（读取时工作区）：** 见「背景」：发布 `stamp=now()`、`age()` 钳 0、`history_` 与 reset 无关联。

**本批取证实测（本机真实二进制 / 实跑）：** 见
`experiments/w6-task-eval/<run_id>/r039/bench.json` —— 修复后：odom 的 `Header.stamp` **逐条等于**源样本 stamp
（`unmatched_stamps=0`）；`time_jump`（future stamp）→ `state=clock_anomaly`（age<0）且 `valid=false`；
`teleport`（新 epoch + 跳变）→ 重建前速度 0、**无尖峰**（`max_speed_after_teleport ≤ 2.0 m/s`，旧实现跨 teleport
差分约 ~600 m/s），并与同轴真值速度误差 ≤ 0.35 m/s。

## 决策

### (a) 源采样时刻 = 发布时刻（方案 A：仅新样本发布，不静默外推）

- `onGt()` 记录**源样本时刻** `src_stamp_`（`rclcpp::Time(msg.header.stamp)`）；`tick()` 仅在
  `src_stamp_ != last_pub_stamp_`（**有新样本**）时发布 odom / TF，`header.stamp = src_stamp_`。
- **不用 `now()` 冒充测量时刻**；**不引入**任何静默外推。TF 连续性由「源率 = odom 率（均 30 Hz）」承载；
  无新样本时**停发** odom / TF 并以 `valid=false` 显式表达——与 tf2「缺变换即显式错误」惯例一致。

### (b) 负年龄 / 时钟回拨 → 显式无效（移除钳 0）

- 年龄改为**有符号** `age = now − 源采样时刻`：`age < 0` → `state=clock_anomaly` + `valid=false`；
  `age > odom_max_age_s_` → `state=stale` + `valid=false`。**删除** `std::max(0.0, ...)` 钳制与 `last_rx_sim_`。

### (c) 复位 epoch 清空速度历史（+ 位置跳变守卫）

- 订阅 `/agc/scenario/reset_epoch`（`std_msgs/Int32`，`transient_local`，与 `scenario_reset` 同话题）；
  **epoch 变化即清空 `history_`**，重建 ≥2 个新样本前**速度输出 0**（`state=reset_rebuild`）——
  复位后首个有效速度**不**由 teleport 位移差分成尖峰。
- 另有**位置跳变守卫**（`reset_jump_clear_m`，默认 0.5 m）：相邻样本位移超过阈值即清空历史，
  与 epoch 订阅**双保险**（覆盖 teleport 样本先于 epoch 消息到达的时序）。

### (d) 诊断话题（非 ABI）

- 新增 latched `std_msgs/String` `/agc/ground/odom_diag`（默认值），内容
  `state=<ok|stale|clock_anomaly|no_sample|reset_rebuild> age=<秒> epoch=<n>`；承载参数写入
  `config/perception.yaml` 的 `ground_odom_adapter:` 段（launch 单一设置，节点不覆写）。

### (e) 验收

- `scripts/run_w6_r039.sh` + `scripts/w6_r039_bench.py`（真实二进制 + 合成真值 / epoch，四场景：断源 / 旧样本 /
  teleport / time jump，**同一时间轴**记录真值 / odom 速度）+ `scripts/w6_r039_check.py`（自包含核查 (a)…(k)）。

### (f) 边界（不改契约）

**不**改冻结 9 消息 / 服务 ABI，**不**变唯一执行口语义，**不**新增第二 ROS 命令者；新增均为诊断 / run config
（`/agc/ground/odom_diag` 为 `std_msgs`；`reset_epoch_topic` / `reset_jump_clear_m` 为 run config）；
`/agc/ground/odom` 的 `frame_id=world` / `child_frame_id=car/base_link` 与 `/agc/ground/odom_valid`（Bool）语义**不变**；
twist 仍按 ROS 约定表达在 child（车体）系。

## 备选

1. **方案 B：无新样本时以显式速度外推模型推到当前时刻**（`stamp=now`、诊断标注 `extrapolated`）——**否决**：
   外推到当前时刻会把「缓存位置」重新包装成「当前测量」，`age` 又趋 0，**恰恰重造** R-039 要消除的口径混淆；
   且 TF 连续性在本项目由「源率 = odom 率」已满足。
2. **保留 `std::max(0.0, ...)` 钳制，仅加告警**——**否决**：坏时钟仍算新鲜，与「需显式无效」冲突。
3. **只加告警不改发布 `stamp`**——**否决**：消费者仍按 `Header` 误判年龄。
4. **只订阅 `/agc/scenario/reset` 服务调用**——**否决**：服务调用不携带「已完成 teleport」的可靠时序；
   改用 `scenario_reset` **已发布**的 `reset_epoch`（transient_local）作为单调 epoch 信号。

## 后果

- **正面**：`Header.stamp` **如实**表达源采样时刻（消费者按 `Header` 得**真实年龄**）；负年龄 / 时钟回拨
  **显式无效**；reset epoch（+ 跳变守卫）**清空速度历史**，teleport **不再**产生速度尖峰；诊断话题使
  `ok / stale / clock_anomaly / reset_rebuild / epoch` 可判。
- **代价 / 限制**：发布率**随源率**（30 Hz），无新样本时**停发** odom / TF——若下游在源静默期仍期望连续 odom，
  须改以 `valid=false` / 诊断判失效（本项目下游 `task_manager` 门禁用 `odom_valid`、`ground_baseline` 读最新样本，
  已适配）。**未决**：是否需要为极低源率场景引入**显式**外推模型（届时须诊断标注 `extrapolated`）——本批不做，
  按需另议。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-039)
- 规格：[fix-ground-odom-map-identity-goal-r39-r41 spec](../../.trae/specs/fix-ground-odom-map-identity-goal-r39-r41/spec.md)（R-039）
- 契约：[#namespace](../ROS2迁移交接文档.md#namespace) / [#execution-port](../ROS2迁移交接文档.md#execution-port) /
  [#task-gating](../ROS2迁移交接文档.md#task-gating) / [#params](../ROS2迁移交接文档.md#params) /
  [#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) / [#messages](../ROS2迁移交接文档.md#messages)
- Nav2 / tf2 / RL 参照：`tf2_ros::MessageFilter`（以消息 stamp 查变换）/ `tf2::ExtrapolationException`
  与 `TimePointZero` / `nav2_amcl::transform_tolerance_`（显式发布前移）/
  `robot_localization::ros_filter` 的 `tf_time_offset_`（future/past dating）与 `last_set_pose_time_`（旧包）/
  `reset` 服务与 `measurement_history_`（复位重置历史）
- 前序：[ADR-0020](0020-perception-as-truth-derived-simulation.md)（真值派生感知）/
  [ADR-0030](0030-scenario-reset-controlled-transaction.md)（受控复位事务 / epoch）/
  [ADR-0008](0008-nav2-paradigm-absorption-as-code-principle.md)（Nav2 吸收）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 实现：`ground_odom_adapter.cpp` / `config/perception.yaml` / `run_w6_r039.sh` /
  `w6_r039_bench.py` / `w6_r039_check.py`
