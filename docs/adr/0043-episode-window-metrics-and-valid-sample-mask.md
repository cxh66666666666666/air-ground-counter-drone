# ADR-0043: episode 窗口统计、终态锁存与有效样本掩码（R-028）

## Status

accepted

## Date

2026-10-16

## 背景

评审风险 **R-028（统计窗口与异步快照）**（[10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-028)）指出：
[`run_logger`](../../src/agc_task/src/run_logger.cpp) 把**进程全过程**（含预热 / 关闭）当统计窗口；
`terminal_state` 实为**最后任意 `state`**（会被覆写）；`scan` 与 `collision action` 无采样新鲜度关联，
陈旧 `scan` 会按新 `odom` **重复计数**；没触发 / 没 `scan` 时默认记 0。后果：预热比例改变统计；
**缺数据反而使指标更好**；一份 `summary` 说不清 0 来自安全、没触发，还是**根本没有任务**。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C9**（必报指标）、
[#params](../ROS2迁移交接文档.md#params)（单一注入，节点不得覆写）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一执行口语义不变，
`/agc/ground/cmd_vel` 仍是唯一执行口）、[#messages](../ROS2迁移交接文档.md#messages)（9 消息 ABI
不得变更）；产物纪律 [ADR-0007](0007-experiment-artifacts-convention.md)（唯一落点
`experiments/<campaign>/<run_id>/`）。R-027 已引入 `proxy_v1`（进程全时长）与 `gz_contact_v1`
（真实接触真值）双口径，见 [ADR-0042](0042-safety-infeasibility-honest-metric-and-contact-evidence.md)。

分类：**设计风险与统计口径**。本 ADR 只对统计**加窗口分段与掩码**，**不**否定 C9 必报要求，
**不**回退任何已验收组件，**不**改契约冻结 9 消息 ABI。

## 决策

### (a) episode 事件窗口分段（窗口口径 ≠ 进程口径）

以 task 进入 / 离开 **active**（`AIR_SEARCH` / `TARGET_ACQUIRED` / `GROUND_APPROACH`）为事件边界：
进入 active 且无开窗 → **开窗**；有开窗且收到**终态**（`TRACKING_SUCCESS` / `INTERCEPT_CONFIRMED` /
`MISSION_FAILED` / `ABORTED` / `INFRA_FAILURE`）或 `IDLE` → **关窗**（记 `leave_s` / `closed` /
`exit_state` / `exit_reason`）。开窗期间**每一拍 `odom`** 计入当前窗口计数（`ticks_total` …
`scan_samples`）。`finalize()` 时若仍有开窗 → `closed=false`（显式标注未闭合），
`leave_s = have_sim_ ? sim_end_s_ : 最后已知时刻`。窗口分段使**预热 / 关闭时长不改变**窗口统计。

### (b) 同时间快照 + 有效样本掩码（陈旧 scan 不重复计数）

每拍（`odom` 回调）以该拍时刻 `t_now = now().seconds()` 计算：
`scan_fresh = have_scan_ && (t_now - scan_rx_s_) <= scan_max_age_s_`、
`action_fresh = have_cm_state_ && (t_now - cm_rx_s_) <= action_max_age_s_`、
`tick_valid = scan_fresh && action_fresh`。窗口计数规则：`!scan_fresh` → `ticks_invalid_scan`；
否则 `!action_fresh` → `ticks_invalid_action`；否则 `ticks_valid`（其中 `triggered` 计
`triggered_ticks`，`contact||ttc<ttc_min_s_` 计 `infeasible_ticks`）。**陈旧 `scan` 段落一律进
`ticks_invalid_scan`，不计入窗口 valid / triggered / infeasible**。新增可标定参数
`scan_max_age_s` / `action_max_age_s`（默认 0.5 s），由 launch **单一注入**（契约
[#params](../ROS2迁移交接文档.md#params)），节点不得覆写。**`proxy_v1` 计算逻辑保持不变**
（触发判定仍用 `cm_action_==1||==2`），历史字段数值不变。

### (c) 终态锁存（首个终态）

新增 `terminal_latched_` 与首个终态 `terminal_state_`：仅当收到**终态**且**尚未锁存**时写入一次，
**之后任何 `state` 消息都不得覆写**。`summary` 的 `"terminal_state"`：有锁存终态则用之；否则回落到
`last_state_name_`（再空则 `"IDLE"`）。新增键 `"terminal_state_latched"`
（`"first_terminal_latched" | "last_state_no_terminal" | "none"`）。既有 `"reason_code"`
（最后一次 `state` 的 `reason_code`）**保留**；另加 `"terminal_reason_code"` 记锁存终态的 reason。

### (d) 无任务 / 无 scan → **拒绝**生成可用评价（不以 0 冒充）

聚合窗口量 `agg_ticks_*` / `agg_scan_samples`；`na_reasons`（有序数组）：无窗口 →
`"no_episode_window"`；否则 `agg_ticks_total==0` → `"no_odom_ticks_in_window"`、
`agg_scan_samples==0` → `"no_scan_in_window"`。`evaluation_applicable = (na_reasons 为空)`。
新增 `"episode_window_metrics"`（`window_count` / 分子分母 / `ticks_missing_ratio_scan` /
`ticks_missing_ratio_action`（`ticks_total==0` 时 `null`）/ `shield_infeasible_rate_window`）与
`"episode_windows"`（逐窗口）；`"evaluation_applicable"` / `"evaluation_na_reasons"`
（空数组 = 可评价）。窗口 `shield_infeasible_rate` 在 `evaluation_applicable==false` 或
`triggered==0` 时输出 **`null`**（**不**以 0 冒充）。`"not_applicable"` 扩展
`episode_window` / `window_scan` / `window_action` 三键。口径版本 `"definition_version"` 增
`"shield_infeasible_rate_window": "window_valid_v1"`、`"terminal_state": "first_terminal_latch_v1"`。

### (e) 进程口径与窗口口径**互不冒充**

`duration_s` / `odom_valid_ratio` / `cmd_vel_rate_hz` / `ticks_*` / `shield_infeasible_rate`
（`proxy_v1`）**保留**为**进程全时长**口径，另以 `"process_metrics_note"` 显式标注，与
`episode_window_metrics`（`window_valid_v1`）互不冒充。三口径（`proxy_v1` / `window_valid_v1` /
`gz_contact_v1`）均**不与旧 RL 安全集指标直接对照**，写于 `metrics_comparability` 与
`definition_version`。

### (f) 验收脚本与产物

新增 [`w6_r028_harness.py`](../../scripts/archive/w6_r028_harness.py)（确定性合成 harness，只用
`/clock` / `odom` / `scan` / `/collision_monitor_state` / `/agc/task/state`，**不发布**
`/agc/ground/cmd_vel`）、[`run_w6_r028.sh`](../../scripts/archive/run_w6_r028.sh) 与
[`w6_r028_check.py`](../../tests/w6_r028_check.py)。产物落
`experiments/w6-task-eval/<run_id>/{summaries,metrics/r028.json,log}`。

## 备选

1. **继续用进程全时长口径**：最简；但预热 / 关闭使统计漂移、缺数据使指标虚高 —— 否决（窗口口径
   与进程口径并列更诚实）。
2. **陈旧 scan 按新 odom 重复计数**（不设掩码）：会让重复计数虚增不可行率 —— 否决（以采样新鲜度
   关联掩码）。
3. **无任务 / 无 scan 时输出 0**：便于画表；但**以 0 冒充有效统计** —— 否决（`null` +
   `evaluation_applicable=false` + `evaluation_na_reasons`）。
4. **以「最后任意 state」当终态**：会被后续状态覆写（正是病灶） —— 否决（锁存**首个**终态）。

## 后果

- **正面**：`summary` 按 episode 事件窗口分段，预热 / 关闭**不改变**窗口统计；陈旧 `scan` 不再按新
  `odom` 重复计数；终态锁存为**首个**终态；无任务 / 无 scan **拒绝**生成可用评价（显式
  `not_applicable` / `evaluation_na_reasons`，**不**以 0 冒充）；进程口径与窗口口径**互不冒充**。
- **代价 / 限制**：窗口计数依赖 **`odom` 拍驱动**（无 `odom` 则无拍）；合成 harness 的 `/clock`
  为 20 ms 拍步进，与墙钟 / GZ 物理步长存在差异；`scan_max_age_s` / `action_max_age_s` 为可标定
  阈值（默认 0.5 s，与状态新鲜度门限量级一致）。
- **未决 / 后续**：`run_id` 唯一不可覆盖（R-029）、既有 `summary` 身份指纹与复现（R-029 / R-030）属
  后续项；本 ADR 仅落 R-028。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- [10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-028)（R-028）
- [spec](../../.trae/specs/fix-scenario-axis-metrics-identity-r26-r30/spec.md#requirement-episode-窗口统计终态锁存与有效样本掩码r-028)（R-028 Requirement）
- [ADR-0042](0042-safety-infeasibility-honest-metric-and-contact-evidence.md)（proxy_v1 / gz_contact_v1）
- [ADR-0007](0007-experiment-artifacts-convention.md)（产物落点）
