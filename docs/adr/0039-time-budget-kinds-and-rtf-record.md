# 三类时间预算口径与 RTF 记录（R-024：mission_sim_budget / episode_wall_watchdog / 进程时长）

## Status

accepted

## 背景

评审风险 **R-024（三种时间口径）**（[10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-024)）指出系统内存在**三处互不对应**的计时：

1. **FSM 超时（`mission_sim_budget`）**：[`task_manager.cpp`](../../src/agc_task/src/task_manager.cpp) 的 C4 用
   `(now() − start_time_) > t_max_s`，`now()` 为 **ROS 仿真时钟**（`use_sim_time:=true`，来自 `/clock`）。
2. **runner 墙钟看门狗（`episode_wall_watchdog`）**：[`episode_runner.py`](../../scripts/episode_runner.py) 用
   **单调墙钟**（`time.monotonic`）等待终态，超时后把 `terminal_state` 写 `ACTIVE`。
3. **进程时长（`process_wall_duration`）**：[`run_logger.cpp`](../../src/agc_task/src/run_logger.cpp) 用
   `std::chrono::steady_clock` 记**从构造到 finalize** 的墙钟时长（含预热 / 关闭），字段 `duration_s`。

后果：**不同 RTF（实时因子）的成功率 / 耗时不可比**（暂停仿真不会触发 FSM 超时，慢仿真可能先耗尽 runner 预算）；
**C4 任务超时失败**与**基础设施看门狗**未清楚区分；训练侧若另用 step budget 会产生**第三套结果**。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C4**（超时一律算失败、只有显式完成才算成功）、
[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)、[#params](../ROS2迁移交接文档.md#params)（launch / config
**单一注入**，节点不得覆写）、[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一执行口语义不变）、
[#messages](../ROS2迁移交接文档.md#messages)（9 消息字段 / 顺序不得变更）。

**先学后做（取证依据）**：核验本机同版本语义——① ROS 2 仿真时钟 vs 墙钟（`use_sim_time` + `/clock` + RTF）；
② Nav2 lifecycle `bond` / 看门狗口径（**「停住」≠「完成」**，外部看门狗只判活性/超时，不取代任务结果）；
③ 现三处计时的实际实现（上列 1–3）。结论：三者**语义不同**，应**分别命名、分别标注、互不冒充**。

分类：**设计风险与未决事项**。风险原文即注明「不建议把所有 timer 一律改墙钟」；本 ADR 给出「应选哪种任务预算」的明确决定。

## 决策

### (a) 三类口径成文并命名（互不冒充）

| 口径 | 时钟 | 归属 | 参数入口 | 语义 |
|---|---|---|---|---|
| `mission_sim_budget` | `ros_sim`（仿真时钟 `now − start_time_`） | `task_manager`（C4） | `t_max_s`（`config/w6_base.yaml` 单一注入） | **任务预算**：本 episode 任务超时判据（超时算失败） |
| `episode_wall_watchdog` | `wall_monotonic`（`time.monotonic`） | `episode_runner` | `--wall-watchdog-s`（缺省回落 `--timeout`） | **外部看门狗**：墙钟等待 episode 结束的上限；超时判 **infra**、**不算成功** |
| `process_wall_duration` | `wall_steady`（`steady_clock`） | `run_logger` | `duration_s`（run 产物字段） | **进程时长**（含预热 / 关闭）；仅记录，**不**参与 episode 判定 |

三者以 `TIME_BUDGETS`（[`episode_runner.py`](../../scripts/episode_runner.py)）显式标注 `clock` / `owner` / `param`。

### (b) 应选哪种任务预算为**设计决定**：`mission_sim_budget`（ROS 仿真时钟）

任务成功 / 失败是**任务语义时间**上的判据（C4），故任务预算**必须**跟随仿真时钟：
仿真暂停（`/clock` 停发 / RTF→0）时任务时间**不推进**，**不得**因此误判任务超时。
**不**把所有 timer 一律改墙钟；`episode_wall_watchdog` 仅作**外部兜底**（防 runner 无限等待），进程时长仅作运行观测。

### (c) 墙钟看门狗超时的显式分类（且不算成功）

`episode_runner` 的 episode 结束判定**收敛到唯一函数** `classify_episode_outcome(...)`：

- 达终态且为 `TRACKING_SUCCESS` / `INTERCEPT_CONFIRMED` → `mission_terminal` / `classification=mission` /
  `budget_kind=mission_sim_budget`；
- 达终态 `MISSION_FAILED` / `ABORTED` → `mission_terminal` / `mission` / `mission_sim_budget`；
- 终态 `INFRA_FAILURE` → `infra_failure` / `infra`；
- **未达终态（等待被墙钟看门狗停止）** → `wall_watchdog_timeout` / **`infra`** /
  `budget_kind=episode_wall_watchdog` / **`counted_as_success=False`**。

即：**外部看门狗超时写入显式 infra 分类，且不计为成功**（区别于 C4 的 `MISSION_FAILED` 任务失败）。

### (d) RTF 与起止事件记录（同一 run 可比）

- **`episode_runner`**：每个 episode 记 `wall_start_s` / `wall_end_s` / `wall_elapsed_s`、`sim_start_s` /
  `sim_end_s` / `sim_elapsed_s` 与 **`rtf`**（= 仿真增量 / 墙钟增量）；`report.json` 汇总 `rtf_mean`。
  记录自 episode 开始（`Monitor` 于 `Popen` 前建立、订阅 `/clock`），故不同 RTF 的**成功率 / 耗时**可按口径区分
  （「仿真时间」与「墙钟时间」）。
- **`run_logger`**：`summary.json` 新增 `duration_kind="process_wall_incl_warmup_shutdown"`（进程时长口径标注）、
  `wall_start_unix_s` / `wall_end_unix_s`（起止事件）、`sim_start_s` / `sim_end_s` / `rtf`（RTF 证据）；
  `duration_s` 语义不变（进程墙钟）。

### (e) train / eval 同一预算语义（不产生第三套口径）

`classify_episode_outcome` 为 **episode 结束判定的唯一来源**：评估 adapter（`episode_runner`）直接调用；
**W7 训练 adapter 必须复用同一函数**（当其在 W7 落地时），**不得**自造独立 step budget / 结束语义。
参数单一注入：mission 预算 `t_max_s` 经 `config/w6_base.yaml`；watchdog 预算经 runner CLI。

### (f) 契约 / ABI 边界

**未改**契约冻结 9 消息 / 服务字段 / 顺序；`capture_hit` 仍恰为 `CAPTURE_NET`；新增字段仅落在 **run 产物**
（`report.json` / `summary.json`，非 ROS 接口）；**未新增**第二 ROS 命令者；**未改**
[#execution-port](../ROS2迁移交接文档.md#execution-port) 唯一执行口语义；契约文档未改。

## Considered Options

- **维持现状（三套口径互不对应、runner 超时写 `ACTIVE` 无分类）** —— 被否：跨 RTF 不可比，infra 看门狗与 C4 失败混淆，即 R-024 缺口。
- **把所有 timer 一律改墙钟** —— 被否：仿真暂停 / 慢仿真会**误判任务超时**，违反 C4 的任务语义时间口径（风险原文亦不建议）。
- **只改 runner、不标注 logger 进程时长** —— 被否：进程时长含预热 / 关闭，与 episode 预算混用即「冒充」，且缺 run 证据。
- **训练侧另立 step budget（第三套口径）** —— 被否：违反「train / eval 同一预算语义」。
- **新增带时间字段的冻结 9 消息 / 新命令者承载 RTF** —— 被否：违反 [#messages](../ROS2迁移交接文档.md#messages)、
  [#execution-port](../ROS2迁移交接文档.md#execution-port)；RTF / 起止仅落 run 产物即可。
- **(a)–(f) 组合（三类命名 + 选 mission_sim_budget + watchdog 显式 infra 分类 + RTF/起止记录 + 单一来源 + ABI 边界）** —— **采纳**。

## Consequences

- 三类时间口径**分别标注、互不冒充**；外部看门狗超时写**显式 infra 分类**且**不计成功**。
- **暂停仿真不触发** `mission_sim_budget`（任务时间不推进），而 `episode_wall_watchdog` **触发**并判 `infra`；
  恢复仿真后 C4 才按**仿真时钟**触发 `MISSION_FAILED / TIMEOUT`（对照明确）。
- run 证据含 **RTF 与起止事件**；不同 RTF（加速 / 减速）的耗时可按「仿真时间」与「墙钟时间」分别比较。
- **不**把所有 timer 改墙钟；C4 任务预算仍为 `mission_sim_budget`（设计决定）。
- episode 结束判定**只有一个来源**；W7 训练 adapter 落地时**必须复用**，不产生第三套结果。
- **ABI 不变**：新增字段均落 run 产物；唯一执行口未改；未新增命令者；契约文档未改。
- 只读旧树边界：本次未读 `reference/`；未新增契约 / ABI 变更。

## Evidence

命令化验收 `bash scripts/run_w6_r024.sh headless 20261016-w6-r024-0`（自包含 harness，无 gz / launch），
结果 `experiments/w6-task-eval/20261016-w6-r024-0/metrics/r024.json`（`overall_pass: true`，5/5）：

| 用例 | 注入 | 观测（关键判据） |
|---|---|---|
| a 暂停仿真 vs 墙钟看门狗 | `use_sim_time: true` + 自发布 `/clock`；`t_max_s=3.0`；仿真进行到 `sim_ns≈2.55s` 起 `start` 后**停发 `/clock`** 5s 墙钟 | 冻结期 `state=AIR_SEARCH`、`mission_budget_not_triggered=true`（仿真时钟不推进 → C4 不触发）；`classify_episode_outcome('ACTIVE',…,5,3)` → `wall_watchdog_timeout / infra / episode_wall_watchdog / counted_as_success=false`；**恢复 `/clock`** 后 `state=MISSION_FAILED reason=TIMEOUT`（C4 按仿真时钟触发） |
| b RTF 与起止事件 | `build_time_evidence` 两组（墙 10s/仿真 20s；墙 40s/仿真 20s） | `sim_elapsed_s` 均 `20.0`（按仿真时间**可比**）、`wall_elapsed_s` 不同、`rtf=2.0 / 0.5`；无仿真时钟 → `rtf=None` |
| c 三类口径标注 | `TIME_BUDGETS` + 源码检查 | 恰 3 键；`clock`/`owner` 三值互异；`MISSION_FAILED→mission_sim_budget`、`ACTIVE` 超时→`episode_wall_watchdog`（`no_impersonation=true`）；`run_logger.cpp` 含 `process_wall_incl_warmup_shutdown` + `rtf`；`task_manager.cpp` C4 用 ROS 时钟判据串 |
| d train / eval 单一来源 | 全仓 `def classify_episode_outcome` 计数 | 定义**恰 1 处**（`episode_runner.py`）；eval adapter 调用之；无第三套 `budget_kind` |
| e 冻结值 / ABI | `grep` 源码 + `git` 工作树 | `hit.data = "CAPTURE_NET";` 存在；`src/agc_msgs` 未改 |

回归：`bash scripts/run_w6_r023.sh headless 20261016-w6-r024-regress` **PASS**
（`experiments/w6-task-eval/20261016-w6-r024-regress/metrics/r023.json` `overall_pass: true`，6/6）
—— 连续跟踪 dwell 中断语义未回退。
