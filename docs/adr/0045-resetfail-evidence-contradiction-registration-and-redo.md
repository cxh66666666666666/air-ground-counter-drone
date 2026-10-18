# resetfail 证据矛盾登记与唯一新 run 重做（R-030）

## Status

accepted

## Date

2026-10-16

## 背景

评审风险 **R-030（文档与产物矛盾）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-030)）指出：
resetfail（非法 spawn → 净空不足 → 复位重试耗尽）验收的**声明**与其**所指向证据**相反，不能据此背书
契约 C9（[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)）。本 ADR 对**据实登记**与
**唯一新 run 重做**做决定，**不**回退任何已验收组件，**不**改契约冻结 9 消息 ABI，**不**新增第二 ROS 命令者。

### ① 声明与所见原件之矛盾（R-030）

W6 spec `.trae/specs/w6-task-eval/tasks.md` 的 **Task 7.3** 声明「非法 spawn → `reset_retries=3`、
`reset_failed=true`、start 返回 `reset_failed`」，**Task 8.4** 声明「`reset_failure_rate=1.0`」，
且完成记录以 `20261014-c9-resetfail-0/metrics/summary.json` 为证据。但本批评审所见**同路径** summary
实为 `terminal_state=IDLE`、`reset_retries=0`、`reset_failed=false`、`reset_failure_rate=0`。
二者**直接相反**；完成记录所指向的证据**已不足以复核**该测试，C9 失败场景「非零」结论**不能**由该路径重现。

### ② 证据完整性事故（2026-10-16）

- **误删范围**：执行 R-028 的子代理在清理残留进程时误执行 `rm -rf experiments/w6-task-eval/20261014-*`，
  删除全部 `20261014-*` 旧实验产物，**含 R-030 保全目标 `20261014-c9-resetfail-0`**。
- **不可恢复依据**：`git ls-files experiments` 中 `w6-task-eval` 记录为 **0 条**（`.gitignore` 仅放行
  `experiments/**/config/`）；`find /home/chen -name "*c9-resetfail*"` 无副本；无备份、无回收站、无 stash。
  → **本环境不可恢复**，旧原件与其原始日志 / 运行配置 / 产物身份**一并永久丢失**。
- **受影响面**：丢失清单（部分）含 `20261014-c9-resetfail-0`（R-030 目标）、`20261014-campaign-*`、
  `20261014-w6-full-s02/-s03`、`20261014-w6-jam-0`、`c9-infeasible-0`、`r6-random-0` 等
  （详见 [spec 证据丢失修订](../../.trae/specs/fix-scenario-axis-metrics-identity-r26-r30/spec.md#requirement-矛盾原件登记与唯一新-run-重做r-030)）。
- **评审文档悬空引用清单（5 处死链）**：
  1. `experiments/w6-task-eval/20261014-c9-resetfail-0/metrics/summary.json`（R-030 本体）
  2. `experiments/w6-task-eval/20261014-campaign/report.json`
  3. `experiments/w6-task-eval/20261014-w6-full-s02/…/full_linear.json`
  4. `experiments/w6-task-eval/20261014-w6-full-s03/…/full_maneuver.json`
  5. `experiments/w6-task-eval/20261014-w6-jam-0/metrics/jam.json`
  以上引用分布在 [10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-030) 与
  [00-总览](../../docs/temp/agc-review-2026-10-14/00-总览.md#judgment)。

分类：**事实错误与证据完整性事故**。原件已不可恢复，问题从「保全并标注矛盾」转为「**据实登记 + 唯一新 run 重做**」。

## 决策

### (a) 据实登记，不伪造、不冒充复核

- **不**伪造 / 重建 `20261014-c9-resetfail-0/metrics/summary.json`，**不**把数字改写成期望值；
- **不**声称旧 `reset_retries=3` / `reset_failed=true` / `rate=1.0` 数字**已复核**（其原件已丢失，无从复核）；
- 矛盾与丢失事实**只**登记于本 ADR 与 spec 证据丢失节，**不**改写评审文档结论。

### (b) 唯一新 run 重做 resetfail 验收

在 **R-029 不可覆盖 `run_id`**（[ADR-0044](0044-unique-run-identity-and-reproducible-evidence.md)）下**重做**
非法 spawn 验收：`run_id` 形如 `<YYYYMMDD>-c9-resetfail-0-<HHMMSS>-<rand4>`，独占创建、已存在即拒（rc=2）。
新 run **补**原始日志 + 运行配置 + 产物身份：

- **原始日志**：`start` 服务应答（应 `success=false`、`message="start rejected: reset_failed"`）、
  `launch.log` 中的复位拒绝原因（`car_target_clearance`）与重试记录；
- **运行配置**：`config/params_applied.yaml`（非法 spawn 展开参数，净空不足可见）+ 代码身份
  （`git_head.txt` / `git_diff.patch` / `git_status_porcelain.txt` / `git_untracked.txt` /
  `workspace_fingerprint.json`）+ 工具链与世界哈希（`toolchain_world.json`）；
- **产物身份**：`reset_retries` / `reset_failed` / `reset_failure_rate` **非零**的 summary，可独立复核。

### (c) 不凭推断改写 Runbook 完成状态

**不**凭推断改写 [Runbook `#status`](../ROS2迁移步骤.md#status)；重做结果与 W6 tasks 声明一致或不一致均
**据实记录**。R-030 的完成状态**保留**为进度事实，由 Main 代理据新 run 证据决定。

### (d) 引用纪律

跨文档一律用**稳定锚点** `<文件>#<锚点>`；**禁止**以章节号记法（section-number）做跨文档引用。

### (e) 验收脚本与产物

新增 [`run_w6_r030.sh`](../../scripts/run_w6_r030.sh) + [`w6_r030_check.py`](../../scripts/w6_r030_check.py)：
起无头 `m0_skeleton`（注入 `w6_c9_resetfail.yaml` 参数与 `world_file`）→ **有界轮询** `/agc/task/start`
就绪 → 调 `start`（预期被拒）→ 优雅停止（`SIGINT` 让 `run_logger` 写 summary）→ 拷 summary /
config 至独立 run 目录 → 自检 a…h。产物落
`experiments/w6-task-eval/<run_id>/{log,resetfail,metrics/r030.json}`（[ADR-0007](0007-experiment-artifacts-convention.md)）。

## 备选

1. **重建旧 summary 使数字「与声明一致」**：看似恢复证据；但属**伪造**历史产物，违反据实纪律 —— **否决**
   （据实登记 + 新 run 重做）。
2. **直接改 W6 tasks / Runbook 完成状态以求自洽**：省事；但**凭推断**改写完成状态，掩盖证据丢失 —— **否决**
   （Runbook 状态不动，矛盾据实记录）。
3. **在旧路径名下新跑一次冒充原件**：可复用旧 glob；但会**冒充**已丢失原件、破坏唯一身份 —— **否决**
   （新 run 用 R-029 唯一 `run_id`，旧路径名**不复存在**，check 断言其不存在）。
4. **仅登记、不重做**：最省；但 C9 失败场景「非零」仍无可复核证据 —— **否决**（唯一新 run 重做补证）。

## 后果

- **正面**：R-030 矛盾与证据丢失事实**据实登记**；resetfail 非法 spawn 验收在**唯一不可覆盖 run** 下重做，
  给出 `reset_retries` / `reset_failed` / `reset_failure_rate` **非零** + 原始日志 + 运行配置 + 产物身份，
  可独立复核；评审死链被显式登记，不误导后续引用。
- **代价 / 限制**：旧原件**永久不可恢复**（无副本）；旧 `20261014-c9-resetfail-0` 数字**永久无法复核**，
  只能以新 run 为准；`reset_retries` 由 `task_manager` 以 `transient_local` 发布，`run_logger` 须在优雅退出前
  收到该值（脚本以「调用后等待 + `SIGINT` 优雅退出」保证）。
- **未决 / 后续**：新 run 结果与 W6 tasks 声明的**一致性结论**待据实回填；评审文档 5 处死链的**正文**由 Main
  代理决定是否补注（本 ADR 不改动画评审文档）。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-030) / [00-总览](../../docs/temp/agc-review-2026-10-14/00-总览.md#judgment)
- 规格：[R-030 Requirement](../../.trae/specs/fix-scenario-axis-metrics-identity-r26-r30/spec.md#requirement-矛盾原件登记与唯一新-run-重做r-030)
- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C9 必报指标）/ [#params](../ROS2迁移交接文档.md#params)（单一注入）
- 前序：[ADR-0044](0044-unique-run-identity-and-reproducible-evidence.md)（唯一 run 身份）/ [ADR-0043](0043-episode-window-metrics-and-valid-sample-mask.md)（episode 窗口统计）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 实现：`run_logger.cpp` / `task_manager.cpp` / `scenario_reset.cpp` / `run_w6_r030.sh` / `w6_r030_check.py`
