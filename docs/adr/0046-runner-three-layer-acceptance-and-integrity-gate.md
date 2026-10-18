# runner 三层验收判定与完整性硬门（R-031）

## Status

accepted

## Date

2026-10-17

## 背景

评审风险 **R-031（runner 假绿）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-031)）指出：
[`episode_runner`](../../scripts/episode_runner.py) 的回合判定只看**终态字符串**（`expected_match`）——
`started` / `summary_missing` / 子进程退出码 / 指标有效性**均不进入** `PASS` 判定；**缺 `summary`** 的
回合被**剔除**均值分母，空集合 `sum/max(len,1)` **返回 0**；按状态字符串等待**无本 run 关联**；
`SIGKILL` 后**无 `wait` 确认**。后果：基础设施失败或旧状态可呈现**符合预期的绿灯**；统计**分母选择性
减少**；批跑故障进程与迟到消息可**跨回合干扰**。R-029（[ADR-0044](0044-unique-run-identity-and-reproducible-evidence.md)）
已引入唯一 `run_id` / `same_run_verified` / 有界轮询就绪，但**完整判定分层**（启动/运行完整性 +
终态命题 + 指标有效性**全必过**）不在其内。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C9**（必报指标）、
[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)（终态语义）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一执行口语义不变）、
[#messages](../ROS2迁移交接文档.md#messages)（9 消息 ABI 不得变更）；产物纪律
[ADR-0007](0007-experiment-artifacts-convention.md)（唯一落点 `experiments/<campaign>/<run_id>/`）。

分类：**设计风险**。本 ADR 只对**批量评估 runner 的验收判定与进程管理**做决定，**不**回退已验收组件，
**不**改契约冻结 9 消息 ABI，**不**新增第二 ROS 命令者。

## 取证（先学后做）

按 [AGENTS.md](../../AGENTS.md) 铁律「先学后做」，**先**对齐同版本 Nav2 的成功用法：

- **看门狗 / 就绪协议**：`nav2_lifecycle_manager` 以 **bond 心跳超时**（`bond_timeout`，超时即判节点失联，
  见 `bondcpp` / `nav2_lifecycle_manager`）检测**节点死亡**，**不**以固定等待当就绪；`nav2_bringup` 以
  `autostart` + 生命周期状态为准入门槛。→ 本批把「回合结束」判据拆为**完整性 + 命题 + 有效性**三层，
  任一未过即 `pass=false`。
- **恢复（recovery）语义**：Nav2 无有效输入时进入**显式恢复动作**，而非静默沿用旧目标。

**本项目现状差距**（读取时工作区）：`episode_runner.py` 的 `pass = expected_match and backend`（仅命题层）；
均值列表**剔除**缺 summary 回合；`sum/max(len,1)`；`wait_terminal` 仅看 `state` 无 run 关联；`finally`
仅 `killpg(SIGINT)` + `time.sleep(5)` + `killpg(SIGKILL)`，**无 `wait` 确认**。

## 决策

### (a) 三层验收判定，全必过方 `PASS`

`classify_acceptance_layers(result)` 输出三层布尔：

- `layer_integrity`：`started` **且** 子进程退出确认（`proc_exit_confirmed`）**且** `summary` 存在
  （`summary_present`）**且** 同 run 关联（`same_run_verified` + `state_run_verified`）。
- `layer_proposition`：`expected_match`（`expected_terminal` + `expected_reason`）。
- `layer_metrics`：C9 两指标为**有效数值**（非 `None` / 非 `not_applicable`）。

`pass = layer_integrity and layer_proposition and layer_metrics`；`expected_match` 仅属命题层，**不再**单独
决定 `PASS`。`main()` 退出码改为以 `pass_count == n_episodes`（且 `n_episodes > 0`）为准。

### (b) 缺 summary / 空指标集合诚实记账（不剔分母、不以 0 冒充）

`summarize_metrics(episodes)`（**不**剔除任何回合）：对每指标统计 `valid_count` / `missing_count` /
`missing_ratio`；均值**仅基于有效样本**；**空有效集合 → `value='not_applicable'`**（**不**返回 0）。
新增 `metric_completeness` 块承载两指标的完整性与缺失比例；`summary_missing_episodes` /
`integrity_failures` 如实列出失败场景（**不含** `unsupported` 回合）。

### (c) 同一 run 关联：summary 唯一 id + `/agc/task/state` 同 episode 窗口

沿用 R-029 的 `same_run_verified`（`run_id` 含 `-<scenario>-<seed>-`）；**新增** `state_run_verified`：
`Monitor` 记录**首个终态观测的墙钟时刻**（`terminal_wall`），要求终态在**本次 start 调用之后**观测
（越窗终态 = 旧回合污染 → `state_run_verified=false`）；未达终态（无越窗终态可判）记 `true`。

### (d) 受限进程组回收 + `wait` 确认退出

`_reap_process_group(proc, pgid)`：`pgid` 在 Popen 后即取（防 leader 先死无法反查组）；`killpg(SIGINT)` →
`proc.wait(REAP_SIGINT_S)` → **无论 launch 是否已退，都补发组 `killpg(SIGKILL)` 清理可能忽略 SIGINT 的
游离子进程（如 gz sim）** → `proc.wait(REAP_SIGKILL_S)` → **轮询 `killpg(pgid,0)` 至整组回收**；
返回 `(exit_confirmed, returncode)`。仅回收 **Popen `start_new_session=True` 的本 run 进程组**；
**不**匹配裸名、**不**杀其它会话、**不**用 `rm -rf`（对齐 R-030 证据丢失事故教训，
[ADR-0045](0045-resetfail-evidence-contradiction-registration-and-redo.md)）。等待 `wait` 取代原固定
`time.sleep(5)`——`ros2 launch` 在 SIGINT 下先关闭节点（`run_logger` finalize 写 summary）再退出，故
`wait` 返回即 summary 已落盘。实测发现：**仅当 launch 仍在时才发 SIGKILL** 会遗留忽略 SIGINT 的 gz sim，
导致后续回合 `gt_stale` 假 infra（跨回合干扰），故改为**无条件补发组 SIGKILL + 整组回收确认**。

### (e) 就绪门槛 = 服务就绪 + 真值新鲜（不以固定等待代就绪）

`wait_ready(mon, timeout_s)` 由「仅 `/agc/task/start` 服务就绪」**加强**为「服务就绪 **且** 车/目标真值
旁路**新鲜**」：`Monitor` 订 `/agc/gt/agc_car` 与 `/agc/gt/intruder_uav`（`PoseStamped`），`gt_fresh()`
要求两者接收年龄均 `< GT_FRESH_S`。原因：`scenario_reset` 的复位前置要求 gt 新鲜（年龄 ≤
`readback_max_age_s`=0.5 s），**仅看服务存在**会令 `start` 早于 gt/ros_gz 桥就绪 → `gt_stale` → 3 次
重试耗尽 → `reset_failed` 假 infra（实测）。此为**真就绪协议**（有界轮询），**不**以固定 `sleep` 代就绪。

### (f) 验收脚本与产物

新增 [`run_w6_r031.sh`](../../scripts/archive/run_w6_r031.sh) + [`w6_r031_check.py`](../../tests/w6_r031_check.py)：
Phase A **实跑** episode_runner（轻量栈 s01 seed 0/1 两回合）；Phase B 自包含核查——Case a 三层判定逻辑、
Case b 缺 summary 不剔分母、Case c 空集合 `not_applicable`（直接 import 判定 / 汇总纯函数）、Case d **实跑**
报告三层全过、Case e 脚本卫生（无 `rm -rf` / 裸名 `pkill`，含 `killpg` + `wait`）、Case f ADR-0046、
Case g 9 消息 ABI 未改。产物落 `experiments/w6-task-eval/<run_id>/{runner,metrics/r031.json,log}`。

## 备选

1. **继续只看 `expected_match`**：最简、与旧报告一致；但**缺 summary / 缺日志 / 基础设施失败**仍可假绿
   （评审病灶）——否决（三层全必过）。
2. **缺 summary 剔除分母**：使均值更「干净」；但**选择性减少分母**会系统性抬高成功率——否决
   （缺失计入分母并如实报缺失比例，空集合 `not_applicable`）。
3. **`wait_terminal` 仅看状态字符串**：免新增字段；但**无法**区分本回合终态与旧回合污染——否决
   （记录终态墙钟并要求在 start 之后观测）。
4. **`SIGINT` 后固定 `time.sleep` 再 `SIGKILL`**：简单；但**不确认退出**且固定等待非就绪协议——否决
   （`wait(timeout)` 两步确认）。

## 后果

- **正面**：回合 `PASS` 需**三层全必过**；缺 summary / 缺日志**不**剔分母且如实报缺失计数 / 比例；空集合
  `not_applicable`（**不**以 0 冒充）；`/agc/task/state` 终态以**本 episode 窗口**关联；进程组回收
  **每一步 `wait` 确认退出**，无裸名杀 / 无 `rm -rf`。Case a…g 实跑对照。
- **代价 / 限制**：新增字段（`layer_*` / `metric_completeness` / `summary_present` / `proc_exit_confirmed` /
  `state_run_verified`）均为**报告 JSON 字段**（run 产物），**未**改冻结 ABI；`state_run_verified` 依赖
  `Monitor` 的墙钟记录，跨机时钟差异不影响（单机 wall monotonic）。
- **未决 / 后续**：`run_w6_r031.sh` Phase A 只跑轻量栈（不含 PX4 / 空中链路）；整链 runner 调用（含
  GUI / full）属 P2 后续。Case b/c 为**纯函数级**逻辑验证（**不**跑仿真），已如实标注。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-031)
- 规格：[fix-runner-green-freeze-uav-safety-r31-r33 spec](../../.trae/specs/fix-runner-green-freeze-uav-safety-r31-r33/spec.md)（R-031）
- Nav2 参照：`nav2_lifecycle_manager`（bond / 看门狗）/ `nav2_bringup`（autostart）
- 前序：[ADR-0044](0044-unique-run-identity-and-reproducible-evidence.md)（唯一 run 身份 / 同 run 关联）、
  [ADR-0045](0045-resetfail-evidence-contradiction-registration-and-redo.md)（证据丢失教训）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 实现：`episode_runner.py` / `run_w6_r031.sh` / `w6_r031_check.py`
