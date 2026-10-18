# 捕获事件 episode 隔离与事件身份（R-021：任务窗口 / 输入寿命 / 一次性复位 / 先命中者胜按时刻）

## Status

accepted

## 背景

评审风险 **R-021（捕获跨回合污染）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-021)）指出捕获裁决与消费侧
存在跨回合污染：

1. **无任务门**：`capture_adjudicator`（[源码](../../src/agc_task/src/capture_adjudicator.cpp)）进入包络即
   `fired_=true` 并发布 `capture_hit`，**不**检查任务状态——任务**未开始**即可先命中，随后 `task_manager`
   进入 `GROUND_APPROACH` 时消费**旧原因**。
2. **无输入寿命 / 无身份**：`car_.valid` / `target_.valid` 在**源停后不失效**；**不**检查 target `confidence` /
   `identity`——缓存位姿可持续产出命中。
3. **一次性标志不复位**：`fired_` 为进程级 latch，**不**随回合复位——第二回合可能**永不重发**或沿用第一回合成功。
4. **消费侧不隔离**：`task_manager`（[源码](../../src/agc_task/src/task_manager.cpp)）`start` **不清**
   `capture_reason_` / `jam_reason_`；`capture_hit` 以 `transient_local` 发布而 task 侧以 `volatile` 订阅，
   启动顺序还可能**漏早发事件**；`GROUND_APPROACH` **先判 capture 再判 jam** → 「先命中者胜」实际**固定 capture 优先**。

**接口演进争议（[X-04](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-04)）**：永久冻结九消息（契约
[#messages](../ROS2迁移交接文档.md#messages)）不足以表达 `episode` 关联；但**无需破坏旧 ABI**——应**新增**版本化配套
事件 / 话题，原消息**不改字段顺序**。本 ADR 记录该边界（**不静默**改 ABI）。

**先学后做（取证依据）**：契约 [终态语义](../ROS2迁移交接文档.md#terminal-semantics)（`reason_code ∈ {CAPTURE_NET, JAM_*}`、
`capture_hit` 精确匹配 `CAPTURE_NET`）、[参数入口](../ROS2迁移交接文档.md#params)（单一注入）、
[唯一执行口](../ROS2迁移交接文档.md#execution-port)（不新增第二命令者）；同版本已落地做法——R-013 的**到达时刻新鲜度门**、
R-015 的 `/agc/scenario/reset_epoch`（`std_msgs/Int32`，`transient_local`）、R-016 `intruder_driver` 的
`depth=1 + RELIABLE + TRANSIENT_LOCAL` epoch 订阅、R-018 `jam_confirm` 已确认事件（`JAM_*`）、
ROS 2 `transient_local` vs `volatile` 投递语义；以及既有 harness 桩环境结构（`w6_r020_check.py` / `w6_r013_check.py`
/ `w6_r015_check.py`）。本变更在既有结构内落地，**不**新增 ABI / 命令者。

分类：**设计风险**。

## 决策

### (a) 任务窗口门控（仅有效回合产出）
`capture_adjudicator` 订阅 `/agc/task/state`（`AgcTaskState`，订阅态）；**仅当** `state == GROUND_APPROACH`
（有效任务窗口）**且**输入新鲜**且**身份匹配时才允许**产出捕获命中。

### (b) 输入寿命与身份
增参（`launch/config` **单一注入**，节点不得覆写；契约 [#params](../ROS2迁移交接文档.md#params)）：
`car_max_age_s`（默认 `0.5`）、`target_max_age_s`（默认 `0.5`）、`confidence_min`（默认 `0.5`）、
`required_target_id`（默认 `""` = 不校验身份）。输入到达时刻（`car_rx_` / `target_rx_`）记录年龄；
**源停 / 超龄 / `confidence < confidence_min` → 判失效，不产出**（含不发布作用包络）；`required_target_id` 非空时
`target_id` 不匹配 → **不产出**。

### (c) episode 一次性复位
订阅 `/agc/scenario/reset_epoch`（`std_msgs/Int32`，`depth=1 + RELIABLE + TRANSIENT_LOCAL`，对齐 R-015 发布端）；
**epoch 值变化 = 新 episode → 复位** `fired_` 与标注消费态（`last_skip_label_`），使连续回合可**独立命中**。

### (d) 配套事件（新增，**不改**冻结 9 消息 ABI）
新增 `/agc/task/capture_event`（`std_msgs/String`，`transient_local`，**非**命令者），值
`CAPTURE_NET ep=<epoch> t=<ros_sec> target=<id>`（可解析）。`/agc/task/capture_hit` **仍恰为** `CAPTURE_NET`
（契约 [#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)，向后兼容）；`capture_evidence` 保留。
增参 `capture_event_topic`（默认 `/agc/task/capture_event`）。

### (e) 消费侧 episode 隔离 + 先命中者胜按事件时刻
- `task_manager` 订阅 `/agc/task/capture_event`（`transient_local`）结算 capture（**不再消费裸 `capture_hit`**）；
  解析 `ep`，**仅当 `ep == episode_epoch_`**（本 episode）才接受；不匹配 → **拒收**。
- jam 仍订 `/agc/task/jam_confirm`（**保持** R-018 值 `JAM_*`，**不**追加 factor——避免破坏 R-018 验收）；
  因不含 `ep`，以「已存在本 episode（`start` 后到达）**且**非 start 前」为门槛：`episode_epoch_ < 0`（尚无 episode）
  的 jam 一律忽略；`start` 成功亦**清空** `capture_reason_` / `jam_reason_` 并记录本 episode `episode_epoch_`。
- **先命中者胜按事件到达时刻定序**：分别记录最近一次**有效** capture / jam 事件的到达时刻（`capture_rx_` / `jam_rx_`）；
  两者并存时取**更早**者结算，**不再固定 capture 优先**。
- 学习当前 epoch：订阅 `/agc/scenario/reset_epoch`（transient_local），`start` 成功后 `episode_epoch_ = 当前 epoch`。

### (f) ABI / 命令者边界（X-04）
新增话题均为 `std_msgs` 状态 / 事件（**非**命令者）；**未改**契约冻结 9 消息 / 服务字段 / 顺序；`/agc/task/capture_hit`
取值仍恰为 `CAPTURE_NET`；**未新增**第二 ROS 命令者，**未改**唯一执行口（[#execution-port](../ROS2迁移交接文档.md#execution-port)）
语义。`task_manager` 的 `start` / `abort` 幂等与终态锁存属 **R-022（ADR-0037，另项）**，本 ADR 不越界。

## Considered Options

- **维持旧式（无任务门 / 无寿命 / 不复位 / 固定 capture 优先）** —— 被否：即 R-021 缺口（跨回合污染）。
- **直接扩冻结 9 消息（加 `episode` 字段）** —— 被否：违反契约 [#messages](../ROS2迁移交接文档.md#messages) 字段冻结；
  [X-04](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-04) 明确「无需破坏旧 ABI，替代为新增配套事件」。
- **把 `episode` 挤进 `reason` 字符串 / 给 `capture_hit` 追加标签** —— 被否：`capture_hit` 必须**恰为** `CAPTURE_NET`；
  字符串化 `reason` 令协议脆弱。
- **为 jam 统一新增带 `ep` 的配套事件** —— 被否：会改变 R-018 `jam_confirm` 值（`JAM_*`）口径、破坏已验收 R-018；
  改为「只对 capture 强制 `ep` 匹配、对 jam 用『新 episode 清 + 到达时刻定序 + 仅 start 后到达』门槛」。
- **(a)–(f) 组合（任务窗口 + 输入寿命 + episode 复位 + 配套事件 + 消费侧隔离 + 时刻定序）** —— **采纳**。

## Consequences

- 任务**未开始**（或非 `GROUND_APPROACH`）时目标入包络**不再**产出命中；源停 / 超龄 / 低 confidence / 身份不匹配
  **不再**产出；`fired_` 随 `epoch` 复位，连续回合可**独立命中**；旧 / 异 episode 事件被**拒收**；「先命中者胜」
  按**事件时刻**定序（`JAM_*` 亦可先胜）。
- **ABI 不变**：新增话题均为 `std_msgs` 状态 / 事件；冻结 9 消息 / 服务未改；唯一执行口未改；未新增命令者。
- **风险标注 / 置信度**：以自包含 harness（假源 + 假 reset 服务 + 假门禁数据）验证，见 [Evidence](#evidence)；
  jam 侧因无 `ep` 只能以「start 后到达」为门槛（如实限制）；若后续 jammer 能带 `ep`，可升级为强匹配（另项）。
  真实火控 / 弹道整链对拍留待后续（与本仓既有 harness 同口径）。
- 只读旧树边界：本次未读 `reference/`；未新增契约 / ABI 变更。

## Evidence

命令化验收 `bash scripts/run_w6_r021.sh headless 20261016-w6-r021-0`（自包含 harness，无 gz / launch），
结果 `experiments/w6-task-eval/20261016-w6-r021-0/metrics/r021.json`（`overall_pass: true`）：

| 用例 | 注入 | 观测（关键判据） |
|---|---|---|
| a 任务未开始 | `state=IDLE` + 目标入包络 | **无** `capture_hit` / `capture_event` |
| b 源停 / 超龄 / 低 confidence | b1 `confidence=0.1`；b2 源停后超龄再开窗口 | 两组均**不产出** |
| c 连续两回合 | 开窗口命中后 `epoch 1→2` | 事件 `ep=1`、`ep=2` 各一次；`capture_hit` 值恰 `CAPTURE_NET` |
| d 旧事件拒收 | start 前旧 `JAM_RTL`；`capture_event ep=999`（不匹配） | 状态保持 `GROUND_APPROACH`（不结算）；匹配 `ep` 后 `INTERCEPT_CONFIRMED CAPTURE_NET` |
| e 目标切换 | `required_target_id=intruder_uav`，先 `other_uav` 后匹配 | 不匹配**不产出**；匹配后命中 |
| f 同拍定序 | `fsm_hz=2`：jam 先→capture 后 ⇒ `JAM_HOLD`；capture 先→jam 后 ⇒ `CAPTURE_NET` | 先到者胜，**非**固定 capture 优先 |
| g 冻结值 / ABI | `grep` 源码 + `git` 工作树 | `hit.data = "CAPTURE_NET";` 存在；`src/agc_msgs` 未改 |

回归：`bash scripts/run_w6_r020.sh headless 20261016-w6-r020-0` **PASS**（`r020.json overall_pass: true`）；
`w6_r020_check.py` 假源已补发 `/agc/task/state`（`GROUND_APPROACH`）与 `/agc/scenario/reset_epoch` 以适配新语义。
