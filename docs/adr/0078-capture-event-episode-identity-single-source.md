# capture 事件的 episode 身份改由 task episode 单一来源（启动双复位竞态致命中被丢弃）

## Status

accepted

## Date

2026-10-21

## 背景

W7-b P1 在新终态语义下的重跑（`w7-rl-b/20261021-w7-rl-b-104120-1456`，40 episodes）出现 **12/40 `MISSION_FAILED(TIMEOUT)`**，其中 4 个回合**已经打出 `CAPTURE_NET` 命中却仍未结算**：

| 回合 | `CAPTURE_NET` 次数 | 原始 `fco_rate` | 终态 |
|---|---|---|---|
| `p1_coop/seed9` | 10 | 0.602 | `TIMEOUT` |
| `p1_coop/seed12` | 4 | 0.141 | `TIMEOUT` |
| `p1_coop/seed18`（重试后） | 19 | 0.573 | `TIMEOUT` |
| `p1_noncoop/seed5` | 8 | 0.236 | `TIMEOUT` |

合计 **41 次命中事件被丢弃**。

**根因（启动期双复位竞态）**：

1. 每条 episode 启动时存在两条独立复位路径：`scenario_reset` 自身的 initial gated reset（[`scenario_reset.cpp`](../../src/agc_task/src/scenario_reset.cpp) 的启动回调）与 `task_manager` 在 `onStart` 内发起的 `/agc/scenario/reset`（[`task_manager.cpp`](../../src/agc_task/src/task_manager.cpp)）；
2. `task_manager` 在自身 reset 后**只等「epoch 相对 reset 前发生变化」**就锁存 `episode_epoch_` 并发布 `/agc/task/episode`；若 initial reset 落在其后，物理 epoch 再 +1；
3. `capture_adjudicator` 当时以 `/agc/scenario/reset_epoch`（物理 epoch）给配套事件打 `ep=`；`task_manager` 按 `ep != episode_epoch_` **静默拒收**。

**日志签名**（全部 40 个最终尝试）：

- reset 顺序为 `reset epoch=1 → reset epoch=2 → initial reset (gated): ok`（initial 在最后）：**7 个 TIMEOUT，0 个成功**；
- initial 在 task reset 之前（或 initial 失败）：**20 个成功，5 个 TIMEOUT**（后者为观测 / 火控原因，无命中事件）。

即：**该顺序与超时完全共现**；4 个有命中事件的回合证明事件被链路丢弃，而非任务失败。

## 决策

### (a) episode 身份**单一来源** = `/agc/task/episode`

- `capture_adjudicator` 改订 `task_manager` 发布的 `/agc/task/episode`（`std_msgs/Int32`，`transient_local`），并以该值作为：
  - 配套事件 `capture_event` 的 `ep=`；
  - `shot_id` 的 epoch 前缀；
  - 新 episode 的火控复位触发（`onEpisodeId`）。
- 参数由 `epoch_topic` 改为 `episode_topic`（缺省 `/agc/task/episode`），仍**单一注入**（[`w6_base.yaml`](../../src/agc_task/config/w6_base.yaml)）。
- `have_episode` 之前**不允许开火**——杜绝无身份事件。

### (b) 拒收不再静默

`task_manager` 对 `ep != episode_epoch_` 的配套事件加**限频告警**（2 s），如实暴露身份错位，避免再次以 TIMEOUT 形式静默吞掉命中。

### (c) 契约边界（**未**改动）

- 冻结 9 消息 / 服务 ABI **未改**；`capture_event` 仍是 `std_msgs/String`；
- 场景难度 / `t_max` / 火控参数 / 期望终态**未改**；
- 物理 reset epoch 仍用于 dwell 连续性（`current_epoch_`）与感知 / 复位簿记，**不**被本改动替代；
- 唯一执行口 / 唯一命令者**未改**（本改动只影响事件关联身份）。

## 备选

1. **serialize `scenario_reset` 的 initial 与 service reset** —— 不充分：若 service reset 先到，initial 仍可能后到；除非把两者合并为一次复位（改动面更大）。
2. **`task_manager` 等待 epoch 稳定后再锁存** —— 依赖时序窗口（实测 initial 最晚 +1.4 s，hover 约 10 s），非构造性保证。
3. **解析 reset 应答中的 epoch 作为 episode 身份** —— 不充分：后到的 initial reset 仍会再 +1，事件仍不匹配。
4. **取消 `ep` 校验** —— 否决：破坏 R-021 的 episode 隔离（旧事件可能结算新 episode）。

## 验证（2026-10-21，红-绿）

验证 run：`w7-rl-b/20261021-w7-rl-b-132023-5915`（`p1_coop` seeds 1/9/12/18，`backend:=full`）。

| seed | 本次 reset 顺序 | 修复前终态 | 修复后终态 | 说明 |
|---|---|---|---|---|
| 1 | `R1 → R2 → initial ok`（原错位序） | `TIMEOUT`（23 发 0 中） | **`INTERCEPT_CONFIRMED`** | 事件 ep=1 被接受 |
| 12 | `R1 → R2 → initial ok`（原错位序） | `TIMEOUT`（4 次命中被丢弃） | **`INTERCEPT_CONFIRMED`** | 事件 ep=1 被接受 |
| 18 | `R1 → R2 → initial ok`（原错位序） | `TIMEOUT`（19 次命中被丢弃） | **`INTERCEPT_CONFIRMED`** | 事件 ep=1 被接受 |
| 9 | `R1 → initial ok → R2`（本就匹配） | `TIMEOUT`（10 次命中被丢弃） | `TIMEOUT`（本次 0 次命中，全部打空） | 未复现错位序；属另一类（火控解算偏差） |

- 3 个**复现了原错位顺序**的回合全部转为成功，且捕获事件 `ep` 与 `task episode id` 一致（`shot=1-*`，物理 reset epoch 为 2）——修复对根因有效。
- `seed9` 本次启动顺序本已匹配，超时原因转为全部射击打空（miss 0.5–1.7 m），**不**属于本 ADR 的缺陷。

## 后果

- **正面**：capture 事件的产生方与消费方共享同一身份来源，启动复位顺序不再影响结算；身份错位有显式告警。
- **代价 / 限制**：`capture_event` 的 `ep` 语义由「物理 reset epoch」变为「task episode id」（与 `run_logger` 已用的 `episode_id` 同源）；历史 run 的事件 ep 与新语义不可逐值对照。
- **证据影响**：`w7-rl-b/20261021-w7-rl-b-104120-1456` 的 P1 判定（[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(h) 修订一）受本缺陷影响，仅作历史留痕；修复后重跑 `w7-rl-b/20261021-w7-rl-b-133724-7514`（**0 条拒收**、8 个超时均 0 命中）判定 P1 成立（primary 均值配对差 **+0.3526**，95% CI [+0.2052, +0.5000]，p=7.8e-05），见 [ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(h) 最终重跑子节。
- **回归防线**：拒收告警可被日志检查发现；单跑以 `p1_coop/seed9` 等原错位序作红-绿验证（本次验证 run `w7-rl-b/20261021-w7-rl-b-132023-5915`）。
- **未决 / 后续**：剩余超时（观测饥饿 / hold 正反馈、火控解算偏差、包络不满足）另行诊断；P2 判定、消融与 W7-c 未执行。

## 参考

- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)、[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)、[#params](../ROS2迁移交接文档.md#params)
- 前序 ADR：[ADR-0024](0024-capture-adjudication-fire-control-window.md)、[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)、[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)、[ADR-0076](0076-capture-scenario-tracking-milestone-not-terminal.md)
- 代码：[`capture_adjudicator.cpp`](../../src/agc_task/src/capture_adjudicator.cpp)、[`task_manager.cpp`](../../src/agc_task/src/task_manager.cpp)
