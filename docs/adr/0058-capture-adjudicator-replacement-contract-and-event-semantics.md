# 捕获裁决器替换契约澄清：对外视图不变，**不**排除内部事件语义演进（X-05）

## Status

accepted

## 背景

[ADR-0024](./0024-capture-adjudication-fire-control-window.md) (c) 记录了一项保证：
「该替换（后续补真实弹道）在 [ADR-0022](./0022-launched-net-capture-mechanism.md) (c) 预留的**可插拔接口**内完成，
**调用方 `task_manager` 不改**」。

评审挑战 **X-05**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-05)）指出：该保证**不应**被读成
「裁决器无需任何内部事件语义演进」。现实现
（[`capture_adjudicator.cpp`](../../src/agc_task/src/capture_adjudicator.cpp)）的证据：

- 接口为**匿名 namespace 内**的纯几何函数 `evaluate(Pose2 car, Pose2 target) -> CaptureDecision`
  （[`:88`](../../src/agc_task/src/capture_adjudicator.cpp#L88)），**构造硬编码** `EnvelopeParams`（[`:58`](../../src/agc_task/src/capture_adjudicator.cpp#L58)）；
- **无**时间 / 目标速度 / **射击请求** / **shot 生命周期**（射击 / 飞行 / 命中 / 失手 / 取消 / 冷却）；
- 真实弹道判定需要**异步效果**（发射后按 `t_flight` 预测目标位置）与**失手 / 取消**语义，
  这不是"仅换实现"能覆盖的。

评审结论原话取向：**不能**把「`task_manager` 绝不改」当作**物理正确性**的前提；同时**不必**破坏旧 ABI
（可保持最终任务视图与冻结 9 消息兼容）。承接 [ADR-0036](./0036-capture-event-episode-isolation-and-event-identity.md)
（episode 隔离 + 事件身份）与 [ADR-0040](./0040-full-chain-coverage-matrix-and-same-run-causal-record.md) (e)（不改 ABI）的既有方向。

分类：**更优路线（挑战既定约束）**。本 ADR **澄清** ADR-0024(c) 的**适用范围**，**不**推翻其主决策
（"闭式占位 → 火控窗口 + 瞄准提前量"仍成立），**不**改契约 / 冻结 ABI；裁定**已获用户批准**（见「决策（已采纳 (b)）」）。

## 决策（已采纳 (b)）

**(b) 收窄并澄清"调用方不改"（已采纳）**：

- **(a) 维持 ADR-0024(c) 字面「调用方完全不改」** —— **被否**：会**阻止异步裁决事件语义**（发射请求、shot 生命周期、
  失手 / 取消），使"真实弹道"在**不改任务层**约束下**无法落地**；即评审所指"零成本替换的隐含承诺"。
- **(b) 收窄为"对外可观测契约不变"（已采纳）**：
  1. **对外不变**：最终终态 `INTERCEPT_CONFIRMED` 且 `reason_code = CAPTURE_NET`；冻结 **9 消息**
     （[#messages](../ROS2迁移交接文档.md#messages)）字段 / 顺序**不变**；唯一执行口
     （[#execution-port](../ROS2迁移交接文档.md#execution-port)）**不变**；**不新增第二 ROS 命令者**。
  2. **内部允许演进**：裁决器**内部**可有 **shot 生命周期**（射击请求 / 发射 / 到达 / 命中 / **失手** / **取消** / 冷却）
     与**异步事件**；消费侧可在**既有 ABI 内**做必要的状态清理 / 关联（承接
     [ADR-0036](./0036-capture-event-episode-isolation-and-event-identity.md) 的 episode / 事件身份）。
  3. **执行顺序**：**先**完成 R-021 的 episode 隔离与事件身份（[ADR-0036](./0036-capture-event-episode-isolation-and-event-identity.md)），
     **再**替换弹道判定器；替换**不是**"零改动"，而是"**对外零破坏**"。

**ADR 处置（已执行）**：已在 [ADR-0024](./0024-capture-adjudication-fire-control-window.md) (c) 补一条**范围澄清**（**措辞澄清，非语义变更、非推翻主决策**），并登记于契约 [#decisions](../ROS2迁移交接文档.md#decisions)；**未**动契约真源文字。

## Considered Options

- **维持 ADR-0024(c) 字面零改动** —— 被否：阻止异步事件语义，真实弹道不可落地。
- **收窄为"对外 ABI / 终态视图不变，内部允许事件语义演进"** —— **推荐**。
- **本轮即实现 shot 状态机与弹道** —— 被否：超本次出口（[ADR-0016](./0016-execution-scope-exit-w6.md)）范围；
  须先完成 episode 隔离（[ADR-0036](./0036-capture-event-episode-isolation-and-event-identity.md)）再替换。

## Consequences

- **"task_manager 不改"限定为对外视图**：终态 / 冻结消息 / 唯一执行口不变；**不**再被读成"裁决器无内部事件语义"。
- **物理正确性不再被 I/O 稳定绑架**：真实弹道可引入异步发射、失手、取消与冷却，而**不改**冻结 9 消息与命令者。
- **接口边界需明确**：裁决器成为**内部事件协议 + 插件**；需 shot 状态机与**回放验收**。
- **迁移成本：中至高**：裁决插件边界、shot 状态机、异步命中 / 失手 / 取消事件的消费与回放验收。
- **只读边界**：本 ADR 只读 [10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-05)、既有 ADR-0022 / 0024 / 0036 与源码；
  未读 `reference/`；未改任何源码 / 契约 / 冻结 ABI。

## 验收方法（裁定后）

1. **对外零破坏**：`git status --porcelain src/agc_msgs` 为空；`/agc/task/capture_hit` 仍**恰为** `CAPTURE_NET`；
   唯一 `/agc/ground/cmd_vel` 命令者不变（回归 [ADR-0040](./0040-full-chain-coverage-matrix-and-same-run-causal-record.md) (e)）。
2. **内部事件语义可复核**：shot 生命周期事件（射击请求 / 发射 / 到达 / 命中 / 失手 / 取消）可关联 episode / 时刻 / `target_id`；
3. **失手与取消**：目标在 `t_flight` 后不在可及集 → **不**判命中；发射前取消 → **无**终态副作用；
4. **回放一致**：以同一 run 记录回放，任务层对外可观测结果与在线一致。
