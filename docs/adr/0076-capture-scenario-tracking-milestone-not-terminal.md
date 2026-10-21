# capture 类场景中跟踪里程碑**不再等价终态**（捕获交火窗口不被提前关闭）

## Status

accepted

## Date

2026-10-20

## 背景

W7-b 步 3 的 P1 配对 run（`w7-rl-b/20261020-w7-rl-b-150408-8486`，40 episodes）暴露出**终态与场景期望的系统性不一致**：

- 场景真源 [`scenarios.yaml`](../../src/agc_task/config/scenarios.yaml) 的 `s08_static_none_nominal_capture` 登记 `expected_terminal: INTERCEPT_CONFIRMED` / `expected_reason: CAPTURE_NET`，且已按 [ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(b) 将该场景的 `capture_adjudicator.interception_authorized` 覆盖为 `true`（授权射击 = 任务决策）；
- 实测协同臂 **20/20** 均为 `TRACKING_SUCCESS(tracking_milestone)`、`expected_match=false` / `pass=false`；非协同臂 18×`TRACKING_SUCCESS` + 1×`INTERCEPT_CONFIRMED` + 1×`MISSION_FAILED(TIMEOUT)`。

**根因**（代码级）：

1. `task_manager` 在 `GROUND_APPROACH` 中一旦「连续驻留 ≥ `t_dwell_s`」就 `enterTerminal(TRACKING_SUCCESS, "tracking_milestone")` 并**锁存**（[`task_manager.cpp`](../../src/agc_task/src/task_manager.cpp) 的 milestone 分支）——实测约 16 s 触发；
2. `capture_adjudicator` 的**开火窗口**严格绑定 `task_state_ == GROUND_APPROACH`（[`capture_adjudicator.cpp`](../../src/agc_task/src/capture_adjudicator.cpp) 的 `window_open`）；
3. ⇒ 里程碑先锁存终态 → 状态离开 `GROUND_APPROACH` → **交火窗口关闭** → `CAPTURE_NET` 永不发生。

即：捕获场景的「跟踪里程碑」与「捕获结局」之间存在**时序竞态**，而里程碑总是赢。

## 决策

### (a) 新增**单一注入**参数：`task_manager.tracking_milestone_is_terminal`

- **缺省 `true`** → 既有行为**完全不变**（跟踪类 / 其它场景零影响）；
- `false` → 里程碑仍置 `tracking_milestone_met_ = true`（随 `/agc/task/state` 正常发布，**指标口径不变**），但**不**调用 `enterTerminal`，FSM 留在 `GROUND_APPROACH`，结局交由：
  - `capture_adjudicator` → `INTERCEPT_CONFIRMED(CAPTURE_NET)`；或
  - `t_max_s` 超时 → `MISSION_FAILED(TIMEOUT)`（契约 [#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) C4：超时一律失败）。
- 真源：基参 [`w6_base.yaml`](../../src/agc_task/config/w6_base.yaml)（`true`）；**仅** capture 场景在场景层覆盖。

### (b) 作用范围：**只改 capture 类场景中「已授权射击」的那一个**

- 已覆盖：`s08_static_none_nominal_capture` → `tracking_milestone_is_terminal: false`（本段唯一**可运行**的 capture 场景）。
- **未**覆盖：`s12_linear_dynamic_nominal_capture` —— 其 `support_status: unsupported:obstacle` 且**未**覆盖 `interception_authorized`（缺省 `false`）；若同时关闭里程碑终态，则注定超时，故**不动**。该不一致**显式登记**为待办（见后果）。
- 跟踪类 / 消融 / 安全场景（`s01`–`s07`、`s09`–`s11`）**不受影响**。

### (c) 契约边界（**未**改动）

- 不新增消息字段、不改冻结 9 消息 ABI；`AgcTaskState.tracking_milestone_met` 语义与发布时机**不变**；
- 不改 `t_dwell_s` / `d_track_m` / `t_max_s` / 网弹道 / 火控门限 → **任务难度冻结**（[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(b) 仍成立）；
- 不改场景的 `expected_terminal`（**不**用"改期望"来迁就实现）；
- 唯一执行口 / 唯一命令者（[#execution-port](../ROS2迁移交接文档.md#execution-port)）不变——本改动只影响**终态判定的时序**。

## 备选

1. **把 s08 的 `expected_terminal` 改为 `TRACKING_SUCCESS`** —— 否决：等于把"捕获场景"降级成"跟踪场景"，抹掉场景的验收语义，且反向迁就实现。
2. **让火控提前开火 / 放宽交战包络** —— 否决：改变任务难度，与 [ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(b)「替换后冻结难度再训练」冲突。
3. **在 `capture_adjudicator` 内把窗口放宽到「曾进入 GROUND_APPROACH」** —— 否决：会让终态与"当前任务阶段"解耦（车已停下仍可开火），且 `task_manager` 一旦锁存 `TRACKING_SUCCESS`，捕获结局将无法改写；问题源头在里程碑的**终态语义**，应在 `task_manager` 修正。
4. **无条件取消里程碑终态（不加参数）** —— 否决：会改变所有跟踪类场景的既有行为，破坏 W6 回归与历史证据可比性。

## 后果

- **正面**：`s08` 恢复其登记的语义 —— 单跑验证（`w7-rl-b/20261020-w7-rl-b-220524-1123`，`backend:=full` / seed 0 / `p1_coop`）：**`INTERCEPT_CONFIRMED`(`CAPTURE_NET`)、`expected_match=true`、`pass=true`**，日志有 `SHOT_FIRED shot=2-1` / `shot=2-2`；里程碑仍在（`tracking_milestone_met`），`vis_rate=0.991`、`fco_rate=0.301`。
- **代价 / 限制（必须记住）**：**episode 时长变了**（~16 s → ~28 s 或至 `t_max`）→ `window_fixed_v1`（自首个 active 拍起固定 60 s，[ADR-0074](0074-visibility-metric-fixed-observation-window.md)）窗口内的行为随之改变 ⇒ **本改动前后的 P1 数值不可混用**；[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(h) 的判定产生于**旧终态语义**下，须在**新语义**下重跑 N=20 并重判（旧判定**保留原样**、标注"语义已被 ADR-0076 取代"）。
- **历史证据影响**：此前 `s08` 得到 `TRACKING_SUCCESS` 的 run（含 W6 回归与 P1 首轮）属于**偏离登记期望**的结果，本改动后应落 `INTERCEPT_CONFIRMED` 或 `TIMEOUT`；这些旧 run 的 `expected_match=false` **不**再代表实现缺陷。
- **回归防线（2026-10-20 落地）**：① 启动前断言 —— [`w6_composition.assertions`](../../src/agc_task/launch/w6_composition.py) 新增 `capture_authorized_requires_nonterminal_milestone`（授权射击 ⇒ 必须关闭里程碑终态），配置自相矛盾时**拒绝运行**；② 静态测试 —— [`test_scene_config_invariants.py`](../../src/agc_task/test/test_scene_config_invariants.py) 全场景扫描（已接入 `colcon test`，红-绿验证：改坏 s08 即失败）。
- **未决 / 后续**：① **已完成（2026-10-21）**：新语义下 P1 N=20 重跑并按预注册重判，修订见 [ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(h) 修订子节（`run_id w7-rl-b/20261021-w7-rl-b-104120-1456`）；② `s12` 的 `interception_authorized` 与里程碑语义一致性（待其 `unsupported:obstacle` 解除时一并处理）；③ W7-b 出口证据已按新语义刷新（见 ADR-0071 修订子节）。

## 参考

- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C4 超时一律失败 / C8 不降级）、[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)、[#params](../ROS2迁移交接文档.md#params)（单一注入）、[#execution-port](../ROS2迁移交接文档.md#execution-port)、[#messages](../ROS2迁移交接文档.md#messages)（冻结 9 消息 ABI **未改**）
- 前序 ADR：[ADR-0014](0014-research-mainline-mobile-interception-system.md)、[ADR-0024](0024-capture-adjudication-fire-control-window.md)、[ADR-0040](0040-full-chain-coverage-matrix-and-same-run-causal-record.md)、[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)、[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)、[ADR-0074](0074-visibility-metric-fixed-observation-window.md)、[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)
- 规格：[`.trae/specs/w7-rl-b/`](../../.trae/specs/w7-rl-b/spec.md)
