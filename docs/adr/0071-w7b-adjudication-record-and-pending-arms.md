# W7-b 裁定记录：火控占位升级、射击授权为任务决策、臂可运行性证据与待建臂

## Status

accepted

## Date

2026-10-19

## 背景

[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md) 已裁定 W7-b 的科研设计（命题 / 变量 / 对照 / 前置能力 / 判定规则）。本 ADR 记录 **W7-b 执行过程中产生的裁定与证据**：入口前置能力（E1–E5）落地、火控占位升级带来的**行为变更**及其处置、臂与消融的**可运行性证据**，以及**尚未承载**的臂。

依据：契约 [#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)、[#execution-port](../ROS2迁移交接文档.md#execution-port)、[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)；评审 [F-04](../reviews/agc-review-2026-10-14/30-未来.md#f-04)、[F-06](../reviews/agc-review-2026-10-14/30-未来.md#f-06)、[F-10](../reviews/agc-review-2026-10-14/30-未来.md#f-10)、[F-11](../reviews/agc-review-2026-10-14/30-未来.md#f-11)、[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)、[F-16](../reviews/agc-review-2026-10-14/30-未来.md#f-16)。

## 决策

### (a) 入口前置能力 E1–E5 已落地（每条有可复现 run 证据）

| # | 能力 | 证据（`campaign/run_id`） | 判据 |
|---|---|---|---|
| **E1** | raw / effective **同拍**归因（F-06） | `w7-rl-b/20261019-w7-rl-b-170941-5020`（GUI·shield=true：`intervened=276/439`、干预时长 13.09 s）+ `-171108-3459`（shield=false：`intervened=0`、Δ=0） | 同拍配对 + 干预标注；两臂齐备 |
| **E2** | **最小动态火控**（F-11） | `w7-rl-b/20261019-w7-rl-b-180124-5536`（headless）+ `-180212-4519`（GUI）：`SHOT_FIRED 2-1` → 0.70/0.75 s 后 `CAPTURE_HIT`（最近逼近 0.487/0.442 m） | 完整射击闭环 + 逐帧与 GZ 真值位姿比对 |
| **E3** | 可见性度量按 **active 窗口**（F-10） | `w7-rl-b/20261019-w7-rl-b-181708-7472`（s02/s06）：分母 = 窗口拍数（378/395） | 分母切分正确、可解释 |
| **E4** | `obs_version` **v2**（F-04） | `w7-rl-b/e4-obs-versioning/metrics/e4.json`：v1 **逐位**可回放；v2 = v1+4 维；元数据**可区分** | 旧权重不静默失效 |
| **E5** | **离线数据集**（F-16） | `w7-rl-b/20261019-w7-rl-b-182643-3111`：条目带 `run_id`/sha256，**可重建**（mismatch=0） | 可重建 + 带身份 |

### (b) 火控占位升级**改变**了捕获时点 → 以「射击授权＝任务决策」冻结任务难度（F-11）

**事实**：闭式包络占位（W6）与最小动态火控（W7-b）在「跟踪类」场景（如 `s02`）会给出**不同结局**——网的**飞行**使捕获可在车体尚未抵达早前判据距离时发生，故 `s02` 曾由 `TRACKING_SUCCESS` 变为 `INTERCEPT_CONFIRMED`。

**裁定**：捕获裁决的**射击时刻**属**任务决策**，**非自动**——新增参数 `interception_authorized`（缺省 `false`，由场景按 `outcome` 轴覆盖：`capture` 场景授权，跟踪类不授权）。故：
- 跟踪类场景结局**不变**（`s02` 复归 `TRACKING_SUCCESS`，W6 回归保持）；
- `capture` 场景（`s08`）在授权下按**动态火控**判定（`INTERCEPT_CONFIRMED` / `CAPTURE_NET`）。

这是 [F-11](../reviews/agc-review-2026-10-14/30-未来.md#f-11)「替换后**冻结任务难度**再训练」在本段的落地方式：**不改**网/高度/适用参数（逐字沿用 W6 值），只把「何时开火」显式化为任务决策。

### (c) 臂与消融：**可运行性证据**（R1–R3 的可行子集）

| 臂 | 组 | 命令源 | shield | 证据（同 run） | 终态 |
|---|---|---|---|---|---|
| `baseline_anchor` | R3 | `baseline`（Navfn + 自研 LQR） | true | `w7-rl-b/20261019-w7-rl-b-182921-9376` | `TRACKING_SUCCESS` |
| `p2_c2_low_level` | P2 臂 A | `rl`（直接出 `cmd_vel`） | true | 同上 | `TRACKING_SUCCESS` |
| `p2_rl_high` | P2 臂 B | `rl_high`（高层拦截点 + 成熟低层 Navfn+LQR） | true | 同上 | `TRACKING_SUCCESS` |
| `ablation_with_shield` | 消融·安全干预 | `rl` | true | 同上 | `TRACKING_SUCCESS` |
| `ablation_without_shield` | 消融·安全干预 | `rl` | **false**（纯仿真，不用于部署） | 同上 | `TRACKING_SUCCESS`（`intervened=0`） |

**纪律**：各臂**同**场景 / 同 seed / 同安全层 / 同终态语义，**仅**自变量（命令源 / shield）不同（[F-14](../reviews/agc-review-2026-10-14/30-未来.md#f-14)）；**无第二命令者**（契约 [#execution-port](../ROS2迁移交接文档.md#execution-port)）。

**诚实标注**：`p2_rl_high` 的**高层策略为规则实现**（目标线性外推正提前量），**非**已训练 RL；该臂考察**职责分层**，RL 训练属 [F-02](../reviews/agc-review-2026-10-14/30-未来.md#f-02) 后续。

### (d) 尚未承载的臂（**显式登记**，不静默假设）

| 待建臂 | 理由 |
|---|---|
| **P1 协同 vs 非协同**（R1） | 「协同」由**空中链路**承载；轻量栈**无空中域** → 需**整链双 PX4**（`backend:=full`）。本段未承载。 |
| 消融·**协同观测 / 目标预测** | 同上（依赖空中协同观测通道）。 |

**更新（2026-10-19）**：整链承载已**落地**（控制权交接 + 观测链修复，见 (g)），故 P1 两臂**具备可运行条件**；但**配对 run 与判定仍未执行**（预注册阈值下）。

**如实标注（P1 观测自变量 α = `observe_yaw_mode`）**：`observe_yaw_mode=target` 臂的**指向信息**用**真值方位** `/agc/gt/intruder_uav` → 该臂 = **显式 oracle 指向臂**（[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md) 允许「显式 oracle 臂」须标注）；**测量**仍走真实感知通道（噪声 / 丢包 / FOV 照常生效，不因指向而放宽）。

### (e) 确证实验**预注册**（R4，先于确证 run）

判定规则 / 样本量 / 阈值（依效应大小）写定于 [`src/agc_rl/config/w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)（单一来源）；**确证对比 run 尚未执行**。本 ADR 中的臂证据（(c)）是**可运行性**验证，**不**构成判定结论。

### (f) 裁定结论（**部分**）

- **P2**：臂 A / 臂 B 均**可运行**；v2 批次（2026-10-22）因**臂信息不等价**（A 为 oracle 固定目标）+ **primary DV 终态后污染**被判为 **confounded**，**不作为 P2 判定证据**（[ADR-0083](0083-w7b-p2-arm-information-equivalence-and-active-window-fco.md)）；v3（同源感知目标 + active-window FCO）已执行 N=20 并判定：**证伪（primary 方向为负且显著）→ 保留 c2**（见 (j)）；主控仍按 [ADR-0004](0004-rl-primary-controller-nav2-safety-layer.md) 保持 c2（未改判）。
- **P1**：整链**已具备承载**（(g)）；配对 run 与判定见 (h)（fixed-window，历史）、(k)（s08 active-window，历史）与 **(l)**（s15 机制激活场景，**现行结论：未达预注册 MDE → 证伪；效应方向正确且显著**）。
- **火控**：占位升级为最小动态火控，且**任务难度已冻结**（(b)）。

### (g) 整链入口落地与观测链根因（W7-b 步 1，2026-10-19）

- **甲-1 交接**：由 `task_manager`（唯一任务协调者）在 `reset_phase` + 门禁后**等「起飞完成（悬停）」**再调 `/agc/air/release_control`（此前**全仓无节点调用**该服务，见 [ADR-0040](0040-full-chain-coverage-matrix-and-same-run-causal-record.md) 登记缺口）；参数单一注入（契约 [#params](../ROS2迁移交接文档.md#params)）。
- **观测链根因**：`px4_adapter` 姿态四元数字段序解析错（PX4 为 **[w,x,y,z]**，原按 [x,y,z,w]）→ `/agc/air/odom` 姿态错 ~58° → `air_perception` **FOV 门**丢目标（`visibility_rate≈1.4%`）→ `TIMEOUT`。修复与影响面见 [ADR-0072](0072-px4-attitude-quaternion-field-order-and-full-chain-observation-fix.md)。
- **复验（同 run）**：`w7-rl-b/20261019-w7-rl-b-224947-6509`（`backend:=full` / `s08` / seed 0）：`相位复位→悬停→release_control 移交→tracker 接管→start OK`；终态 **`TRACKING_SUCCESS`**、`visibility_rate=0.982`（summary `w6-task-eval/20261019-s08_static_none_nominal_capture-0-225005-4480`）。
- **未决**：`s08` 期望 `INTERCEPT_CONFIRMED` 而实测 `TRACKING_SUCCESS`（捕获裁决 / `tracking_milestone` 先后与 `interception_authorized` 待另行裁定，**不**静默改场景语义）。

### (h) P1 确证判定**已执行**（2026-10-20；预注册口径 `version: 1`）

> **⚠ 语义已被取代（同日）**：本节判定产生于**旧终态语义**（capture 场景中跟踪里程碑提前锁存 `TRACKING_SUCCESS`）。该语义已由 [ADR-0076](0076-capture-scenario-tracking-milestone-not-terminal.md) 修正（`tracking_milestone_is_terminal: false`，仅 capture 场景）→ episode 时长与固定窗口内的行为随之改变，故本节的**数值不作为最终结论**；原记录**保留不动**，待新语义下重跑 N=20 后在本节追加"修订"子节。

- **承载 run**：`w7-rl-b/20261020-w7-rl-b-150408-8486`（`backend:=full` / `s08` / seeds 0–19 / CRN 同 seed 配对 / 40 episodes / 墙钟 1.32 h），判定脚本 `scripts/w7b_p1_adjudicate.py`。
- **先于本 run 落地的前置**（各有独立 ADR）：臂公平性 [ADR-0073](0073-p1-arm-fairness-noncoop-fixed-trajectory.md)、DV 口径（固定观测窗口）[ADR-0074](0074-visibility-metric-fixed-observation-window.md)、整链 infra 处置（运行前清场 + infra 重试）[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)。
- **结果**：

| DV | 角色 | n 配对 | 剔除 | 均值配对差 | 95% CI | 配对 t p | Wilcoxon p | 判定 |
|---|---|---|---|---|---|---|---|---|
| `fire_control_opportunity_rate` | primary | 20 | 0 | **+0.2885** | [+0.2023, +0.3747] | 1e-06 | 0.0（含并列 → 正态近似） | **成立** |
| `min_clearance` | secondary | 20 | 0 | +0.0453 | [−0.3261, +0.4168] | 0.801 | 0.729 | **不显著（如实登记）** |

- **各臂画像**：协同臂 `visibility_rate` 均值 **0.956**、`fco_rate` 均值 **0.401**（20/20 `TRACKING_SUCCESS`）；非协同臂 `visibility_rate` 均值 **0.365**、`fco_rate` 均值 **0.112**（18×`TRACKING_SUCCESS` + 1×`INTERCEPT_CONFIRMED` + 1×`MISSION_FAILED(TIMEOUT)`）。
- **口径纪律**：primary 的**失败 / infra / 缺值一律按 0 计入分母**（`primary_honored=true`、剔除 **0** 对、零填 2 个），阈值 / 样本量 / 自变量**未改**；判定脚本旧版"缺值整对剔除"与预注册 `keep_all_failures_and_infra` 不一致的问题已随 [ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md) 修正并复验。
- **infra 环境（如实登记）**：终态 infra **0/40**；2 个 episode 各触发 1 次 infra 重试（重试后均达任务终态）。该分布为**机器特性**，不得读作臂效应。
- **结论与适用边界**：**P1（协同观测 > 非协同观测）在 primary DV 上成立**。但协同臂的**指向信息**取**真值方位** `/agc/gt/intruder_uav` → 该臂为 **显式 oracle 指向臂**（(d) 已标注），**测量**仍走真实感知通道（噪声 / 丢包 / FOV 照常生效，不因指向而放宽）⇒ 本结论的适用范围限"**oracle 指向条件下**"，**不**等同于"已训好的协同策略收益"。secondary DV 不显著 → 本段**不**主张协同在**安全裕度**上的收益。
- **未决**：`s08` 期望 `INTERCEPT_CONFIRMED` 而协同臂 **20/20** 实测 `TRACKING_SUCCESS` → 捕获裁决 / `tracking_milestone` 先后与 `interception_authorized`（(g) 已登记）仍待**另行裁定**；本判定**不**触碰场景语义。

#### 修订一（2026-10-21，新终态语义；含 ADR-0078 前的缺陷批次）

- **承载 run**：`w7-rl-b/20261021-w7-rl-b-104120-1456`（`backend:=full` / `s08` / seeds 0–19 / CRN 同 seed 配对 / 40 episodes / 墙钟 1.92 h），判定脚本 [`scripts/w7b_p1_adjudicate.py`](../../scripts/w7b_p1_adjudicate.py)，判定产物 `metrics/p1_adjudication.json`（同 run 目录）。
- **语义前提**：[ADR-0076](0076-capture-scenario-tracking-milestone-not-terminal.md)（`tracking_milestone_is_terminal: false`，仅 `s08`）；运行前按源码增量重建 `agc_task`（含 2026-10-20 的 `air_hover_timeout_s` 缺省值修复），使二进制与源码一致。
- **结果**：

| DV | 角色 | n 配对 | 剔除 | 均值配对差 | 95% CI | 配对 t p | Wilcoxon p | 判定 |
|---|---|---|---|---|---|---|---|---|
| `fire_control_opportunity_rate` | primary | 20 | 0 | **+0.2022** | [+0.0513, +0.3531] | 0.0113 | 0.0168 | **成立** |
| `min_clearance` | secondary | 20 | 0 | −0.0959 | [−0.6313, +0.4395] | 0.712 | 0.571 | **不显著（如实登记）** |

- **各臂画像**：协同臂 `visibility_rate` 均值 **0.859**、`fco_rate` 均值 **0.440**，终态 **12×`INTERCEPT_CONFIRMED` + 8×`MISSION_FAILED(TIMEOUT)`**；非协同臂 `visibility_rate` 均值 **0.400**、`fco_rate` 均值 **0.177**，终态 **16×`INTERCEPT_CONFIRMED` + 4×`MISSION_FAILED(TIMEOUT)`**。
- **口径纪律**：primary 的**失败 / infra / 缺值一律按 0 计入分母**（`primary_honored=true`、剔除 **0** 对、零填 **12** 个 = 两臂失败回合总数 12）；阈值 / 样本量 / 自变量**未改**。
- **infra 环境（如实登记）**：终态 infra **0/40**；**7** 个 `(arm, seed)` 触发重试（协同 2 / 非协同 5，单条最多 3 次），重试后除 `p1_coop/seed18` 落 `TIMEOUT`（mission，按契约 C4 不再重试）外均达任务终态。该分布为**机器特性**，不得读作臂效应。
- **结论与适用边界**：**P1（协同观测 > 非协同观测）在 primary DV 上仍成立**。须同时如实登记：协同臂**任务成功率更低**（12/20 vs 16/20）——primary 的优势来自**成功回合内更高的有效火控机会率**，而非更高的任务成功率；二者不可混读。协同臂仍为**显式 oracle 指向臂**（(d)），结论限"**oracle 指向条件下**"；secondary 不显著 → **不**主张协同在**安全裕度**上的收益。
- **未决**：`s12` 的 `interception_authorized` 与里程碑语义一致性（待其 `unsupported:obstacle` 解除时一并处理，[ADR-0076](0076-capture-scenario-tracking-milestone-not-terminal.md) 后果节）；W7-b 其余出口（P2 判定、协同观测 / 目标预测消融）与 W7-c 多 worker。
- **后续修订（2026-10-21）**：本 run 的 12 个超时经日志诊断，其中 4 例为**启动双复位竞态导致 41 次 `CAPTURE_NET` 被丢弃**（[ADR-0078](0078-capture-event-episode-identity-single-source.md)）。该缺陷主要落在协同臂（3 例 vs 非协同 1 例）⇒ 本子节的 **+0.2022 仍成立且偏保守**，但"协同臂成功率 12/20 < 16/20"**不可读作臂效应**；修复后须重跑 N=20 刷新本节。

#### 修订二（2026-10-21，修复 ADR-0078 后；含 B2 火控缺陷批次）

- **承载 run**：`w7-rl-b/20261021-w7-rl-b-133724-7514`（`backend:=full` / `s08` / seeds 0–19 / 40 episodes / 墙钟 1.61 h）；判定产物同 run `metrics/p1_adjudication.json`。
- **缺陷清除证据**：本 run 全日志 **0 条 `capture_event 拒收`**；8 个超时回合的 `CAPTURE_NET` 次数均为 **0**（不存在"已命中却被丢弃"），即 [ADR-0078](0078-capture-event-episode-identity-single-source.md) 的 A 类机理已消除；剩余超时均属观测 / 火控解算类（B 类，另行诊断）。
- **结果**：

| DV | 角色 | n 配对 | 剔除 | 均值配对差 | 95% CI | 配对 t p | Wilcoxon p | 判定 |
|---|---|---|---|---|---|---|---|---|
| `fire_control_opportunity_rate` | primary | 20 | 0 | **+0.3526** | [+0.2052, +0.5000] | 7.8e-05 | 7.2e-04 | **成立** |
| `min_clearance` | secondary | 20 | 0 | −0.1064 | [−0.2652, +0.0524] | 0.177 | 0.349 | **不显著（如实登记）** |

- **各臂画像**：协同臂 `visibility_rate` 均值 **0.932**、`fco_rate` 均值 **0.462**，终态 **17×`INTERCEPT_CONFIRMED` + 3×`MISSION_FAILED(TIMEOUT)`**；非协同臂 **0.375 / 0.107**，终态 **15×`INTERCEPT_CONFIRMED` + 5×`MISSION_FAILED(TIMEOUT)`**。
- **口径与 infra**：primary 的失败 / 缺值按 0 计入分母（零填 **8**、剔除 **0**）；终态 infra **0/40**；**4** 个 `(arm, seed)` 触发 1 次 infra 重试（重试后 3 个成功、1 个仍 `TIMEOUT`），属机器特性、不计作臂效应。
- **结论**：**P1（协同观测 > 非协同观测）在 primary DV 上成立**，且效应量（+0.353）大于缺陷批次的 +0.202。协同臂仍为**显式 oracle 指向臂**（(d)），结论限"**oracle 指向条件下**"；secondary 不显著 → **不**主张协同在**安全裕度**上的收益。成功率 17/20 vs 15/20 仅作画像，**不**作为 P1 的判定依据。
- **取代关系**：本节取代"修订一"的 `20261021-w7-rl-b-104120-1456` 数值（该批次受 ADR-0078 缺陷影响，仅作历史留痕）。

#### 最终重跑（2026-10-21，修复 ADR-0079 B2 火控后 · **本 ADR 的现行 P1 结论**）

- **承载 run**：`w7-rl-b/20261021-w7-rl-b-175313-3256`（`backend:=full` / `s08` / seeds 0–19 / 40 episodes / 墙钟 1.58 h）；判定产物同 run `metrics/p1_adjudication.json`。
- **前置修复**：[ADR-0078](0078-capture-event-episode-identity-single-source.md)（episode 身份）+ [ADR-0079](0079-b2-fire-control-target-motion-estimation.md)（目标运动估计 / 发射解算门）。本 run **0 条事件拒收**。
- **结果**：

| DV | 角色 | n 配对 | 剔除 | 均值配对差 | 95% CI | 配对 t p | Wilcoxon p | 判定 |
|---|---|---|---|---|---|---|---|---|
| `fire_control_opportunity_rate` | primary | 20 | 0 | **+0.2988** | [+0.1490, +0.4485] | 5.1e-04 | 1.4e-03 | **成立** |
| `min_clearance` | secondary | 20 | 0 | −0.1704 | [−0.6002, +0.2595] | 0.417 | 0.021 | **不显著（按预注册 t 检验）** |

- **各臂画像**：协同臂 **20/20 `INTERCEPT_CONFIRMED`**；非协同臂 **18/20 成功 + 2×`MISSION_FAILED(TIMEOUT)`**（`seed6`/`seed18`，各仅 1 次开火机会、`fco≈0.001/0.002` → 属固定航迹的包络行为，非链路缺陷）。
- **口径与 infra**：primary 失败 / 缺值按 0 计入分母（零填 **2**、剔除 **0**）；终态 infra **0/40**；**7** 个 `(arm, seed)` 触发 1 次 infra 重试，均恢复；属机器特性、不计作臂效应。
- **结论**：**P1（协同观测 > 非协同观测）在 primary DV 上成立**。协同臂仍为**显式 oracle 指向臂**（(d)），结论限"**oracle 指向条件下**"；secondary 不显著 → **不**主张协同在**安全裕度**上的收益。成功率 20/20 vs 18/20 仅作画像，**不**作为 P1 的判定依据。
- **环境边界（如实登记，不修改）**：本判定在 `range_max=12 m`、空中站距带 `[8,15] m` 下取得；超出量程的时段对两臂**对称**且已由 `visibility_rate` 计入。该几何盲区不构成臂间偏差，故保留现配置；批次产物 `metrics/perception_health.json` 自动报告 `rmax/no_pose/fov` 占比（[ADR-0080](0080-air-lifecycle-startup-robustness.md)）。
- **取代关系**：本节取代"修订一"与"修订二"的数值（前两批次分别受 ADR-0078 / B2 火控缺陷影响，仅作历史留痕）。

### (i) P2 v2 批次**已执行但判为 confounded**（2026-10-22；预注册 v2；**不作判定**）

> **2026-10-22 审计更新**：本批次因臂信息不等价（A 只订 `/agc/ground/goal`；s08 固定目标 = 目标真值；B 订 `/agc/targets`）与 primary DV 终态后污染（约 97%/100% 机会拍在终态之后）判为 **confounded**；以下数值仅作历史 / 探索证据，**不**构成 P2 成立或证伪。现行 P2 = v3 已执行并判定（见 (j)；[ADR-0083](0083-w7b-p2-arm-information-equivalence-and-active-window-fco.md)）。

- **承载 run**：`w7-rl-b/20261022-w7-rl-b-112833-1155`（`backend:=full` / `s08_static_none_nominal_capture` / seeds 0–19 / CRN 同 seed 配对 / 40 episodes / 墙钟 2.37 h）；判定产物同 run `metrics/p2_adjudication.json`；脚本 [`scripts/w7b_p2_adjudicate.py`](../../scripts/w7b_p2_adjudicate.py)。
- **臂与条件**：臂 A `p2_c2_low_level`（`command_source=rl`，直接出 `cmd_vel`）；臂 B `p2_rl_high`（**规则高层**拦截点 + 成熟低层 Navfn+LQR，[ADR-0082](0082-w7b-p2-arm-b-rule-high-level-first.md)）；固定协同、同安全层 / 唯一执行口 / 终态；两臂训练 / 调参预算均为 0，推理算力单列、不声称等算力。
- **结果**：

| DV | 角色 | n 配对 | 剔除 | 均值配对差 (B−A) | 95% CI | 配对 t p | Wilcoxon p | 判定 |
|---|---|---|---|---|---|---|---|---|
| `fire_control_opportunity_rate` | primary | 20 | 0 | **−0.0697** | [−0.2528, +0.1135] | 0.436 | 0.658 | **未达 MDE=0.20 / 不显著 → P2 证伪** |
| `min_clearance` | secondary | 20 | 0 | +0.2182 | [−0.5590, +0.9954] | 0.564 | 0.294 | 不显著（仅报告） |

- **审计敏感性（2026-10-22）**：active-window FCO（`episode_window_metrics`，GROUND_APPROACH→终态）A 0.109 vs B 0.042，Δ = −0.067，n=19（剔除 infra seed 8），p = 0.029；方向仍为 B 不优，但 v2 primary 的约 97%/100% 机会拍来自终态后，故 v2 数值不可作为判定。**A 另有 oracle 目标信息优势**（`goal_follow_target=false` + goal=目标真值），v2 不可读作职责分层结论。
- **各臂画像**：臂 A **16/20 `INTERCEPT_CONFIRMED` + 4×`MISSION_FAILED(TIMEOUT)`**（seeds 3/8/11/13），`fco` 均值 0.376（含 4 个失败零填；成功回合 0.470）、`visibility` 0.871、`min_clearance` 4.868；臂 B **19/20 `INTERCEPT_CONFIRMED` + 1×`ACTIVE`**（seed 8，infra 重试 3 次耗尽），`fco` 均值 0.306（含 infra 零填；成功回合 0.322）、`visibility` 0.924、`min_clearance` 5.086。任务成功率 B 更高仅作画像，**不**作为 P2 判定依据；primary 为 FCO rate（与 P1 同口径）。
- **口径纪律**：primary 的失败 / infra / 缺值按 0 计入分母（`primary_honored=true`、剔除 **0** 对、零填 **5** = 臂 A 4 个失败 + 臂 B 1 个 infra）；终态 infra **1/40**（B seed 8），**10** 个 `(arm, seed)` 触发重试（A 5 / B 5，单条最多 3 次），单列、不计作臂效应（[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)）。
- **结论（已撤销）**：v2 的"证伪"**不成立**——本批次未隔离 IV，且 primary 被终态后停泊污染（[ADR-0083](0083-w7b-p2-arm-information-equivalence-and-active-window-fco.md)）。**P2 当前状态 = v3 已执行并判定（见 (j)）**；主控仍按 [ADR-0004](0004-rl-primary-controller-nav2-safety-layer.md) 保持 c2。臂 B 为**规则高层**，本记录**不**回答"训练 RL 是否优于 c2"；训练 RL 仍按 [ADR-0082](0082-w7b-p2-arm-b-rule-high-level-first.md)(d) 后置。
- **解释边界**：任务成功率 B 19/20 > A 16/20，但 primary FCO rate 不显著且均值差为负；本判定**不**主张 B 更差，只主张"**B 的优越性未成立**"。若后续以任务成功率为决策终点，须**新预注册**（不得事后改 primary）。
- **未决 / 后续**：消融（协同观测 / 目标预测）与 W7-c 多 worker 仍未执行。

### (j) P2 v3 判定**已执行**（2026-10-22；预注册 v3；**现行 P2 结论**）

- **承载 run**：`w7-rl-b/20261022-w7-rl-b-v3-n20r`（`backend:=full` / `s08_static_none_nominal_capture` / seeds 0–19 / CRN 同 seed 配对 / 40 episodes / 墙钟 2.39 h）；判定产物同 run `metrics/p2_adjudication.json`。
- **设计修正**（[ADR-0083](0083-w7b-p2-arm-information-equivalence-and-active-window-fco.md)(b)(c)(g)）：两臂 `goal_follow_target=true`（同源感知目标）；primary 改 **active-window FCO**（`GROUND_APPROACH → 终态`，不含终态后停泊）；臂 B 高层加目标低通 + 前向限幅；预注册 v3（N=20 / α=0.05 / MDE=0.10）。
- **结果**：

| DV | 角色 | n 配对 | 剔除 | 均值配对差 (B−A) | 95% CI | 配对 t p | Wilcoxon p | 判定 |
|---|---|---|---|---|---|---|---|---|
| `fire_control_opportunity_rate_active` | primary | 20 | 0 | **−0.0561** | [−0.0971, −0.0151] | **0.010** | 0.011 | **未达 MDE=0.10（方向为负且显著）→ P2 证伪、保留 c2** |
| `min_clearance_m` | secondary | 20 | 0 | −0.0879 | [−0.5920, +0.4161] | 0.719 | 0.985 | 不显著（仅报告） |

- **各臂画像**：臂 A **19/20 `INTERCEPT_CONFIRMED`** + 1×`MISSION_FAILED(TIMEOUT)`（seed1）；臂 B **17/20** + 3×`MISSION_FAILED(TIMEOUT)`（seeds 1/5/15；seed5 `min_clearance=0.685 m`，如实登记）。终态 infra **0/40**；两臂 `goal_follow_target=true` 均生效；`layer_integrity` 全 true。
- **口径纪律**：primary 的失败 / infra / 缺值按 0 计入分母（`primary_honored=true`、剔除 **0** 对、零填 **4** = A seed1 + B seed1/5/15）；n=20 与预注册一致；两臂均触发过 infra 重试（A 6 / B 13 次），终态 infra 0。
- **描述性对照（legacy）**：`window_fixed_v1` 的 `fire_control_opportunity_rate` A 0.621 vs B 0.221，Δ = −0.400（p≈0）；该口径约 97%/100% 机会拍来自终态后停泊，**不参与 v3 判定**，仅说明 v2 的 DV 污染。
- **结论**：按 v3 预注册，**P2 未达优越性判据 → 证伪；保留 c2**（[ADR-0004](0004-rl-primary-controller-nav2-safety-layer.md) 不变）。本轮 primary **方向为负且显著**（B 的 active-window FCO 低于 A），但效应量 **0.056 < MDE 0.10**；不得以此表述"B 在任务成功率上更差"（A 19/20 vs B 17/20 仅作画像，且 seed1 为共因失败）。臂 B 为**规则高层**，本判定**不**回答"训练 RL 是否优于 c2"；训练 RL 仍按 [ADR-0082](0082-w7b-p2-arm-b-rule-high-level-first.md)(d) 后置。
- **解释边界**：两臂同源感知目标 / 同安全层 / 唯一执行口 / 终态；B 的 3 个失败与较低 active FCO 是该架构在本场景、本预算下的实测结果。若要改以"任务成功率 / 捕获时长"为决策终点，须**新预注册**，不得事后改 primary。
- **证据链**：v3 预注册在 run 前写定（本会话工作区；文件 mtime 早于 run；git 提交待固化）；v2 run `20261022-w7-rl-b-112833-1155` 仍为 confounded / invalidated（(i)）。

### (k) P1 v4 口径复核**已执行**（2026-10-23；预注册 v4；**现行 P1 结论**）

> **取代关系**：(h) 及其修订子节（fixed-window，+0.2988）保留为**历史记录**，不再作为 P1 的现行结论。现行结论以本节为准（[ADR-0084](0084-w7b-p1-active-window-dv-recheck.md)）。

- **背景**：[ADR-0083](0083-w7b-p2-arm-information-equivalence-and-active-window-fco.md) 登记 P1 与 P2 使用同一固定窗口的风险：P1 v1 的 fixed-window Δ=+0.299（p=0.0005），active-window Δ=+0.056（p=0.098），约 92%/86% 机会拍在终态后。P1 的"成立"因此不能作为 W8 冻结依据。
- **决策与预注册**：[ADR-0084](0084-w7b-p1-active-window-dv-recheck.md) 将 P1 v4 的 primary 改为 `fire_control_opportunity_rate_active`（`GROUND_APPROACH → 终态`）；N=20 / α=0.05 / MDE=0.10 / 臂定义**均不变**；fixed-window 降为 legacy/描述量。
- **承载 run**：`w7-rl-b/20261023-w7-rl-b-p1v4-n20`（`backend:=full` / `s08_static_none_nominal_capture` / seeds 0–19 / CRN 同 seed 配对 / 40 episodes / 墙钟 1.73 h）；判定产物同 run `metrics/p1_adjudication.json`。
- **结果**：

| DV | 角色 | n 配对 | 剔除 | 均值配对差（协同 − 非协同） | 95% CI | 配对 t p | Wilcoxon p | 判定 |
|---|---|---|---|---|---|---|---|---|
| `fire_control_opportunity_rate_active` | **primary** | 20 | 0 | **+0.0278** | [−0.0113, +0.0669] | **0.153** | 0.498 | **未达 MDE=0.10 且不显著 → 证伪** |
| `min_clearance_m` | secondary | 20 | 0 | +0.1911 | [−0.5000, +0.8822] | 0.570 | 0.729 | 不显著（仅报告） |
| `fire_control_opportunity_rate`（`window_fixed_v1`） | legacy | 20 | 0 | +0.2812 | [+0.1543, +0.4082] | 1.8e-04 | 5.9e-04 | 仅描述，不参与 v4 判定 |

- **口径纪律**：primary 的失败 / infra / 缺值按 0 计入分母（`primary_honored=true`、剔除 **0** 对、零填 **3** = 协同 seeds 4/5 + 非协同 seed 13）；N=20 与预注册一致；阈值 / 样本量 / 自变量**未改**。
- **infra（如实登记）**：终态 infra **0/40**；**9** 个 `(arm, seed)` 触发 infra 重试（协同 3 / 非协同 6，单条最多 3 次），均恢复；属机器特性，不计作臂效应（[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)）。
- **各臂画像**：协同 **18/20 `INTERCEPT_CONFIRMED`**（seeds 4/5 `MISSION_FAILED`）；非协同 **19/20**（seed 13 失败）。`visibility_rate` 均值协同 0.946 / 非协同 0.389（fixed-window 口径，含终态后停留），仅作描述。
- **结论（现行 P1 结论）**：在 **clean DV（active-window FCO）** 下，**协同观测相对非协同观测的收益未成立**。v1 的 fixed-window"成立"被确认为**终态后停泊污染的产物**：同一新 run 中 legacy 仍为 +0.2812（p=1.8e-04），而 active 仅 +0.0278（p=0.153）。
- **解释边界**：本结论**不**主张"协同策略无效"——自变量是**观测指向 + 跟踪参考**的组合，且协同臂为**显式 oracle 指向臂**（(d)）；测量通道仍为真实感知（噪声 / 丢包 / FOV 照常生效）。结论限"已验证通道族"，不涉及真实检测 / sim-to-real（[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)(i)）。
- **机制边界（场景不足以激活 α，2026-10-23 补记；详见 [ADR-0084](0084-w7b-p1-active-window-dv-recheck.md)(f)）**：本 run 的 active 窗口内，两臂**可见率 0.932 vs 0.869**（仅差 6 pt），而 FCO 门 = **包络内 ∧ 目标可见**、包络占用由地面路径决定——s08 的 `goal_follow_target=false` 使 goal = 目标真值、**地面路径与感知无关**（两臂同源），故 α 的因果链只剩"可见"一项；该项又因目标 **static + 无遮挡 + 标称 FOV（100°/70°）** 而接近饱和。⇒ v4 的"证伪"**限本场景**（静态 / 无障碍 / 标称感知 / 固定目标），**不得**读作"协同观测在一般情况下无收益"。机制激活需目标运动 / 遮挡 / 丢包 / FOV 收窄 / 任务目标跟随感知等条件——s08 全部缺席。
- **未决 / 后续**：若改以**任务成功率 / 捕获时长 / 首次火控机会时间**为决策终点，须**新预注册 v5** 并重新匹配样本量 / MDE，不得事后改 primary（[ADR-0084](0084-w7b-p1-active-window-dv-recheck.md)(c)）。

### (l) P1 v5 机制激活重测**已执行**（2026-10-23；预注册 v5；**现行 P1 结论**）

> **取代关系**：(h)（fixed-window，成立）与 (k)（s08 active-window，证伪）保留为**历史**；现行结论以本节为准。

- **背景与场景选择**：[ADR-0084](0084-w7b-p1-active-window-dv-recheck.md)(f) 指认 s08 场景不足以激活 α（FCO 门中的**包络占用**由固定目标决定、**可见**仅差 6 pt）；[ADR-0085](0085-w7b-p1-mechanism-activating-scenario-pilot.md) 用**先定规则**的两轮 pilot（候选：linear / linear+fov / static+fov / static+dropout）选中 **C = `s15_static_none_fov_capture`**（static + 窄视场 30°/20°）；linear 候选因 **`rmax`（>12 m 量程）+ C10 hold 自锁**退化，dropout 候选在 active 窗口不降可见率，二者均被排除。
- **承载 run**：`w7-rl-b/20261023-w7-rl-b-p1v5-n20`（`backend:=full` / `s15` / seeds 0–19 / CRN 同 seed 配对 / 40 episodes）；判定产物同 run `metrics/p1_adjudication.json`（`prereg_version: 5`）；机制细节见 [ADR-0085](0085-w7b-p1-mechanism-activating-scenario-pilot.md)(h)。
- **结果**：

| DV | 角色 | n 配对 | 剔除 | 均值配对差（协同 − 非协同） | 95% CI | 配对 t p | Wilcoxon p | 判定 |
|---|---|---|---|---|---|---|---|---|
| `fire_control_opportunity_rate_active` | **primary** | 20 | 0 | **+0.0804** | [+0.0281, +0.1328] | **0.0046** | 0.0022 | **Δ < MDE 0.10 ⇒ 按预注册判据证伪** |
| `min_clearance_m` | secondary | 20 | 0 | −0.4125 | [−1.1180, +0.2929] | 0.236 | 0.330 | 不显著（仅报告） |

- **机制激活（实测）**：active 窗口可见率 **协同 0.473 vs 非协同 0.066（Δ≈41 pt）**——α 的因果链（指向 / 跟踪 → 可观测性 → 火控机会）在本场景**确被激活**，与 s08 的 6 pt 形成对照。
- **臂画像**：协同 **15/20 `INTERCEPT_CONFIRMED`** + 4 `MISSION_FAILED` + 1 重试耗尽 infra；非协同 **13/20** + 7 `MISSION_FAILED`；infra 重试 14 次（9/5），终态 infra 1。primary 零填 **12**（协同 5 / 非协同 7），剔除 0。
- **结论（现行 P1 结论）**：在机制已激活的固定条件下，协同观测的 FCO 优势为 **+0.080（绝对值）、CI 不含 0、p=0.005**——**方向与假设一致且统计显著**，但按**当日预注册口径**（v5：`p<α AND Δ≥MDE=0.10`）"未通过"。**三态读法（ADR-0086，2026-10-23 补记）**：CI95 [+0.0281, +0.1328] **覆盖** MME 0.10 ⇒ **`inconclusive`（未通过门槛）**——既**未支持**"收益 ≥ 10 pt"，也**未排除**它（要排除需 CI 上限 < 0.10）。故：**不得**称"协同无效"（效应方向一致、显著），**也不得**称"收益达到 10 pt"；须并列报告点估计 + CI + 决断所需样本量（[ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md)(d)）。
- **解释边界**：协同臂仍为**显式 oracle 指向臂**（(d)）；结论限已验证通道族（本场景为窄视场通道），不涉真实检测 / sim-to-real（[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)(i)）。
- **描述性拆解（[ADR-0085](0085-w7b-p1-mechanism-activating-scenario-pilot.md)(i)，2026-10-23；不改 verdict）**：把"任务成功"与"过程中的火控机会"拆开后——**零填压缩很小**（判定 +0.0804 vs 全回合真实 +0.0846）；**可见率并没等比例转化为机会**（池化：可见率 0.327 vs 0.065，但**可见拍中包络占比 0.134 vs 0.227**）；失败回合最长且机会极少（真实 FCO ≤0.012）使池化 Δ 降到 +0.029。产物 `metrics/p1_fco_decomposition.{json,md}`（[`scripts/w7b_p1_decompose_fco.py`](../../scripts/w7b_p1_decompose_fco.py)）。另登记**候选缺陷**：协同 seed5 第二发 `SHOT_FIRED` 无结果行且此后窗口未再开启（待查）。
- **机制消融（[ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)(f)，2026-10-24；估计型，不改 verdict）**：2×2（指向 × 跟踪参考）N=20 —— **收益主要来自"指向"**（指向主效应 +0.066 / +0.135，均显著）；**"跟踪参考"在 pointing=target 时为负**（①−③ = −0.072，p=0.011）；交互 −0.067；`p1_point_only` 的 FCO（0.155）约为 `p1_coop`（0.083）的 1.9×，但成功率 **15/20 vs 19/20**（权衡，未 adjudicate）。**限 s15 静止目标**，不得外推到移动目标；"跟踪对运动目标的价值"仍**未测**。
- **未决 / 后续**：如需取得**决断性**结论（`supported` / `falsified`），须按 [ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md)(d) 以**新预注册**加大样本量（MME 0.10、σ≈0.112 时：余量 0.03 → N≈109；余量 0.02 → N≈246），**不得**事后改 v5 阈值；**P1 机制消融已完成**（[ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)(f)）；**P2 侧消融（目标预测 / 协同观测）与 W7-c 仍未执行**。

## 备选

1. **让火控自动开火（不改任务难度）** —— 否决：会使跟踪类场景结局漂移（W6 回归破），且 F-11 要求替换后冻结难度。
2. **按场景 outcome 直接改 `expected_terminal`** —— 否决：等于把模型升级掩盖为"基线调整"，且未经授权改 W6 场景语义。
3. **在轻量栈内伪造"协同"** —— 否决：无空中域即无协同，属**静默假设**。

## 后果

- **正面**：E1–E5 有证据；臂可运行；火控升级的**行为变更**被显式裁定而非静默；待建臂显式登记；整链入口（控制权交接 + 观测链）已**评价可用**（(g)）。
- **代价 / 限制**：P1 在 s08 与 s15 两个固定条件下均**未达预注册门槛**（(k)(l)）——s15 下效应显著（Δ=+0.080、p=0.005）但低于 MME 0.10 ⇒ 按 v6 三态为 **`inconclusive`（既未支持也未排除）**；W7-b 出口**部分达成**（臂可运行 + 预注册判定已执行 + P1 消融完成；P2 侧消融未执行）；依赖 `/agc/air/odom` 姿态的**旧证据须重验**（[ADR-0072](0072-px4-attitude-quaternion-field-order-and-full-chain-observation-fix.md)(c)）。
- **未决 / 后续**：`s12` 的 `interception_authorized` 与里程碑语义一致性；P1 是否以 **v7**（MME <0.10）重判——须满足 [ADR-0088](0088-mme-change-governance-independent-justification-and-seed-isolation.md)：**独立工程依据**（不得以观测 0.0804 倒推）、**新种子 seeds ≥20**（0–19 已用于场景选择 / v5 / 消融）、**按实际判定规则重算 N**；协同观测 / 目标预测消融定义；训练 RL 高层（[ADR-0082](0082-w7b-p2-arm-b-rule-high-level-first.md) 裁定 P2 v1 **不**升级）；W7-c 多 worker。

## 参考

- 前序 ADR：[ADR-0004](0004-rl-primary-controller-nav2-safety-layer.md)、[ADR-0024](0024-capture-adjudication-fire-control-window.md)、[ADR-0052](0052-sensor-and-vehicle-model-limitation-and-capability-calibration.md)、[ADR-0069](0069-w7-entry-and-exit-definition.md)、[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)
- 后续 ADR：[ADR-0072](0072-px4-attitude-quaternion-field-order-and-full-chain-observation-fix.md)（整链控制权交接 + 观测链根因修复，(g)）
- 规格：[`.trae/specs/w7-rl-b/`](../../.trae/specs/w7-rl-b/spec.md)
- 预注册：[`src/agc_rl/config/w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)

## 闭合注记（2026-10-24）

W7-b 已按 [ADR-0093](0093-w7b-exit-closure-and-prefix-provenance.md) 闭合。P1 v5 = `inconclusive`、P2 v3 = `falsified` / 保留 c2、P2 v2 = `confounded` 的原文与 `executed_*` 字段均保留；全部确证 / 消融 / 诊断 run 标注 **pre-fix**（火控 [ADR-0091](0091-fire-control-inflight-lifecycle-decoupling.md)；P2 v3 亦早于 [ADR-0089](0089-map-signature-lethal-only-for-replan-trigger.md)）。P2 侧协同观测 / 目标预测消融为**显式范围裁定**（不纳入 W7-b 出口）。v7 未注册；在线 RL 未进入；W7-c / W8 未开始。
