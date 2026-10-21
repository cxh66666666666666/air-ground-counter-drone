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

- **P2**：臂 A / 臂 B 均**可运行**且达任务出口 → 具备开展等预算对比的条件；**判定未执行**（无预注册阈值下的对比）。
- **P1**：整链**已具备承载**（(g)）；配对 run 与判定已分别在 (h) 与其**修订**子节执行（后者为新终态语义下的现行结论）。
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

## 备选

1. **让火控自动开火（不改任务难度）** —— 否决：会使跟踪类场景结局漂移（W6 回归破），且 F-11 要求替换后冻结难度。
2. **按场景 outcome 直接改 `expected_terminal`** —— 否决：等于把模型升级掩盖为"基线调整"，且未经授权改 W6 场景语义。
3. **在轻量栈内伪造"协同"** —— 否决：无空中域即无协同，属**静默假设**。

## 后果

- **正面**：E1–E5 有证据；臂可运行；火控升级的**行为变更**被显式裁定而非静默；待建臂显式登记；整链入口（控制权交接 + 观测链）已**评价可用**（(g)）。
- **代价 / 限制**：P1 已在新终态语义下重跑并判定（(h) 修订子节）→ W7-b 出口**部分达成**（P2 判定、消融未执行）；依赖 `/agc/air/odom` 姿态的**旧证据须重验**（[ADR-0072](0072-px4-attitude-quaternion-field-order-and-full-chain-observation-fix.md)(c)）。
- **未决 / 后续**：`s12` 的 `interception_authorized` 与里程碑语义一致性；P2 预注册对比（c2 / 高层 RL）；协同观测 / 目标预测消融定义；高层策略由规则升级为 RL；W7-c 多 worker。

## 参考

- 前序 ADR：[ADR-0004](0004-rl-primary-controller-nav2-safety-layer.md)、[ADR-0024](0024-capture-adjudication-fire-control-window.md)、[ADR-0052](0052-sensor-and-vehicle-model-limitation-and-capability-calibration.md)、[ADR-0069](0069-w7-entry-and-exit-definition.md)、[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)
- 后续 ADR：[ADR-0072](0072-px4-attitude-quaternion-field-order-and-full-chain-observation-fix.md)（整链控制权交接 + 观测链根因修复，(g)）
- 规格：[`.trae/specs/w7-rl-b/`](../../.trae/specs/w7-rl-b/spec.md)
- 预注册：[`src/agc_rl/config/w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)
