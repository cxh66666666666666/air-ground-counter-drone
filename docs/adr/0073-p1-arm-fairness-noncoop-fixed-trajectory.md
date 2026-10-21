# P1 臂公平性修正：非协同臂 = 空中固定航迹（不订阅目标跟踪参考）

## Status

accepted

## Date

2026-10-20

## 背景

W7-b P1（**协同 vs 非协同**，[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)）首轮 N=20 确证 run（`w7-rl-b/20261020-w7-rl-b-094812-9910`）暴露 **臂定义与 ADR-0070 不符**导致的**混淆（confound）**：

- 两臂**仅**差 `observe_yaw_mode`（`target` / `velocity`），但**两侧都**跑同一 `standoff_follower` 目标跟踪栈（tracker 订阅 `/agc/air/tracking_ref`）。
- 非协同臂在**接管瞬间**短暂获得目标（`acquired_=true`），随即丢失 → 触发契约 [#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C10** / [ADR-0047](0047-c10-freeze-downstream-closure-source-time-and-offset-lifetime.md) 的「已获目标后…**hold（不回落八字）**」：`vel=0`，**停在空中原地**；`velocity` 下机头**不再转动** → 目标永久出视场 → **自我锁死** → `MISSION_FAILED(TIMEOUT)`，`visibility_rate` 0.01–0.02、`fco_rate=0`。
- **实测**（2026-10-20 整链，`velocity` + `tracking_ref=on`）：`start` 后约 2 s 无人机即冻结在 (-2.7, -1.5, 3.6)、`yaw` 冻结 −107.8° 达 42 s；目标方位 ≈+32° → **Δ≈+140°**（远超 ±45°/±50° FOV）；节点日志逐条 `已获目标后参考 / 目标失效 → 显式 hold`。
- **后果**：α 由「**观测指向**」退化为「**能否持续观测**」（协同臂 ≈0.98，且从不进入 hold）——P1 对比**混入"观测丢失→停机自锁"机制**，`Δvis≈+0.96` / `Δfco≈+0.16` **偏乐观**。

## 决策

### (a) 非协同臂按 [ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md) 字面「**空中固定航迹**」重新定义

`w6.launch.py` 新增 `air_tracking_ref := on|off`（缺省 `on`，**既有行为不变**）。非协同臂取 `off`：把 `trajectory_tracker.tracking_ref_topic` 指向**未使用话题** → tracker **永不** `acquired_` → 只飞**基础 figure-8**（固定航迹），**不**触发 hold。

- **不改**契约 C10 / [ADR-0047](0047-c10-freeze-downstream-closure-source-time-and-offset-lifetime.md) 的 hold 语义（**红线保持**）——只改"**本臂是否喂入跟踪参考**"。
- 落地：`episode_runner.run_episode` 透传 `air_tracking_ref`；`w7b_arms_runner` 的 `p1_noncoop` 设 `air_tracking_ref: off`。

### (b) 协同臂保持（跟随目标 + 对准目标）

`p1_coop` 不变（`tracking_ref` on + `observe_yaw_mode=target`）。
→ α = {**协同**：跟踪 + 指向；**非协同**：固定航迹 + 不指向}，对齐 ADR-0070 的 α 定义（"空地协同策略" vs "非协同（地面单体 / **空中固定航迹**）"）。

### (c) 首轮 N=20 结果**作废（登记）**

首轮确证 run 的配对结论因上述混淆**不采信**（产物保留于 `experiments/w7-rl-b/20261020-w7-rl-b-094812-9910` 作为**反例证据**）；修正臂定义后**重跑**，按预注册（[`w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)）判定。判定规则 / 样本量 / 阈值**均未**改动（[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)）。

## 复验

- **参数级**：`atr=off` 生成的空中参数中 `trajectory_tracker.tracking_ref_topic = /agc/air/tracking_ref__inactive_p1_fixed_traj`。
- **单元级**（整链 s08 seed0 / `velocity` / `atr=off`）：`start OK` → **接管** → **无 `显式 hold`** → 飞行**连续**（位置沿大图-8 移动、`yaw` 随速度扫过 13°→52°→163°→−131°→−79°→−11°）、终态 `TRACKING_SUCCESS`、`vis_rate=0.986`、`fco_rate=0.157`。
- **新发现（未决）**：修正后**非协同臂** `vis_rate` 亦 ≈0.98 → 与协同臂**无差**。原因：`visibility_rate` 分母为 **episode active 窗口**（start→终态），两臂都很快达 `TRACKING_SUCCESS`（≈16 s）；且"成功需目标可见"构成**选择效应** → 该 DV 在"早成功"臂上**饱和**，对"指向"自变量**不敏感**。

## 备选

1. **改 tracker 的 hold 行为（丢失后回落 figure-8）** —— 否决：违反 C10 / ADR-0047 的**自保**语义（观测丢失即冻结为**契约要求**），不得为实验方便而改。
2. **两臂都关 `tracking_ref`（只留 yaw 差异）** —— 保留备选：可得"**单一自变量 = 观测指向**"的最干净隔离；但协同臂失去"空中跟随"维度，与 ADR-0070 的"空地协同策略"定义不完全一致。
3. **接受现状并如实标注混淆** —— 否决：会把"停机自锁"当作"协同收益"，命题失去可证伪性。

## 后果

- **正面**：非协同臂成为**公平基线**（固定航迹 + 不指向），解耦"观测策略"与"停机自锁"；预注册纪律保持（规则未改）。
- **代价 / 限制**：α 为**两项差异组合**（跟踪参考 + 指向，依 ADR-0070 的 α 定义属设计内）；首轮 run 作废、需重跑。
- **未决 / 后续**：① ~~**DV 饱和 / 选择效应**~~ → 已裁定：[ADR-0074](0074-visibility-metric-fixed-observation-window.md)（改固定观测窗口 `window_fixed_v1`）；② ~~full 链 infra 率（`air_hover_timeout` 等）另治~~ → 已处置：[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)（真根因=残留孤儿进程；运行前清场 + infra 重试）；③ ~~重跑 P1（N=20）~~ → 已跑两轮，判定见 [ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(h)（其语义随后被 [ADR-0076](0076-capture-scenario-tracking-milestone-not-terminal.md) 取代，**待新语义重跑**）。

## 参考

- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C10）/ [#execution-port](../ROS2迁移交接文档.md#execution-port) / [#params](../ROS2迁移交接文档.md#params)
- 前序 ADR：[ADR-0043](0043-episode-window-metrics-and-valid-sample-mask.md)、[ADR-0047](0047-c10-freeze-downstream-closure-source-time-and-offset-lifetime.md)、[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)、[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)、[ADR-0072](0072-px4-attitude-quaternion-field-order-and-full-chain-observation-fix.md)
- 预注册：[`src/agc_rl/config/w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)
