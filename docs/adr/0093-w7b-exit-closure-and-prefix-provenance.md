# W7-b 出口闭合、修复前溯源与剩余消融范围裁定

## Status

accepted

## Date

2026-10-24

## 背景

W7-b 的出口定义见 [ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)(k)：**命题卡 + 臂可运行证据 + 裁定文档**，**不含**论文级统计结论（属 W8）。截至 2026-10-24：

- P1 v5 = `inconclusive`（CI [+0.0281, +0.1328] 覆盖 MME 0.10；[ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md) / [ADR-0090](0090-w7b-post-audit-v7-gate-and-implementation-fixes.md)(a)(b)）；
- P2 v3 = `falsified`、保留 c2；P2 v2 = `confounded`（[ADR-0083](0083-w7b-p2-arm-information-equivalence-and-active-window-fco.md) / [ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(i)(j)）；
- 火控状态机在飞生命周期缺陷已由 [ADR-0091](0091-fire-control-inflight-lifecycle-decoupling.md) 修复（独立变更、独立回归、`model_id=net_ballistic_frame_v2`）；
- 移动目标 costmap marking 触发已由 [ADR-0092](0092-moving-target-costmap-marking-and-replan-policy.md) 独立处置（地图签名只计 static 层）；
- P1 / P2 的全部确证 run、消融与诊断批次均在**上述两项修复之前**执行。

本 ADR 是 W7-b 的**闭合记录**：保留历史 verdict 原文，标注修复前溯源，处理剩余出口缺项，并关闭 W7-b。**不产生新 verdict**，**不改写**任何 `executed_*` 字段。

## 决策

### (a) 保留历史结论，不因修复重判

- P1 v5 维持 `inconclusive`；P1 v4（s08，`falsified`）与 v1（fixed-window，历史成立）保持各自历史读法。
- P2 v3 维持 `falsified`、保留 c2；P2 v2 维持 `confounded`。
- **不**以修复后的小批 / 诊断结果重判 P1 / P2；**不**注册 v7；**不**进入在线 RL。
- 所有历史 `executed_*` / `verdict` 字段逐字保留；本 ADR 只追加溯源与闭合注记。

### (b) 修复前溯源（pre-fix provenance）

| 证据 | 结论 | pre-fix 标记 | 受影响 / 需注意的字段 |
|---|---|---|---|
| P1 v1 `20261021-w7-rl-b-175313-3256` | fixed-window 历史成立 | 火控缺陷前 | 射击时序；fixed-window 含终态后停泊（ADR-0083） |
| P1 v4 `20261023-w7-rl-b-p1v4-n20` | `falsified`（s08，历史） | 火控缺陷前 | 射击 / 命中时序 |
| P1 v5 `20261023-w7-rl-b-p1v5-n20` | `inconclusive`（现行） | 火控缺陷前；s15 seed5 第二发无结局 | 射击时序；FCO 机会拍在输入失效拍可能受 `envelope_in` 未发布影响（**未量化**） |
| P1 消融 `20261023-w7-rl-b-p1abl-n20` | 估计型（指向主效应） | 火控缺陷前 | 真实过程 FCO；射击时序 |
| P2 v2 `20261022-w7-rl-b-112833-1155` | `confounded` | 火控缺陷前；ADR-0089 前 | 臂信息不等价 + 终态后污染；不作判定 |
| P2 v3 `20261022-w7-rl-b-v3-n20r` | `falsified` / c2 | 火控缺陷前；**ADR-0089 前** | `flight` / `miss_dist` 时序；重规划计数（含 255/253 伪触发与移动目标 marking） |
| 诊断 run（p1fail、p2 predchain / retry2、replan audit、map trigger audit） | 仅诊断 | 火控缺陷前 | 只用于解释，不作判定 |

> 结论：上述 run 的**射击 / 命中时序证据**受火控缺陷影响；P2 v3 的重规划计数还受 ADR-0089 之前的签名口径影响。历史 verdict 是**当时实现的判定**，本 ADR **不**改写，也**不**用修复后结果替换。

### (c) 出口缺项盘点

| 出口 | 状态 | 说明 |
|---|---|---|
| E1–E5 入口前置 | ✔ | 各有可复现 run（[`w7-rl-b/checklist.md`](../../.trae/specs/w7-rl-b/checklist.md)） |
| R1 P1 协同收益 | ✔（`inconclusive`） | 现行读法保留；修复前实现 |
| R2 P2 职责分层 + 消融 | ✔（可运行子集）/ 范围裁定 | P2 两臂、P1 2×2、安全干预 with/without shield 有证据；**P2 侧协同观测 / 目标预测消融未执行**，按 (d) 显式范围裁定 |
| R3 基线锚 | ✔ | Navfn + 自研 LQR，准确命名 |
| R4 预注册 | ✔ | P1 v5 / P2 v3 由各自历史预注册承载；v6 只提供三态读法（[ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md)） |
| R5 裁定文档 | ✔ | [ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md) + 本 ADR 的 pre-fix 溯源 |
| 横切 R6–R8 | ✔ | 单一入口 / 产物纪律 / 引用与结论域边界 |

### (d) 剩余消融的显式范围裁定

**未执行**：P2 侧「协同观测」与「目标预测」消融。

**裁定：不纳入 W7-b 出口，留待 W8 / 新战役。** 理由：

1. **不能改变已登记 verdict**：P2 v3 的注册假设（规则高层 + 成熟低层 vs c2）已判 `falsified` 并保留 c2；在 B 臂内部再开关协同观测 / 目标预测，得到的是**新假设 / 新臂**，不是对 v3 的重读。
2. **修复前污染**：这些消融若在修复前执行，射击时序与重规划计数会被 ADR-0091 / ADR-0092 的缺陷污染；修复后执行则须**新预注册 + 新种子 + 重新规划 N**，不能与 v3 混用。
3. **出口边界**：W7-b 出口是命题卡 + 臂可运行证据 + 裁定文档，**不是**论文级机制消融全集（[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)(k)）；机制消融的完整矩阵属 W8。
4. **主张门槛**：若未来要主张「仅指向足够 / 协同观测足够 / 目标预测足够」，须同时提供**机会收益 + 预先定义的任务成功率保障 + 净空 / 干预指标**，并按实际判定规则重新规划 N（[ADR-0085](0085-w7b-p1-mechanism-activating-scenario-pilot.md)(i)(i-1)、[ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)(f)、[ADR-0088](0088-mme-change-governance-independent-justification-and-seed-isolation.md)）。

### (e) 火控修复后的可用证据（不改变出口结论）

- **确定性**：`test_dynamic_fire_control` 13 项全过（目标过期 / 低置信度 / 越出高度 / 越出射程 / 车辆位姿缺失 / 落地 / 超时 / 冷却 / 复位中止 / 时钟回退 / 唯一终局 / 正常命中）。
- **节点级**：`tests/w7b_fire_control_lifecycle_check.py` A–F 全过（合成话题，覆盖目标失效、低置信度、越出高度、odom 缺失、复位、冷却）。
- **lightweight E2**（GUI + headless）：`20261024-w7-rl-b-145517-2702`（s08 headless）、`20261024-w7-rl-b-152651-9675`（s08 GUI）、`20261024-w7-rl-b-152800-3174`（s15 seed5 headless，4 发全部闭合）、`20261024-w7-rl-b-152850-0348`（s15 seed5 GUI）。全部 `SHOT_FIRED` 有唯一终局，`flight` 在 1.30 s 级（≤3.15 s），无 5007 m `miss_dist`。
- **整链小批（P1 s15 seed5 / P2 B s08 seeds 0–2）**：受 uXRCE `create entities failed: participant: 255`（agent participant 创建 1.5–1.9 s > PX4 1 s 预算）阻塞，见 `experiments/w7-rl-b/20261024-w7-rl-b-firectl-fullchain-infra/metrics/fire_control_fullchain_infra.md`。**infra 单列**，不伪装成任务失败或成功；待环境恢复后补跑，届时另立 run，不改历史 verdict。

### (f) 闭合状态

- **W7-b 闭合**：出口 = 命题卡（[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)）+ 臂可运行证据 + 裁定文档（[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md) + 本 ADR）。
- P1 v5 = `inconclusive`；P2 v3 = `falsified` / c2；P2 v2 = `confounded`。
- **v7 未注册**；**在线 RL 未进入**；**W7-c 未开始**；**W8 未开始**。
- 火控模型身份 `net_ballistic_frame_v2`；修复前证据保留为 pre-fix。

## 备选

1. **用修复后小批重判 P1 / P2** —— 否决：样本量不足、非确证，且违反「不得用诊断 run 重判」与预注册纪律。
2. **补跑 P2 侧协同观测 / 目标预测消融后再闭合** —— 否决：会引入新臂 / 新预注册；且当前整链 uXRCE 环境不可用；W7-b 出口不要求该全集。
3. **注册 v7 以消除 `inconclusive`** —— 否决：无独立工程理由；成本与门槛治理见 [ADR-0090](0090-w7b-post-audit-v7-gate-and-implementation-fixes.md) / [ADR-0088](0088-mme-change-governance-independent-justification-and-seed-isolation.md)。
4. **把整链 infra 失败混入成功率** —— 否决：违反「infra 单列」与 [ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)。

## 后果

- **正面**：W7-b 以「命题 + 臂证据 + 裁定」闭合；历史 verdict 完整；修复前后证据以 `model_id` / ADR 明确分离；剩余消融有显式范围裁定而非沉默跳过。
- **代价 / 限制**：P1 仍 `inconclusive`、P2 仍 `falsified`；P2 侧机制消融与整链小批留待 W8 / 环境恢复；修复前 run 的射击时序不可与修复后直接混用。
- **未决 / 后续**：整链小批补跑（infra 恢复后）；P2 侧机制消融（W8 / 新预注册）；W7-c 多 worker；W8 对照与冻结；实机（[ADR-0009](0009-sim-first-with-hardware-hooks.md)）。

## 证据

- 预注册：[`src/agc_rl/config/w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)（v6；历史 `executed_*` 未改）。
- 裁定：[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)（(h)(i)(j)(k)(l)）。
- 火控修复：[ADR-0091](0091-fire-control-inflight-lifecycle-decoupling.md)、`src/agc_task/test/test_dynamic_fire_control.cpp`、`tests/w7b_fire_control_lifecycle_check.py`。
- 地图触发：[ADR-0092](0092-moving-target-costmap-marking-and-replan-policy.md)、`20261024-w7-rl-b-p2-map-trigger-bench`（C7 4→0、C8 4→1）。
- 整链 infra：`20261024-w7-rl-b-firectl-fullchain-infra/metrics/fire_control_fullchain_infra.md`。
- lightweight E2：`20261024-w7-rl-b-145517-2702`、`20261024-w7-rl-b-152651-9675`、`20261024-w7-rl-b-152800-3174`、`20261024-w7-rl-b-152850-0348`。

## 参考

- 出口：[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)(k)、[`w7-rl-b/spec.md`](../../.trae/specs/w7-rl-b/spec.md)、[`w7-rl-b/checklist.md`](../../.trae/specs/w7-rl-b/checklist.md)。
- 判据：[ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md)、[ADR-0088](0088-mme-change-governance-independent-justification-and-seed-isolation.md)。
- 修复：[ADR-0090](0090-w7b-post-audit-v7-gate-and-implementation-fixes.md)、[ADR-0091](0091-fire-control-inflight-lifecycle-decoupling.md)、[ADR-0092](0092-moving-target-costmap-marking-and-replan-policy.md)。
- infra：[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)。
