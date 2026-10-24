# W7-b 核实后的决策门：不注册 v7、保留 `inconclusive` / c2，先修火控状态机

## Status

accepted

## Date

2026-10-24

## 背景

本 ADR 是核实子阶段 [`verify-w7b-p1-p2-before-v7`](../../.trae/specs/verify-w7b-p1-p2-before-v7/spec.md) 的 T5 决策记录。它汇总 P1 / P2 的诊断证据，回答「是否注册 v7」与「哪些实现缺陷须先修」，**不产生新的 P1 / P2 verdict**，也**不改写**历史 `executed_*` 字段或既有判定原文。

### P1：旧失败未复现，`inconclusive` 维持

- P1 v5（run `w7-rl-b/20261023-w7-rl-b-p1v5-n20`）primary `fire_control_opportunity_rate_active` 的 95% CI [+0.0281, +0.1328] **覆盖** MME=0.10，按 [ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md) 三态读法为 **`inconclusive`（未通过门槛）**。
- 2×2 消融（run `w7-rl-b/20261023-w7-rl-b-p1abl-n20`，[ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)(f)）为**估计型**：收益主要来自「指向」；`p1_point_only` 的**真实过程 FCO** 0.155 高于 `p1_coop` 0.083，但任务成功率 **15/20 对 19/20** 更低。该消融**不**判定「仅指向足够」。
- `p1_point_only` 的 5 个超时 seeds（1 / 4 / 8 / 10 / 18）在 2026-10-24 诊断 run 中**未复现**：
  - `w7-rl-b/20261024-w7-rl-b-p1fail-smoke`：seed 8 捕获；
  - `w7-rl-b/20261024-w7-rl-b-p1fail-diag`：seeds 1 / 4 / 10 捕获，seed 18 为 infra（ACTIVE）耗尽；
  - 同类的盲 / 低高度失败反而出现在协同臂（smoke 的 coop seed8 可见率 0；diag 的 coop seed4 可见率 0.084；coop seed18 低高度、5 发全失）。
  ⇒ 旧「更多机会但更少捕获」**不是稳定的臂属性**，更符合运行期靶机高度 / 几何随机性（或 infra）。证据见 `20261023-w7-rl-b-p1abl-n20/metrics/p1_point_only_failures.{json,md}` 与 [ADR-0085](0085-w7b-p1-mechanism-activating-scenario-pilot.md)(i)(i-1)。
- **没有独立工程理由降低 MME**，也没有**预先定义的任务成功率保障**。若未来要主张「仅指向足够」，须新预注册 + 新种子 + 机会收益 + 任务成功率保障 + 净空 / 干预指标，并按实际判定规则计算 N（[ADR-0088](0088-mme-change-governance-independent-justification-and-seed-isolation.md)）。

### P2：v3 维持 `falsified` / c2；缺陷与离群已分解但未消除

- P2 v3（run `w7-rl-b/20261022-w7-rl-b-v3-n20r`）primary CI [−0.0971, −0.0151] 完全低于 MME=0.10 ⇒ **`falsified` 不变，保留 c2**。
- **重规划审计**（run `w7-rl-b/20261024-w7-rl-b-p2-replan-audit`）：
  - 「目标身份变」实际判据是 `hypot(goal − last_plan_goal) > replan_displacement_m`，不是 `target_id`；受控台架 C1–C3 行为正确，**非缺陷**（仅命名错位，未改名）。
  - 「地图变化」确认**签名口径缺陷**：`cost >= 253` 把 `NO_INFORMATION=255` 与 `INSCRIBED_INFLATED_OBSTACLE=253` 当作硬障碍；C5 自由 / 未知翻转 4 s 内触发 14 次。已由 [ADR-0089](0089-map-signature-lethal-only-for-replan-trigger.md) 修为 **`cost == 254`**，C5 降为 0、C4 真实增障仍触发 1 次，W6 R-041 回归 PASS。
  - 全链新 run `w7-rl-b/20261024-w7-rl-b-p2-predchain-retry2` 的地图触发 30 次全部带 `hard_cells 7–14`，`static_layer=0`、`obstacle_layer=136`、`voxel_layer=320` ⇒ 「未知 255 / inflation 253」伪触发消失；剩余触发来自**移动目标在 obstacle / voxel 层的 lethal marking 迁移**，属 ADR-0089 明确不消除的**另一原因**。
- **预测链分解**（v3 与 retry2 的 `metrics/p2_prediction_chain.{json,md}`，[ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)(d-1)）：
  - v3 的 8.7 m 离群在 `max_lead_m` 上限下**目标估计至少占 7.241 m**，不是高层 lead 或火控提前量；`tracks.front()` 与最高置信度在单航迹通道下构造一致，**无证据**表明其为主因。
  - retry2（B 臂 3 seeds）**3/3 捕获**，6 发 `pred` 误差均值 0.221 m、>0.5 m **0/6**，8.7 m **未复现**；但仅 3 seeds，**不能**推翻 v3。
- **新发现火控状态机时序缺陷**：`DynamicFireControlAdjudicator::step()` 的位姿 / 交战门提前返回位于 `if (in_flight_)` 之前，导致在飞期间门短暂失败时既不推进也不结算 ⇒ `flight > 3 s`（v3 最大 32.592 s）、`miss_dist` 最大 5007.883 m、落地判定延迟（retry2 的 miss 回合 `flight − t_est` 为 +1.21～+1.34 s）。该缺陷与弹道模型 / 目标估计 / 高层 lead 无关，须独立修复。

### 在线 RL

现有证据仍不足以进入在线 RL：P1 为 `inconclusive`，P2 为 `falsified`，且火控状态机缺陷未修。[ADR-0082](0082-w7b-p2-arm-b-rule-high-level-first.md)(d) 的触发条件未满足，维持「规则高层优先」。

## 决策

### (a) 本阶段**不注册 v7**

P1 维持 `inconclusive`，P2 维持 `falsified` 并保留 c2。理由：没有独立工程 / 科学理由降低 MME；没有预先定义的任务成功率保障；诊断 run 未复现旧失败，不构成「仅指向足够」的新证据；不得仅为消除 `inconclusive` 而立项或投入上百对重跑。

### (b) MME 与历史记录不变

MME 保持 0.10，α=0.05、power=0.80、N=20 与三态判据不变；**不**因 v5 观测 0.0804 倒推门槛。所有历史 verdict 与 `executed_*` 字段保持原文，不回溯改写。

### (c) 先修火控状态机，再决定 P2

把「在飞推进 / 落地 / 超时结算」与位姿 / 交战门解耦（或为在飞状态单独维护弹道时钟），作为**独立变更 + 独立 ADR + 回归**。修复前**不注册**新的 P2 确证 run，也不据诊断 run 重判 P2。

### (d) 移动目标地图触发另议

AX-0089 只消除「未知 255 / inflation 253」伪触发；「移动目标被 obstacle / voxel 层标记为 lethal」是否应参与静态地图重规划签名，须**独立 ADR** 决策，不与火控状态机修复合并。

### (e) 不进在线 RL

W7-c / W8 未开始；本决策不改变该状态。

### (f) 本 ADR 的效力边界

本 ADR **不产生新 verdict**，**不修改**任何 `p1_adjudication.json` / `p2_adjudication.json` 的判定结论，**不改写**历史 `executed_*` 字段。所有新增 run 均为**诊断 / 非确证**，真值只用于离线评估。

## 备选

1. **注册 v7（同场景、新种子、加大 N）**——否决：当前没有独立工程理由；「仅指向足够」缺少任务成功率保障；按余量 0.02–0.03 估算 N≈109–246（每对 3–6 min，约 5–10 h），成本不足以由现有证据支撑。
2. **直接下调 MME 到 0.08 使 v5 变 `supported`**——否决：事后改门槛，违背 [ADR-0088](0088-mme-change-governance-independent-justification-and-seed-isolation.md) 与 [F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)。
3. **直接用 3-seed retry2 重判 P2**——否决：样本量不足，且火控状态机缺陷污染飞行 / 命中时序；须先修缺陷、再以新预注册承载。
4. **把移动目标 marking 触发与火控修复合并为一次「优化」**——否决：违背「每次只修一个原因」；两者根因与回归面不同。
5. **进入在线 RL**——否决：证据不足，触发条件未满足。

## 后果

- **正面**：避免为消除 `inconclusive` 进行无依据的大规模重跑；把已确认的实现缺陷（火控状态机、移动目标地图触发）与统计命题分离，后续修复可独立验收；历史判定保持完整。
- **代价 / 限制**：P1 仍停在 `inconclusive`，P2 仍为 `falsified`；在火控状态机修复前，新的 P2 射击 / 命中证据仍不可用于确证；移动目标地图触发仍会消耗重规划预算。
- **未决 / 后续**：① 火控状态机的独立 ADR + 回归；② 移动目标地图触发的独立 ADR；③ 若未来主张「仅指向足够」，须新预注册 + 新种子 + N 规划 + 成功率保障；④ 在线 RL / W7-c / W8 仍未开始。

## 证据

| 用途 | run_id / 产物 |
|---|---|
| P1 v5 判定 | `w7-rl-b/20261023-w7-rl-b-p1v5-n20/metrics/p1_adjudication.json` |
| P1 消融（估计型） | `w7-rl-b/20261023-w7-rl-b-p1abl-n20/metrics/p1_ablation.{json,md}` |
| P1 失败解释 | `w7-rl-b/20261023-w7-rl-b-p1abl-n20/metrics/p1_point_only_failures.{json,md}` |
| P1 复现诊断 | `w7-rl-b/20261024-w7-rl-b-p1fail-smoke`、`w7-rl-b/20261024-w7-rl-b-p1fail-diag`（`metrics/arms.json`、`log/`） |
| P2 v3 判定 | `w7-rl-b/20261022-w7-rl-b-v3-n20r/metrics/p2_adjudication.json` |
| P2 B 臂诊断 / 预测链 | 同 run `metrics/p2_arm_b_diagnosis.{json,md}`、`metrics/p2_prediction_chain.{json,md}` |
| P2 重规划审计 / 台架 | `w7-rl-b/20261024-w7-rl-b-p2-replan-audit/metrics/p2_replan_audit.{json,md}`、`p2_replan_bench*.{json,md}` |
| P2 预测链 / 火控时序诊断 | `w7-rl-b/20261024-w7-rl-b-p2-predchain-retry2/metrics/p2_prediction_chain.{json,md}`、`p2_arm_b_diagnosis.{json,md}` |
| infra 证据（失败 / 空 run） | `w7-rl-b/20261024-w7-rl-b-p2-predchain`、`-r2`、`-retry`（仅日志，无 metrics） |

归档按 [ADR-0064](0064-evidence-immutable-archive-and-intake-manifest.md) 生成 `config/evidence_manifest.json`、只读 `.tar` + `.sha256`、`.retrieval.json`（`verified=true`、`mismatches=[]`），路径 `experiments/_archive/w7-rl-b/<run_id>.tar`。上述诊断批次所引用的 **`w6-task-eval` 逐 episode run**（13 个）与 R-041 回归 run `20261024-w6-r041-p2audit` 亦已按同口径归档，避免 `arms.json` 的 `summary_path` 指向未归档证据。

相关裁定：[ADR-0085](0085-w7b-p1-mechanism-activating-scenario-pilot.md)(g)(h)(i)(i-1)、[ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md)、[ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)(d)(d-1)(f)、[ADR-0088](0088-mme-change-governance-independent-justification-and-seed-isolation.md)、[ADR-0089](0089-map-signature-lethal-only-for-replan-trigger.md)。

## 参考

- 核实 spec：[`verify-w7b-p1-p2-before-v7`](../../.trae/specs/verify-w7b-p1-p2-before-v7/spec.md)
- 判据与治理：[ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md)、[ADR-0088](0088-mme-change-governance-independent-justification-and-seed-isolation.md)
- 消融 / B 臂诊断：[ADR-0087](0087-p1-mechanism-ablation-and-p2-arm-b-diagnosis-scope.md)
- 地图签名修复：[ADR-0089](0089-map-signature-lethal-only-for-replan-trigger.md)
- 证据归档：[ADR-0064](0064-evidence-immutable-archive-and-intake-manifest.md)
- 火控实现：[`capture_adjudicator.cpp`](../../src/agc_task/src/capture_adjudicator.cpp)、[ADR-0079](0079-b2-fire-control-target-motion-estimation.md)
