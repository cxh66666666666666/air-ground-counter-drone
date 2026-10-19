# 决策适用关系与引用纪律

## Status

accepted

## 背景与决策

结构评审 [S-10](../../docs/reviews/agc-review-2026-10-14/20-结构.md#s-10)（中）指出：accepted ADR 的**历史正文**与后续修订 / 下游状态**残留并存**且缺**当前适用关系**；spec / checklist **相对链接层级不符**、稳定锚点用法不一致；进度出现「**第二状态表**」。评审分类明确：**残留措辞不等同进度未完成的裁决**，**保留历史、不全文重写抹除演进**。

**取证（S-10 盘点，2026-10-18，只读脚本 + 静态审查）**：

- **禁用章节号**：`grep -rnE "§[0-9]" docs/ .trae/specs/` **无命中**（本就满足，本轮复核）。
- **决策适用关系**：已有「适用关系」节的 ADR —— [ADR-0003](./0003-nav2-pattern-complete-absorption.md) / [ADR-0008](./0008-nav2-paradigm-absorption-as-code-principle.md)（S-04 校准，口径见 [ADR-0060](./0060-nav2-absorption-applicability-and-command-source-trimming.md)）；[ADR-0041](./0041-scenario-axis-carriers-and-coverage.md) 顶部 S-05 / S-06 注。**唯一需正式化者为 [ADR-0012](./0012-px4-body-as-toolchain-dependency.md)**：其正文记 uXRCE-DDS Agent `v2.4.2`，现行 README / Runbook 为 `v2.4.3`，S-09 只加了**临时顶部注**、**未**正式化。其余 ADR 的旧值已在**自身正文显式标「作废」**（[ADR-0017](./0017-air-ground-collaboration-loop.md) 改写 C10、[ADR-0019](./0019-uav-trajectory-and-tracking-parameter-calibration.md) 改写 C1、[ADR-0021](./0021-ground-safety-thresholds-and-platform-recalibration.md) 旧 C2 值、[ADR-0022](./0022-launched-net-capture-mechanism.md) 旧捕获判据、[ADR-0023](./0023-spawn-clearance-recalibration.md) 旧 `2.4 m`、[ADR-0034](./0034-failsafe-factor-separation-and-runtime-param-lock.md) 旧 failsafe 叙述），**无并存歧义**。
- **相对链接层级**：全仓 spec / checklist 中 `.trae/specs/<ws>/` 指向项目根的链接应为 `../../../`。只读脚本分类：**层级错误 7 处** + **文件名错误 2 处**（`docs/adr/` 内），另有 **3 处**为 [`spec模板.md`](../conventions/spec模板.md) **代码块内**的模板占位（按生成物层级 `../../../`，非真错）。层级错误已修：[w2-sim-world checklist](../../.trae/specs/w2-sim-world/checklist.md) 两处 `../../docs/adr/` → `../../../docs/adr/`；[fix-ground-failstop-r8-r10 tasks](../../.trae/specs/fix-ground-failstop-r8-r10/tasks.md) `../../w5-*` → `../w5-*`；[ADR-0008](./0008-nav2-paradigm-absorption-as-code-principle.md) / [ADR-0060](./0060-nav2-absorption-applicability-and-command-source-trimming.md) 的 `../GLOSSARY.md` / `../.trae` / `../src` → `../../…`。文件名错误已修：[ADR-0038](./0038-continuous-tracking-dwell-interruption.md) → `0028-task-start-transaction-and-freshness-gate.md`；[ADR-0053](./0053-lqr-numeric-model-exposure-and-config-validity.md) → `0018-shared-control-layer-and-lqr-plugin.md`。
- **残留状态**：[w6-task-eval checklist](../../.trae/specs/w6-task-eval/checklist.md#L4) 曾称「唯一遗留为『双 PX4 整链 launch 集成』」，与 Runbook [#status](../ROS2迁移步骤.md#status)（W6 ✅）**矛盾**；[w2-sim-world checklist](../../.trae/specs/w2-sim-world/checklist.md#L4)（「W2-full 待办」而 R6 已 [x]）、[m0 checklist](../../.trae/specs/m0-walking-skeleton/checklist.md#L4)（「除 Runbook 状态表待合并」而 Runbook M0 ✅）同属残留。本轮已把三者的状态行改为**只引用** Runbook [#status](../ROS2迁移步骤.md#status)，**未**凭推断改写完成状态；w6 两个未勾项（R7 报告口径标注 / 通用红线）**据实核定**后补勾并附证据。
- **锚点用法**：`契约 #hard-constraints` 一类**裸锚点文本**散见于 [w0](../../.trae/specs/w0-repo-and-conventions/checklist.md) / [w1](../../.trae/specs/w1-msgs/checklist.md) / [w3](../../.trae/specs/w3-air-link/checklist.md) / [m0](../../.trae/specs/m0-walking-skeleton/checklist.md)（与其它文件的可点击链接不一致）；已统一为可点击稳定锚点。

**决策**：

- **(a) 适用关系节标准**：历史正文与现行并存的 ADR **须**有「**适用关系**」节，显式给出 **supersedes / amends / 适用范围 / 决议锚点**；**保留历史正文**（不全文重写），下游**只链接现适用**决策。[ADR-0012](./0012-px4-body-as-toolchain-dependency.md) 的 S-09 临时注**升格**为标准节（本 ADR 生效；**不**重复堆叠）。
- **(b) 进度唯一真源**：进度**只**引用 Runbook [#status](../ROS2迁移步骤.md#status)；per-workstream 三件套**不**维护**第二状态表**；**残留措辞不**作为进度未完成的裁决（[ADR-0002](./0002-docs-source-of-truth-layering.md) 真源分层在「进度」维度的落地）。
- **(c) 引用纪律**：跨文档一律用**稳定锚点可点击链接**；相对路径按三件套层级（`.trae/specs/<ws>/` → `../../../`）；**禁** `§` 章节号。**纪律 / 禁止句**中被点名的锚点名称（如 "Runbook `#status`"、"契约 `#hard-constraints`"）作为**命名引用**保留，但**可导航引用一律给出链接**。
- **(d) spec 模板**：[`spec模板.md`](../conventions/spec模板.md) **占位符单列**成表（`<WS>` / `<名称>` / `<ws-id>-<slug>` / `<锚点>` / `<#>`）；模板的进度条目引用 Runbook 而非第二状态表；模板内代码块的相对层级按**生成物**（`.trae/specs/<ws>/`）计。

## Considered Options

- **全文重写各 ADR 抹平演进** —— 被否：抹掉演进轨迹，读者无法复核历史结论（违背评审「保留历史」）。
- **在各处维护第二状态表以「对齐」** —— 被否：制造重复真源，与 Runbook 冲突（评审点名的病灶）。
- **仅改被点名两处（ADR-0012 / w6 checklist）** —— 被否：不解决「层级 / 锚点 / 模板」的**系统性**不一致（评审建议「检查相对链接和显式锚点，模板占位单列」）。
- **重写指向已退役 / 已归档产物的悬挂链接** —— 被否：会把历史证据日志改成"当时不存在"的名字，**伪造记录**。按 [ADR-0041](./0041-scenario-axis-carriers-and-coverage.md) 先例，退役入口名以「现适用」注记读作同一拓扑，**不重写**。

## Consequences

- 读者按「**现适用**」决策执行；历史分歧可追溯（适用关系节 + 历史正文并存）。
- 全仓 spec / checklist / ADR 的**层级错误清零**；`§` grep 空；三件套不再出现第二状态表（模板已固化）。
- **未决登记（不静默跳过）**：只读脚本仍检出**悬挂引用**——指向**已退役入口**（`w6_task_eval.launch.py` / `w6_full.launch.py`，[ADR-0061](./0061-w6-runtime-topology-backend-identity-and-single-parameterized-entry.md) 已统一为 `w6.launch.py`）与**已移除 / 已归档实验产物**（`experiments/w6-task-eval/20261014-*`，[ADR-0044](./0044-unique-run-identity-and-reproducible-evidence.md) / [ADR-0064](./0064-evidence-immutable-archive-and-intake-manifest.md)）；分布于 [ADR-0040](./0040-full-chain-coverage-matrix-and-same-run-causal-record.md) / [ADR-0041](./0041-scenario-axis-carriers-and-coverage.md) / [ADR-0049](./0049-lifecycle-repeatable-resource-state-machine-and-liveness-detection.md) / [ADR-0057](./0057-c3-gui-acceptance-scope-authorization.md) 与三件套 `fix-*` 批。**本轮不重写**（保历史事实），按 [ADR-0041](./0041-scenario-axis-carriers-and-coverage.md) 先例**以「现适用」注记读作同一拓扑**；归属 **W8 冻结前**统一以注记方式处置。
- **未改**：契约冻结 9 消息 / 服务 ABI、唯一执行口语义、世界物理几何逐值、Runbook [#status](../ROS2迁移步骤.md#status) 完成状态（W0–W6 ✅ / W7·W8 ☐）。
