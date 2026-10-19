# W7 入口与出口定义：平台出口、复用当前空地仿真栈、前置以可复现证据为准

## Status

accepted

## Date

2026-10-19

## 背景

评审 [F-01](../reviews/agc-review-2026-10-14/30-未来.md#f-01) 指出：Runbook 依赖表把 **W7 前置写作仅 `W2`**（[#status](../ROS2迁移步骤.md#status)），而现 RL 主控链实际消费 **W4**（目标 / 感知）、**W5**（安全 / 控制）、**W6**（状态与 reset）——前置**未反映真实拓扑**；建议先定义 W7 可运行最小闭环的**依赖与出口**，且每个依赖**指向验收证据**。同段另有三条被本 ADR 一并裁定的相关意见：[F-04](../reviews/agc-review-2026-10-14/30-未来.md#f-04)（obs 版本化）、[F-05](../reviews/agc-review-2026-10-14/30-未来.md#f-05)（train/eval adapter）、[F-07](../reviews/agc-review-2026-10-14/30-未来.md#f-07)（种子分解）、[F-08](../reviews/agc-review-2026-10-14/30-未来.md#f-08)（并行隔离）、[F-15](../reviews/agc-review-2026-10-14/30-未来.md#f-15)（架构演进触发）。

本轮修复已把 41 项风险固化为决策（[ADR-0025](0025-gz-side-terminal-failstop-deadman-plugin.md)–[ADR-0068](0068-third-party-asset-provenance-license-and-hash-manifest.md)），`fix-*` 规格 checklist 多标"已核查"（例 [fix-episode-reset-intruder-r13-r17](../../.trae/specs/fix-episode-reset-intruder-r13-r17/checklist.md)）。但按评审自身判据（[00-总览](../reviews/agc-review-2026-10-14/00-总览.md#judgment)）——"**消息已发 ≠ 效果已发生**"、"**实现已改 ≠ 证据已复现**"——W7 前置**不能**以"代码 / ADR 已记录"为满足。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C1–C10 **不**改）。范围：W7 / W8 归"项目结束后后续"（[ADR-0016](0016-execution-scope-exit-w6.md)）；研究主线为**系统论文**、RL 为手段（[ADR-0014](0014-research-mainline-mobile-interception-system.md)）。

## 决策

### (a) W7 第一个出口 = 可信实验平台，不是算法成果

W7 第一阶段（**W7-a 最小平台**）产出"**语义正确、可复现、可被训练的环境 + 一致的 train / eval 协议**"，验收锚 = "同 seed 可复现 / 指标齐全"（Runbook [W7 卡片](../ROS2迁移步骤.md#workstreams)）。**RL 算法、网络、奖励、训练步数、论文问题不在 W7 入口出口内**，属 W7-b（[F-02](../reviews/agc-review-2026-10-14/30-未来.md#f-02)：先定可证伪命题，并**先用规则策略证明任务非平凡且可达**）。

### (b) 复用当前空地仿真栈，不另建训练环境

训练环境 = 现 **W2-full + W4 + W5 + W6** 拓扑；**不建**独立简化训练环境。若需 headless / 降配，作为**同一栈的参数化"训练视图"**，差异写入 W7 spec。理由：评审最致命一类风险正是 **train / eval 语义分叉**（[R-010](../reviews/agc-review-2026-10-14/10-风险.md#r-010)、[R-032](../reviews/agc-review-2026-10-14/10-风险.md#r-032)）；另建环境须补一套转移验证、成本高且易漏。

### (c) 前置以可复现 run 证据为准

W7 spec 的每条前置依赖 **SHALL 指向 `run_id` + 验收命题**，**不**以"代码 / ADR 已记录"为满足。

### (d) W7-a 最小平台 = 四件套

| 项 | 内容 | 依据 |
|---|---|---|
| obs / action 版本化 | `obs_version` + 归一化 + checkpoint 元数据 + 旧版本回放 | [F-04](../reviews/agc-review-2026-10-14/30-未来.md#f-04) |
| train / eval 终止与预算 | 复用 [ADR-0039](0039-time-budget-kinds-and-rtf-record.md) 三类口径；超时一律失败、infra 单列不吞 | [F-05](../reviews/agc-review-2026-10-14/30-未来.md#f-05) |
| 种子分解 | spawn / sensor / channel / target / policy / physics 分离 + 派生规则 | [F-07](../reviews/agc-review-2026-10-14/30-未来.md#f-07) |
| 单 worker 可重放 | 先单 worker；多 worker 隔离属 W7-c | [F-08](../reviews/agc-review-2026-10-14/30-未来.md#f-08) |

### (e) 单 worker 优先

并行 / 向量化隔离**不在** W7 入口范围；触发条件 = "**单 worker 可靠且确需吞吐**"（[F-08](../reviews/agc-review-2026-10-14/30-未来.md#f-08)）。

### (f) 暂不重构架构

包边界**不动**（[F-15](../reviews/agc-review-2026-10-14/30-未来.md#f-15)）；仅在**真实异步裁决（火控发射 / 命中）**或**多 worker 资源管理**出现时，按职责分管理、必要时增包。

### (g) 落地物

本 ADR 定性 → [`.trae/specs/w7-rl/`](../../.trae/specs/w7-rl/spec.md) → Runbook 依赖表（W7 前置）与 [W7 卡片](../ROS2迁移步骤.md#workstreams) 更新。

## 备选

1. **W7 直接绑定算法 / 论文成果** —— 否决：主线是系统论文、RL 为手段（[ADR-0014](0014-research-mainline-mobile-interception-system.md)），会把未定科研问题写成**隐式 ADR**。
2. **另建专用训练环境** —— 否决：语义分叉 + 转移验证成本高。
3. **前置以"ADR 已记录"为准** —— 否决：与评审判据矛盾（"实现已改 ≠ 证据已复现"）。
4. **现在就 vectorize / 重构包边界** —— 否决：成本高、收益未证（[F-08](../reviews/agc-review-2026-10-14/30-未来.md#f-08) / [F-15](../reviews/agc-review-2026-10-14/30-未来.md#f-15)）。

## 后果

- **正面**：W7 前置**显式、可判定、与真实拓扑一致**；修正 Runbook 依赖表漂移。
- **代价 / 限制**：W7-a 先出**证据**、不出算法；科研问题仍**空位**。
- **未决 / 后续**：算法 / 环境细节 / 指标 / 论文方向（W7-b）；火控升级（[F-11](../reviews/agc-review-2026-10-14/30-未来.md#f-11)）先于捕获奖励；多 worker（W7-c）。
- **不改**：契约 C1–C10、唯一执行口语义、W7 / W8"后续"地位（[ADR-0016](0016-execution-scope-exit-w6.md) 不改）。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码（HOW）。

## 参考

- 评审：[F-01](../reviews/agc-review-2026-10-14/30-未来.md#f-01)、[F-02](../reviews/agc-review-2026-10-14/30-未来.md#f-02)、[F-04](../reviews/agc-review-2026-10-14/30-未来.md#f-04)、[F-05](../reviews/agc-review-2026-10-14/30-未来.md#f-05)、[F-07](../reviews/agc-review-2026-10-14/30-未来.md#f-07)、[F-08](../reviews/agc-review-2026-10-14/30-未来.md#f-08)、[F-11](../reviews/agc-review-2026-10-14/30-未来.md#f-11)、[F-15](../reviews/agc-review-2026-10-14/30-未来.md#f-15)（[30-未来](../reviews/agc-review-2026-10-14/30-未来.md)）
- Runbook：[#status](../ROS2迁移步骤.md#status)、[#workstreams](../ROS2迁移步骤.md#workstreams)
- 规格：[w7-rl spec](../../.trae/specs/w7-rl/spec.md)
- 前序：[ADR-0004](0004-rl-primary-controller-nav2-safety-layer.md)、[ADR-0011](0011-node-language-policy.md)、[ADR-0014](0014-research-mainline-mobile-interception-system.md)、[ADR-0016](0016-execution-scope-exit-w6.md)、[ADR-0039](0039-time-budget-kinds-and-rtf-record.md)
