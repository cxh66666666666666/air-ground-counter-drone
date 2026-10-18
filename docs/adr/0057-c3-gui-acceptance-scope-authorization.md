# C3 GUI 验收范围裁定：定性验收与规模统计分开授权（X-02）

## Status

accepted

## 背景

评审挑战 **X-02**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-02)）针对契约
[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C3** 的原文字「实证/验收**使用非无头（GUI）**；
默认仍无头以向后兼容」提出：

1. 现 [campaign 产物](../../experiments/w6-task-eval/20261014-campaign/report.json) 明确标注为 **headless**；
2. W7 规模训练与多 seed 统计若**逐次 GUI**，成本高且可能拖累 RTF；
3. 但**不能静默**把 headless PASS 当作 C3 验收（现已由
   [ADR-0040](./0040-full-chain-coverage-matrix-and-same-run-causal-record.md) (d) 如实标注 backend/mode 缓解，
   **但其明确声明 X-02 的裁定未在彼处消解**）。

即：C3 的文字范围（"每份实证都要 GUI"）与"规模统计用 headless 批跑"的现实需求之间存在**未裁定的冲突**。

**先学后做**：本项目同域既有做法为**观察方式分离**——[ADR-0040](./0040-full-chain-coverage-matrix-and-same-run-causal-record.md)
以 **backend / mode 标签 + pass 命题化**把「轻量 headless PASS」与「整链 / C3 GUI 验收」显式区分；同版本 Nav2 生态亦以
`gui` / `headless` 两种启动模式并存（`nav2_bringup` 的 `slam_launch`/`bringup` 皆可 headless 运行）。故合理方向是
**按观察目的分层**，而非取消 GUI 验收或放弃规模统计。

分类：**更优路线（挑战既定约束）**。本 ADR 记录**已获用户批准的裁定**（含替代语义 + 迁移成本 + 验收方法）；
契约 C3 文字变更**经用户显式批准后**落于契约真源（[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)），**非**静默改。

## 决策（已采纳 (b)）

裁定采纳 **(b) 分层授权**：

- **(a) 现状基线（不采纳为终态）**：C3 字面 = 每份实证 / 验收都须 GUI；headless 产物只作辅助证据
  （即 [ADR-0040](./0040-full-chain-coverage-matrix-and-same-run-causal-record.md) 当前的边界记录）。
  **问题**：W7 规模统计与 C3 持续冲突，且每次裁定都要"绕过"。
- **(b) 分层授权（已采纳）**：
  1. **定性 / 关键验收**（每种拓扑、每个关键场景）**仍须 GUI** 审阅并留记录 → **C3 的价值保留**；
  2. **规模数值统计**（多 seed / 多场景批量）**允许 headless**，但**前提**是已完成
     **GUI ↔ headless 等价核验**（同参数 / 同 seed / 关键指标一致）；
  3. 所有产物**必须**标 `backend` / `mode(GUI|headless)` 标签并在报告引用时**显式区分**；
  4. **不得**把 headless 产物当 C3 定性验收材料。
- **(c) 全面 headless 化**（**被否**）：取消 GUI 验收。**理由**：与 C3 立意冲突（可观察性本身是验收目的）。

**契约处置（已执行）**：已按 (b) 改契约 [#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) 的 **C3** 文字（**唯一约束家**），并登记于契约 [#decisions](../ROS2迁移交接文档.md#decisions)。**约束原文以契约真源为准，本 ADR 不复述**。

## Considered Options

- **维持现 C3 文字 + ADR-0040 边界记录** —— 可行但未解 X-02（规模统计长期与 C3 冲突）。
- **改 C3 文字为「定性 GUI + 规模 headless（须等价核验）」** —— **推荐方向**（需用户批准；本 ADR 仅给候选文本，未落改）。
- **全面 headless 化、取消 GUI 验收** —— 被否：与 C3 立意冲突，且削弱"可观察验收"。

## Consequences

- **不删除 GUI 验收价值**：定性 / 关键场景仍须 GUI 记录；headless 只服务规模统计。
- **产物纪律**：campaign / episode 产物必带 `backend` / `mode` 标签（承接 [ADR-0040](./0040-full-chain-coverage-matrix-and-same-run-causal-record.md) (a)）。
- **裁定前**：现 headless 产物**仅作辅助证据**，C3 仍以 GUI 材料为准（不静默改变现状结论）。
- **迁移成本：低至中**：契约文字裁定 + 报告 backend/mode 标签 + 两模式**配对验收**；不需改 PX4 链路、包边界或冻结 ABI。
- **只读边界**：本 ADR 只读 [10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-02) 与契约；未读 `reference/`；未改任何源码 / 契约。

## 验收方法（裁定后）

1. 每份被声称 **C3 验收** 的场景有 **GUI 记录**（同 run 可复核）；
2. 至少一组 **GUI ↔ headless 配对 run**（同参数 / 同 seed），关键指标一致，作为等价核验证据；
3. 产物 `backend` / `mode` 标签齐全，引用处显式区分，**无**"headless PASS 冒充 C3 验收"。
