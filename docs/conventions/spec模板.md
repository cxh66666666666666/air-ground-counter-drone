# spec 三件套模板

> 每个 workstream 须具备 `.trae/specs/<ws-id>-<slug>/{spec.md,tasks.md,checklist.md}`，由本模板生成。
> 真源分层见 [ADR-0002](../adr/0002-docs-source-of-truth-layering.md)：约束只引用契约真源，不复述。
> **进度真源**：唯一进度真源为 Runbook [状态表](../ROS2迁移步骤.md#status)；三件套**不**维护第二状态表——如需标注执行进度，一律**引用**该锚点，**不**凭推断改写完成状态（[ADR-0066](../adr/0066-decision-applicability-and-reference-discipline.md)）。
> **引用纪律**：跨文档一律用**稳定锚点**可点击链接（`<文件>#<锚点>`），**禁** `§` 章节号；相对路径按三件套所处层级（`.trae/specs/<ws>/` → 项目根为 `../../../`）。

---

## 占位符说明（单列）

| 占位符 | 含义 | 示例 |
|---|---|---|
| `<WS>` | workstream 编号 | `W3` |
| `<名称>` | workstream 名称 | 空中链路 |
| `<ws-id>-<slug>` | 规格目录名 | `w3-air-link` |
| `<锚点>` | 契约真源的稳定锚点名 | `hard-constraints` |
| `<#>` | 需求编号 | `R1` |

> 占位符一律**单列**于上表，模板正文只按名引用，避免散落后无从穷举。

---

## spec.md 模板

```markdown
# <WS> · <名称> — Spec

> **依据**：契约真源 [<锚点>](../../../docs/ROS2迁移交接文档.md#<anchor>)；相关 ADR。
> **上游**：Runbook 的 <WS> 卡片（[#workstreams](../ROS2迁移步骤.md#workstreams)）；进度以 [#status](../ROS2迁移步骤.md#status) 为**唯一真源**。
> **原则**：本 spec 从零撰写，不参考归档旧树的任何 spec / 任务 / 清单。

## Why
<为什么需要本 workstream、不做的后果>

## What Changes
<产出与影响面>

## Impact
<新增/改动，以及不改动什么（尤其：不改动归档旧树）>

## Requirements
### R1 · <需求>
<正文，含可命令化判据>

## Nav2 吸收点   <!-- 强制节：非空 -->
<按 [ADR-0008](../../../docs/adr/0008-nav2-paradigm-absorption-as-code-principle.md) 吸收矩阵逐项列出，
并标注每项来源：`全新重写` / `对齐旧树行为语义（附稳定锚点）` / `直接复用 Nav2 组件`。
未采纳的范式须写明理由。>

## 只读旧树边界   <!-- 强制节：非空 -->
- **允许读**：旧树（`reference/`）以对齐**行为 / 契约事实**（WHAT）。
- **禁止**：复制旧代码；继承其格式、命名、目录、任务粒度（HOW）。
- **不变量**：读旧码对齐行为是**硬性要求**，不是可选项；产出代码一律用新范式重写。

## 非目标
<本 workstream 明确不做什么>
```

---

## tasks.md 模板

```markdown
# Tasks · <WS> <名称>

> 依据 [spec.md](./spec.md)。完成一项即在 checkbox 打勾，并保留可核查证据。

- [ ] Task 1：<标题>（关联 R#）
  - [ ] 1.1 <子项>

# Task Dependencies
<依赖关系>

# 证据索引
（执行后回填：命令与输出摘要）
```

---

## checklist.md 模板

```markdown
# Checklist · <WS> <名称>

> 逐项核查 [spec.md](./spec.md) 与 [tasks.md](./tasks.md)。核查方式以**可执行命令**为准。
> **进度**：唯一进度真源 = Runbook [#status](../ROS2迁移步骤.md#status)；本清单**不**另立第二状态表。

## R<#>
- [ ] <可命令化判据>

## 通用红线（所有 workstream 适用）
- [ ] 若发现任一硬约束（契约 [hard-constraints](../../../docs/ROS2迁移交接文档.md#hard-constraints)）未被满足，已**停止并报告**，未带错继续推进
```