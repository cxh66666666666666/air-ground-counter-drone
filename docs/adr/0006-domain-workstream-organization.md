# 重构以职责域 workstream 组织，而非时间序 P0–P6

## Status
accepted

## 背景与决策
重构按**职责域**切 workstream，每个 workstream 各带 `.trae/specs/<ws>/{spec,tasks,checklist}.md`；执行顺序由**显式依赖图**派生，不再以写死的 P0–P6 时序组织。《步骤文档》升为唯一执行入口（Runbook），按 workstream 编排。

## Considered Options
- 时间序 P0–P6（原方案）—— 被否：P0–P4 阶段根本执行不了硬约束回归，暴露"时序"与"可验证性"脱节。

## Consequences
- 每个 workstream 具备独立验收口径。
- 硬约束回归挂到"首次使该约束可验证"的 workstream，而非每阶段一律执行。
