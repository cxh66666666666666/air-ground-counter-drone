# 文档真源分层与稳定锚点引用

## Status
accepted

## 背景与决策
每类信息只归属一个"家"：契约真源 = 《交接文档》；执行编排 = 《步骤文档》（Runbook）；per-workstream 规格 = `.trae/specs/<ws>/{spec,tasks,checklist}.md`；不可逆决策 = `docs/adr/`；术语 = `GLOSSARY.md`。跨文档引用**一律使用稳定锚点**（`<文件>#<锚点>`），**禁止**用 `§章节号`；`§` 编号只保证本文件内自洽。

## Considered Options
- 单一大文档承载全部内容 —— 被否：已实测出现 §号交叉引用错乱与重复内容各自漂移。

## Consequences
- 重排章节不再使跨文档引用失效。
- 《步骤文档》对硬约束只引用《交接文档》，不复述（避免双真源）。
