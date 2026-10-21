# 公开仓库发布边界：`.trae` 与 `experiments` 不入版本库

## Status

accepted

## Date

2026-10-21

## 背景

项目已推送到 GitHub 公开仓库。首轮推送包含：

- `.trae/`（73 文件 / 2.6 MB）：编辑器 / Agent 本地工作流与 workstream 规格，含归档清单；
- `experiments/`（4625 文件 / **152 MB**）：逐 run 的 `config/` 快照、`git_diff.patch`、
  `git_untracked.txt`、机器指纹与绝对路径。

这些内容属于**本机研发过程物**，不是可复现构建所需的源码 / 文档；推入公开仓库会显著增大克隆体积、
暴露本机路径与内部规格，且与“实验产物唯一落点 `experiments/`”的本地证据纪律并不冲突
（证据留在本机，必要时以 Release / 归档包分发）。

## 决策

### (a) `.gitignore` 排除

```gitignore
/.trae/
/experiments/
```

替换原先“`experiments/*/*/*` 忽略、但放行 `config/`”的规则；今后实验证据**不再随 git 推送**。

### (b) 历史重写

首轮推送已包含上述路径；用 `git filter-branch --index-filter` 从**全部历史**中删除
`.trae` 与 `experiments`，清理 `refs/original` / reflog / 不可达对象后
**force-push** `main`。仓库刚建、无协作者，历史重写风险可控。

### (c) 保留内容

源码（`src/`）、脚本（`scripts/`）、测试（`tests/`）、文档（`docs/`、`GLOSSARY.md`、
`AGENTS.md`、Runbook）、`toolchain/README.md`、模型资产（如 `wheel.stl`）继续入库。

## 备选

1. **只新增停止跟踪提交** —— 否决：GitHub 历史提交仍可见，未达到“不推送”目的。
2. **保留 experiments 的 config 快照** —— 否决：152 MB 且含机器指纹/绝对路径；公开仓库不需要。
3. **Git LFS 存 experiments** —— 否决：当前无跨机器分发需求；需要时用 Release 资产更直接。

## 后果

- **正面**：公开仓库只含源码 / 文档，克隆体积从 ~150 MB 降到源码量级；本机路径与内部规格不外泄。
- **代价 / 限制**：公开 clone **不含** `.trae/specs/`，因此 `AGENTS.md` / 部分文档中指向
  `.trae/specs/...` 的引用在公开仓库中不可用；规格随本地工程分发。
- **证据**：实验 run 的 `config/`、`metrics/`、`log/` 仍全部保留在**本机** `experiments/`；
  需要公开时以 Release 附件 / 归档包提供。
- **可复现性**：源码 + 参数（`src/*/config`）+ ADR + Runbook 足以重建实验；逐 run 的机器指纹不再公开。

## 参考

- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 证据归档：[ADR-0064](0064-evidence-immutable-archive-and-intake-manifest.md)
- 入口索引：[`AGENTS.md`](../../AGENTS.md)
