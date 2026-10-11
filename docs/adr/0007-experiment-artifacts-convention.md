# 实验产物唯一落点与命名规范

## Status
accepted

## 背景与决策
所有实验产物只落一个根，按 campaign → run 两层组织：

```
experiments/<campaign>/<run_id>/
  ├── config/     本次参数快照（必需，可复现）
  ├── bag/        ros2 bag
  ├── log/        运行日志 / run logger 输出
  ├── metrics/    结构化指标（含 shield_infeasible_rate、reset_failure_rate）
  └── figures/    出图
```

`run_id = <YYYYMMDD>-<scenario>-<seed>`；每次 run 必须落 config 快照 + git commit hash。`experiments/` 整体 git 忽略，仅 `experiments/README.md` 与 `experiments/**/config/` 入库。**禁止**在其它任何位置写实验产物。

## Considered Options
- 产物就近散落（旧项目现状）—— 被否：旧项目实验目录错乱是本项目明确要避免的反面教材。

## Consequences
- 需在 W0 定义 `.gitignore` 例外规则，使 `experiments/**/config/` 可入库而其余忽略。
- run 元数据须含 git commit hash —— 前提是项目根为 git 仓库（见 W0 前置项）。
