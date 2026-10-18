# campaign 归属与临时文件唯一（S-07）

## Status

accepted

## Date

2026-10-18

## 背景

结构评审 [S-07](../../docs/temp/agc-review-2026-10-14/20-结构.md#s-07)（运行产物规范）指出：[实验产物规范](../../docs/conventions/实验产物规范.md)
仍写 `run_id = <YYYYMMDD>-<scenario>-<seed>`（**已不可唯一**），而
[ADR-0044](0044-unique-run-identity-and-reproducible-evidence.md) 已实装唯一 run 身份；
[episode_runner](../../scripts/episode_runner.py) campaign `run_id = <YYYYMMDD>-campaign`（同日覆盖）且
campaign 报告直写 run 根（与 `config/` / `metrics/` 分工不同）；[w6_composition](../../src/agc_task/launch/w6_composition.py)
临时参数 / manifest 文件仅按 `scene`+`seed` 命名，缺唯一 run 标记（并发 / 重复 run 互相覆盖）。

产物纪律真源：[ADR-0007](0007-experiment-artifacts-convention.md)（唯一落点）；运行身份真源：
[ADR-0044](0044-unique-run-identity-and-reproducible-evidence.md)。契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C9**、
[#params](../ROS2迁移交接文档.md#params)。

分类：**结构问题与更优路线**。本 ADR 只对**campaign 归属与临时文件唯一**做决定，**不**改契约冻结 9 消息 ABI，
**不**改 C9 指标语义 / 三层验收，**不**改 Runbook `#status`。

## 决策

### (a) 规范同实装

[实验产物规范](../../docs/conventions/实验产物规范.md) 与 [experiments/README](../../experiments/README.md) 同步到
[ADR-0044](0044-unique-run-identity-and-reproducible-evidence.md) 的唯一 run 身份（只索引不复述约束）。

### (b) campaign run_id 唯一

[episode_runner](../../scripts/episode_runner.py) campaign `run_id` 改为 `<YYYYMMDD>-<campaign>-<HHMMSS>-<rand4>`；
同日同 campaign 连跑落到不同目录，前次报告仍可复核。

### (c) campaign 报告归 `metrics/` 且引用每 run

campaign 汇总报告 `report.{json,md}` 落 campaign run 的 `metrics/`；报告含 `run_references`，逐 episode
引用其不可变产物（`run_id` + `config/scene_manifest.json` / `config/params_applied.yaml`）。

### (d) 临时文件带唯一标记

[w6_composition](../../src/agc_task/launch/w6_composition.py) `write_temp_params` / `write_manifest`
（及派生障碍 / 地图、空中参数临时文件）文件名追加唯一 token（时刻 + 进程 + 随机），避免并发 / 重复 run 互相覆盖。

## 备选

1. **保持规范旧格式**：最简；但覆盖风险不止实现犯错（规范自身不可表达唯一身份）——否决。
2. **campaign 报告留 run 根**：简单；但 campaign 与 episode 混层、缺规范归属——否决（归 `metrics/` 并引用每 run）。
3. **临时文件保持 `scene`+`seed`**：省事；但多 worker / 重复 run 互相覆盖——否决（唯一 token）。

## 后果

- **正面**：规范可表达唯一身份；campaign 与 episode 分层清晰；临时文件不互相覆盖；报告可追溯每 run。
- **代价**：依赖固定临时文件名的既有 shell 脚本（`run_w6_r026.sh`）与读 campaign 报告的 check 脚本
  （`w6_r025_check.py` / `w6_r026_check.py` / `w6_r029_check.py` / `w6_r031_check.py`）报告路径同步更新到 `metrics/`。
- **不**改 Runbook `#status`；**不**改契约冻结 9 消息 ABI。

## 参考

- 评审：[20-结构 S-07](../../docs/temp/agc-review-2026-10-14/20-结构.md#s-07)
- 规格：[fix-structure-geometry-evidence-toolchain-s06-s10 spec](../../.trae/specs/fix-structure-geometry-evidence-toolchain-s06-s10/spec.md)
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 运行身份：[ADR-0044](0044-unique-run-identity-and-reproducible-evidence.md)
- 实现：`episode_runner.py` / `w6_composition.py` / `w6.launch.py`
