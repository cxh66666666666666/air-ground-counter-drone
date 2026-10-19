# 证据不可变归档与入库 manifest（S-08）

## Status

accepted

## Date

2026-10-18

## 背景

结构评审 [S-08](../../docs/reviews/agc-review-2026-10-14/20-结构.md#s-08)（**证据保全**，高）指出：
git 只入库 `config/`，`bag/` / `log/` / `metrics/` / `figures/` 一律忽略（[.gitignore](../../.gitignore)），
**无**归档 manifest / 内容哈希 / 访问位置 / 保留期 → **clone 不能复核已完成状态**；
重建机器或清临时目录可**永久失去科研证据**（[ADR-0045](0045-resetfail-evidence-contradiction-registration-and-redo.md)
已登记一次真实证据丢失事故）。评审建议：无需把大 `bag/` 入 git，但应有**不可变本地归档 + 入库 manifest**
（内容哈希 / 大小 / 运行身份 / 生成器版本 / 访问位置 / 保留期），并通过**另一环境读取一次归档**验证「可取回」，
而不只写路径。

**用户裁定**：**本地不可变归档 + 入库 manifest**。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C9 必报指标）、
[#params](../ROS2迁移交接文档.md#params)（单一注入，节点不得覆写）；
产物纪律 [ADR-0007](0007-experiment-artifacts-convention.md)（唯一落点 `experiments/<campaign>/<run_id>/`）；
运行身份 [ADR-0044](0044-unique-run-identity-and-reproducible-evidence.md)（唯一 `run_id` + 展开参数 / 代码身份 /
工具链与世界哈希）。本 ADR 只对**证据归档与索引**做决定，**不**回退任何已验收组件，
**不**改契约冻结 9 消息 ABI，**不**新增第二 ROS 命令者。

## 决策

### (a) 入库 manifest 落 `config/evidence_manifest.json`（只索引不复述）

为每 run 生成 **入库 manifest** `experiments/<campaign>/<run_id>/config/evidence_manifest.json`：

- **逐产物**：`path` / `type`（`config` / `metrics` / `log` / `bag` / `figures` / `other`）/ `size` / `sha256`；
- **运行身份**：`campaign` / `run_id` / `episode_id` / `git_commit` / `backend`（缺失记 `null`，**不**以 0 冒充）；
- **生成器版本**：`generator` + `generator_version` + `manifest_version`；
- **访问位置 / 保留期**：`archive.path` + `archive.checksum_file` + `archive.retrieval_record` +
  `retention`（长期保留，≥ W8 对照冻结）。

`config/**` 已由 [ADR-0007](0007-experiment-artifacts-convention.md) 放行入库（[.gitignore](../../.gitignore)），
故 manifest 天然入库，**无需**改放行规则。manifest **不含自身哈希**（`self_excluded` 显式标注），避免自引用；
归档文件的 sha256 由侧车记录（见 (b)）。

### (b) 本地不可变归档 `experiments/_archive/<campaign>/<run_id>.tar`（只读）

- run 目录打包为 `experiments/_archive/<campaign>/<run_id>.tar`，**含** `config/evidence_manifest.json`；
- 归档成员路径以 `<run_id>/` 为前缀（便于取回还原），`uid/gid/uname/gname` 归一以利确定性；
- 归档文件 `sha256` 记入同名 `.tar.sha256`（`sha256sum` 格式，机器可校验）；
- 归档与侧车在生成后置为**只读**（`0444`），标 **保留期**（`retention`，长期 ≥ W8 冻结）；
- **大归档不入 git**：`experiments/_archive/` 加入忽略规则（[.gitignore](../../.gitignore)）。

### (c) 跨路径取回验证（证明「可取回」，不只写路径）

归档**解包到另一路径**（临时目录，`tempfile.mkdtemp`）并**逐文件哈希比对**（含 manifest 自身）；
结果落 `experiments/_archive/<campaign>/<run_id>.tar.retrieval.json`
（`verified` / `method` / `files_checked` / `mismatches` / `archive_sha256` / `verified_at`）。
`mismatches` 非空即判失败。

### (d) 保护现有证据（不覆盖既有产物）

对**现存** experiments run 生成 manifest + 归档：manifest 与归档为**新增**产物，
**不**改写 run 内既有 `params` / `summary` / `log` / `metrics`；已存在 manifest + 归档的 run 默认 **skip**
（`--force` 可显式重建）。

### (e) 引用纪律（据实登记，不伪造、不冒充复核）

R-030 / R-036 的证据丢失事实**只引用** [ADR-0045](0045-resetfail-evidence-contradiction-registration-and-redo.md)，
本 ADR / 脚本**不**复述细节、**不**伪造 / 重建已丢失原件、**不**以旧路径名冒充复核。
跨文档引用一律用**稳定锚点** `<文件>#<锚点>`；**禁止**章节号记法。

### (f) 实现与产物

新增 [`scripts/evidence_archive.py`](../../scripts/evidence_archive.py)：`--campaign` / `--run-id` 过滤、
`--force` 重建、`--no-verify` 跳过取回验证、`--list` 枚举。验收产物落
`experiments/w6-task-eval/20261018-s08-evidence-archive/{log/evidence_archive.log,metrics/s08.json}`
（[ADR-0007](0007-experiment-artifacts-convention.md)）。

## 备选

1. **把大 `bag/` / `log/` 直接入 git**：最省事、clone 即可见；但仓体膨胀、二进制不可 diff —— **否决**
   （本地不可变归档 + 入库 manifest 索引）。
2. **只写归档路径、不验证可取回**：省一步；但「写路径」不等于「能取回」，重建机器时可能发现归档损坏 ——
   **否决**（解包到另一路径逐文件哈希比对）。
3. **manifest 含自身哈希 / 归档哈希**：看似自足；但归档含 manifest → 自引用循环，哈希不可闭合 ——
   **否决**（manifest 排除自身；归档哈希与取回记录放侧车，侧车不入归档）。
4. **重建已丢失的 R-030 原件以「保护证据」**：属**伪造**历史产物 —— **否决**（据实登记，只引用 ADR-0045）。

## 后果

- **正面**：每 run 有**入库 manifest**（逐产物哈希 / 身份 / 访问位置 / 保留期）与**只读本地归档**；
  「可取回」由**跨路径逐文件哈希比对**证明，而非仅声明路径；现存 136 run 一次性补齐 manifest + 归档，
  既有产物**未被覆盖**；`bag/` / `log/` / `metrics/` 仍不入 git，仓体不受影响。
- **代价 / 限制**：`experiments/_archive/` 为**本地**归档（不随 clone 分发），跨机器复核需另移植归档；
  归档为**一次性快照**，run 后续若追加产物，其归档即**非最新**（需 `--force` 重建并**重记哈希**，
  旧归档哈希随之失效）；`--force` 重建会**改写**归档，故重建须显式、可追溯。
- **未决 / 后续**：外部（异地）归档与分发属 W8 正式冻结项（评审 [S-08](../../docs/reviews/agc-review-2026-10-14/20-结构.md#s-08)
  亦将其列 W8 出口）；「小审定摘要入 git」按项目裁定另行决定。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[20-结构](../../docs/reviews/agc-review-2026-10-14/20-结构.md#s-08) / [10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-030)
- 规格：[fix-structure-geometry-evidence-toolchain-s06-s10 spec](../../.trae/specs/fix-structure-geometry-evidence-toolchain-s06-s10/spec.md)
- 前序：[ADR-0007](0007-experiment-artifacts-convention.md)（唯一落点）/ [ADR-0044](0044-unique-run-identity-and-reproducible-evidence.md)（唯一运行身份）/ [ADR-0045](0045-resetfail-evidence-contradiction-registration-and-redo.md)（证据丢失登记）/ [ADR-0046](0046-runner-three-layer-acceptance-and-integrity-gate.md)（三层验收）
- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C9）/ [#params](../ROS2迁移交接文档.md#params)（单一注入）
- 实现：`scripts/evidence_archive.py` / `.gitignore` / `experiments/README.md` / `docs/conventions/实验产物规范.md`
