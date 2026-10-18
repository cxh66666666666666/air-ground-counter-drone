# 工具链可重建 pin 清单与一致性校验（S-09）

## Status

accepted

## Date

2026-10-18

## 背景

结构评审 [S-09](../../docs/temp/agc-review-2026-10-14/20-结构.md#s-09)（**工具链可重建性**，高）指出：
[`toolchain/`](../../toolchain/README.md) 内容**不入 git**（[.gitignore](../../.gitignore) 仅放行
`/toolchain/README.md`），而 README 原先只有**版本 tag + 局部配方**，
**无**源 SHA / submodule / 补丁与兼容输入来源 / 依赖版本 / 构建选项 / **产物 binary 哈希**清单。

后果：新机器可能**拿不到必要兼容输入**（如 AGC 自研 `fmt_compat.hpp`、离线 `localdeps/`），
或**产出不同 DDS / PX4 行为**（agent 与 ROS 2 的 Fast DDS / fastcdr 版本必须同源，见
[`toolchain/README.md`](../../toolchain/README.md#uxrce-dds-agent-版本必须与-ros-2-的-dds-栈同版本)）；
且「**已有机器构建成功**」**不**等于环境可从空目录独立重建 —— 复制现有 `build/` 不构成重建证明。
另：[ADR-0012](0012-px4-body-as-toolchain-dependency.md) 历史正文记 agent `v2.4.2`，README / Runbook 现行为 `v2.4.3`，
版本与来源**未锁定**。

契约真源：[#decisions](../ROS2迁移交接文档.md#decisions)（**PX4 本体形态 = `toolchain/` 外部依赖（pin `v1.15.4`）；
`px4_msgs` 内联 `src/`**）、[#messages](../ROS2迁移交接文档.md#messages)（冻结 9 消息 ABI，工具链变更**不**触碰）、
[#params](../ROS2迁移交接文档.md#params)（单一注入）；上游决策 [ADR-0012](0012-px4-body-as-toolchain-dependency.md)
（PX4 本体作 `toolchain/` 外部依赖）/ [ADR-0005](0005-px4-link-uxrce-dds.md)（uXRCE-DDS + `px4_msgs`）；
复现步骤见 Runbook [附录 B](../ROS2迁移步骤.md#appendix-b)。**保留**外部依赖决定：
**不**回退 agent 2.4.2、**不**引入 MAVROS。

## 决策

### (a) `toolchain/README.md` = 唯一入库的可重建 pin 清单

`toolchain/` 其余内容被忽略，因此 pin 值**只能**写在**该 README 内**（写到别处即随忽略丢失）。
README 除人读摘要表外，含一个 **fenced ```yaml 机器可读块**（`agc_toolchain_pins:`，`schema: 1`），键：

| 键 | 内容 |
|---|---|
| `git_repos` | PX4（`tag` `v1.15.4` + `head` + **9 个 SITL(gz) 必需子模块逐条 SHA**）、uXRCE-DDS-Agent（`tag` `v2.4.3` + `head`） |
| `artifacts` | 逐条 `sha256`：`bin/MicroXRCEAgent`（包装脚本）/ `build_ros/MicroXRCEAgent`（agent 真实二进制）/ `build/px4_sitl_default/bin/px4`（SITL）/ `microxrce_compat/fmt_compat.hpp`（**AGC 自研兼容头**，非上游文件） |
| `build_options` | PX4 `config=px4_sitl_default` + `config_file=boards/px4/sitl/default.px4board` + `RelWithDebInfo`；Agent `build_ros` + 系统 FastDDS/FastCDR/Logger、`Release`、`-include` 兼容头 |
| `dds_versions` | ROS 侧 `fastrtps` / `fastcdr`；host `spdlog` / `fmt`；`localdeps/sysroot` 的 `spdlog` / `fmt` |
| `localdeps_debs` | `localdeps/debs/` 逐包 `sha256`（离线 `.deb` 的来源与哈希） |

### (b) 新增 `scripts/toolchain_pin_check.py`（只读，记录值 vs 实测）

解析上述 yaml 块并**逐项比对实测**：产物 `sha256`、仓库 `HEAD` / `tag`、子模块 SHA（区分
`missing` / `not_initialized`）、依赖版本（`dpkg-query` + sysroot 头文件宏）、离线包哈希**集合**双向差集。
**缺失即 FAIL**（不静默忽略）；`dpkg-query` 不可用记 **SKIP** 而非 PASS。
`ok=false` 即**退出码 1**。任何不一致**必须显式报出**，不得以「环境差异」为由放行。

### (c) 「从空目录恢复」配方 + 离线包访问方式

README 增「从空目录恢复（配方）」节（克隆 tag + 初始化 9 子模块 → PX4 SITL 构建 → agent 构建 →
入口包装脚本 → 离线 `.deb` → **运行期环境** `PX4_GZ_STANDALONE` / `PX4_GZ_WORLD=agc_world` /
`PX4_SIM_MODEL=gz_x500` / `GZ_SIM_RESOURCE_PATH` → 校验脚本）与「离线包访问方式」节
（GitHub 极慢走 **Gitee 镜像**；pip 走**清华源**；`localdeps/debs` 为 Ubuntu 24.04 noble amd64 离线包；
ROS 侧 DDS 直接来自 `/opt/ros/jazzy`，**无需自建**）。

### (d) 未决登记的验证缺口（**不**静默跳过）

本清单本轮**未**在**真正空目录**端到端实跑重建 —— 校验脚本只证明「**现值**与记录一致」，
**不**证明「空目录可恢复」。该缺口**显式登记为未决**（README 配方节首句 + 本 ADR），
归属：**W3 工具链复现项的可复现出口**，在 **W8 对照与冻结**前以**干净环境/独立目录**实测一次（禁止复制现有 `build/`）。
在实测完成前，任何「环境已可重建」的结论**不成立**。

### (e) 版本适用关系（与 S-10 协同）

[ADR-0012](0012-px4-body-as-toolchain-dependency.md) 正文所记 agent `v2.4.2` 为 **P0 当时适用值**；
**现行适用版本为 `v2.4.3`**（对系统 Fast DDS 2.14 构建，与 ROS 2 Jazzy 同源）。历史正文**保留**，
在 ADR-0012 顶部加**适用关系注**指向本 ADR 与 [`toolchain/README.md`](../../toolchain/README.md)；
下游**只链接现适用版本**。引用纪律：跨文档一律用**稳定锚点** `<文件>#<锚点>`，**禁止**章节号记法。

## 备选

1. **pin 清单入库为独立文件（JSON / 单独 md）**：结构更「干净」；但 `toolchain/` 内容被忽略、git 只放行该 README，
   独立文件**不入库即丢失** —— **否决**（pin 值只写在 `toolchain/README.md` 内）。
2. **把 PX4 / agent 源码 vendor 进 `src/` 或 git**：clone 即可见；但体积大、非 ROS 包、污染 colcon 工作空间，
   且 [ADR-0012](0012-px4-body-as-toolchain-dependency.md) 已裁 **外部依赖** —— **否决**（保留外部依赖决定）。
3. **只记版本 `tag`、不记 `HEAD` / submodule / binary 哈希**：写作最省；但 **tag 不等于内容锁定**
   （浅克隆 / 子模块 / 构建产物均可漂移），正是评审所指缺口 —— **否决**（tag + HEAD + submodule + 产物哈希）。
4. **复制现有 `build/` 到新机器当作「重建证明」**：可快速「看起来能跑」；但评审明确要求
   **不复制现有 build 当重建证明** —— **否决**（登记为未决，另行干净环境实测）。
5. **回退 agent 2.4.2 或改走 MAVROS**：被评审显式排除（跨版本 DDS 不互通、已裁 uXRCE 路线）—— **否决**。

## 后果

- **正面**：工具链**唯一入库索引**含源 SHA / 子模块 / 兼容输入来源 / 依赖版本 / 构建选项 / 产物哈希；
  任一处漂移可被 `scripts/toolchain_pin_check.py` **当场报出**（实测 25 项全 PASS，退出码 0）；
  离线包来源与镜像访问方式成文，新机器有可依据的获取路径；agent 版本适用关系显式化（2.4.2 → 2.4.3）。
- **代价 / 限制**：pin 清单与实机**深度耦合**（`toolchain/` 不入库，故清单本身不含被忽略内容的自校验；
  产物哈希随重新构建而变，须**显式更新 README 并重跑校验**，不得静默接受漂移）；
  「空目录重建」仍**未实测**（见 (d)），当前只证明**现值一致**。
- **未决 / 后续**：干净环境端到端重建实测（W3 复现出口 → W8 冻结前）；
  多机/CI 环境下的工具链校验接入属 W8 范围。裁决**不**改 [Runbook #status](../ROS2迁移步骤.md#status)。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[20-结构](../../docs/temp/agc-review-2026-10-14/20-结构.md#s-09) / [00-总览](../../docs/temp/agc-review-2026-10-14/00-总览.md)
- 规格：[fix-structure-geometry-evidence-toolchain-s06-s10 spec](../../.trae/specs/fix-structure-geometry-evidence-toolchain-s06-s10/spec.md)
- 前序：[ADR-0012](0012-px4-body-as-toolchain-dependency.md)（PX4 本体外部依赖）/ [ADR-0005](0005-px4-link-uxrce-dds.md)（uXRCE-DDS + `px4_msgs`）/ [ADR-0003](0003-nav2-pattern-complete-absorption.md)（Nav2 走 apt + `<depend>` 对照）
- 契约：[#decisions](../ROS2迁移交接文档.md#decisions)（PX4 本体 = 外部依赖）/ [#messages](../ROS2迁移交接文档.md#messages)（ABI 冻结）/ [#params](../ROS2迁移交接文档.md#params)
- 实现：[`toolchain/README.md`](../../toolchain/README.md) / [`scripts/toolchain_pin_check.py`](../../scripts/toolchain_pin_check.py) / Runbook [附录 B](../ROS2迁移步骤.md#appendix-b)
