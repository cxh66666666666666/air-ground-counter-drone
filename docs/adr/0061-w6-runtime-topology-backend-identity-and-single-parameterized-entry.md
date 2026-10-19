# W6 运行拓扑 backend 身份与单一参数化入口

## Status
accepted

## 背景与决策

结构评审 [S-05](../../docs/reviews/agc-review-2026-10-14/20-结构.md#s-05)（**用户裁定 · 正式修订**）指出：`m0_skeleton` / `w6_task_eval` / `w6_full` **重复组装**——后两者各自内联一份**完全相同**的场景配置解析（base/registry/carriers 加载、carrier 校验、参数合并、轴断言、manifest 生成），且参数路径分叉（`full` 另用 `air.yaml`）；`backend` 未贯穿 summary；`launch` 身份字段与**入口名**分离；关键段启动依赖由**绝对时间 `TimerAction`**（12 / 25 / 35 s）承载。后果：配置漂移不易发现、拓扑未成为可审查实体。契约真源：[#params](../ROS2迁移交接文档.md#params)（launch 单一设置 / 节点不得覆写）、[#execution-port](../ROS2迁移交接文档.md#execution-port)、[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C1–C10）、[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)。

**裁定**（**不**改冻结 ABI / 唯一执行口语义 / [Runbook #status](../ROS2迁移步骤.md#status)）：

1. **单一参数化入口**：`agc_task` 顶层 W6 入口 **统一为** [`w6.launch.py`](../../src/agc_task/launch/w6.launch.py)（`backend` ∈ `lightweight` | `full`，默认 `lightweight`）。原 `w6_task_eval.launch.py` / `w6_full.launch.py` **退役**；[`m0_skeleton.launch.py`](../../src/agc_task/launch/m0_skeleton.launch.py) **保留**为骨架基座（被 `backend:=lightweight` include）。
2. **共享配置解析模块为单一真源**：抽取 [`w6_composition.py`](../../src/agc_task/launch/w6_composition.py)（`load_scene` / `carrier_entry` / `coverage_status` / `compose_params` / `assertions` / `build_manifest` / `write_temp_params` / `write_manifest`）；入口**不复述**解析逻辑。约束类信息**只引用不复述**（真源见契约）。
3. **backend 能力表**：`BACKENDS`（`w6_composition.py`）每 backend 声明 `air_link` / `px4` / `target_source` / `required_nodes`（完整展开节点）/ `supported_axes`；`not_carried` 状态由能力表**派生**（`entry.requires_air_link and not BACKENDS[backend].air_link`），**不再**逐文件硬编码「轻量栈无空中链路」。`lightweight`：`air_link=px4=False`、`target_source=gt_adapter`、支持轴仅 `obstacle`/`outcome`；`full`：`air_link=px4=True`、`target_source=air_fusion`、含空中链路 4 节点与 `intruder_driver`、支持全部 4 轴。
4. **入口名 == 报告身份**：manifest 记 `launch='w6.launch.py'` + 实际 `backend` + `nodes`（完整展开节点名）+ 生效参数（`scene`/`seed`/`support_status`/`declared_axes`/`expected_terminal`/`expected_reason`/`carriers`/`effective_params`/`assertions`）。runner / probe 报告的 `launch` 字段同步为 `w6.launch.py`。
5. **就绪依赖替代盲计时**：`intruder_driver` 由 **`OnProcessStart(target_action=<入侵者 PX4 ExecuteProcess>)`** 事件驱动（其自身已就绪门控：无 odom 先 no-op），**移除** 35 s `TimerAction`。己方 PX4（12 s）与入侵者 PX4（25 s）**保留** `TimerAction` 并在代码旁注明理由——二者属 **gz 引擎 spawn 时序**（世界 / 入侵者模型于 gz+3 s spawn，PX4 才具 gz 资源与时钟 / 可 attach 模型），本层**无可达就绪信号**，故保留最小计时。

## Considered Options

- **保留多入口、仅各自抽共享 helper** —— 被否：仍有第二份入口组装与 `backend` 身份分叉，未消解「入口名 != 报告身份」，拓扑仍非单一可审查实体。
- **全部启动依赖改就绪等待** —— 不可行 / 被否：`launch_ros` **无**内置「等话题 / 服务」wait 动作；PX4 与 gz 引擎 spawn 属引擎侧时序、本层无可达就绪信号（被 spawn 的进程本身即目标），强行等待会引入伪信号。
- **把 `backend` 仅作顶层变量、不落能力表** —— 被否：违反「`not_carried` 由能力表派生」，且覆盖状态会退回逐文件硬编码。
- **继续用绝对时间承载 `intruder_driver`** —— 被否：该依赖有**可达就绪信号**（进程 start 事件 + 节点自门控），盲计时会引入不必要的时序脆弱。

## Consequences

- **代码**：[`w6_composition.py`](../../src/agc_task/launch/w6_composition.py) 为共享解析单一真源；[`w6.launch.py`](../../src/agc_task/launch/w6.launch.py) 按 `backend` 分派组装（`lightweight` = include `m0_skeleton` + `capture_adjudicator` + `jammer`；`full` = 双 PX4 整链）；`w6_task_eval.launch.py` / `w6_full.launch.py` 删除。临时参数 / manifest 文件名含 backend 与唯一性（`w6_<backend>_<scene>_<seed>.yaml` / `w6_<backend>_manifest_<scene>_<seed>.json`）。
- **行为**：`lightweight` / `full` 除入口名 / 身份 / 就绪改动外**等价**；`intruder_driver` 改事件驱动后其启动不早于入侵者 PX4（`≈25 s`），与旧 35 s 相比**不晚**、且随实际就绪，行为等价或更稳。
- **同步更新**：引用旧入口名的脚本 / 核查器（`scripts/run_w6_*.sh` / `episode_runner.py` / `w6_r0*_check.py` / `w6_r025_full_probe.py` / `w6_full_check.py`）、config 注释、ADR（[0029](./0029-spawn-clearance-entity-pairs-and-boundary.md) / [0031](./0031-intruder-episode-start-protocol.md) / [0034](./0034-failsafe-factor-separation-and-runtime-param-lock.md) / [0040](./0040-full-chain-coverage-matrix-and-same-run-causal-record.md) / [0041](./0041-scenario-axis-carriers-and-coverage.md) / [0049](./0049-lifecycle-repeatable-resource-state-machine-and-liveness-detection.md)）**就近加中性注**（历史结论正文不改）。
- **冻结不变**：9 消息 / 服务 ABI、唯一执行口语义（**无**第二 ROS 命令者）、硬约束 C1–C10、[Runbook #status](../ROS2迁移步骤.md#status) 均**不改**。
- **未决 / 后续**：gz 引擎 spawn 段落（12 / 25 s）的就绪信号若将来由引擎侧提供可达事件，须回归就绪依赖；`intruder_driver` 事件驱动的实跑时序以整链实跑证据复核。
