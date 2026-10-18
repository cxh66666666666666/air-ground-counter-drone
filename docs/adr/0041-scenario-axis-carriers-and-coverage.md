# 场景轴承载者校验与覆盖记账（R-026：声明轴必须有承载者，未执行轴如实标注）

## Status

accepted

> 注（S-05 / [ADR-0061](./0061-w6-runtime-topology-backend-identity-and-single-parameterized-entry.md)）：W6 顶层入口已统一为 [`w6.launch.py`](../../src/agc_task/launch/w6.launch.py)（`backend` 参数 ∈ lightweight | full），原 `w6_task_eval.launch.py` / `w6_full.launch.py` 退役；本 ADR 历史结论正文不改，下文的旧入口名按现入口读作同一拓扑。
>
> 注（S-06 / [ADR-0062](./0062-scene-geometry-single-source-and-derivation.md)）：静态障碍几何真源已**上移**为 [`agc_sim/config/world_geometry.yaml`](../../src/agc_sim/config/world_geometry.yaml)；[`agc_world.sdf`](../../src/agc_sim/worlds/agc_world.sdf) 改为**不含 `pole_*` 的 base world**，`world_bringup.launch.py` 按 `world_obstacles` 由真源**注入 / 不注入**障碍模型生成运行期世界（本 ADR 历史正文「`none` 由单一真源去除 `pole` 块生成」按此读作**真源上移后的按域派生**）。历史正文不改。

## 背景

评审风险 **R-026（场景轴未执行）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-026)）指出：
[`scenarios.yaml`](../../src/agc_task/config/scenarios.yaml) 登记 `target_motion × obstacle × perception × outcome`，
但运行参数**未实现对应实验因子**——`dynamic` 无障碍驱动覆盖；`s06/s07` 无 drop / fov 参数；
`none/static` 使用**同一世界**（[`agc_world.sdf`](../../src/agc_sim/worlds/agc_world.sdf) 恒含 `pole_1/pole_2`）；
[`w6_full.launch.py`](../../src/agc_task/launch/w6_full.launch.py) 空中节点读**独立** [`air.yaml`](../../src/agc_air/config/air.yaml)
而非场景 merged 参数；轻量栈不起 `intruder_driver`（`linear/maneuver` 不落到执行者）；且
`target_source='gt_adapter'`（真值直发）使感知通道模型（`drop_prob` / `hfov`）**不落在** `/agc/targets`
消费路径。后果：名义 12 场景可能实为**少量配置变种**；对 dropout / FOV / 动态障碍的因果结论**无效**。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C9**（必报指标）、
[#params](../ROS2迁移交接文档.md#params)（单一注入，节点不得覆写）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一执行口语义不变）、
[#messages](../ROS2迁移交接文档.md#messages)（9 消息 ABI 不得变更）；产物纪律
[ADR-0007](0007-experiment-artifacts-convention.md)（唯一落点 `experiments/<campaign>/<run_id>/`）。

**先学后做（取证依据）**：核验本机同版本既有实现——① Nav2 `bringup` 的**参数分层 + 缺失即拒启动**惯例
（参数 schema 校验，缺参在启动期报错而非静默用默认）；② `ros_gz_sim` / gz 的**世界加载**语义
（`gz sim <flags> <sdf>`，世界由参数单一指定）；③ 全仓既有 launch 的参数合并与 run 账本惯例
（[w6_task_eval](../../src/agc_task/launch/w6_task_eval.launch.py) / [run_logger](../../src/agc_task/src/run_logger.cpp)）。
结论：应引入**轴→承载者登记 + 启动时校验**，并以**运行时 manifest** 记录实际生效参数；未执行轴**如实标注**。

分类：**设计风险与未决事项**。本 ADR 给出「如何让声明轴 = 执行轴」的明确决定，**不**回退任何已验收组件。

## 决策

### (a) 轴→承载者登记（`config/scenario_carriers.yaml`）

新增 [`scenario_carriers.yaml`](../../src/agc_task/config/scenario_carriers.yaml)，逐轴值映射到承载它的
**节点 / 参数 / 文件**，并标 `supported`（是否有承载者）与 `requires_air_link`（承担者是否依赖空中链路）：

- **target_motion** `static|linear|maneuver` → `intruder_driver`（`px4_ns=px4_1`），`requires_air_link`；
- **obstacle** `none|static` → 世界障碍生成（`world_obstacles`）+ `obstacles_file`；`dynamic` → `supported: false`（**无承载者**）；
- **perception** `nominal|dropout|fov` → `target_source=air_fusion` + 传感器通道文件（`sensors_file`），`requires_air_link`；
- **outcome** → `task_manager`（+ `capture_adjudicator` / `jammer`）。

各场景在 [`scenarios.yaml`](../../src/agc_task/config/scenarios.yaml) 增 `support_status`
（`supported` / `unsupported:<axis>`）；`s05` / `s12`（`obstacle=dynamic`）标 `unsupported:obstacle`。

### (b) 启动时校验 + 显式拒绝运行

[w6_task_eval](../../src/agc_task/launch/w6_task_eval.launch.py) 与 [w6_full](../../src/agc_task/launch/w6_full.launch.py)
**启动时**：对每个声明轴值查承载者登记；**无登记 / `supported:false` → `raise`（拒绝运行）**并给出显式原因；
`support_status` 为 `unsupported:<axis>` 亦拒绝。**SHALL NOT** 以命名当执行。

### (c) 正负对照的承载实现

- **perception**：`perception_bringup.launch.py` 增 `sensors_file` / `obstacles_file` 两个 launch 参数（缺省=包内默认）；
  上层按轴选择——`nominal`=[`perception_sensors.yaml`](../../src/agc_perception/config/perception_sensors.yaml)（`drop_prob=0`、标称 FOV）、
  `dropout`=[`perception_sensors_dropout.yaml`](../../src/agc_perception/config/perception_sensors_dropout.yaml)（`drop_prob=0.3`、`drop_burst_*`）、
  `fov`=[`perception_sensors_fov.yaml`](../../src/agc_perception/config/perception_sensors_fov.yaml)（`hfov 30 / vfov 20`）。
  整链以 `target_source=air_fusion` 承载（通道模型经 `/agc/air/obs` 落入 `/agc/targets`）。
- **obstacle**：`world_bringup.launch.py` 增 `world_obstacles` 选择——`static` 用原世界；`none` 由**单一真源**
  [`agc_world.sdf`](../../src/agc_sim/worlds/agc_world.sdf) **去除** `pole_1/pole_2` `<model>` 块写入临时世界再用
  （**不**手写第二份完整 world、**不**改世界物理）；`obstacles_file` 对应
  [`obstacles_none.yaml`](../../src/agc_perception/config/obstacles_none.yaml)（空）/ [`obstacles.yaml`](../../src/agc_perception/config/obstacles.yaml)。
- **target_motion**：整链由 `intruder_driver` 承载；整链空中节点参数改为 `air.yaml` 基参 + `scenarios.yaml`
  场景覆盖**合并后注入**（不再是独立 `air.yaml`）。

### (d) 运行时证据 `scene_manifest` 与「实际 == 声明」

顶层 launch 生成 `scene_manifest.json`（声明轴 + 承载者 + 状态 + 实际生效参数 / 文件 + 断言 + `support_status`），
经新增参数 `scene_manifest_file`（及 `params_file`）**单一注入**给 [run_logger](../../src/agc_task/src/run_logger.cpp)，
由 run_logger 拷入 run 的 `config/`（ADR-0007）。launch 内**断言**实际生效参数 == 声明轴，不符即拒绝运行。

### (e) 覆盖账本：`not_carried`（轻量栈）如实标注

承载者需空中链路而后端为**轻量栈**（[episode_runner](../../scripts/episode_runner.py) 恒跑
[w6_task_eval](../../src/agc_task/launch/w6_task_eval.launch.py) = `m0_skeleton` + 裁决器，**无** PX4 / 空中链路）时，
运动轴与感知轴在 manifest 与 runner 报告中标 **`not_carried`**；runner 对 `unsupported` 场景**拒绝运行**（跳过并记原因）。
**不**把轻量栈的声明轴谎称为已执行。

### (f) 验收脚本与产物

新增 [`w6_r026_check.py`](../../tests/w6_r026_check.py) + [`run_w6_r026.sh`](../../scripts/archive/run_w6_r026.sh)：
轻量实跑 none/static/linear/maneuver/dropout/fov 采集 manifest 与「none 世界生成」产物，验证 dynamic 拒绝运行，
核查覆盖账本与「实际 == 声明」；产物落 `experiments/w6-task-eval/<run_id>/{manifests,worlds,lightweight,metrics/r026.json,log}`。

### (g) 边界（未采纳 / 红线）

- **不**改契约冻结 9 消息 / 服务 ABI；**不**新增第二 ROS 命令者（新增均为 `std_msgs` 诊断 / run config）；
  **不**改唯一执行口语义。
- **不**为 R-026 新建**动态障碍执行节点**；**不**改世界**物理**（仅含 / 不含障碍的**生成**对照）；
  `obstacle=dynamic` **显式标 `unsupported`**。
- 参数**单一注入**（契约 [#params](../ROS2迁移交接文档.md#params)），节点不得覆写。

## 备选

1. **仅文档标注、不改参数流**：维持 12 场景登记但旁注「标签」，成本最低；但**声明轴仍与执行脱节**，
   因果结论仍无效——否决（未满足 R-026 判据「逐轴承载者校验 + manifest 实际==声明」）。
2. **为动态障碍新建执行节点 / 改世界物理**：可覆盖 `dynamic`，但**新增 ROS 命令者 / 改世界物理**触红线；
   且 R-026 只需如实标注未执行——否决（显式 `unsupported` 更诚实）。
3. **在 `scenarios.yaml` 内联承载者（不新建登记表）**：减少文件数；但承载者与场景登记**耦合**、
   多场景共享轴值会**重复**，且 launch 校验需遍历——否决（独立登记表更利于单一真源与复用）。

## 后果

- **正面**：声明轴**必有承载者**（缺则拒启动），`dynamic` 明确 `unsupported`；`none/static`、`nominal/dropout/fov`
  有真实正负对照；轻量栈未执行轴**如实标 `not_carried`**；`scene_manifest` 使「用的是哪套栈 / 声明轴 / 实际参数」可复核。
- **代价 / 限制**：轻量栈**仍不执行**运动轴与感知轴（如实标注，**非**冒充）；整链 `w6_full` 的 dropout / fov
  因果证据需**双 PX4 实跑**方可产出（本 ADR 只保证承载接线与 manifest 落地，不保证整链可跑通）。
- **未决**：整链 `perception=dropout|fov` 的**观测级**证据（`/agc/targets` 按概率丢失 / FOV 缺失）依赖双 PX4 实跑，
  在本仓环境未强制跑通——保留 config / 参数级证据，观测级验收留待整链实跑或另项。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。
