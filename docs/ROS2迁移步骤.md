# ROS2 重构 Runbook（唯一执行入口）

> **角色**：本项目唯一的**执行入口（Runbook）**（见 [GLOSSARY](../GLOSSARY.md)）。只编排"现在做什么、如何验收"。
> **不承载**：任何约束 —— 约束一律**引用**契约真源 [ROS2迁移交接文档](./ROS2迁移交接文档.md)，不复述（[ADR-0002](./adr/0002-docs-source-of-truth-layering.md)）。
> **读者**：agent（按本文推进，每步须可判定）为主，人工以检查视角复核。
> **引用约定**：跨文档引用用稳定锚点（`<文件>#<锚点>`），禁止 §章节号。
> **基线**：ROS1 参考实现已冻结只读，**不得删改，不得复制代码**（[ADR-0001](./adr/0001-greenfield-ros2-rewrite.md)）。

---

<a id="usage"></a>
## 0. 用法

1. 按 workstream 推进；每个 workstream 各有规格 `.trae/specs/<ws>/{spec,tasks,checklist}.md`。
2. 卡片的"验收"必须**可命令化判定**；不通过的 workstream 不得进入其下游。
3. 完成一个 workstream，就在 [状态表](#status) 打勾。
4. 遇到约束问题，查契约真源对应锚点，**不改约束、改实现**。

<a id="principles"></a>
## 1. 原则

- **单一项目根** `/home/chen/AGC`（colcon 根即项目根）。
- **全新重写**：旧树只读归档，仅作 W8 行为对照。
- **Walking Skeleton 先行**（[M0](#m0)）：先打通一条最小端到端闭环，再逐域加厚。
- **Nav2 吸收**：按 [ADR-0008](./adr/0008-nav2-paradigm-absorption-as-code-principle.md) 的吸收矩阵，在 W1–W6 **代码层**落地，每个 workstream spec 必须含"Nav2 吸收点"一节。
- **纯仿真优先**，为实机预留接口（[ADR-0009](./adr/0009-sim-first-with-hardware-hooks.md)）。

<a id="status"></a>
## 2. Workstream 总览与状态

| WS | 名称 | 前置 | 挂载硬约束 | 状态 |
|---|---|---|---|---|
| 环境 | P0 工具链（Jazzy/ros_gz/Nav2/PX4/uXRCE） | — | — | ✅ 已就绪（附录 B） |
| **W0** | 仓库与工程规范 | 环境 | C2 | ✅ |
| **W1** | 消息与接口（`agc_msgs`） | W0 | C2 | ✅ |
| **M0** | 最小端到端闭环（Walking Skeleton） | W0·W1·W2(最小)·W5(桩) | — | ✅ |
| **W2** | 仿真世界与模型 | W0 | C3·C7(场景层) | ✅ |
| **W3** | 空中链路 | W2 | C1·C10 | ✅ |
| **W4** | 感知与融合 | W2 | — | ✅ |
| **W5** | 地面导航与安全层 | W2·W4 | C6 | ✅ |
| **W6** | 任务与评估 | W5 | C4·C5·C7·C8·C9 | ✅ |
| **W7** | RL 平台 | W2-full·W4·W5·W6（已验收证据，[ADR-0069](./adr/0069-w7-entry-and-exit-definition.md)） | — | ◐ 进行中（W7-a ✅；W7-b 的 **P1 已在新终态语义 + [ADR-0078](./adr/0078-capture-event-episode-identity-single-source.md) + [ADR-0079](./adr/0079-b2-fire-control-target-motion-estimation.md) 修复后完成 N=20 判定并成立**，P2 判定 / 消融与 W7-c 未执行，见 [ADR-0071](./adr/0071-w7b-adjudication-record-and-pending-arms.md) / [ADR-0076](./adr/0076-capture-scenario-tracking-milestone-not-terminal.md)） |
| **W8** | 对照与冻结 | W6·W7 | C1–C10（容差） | ☐（后续） |

### 依赖图

```
W0 ─┬─> W1 ─────────┐
    │               ├─> M0 骨架闭环 ──> 逐域加厚
    └─> W2 ─┬───────┘
            ├─> W3 空中链路
            └─> W4 感知融合 ─┐
                             └─> W5 地面导航 ──> W6 任务评估 ──> 〔本次出口：ADR-0016〕
（后续）W7 RL 平台（前置：W2-full·W4·W5·W6 已验收证据）· W8 对照冻结
```

> **本次范围**：以 **W6 为出口**；W7 / W8 归"项目结束后后续"（[ADR-0014](./adr/0014-research-mainline-mobile-interception-system.md)、[ADR-0016](./adr/0016-execution-scope-exit-w6.md)）。W5 的地面主控**维持 [ADR-0004](./adr/0004-rl-primary-controller-nav2-safety-layer.md)**，本次以 **RL 最小版**落地。

---

<a id="workstreams"></a>
## 3. Workstream 卡片

### W0 · 仓库与工程规范
- **目标**：确定目录骨架、包边界、命名与产物规范、git 与 `.gitignore`。
- **产出**：目录骨架 + 规范文档 + git 初始化 + `.gitignore` + 旧树归档至 `reference/old_ros1/`。
- **非目标**：CI/CD 流水线（另立 workstream）。
- **验收**：`colcon list` 只列出预期包；`experiments/**/config/` 可入库而其余被忽略（[ADR-0007](./adr/0007-experiment-artifacts-convention.md)）。
- **Nav2 吸收点**：参数声明规范、接口分层约定。
- **spec**：`.trae/specs/w0-repo-and-conventions/`

### W1 · 消息与接口
- **目标**：`agc_msgs` 独立包 + 9 个 rosidl 消息，字段级对齐契约。
- **产出**：`agc_msgs`。
- **验收**：`ros2 interface show agc_msgs/msg/AgcTaskState` 与契约 [消息](./ROS2迁移交接文档.md#messages) **逐字段一致**。
- **Nav2 吸收点**：topic/service/**action** 职责划分。
- **spec**：`.trae/specs/w1-msgs/`

<a id="m0"></a>
### M0 · 最小端到端闭环（Walking Skeleton）
- **目标**：起世界 → 出 odom → 收任务门控 → 下发 `cmd_vel`（占位/直通策略）→ 落一条评估。
- **前置**：W0、W1、W2（最小）、W5（桩）。
- **产出**：一条可复现的 run。
- **验收**：一条 `ros2 launch` 起全链；`/agc/ground/cmd_vel` 有输出；产物落 `experiments/<campaign>/<run_id>/`。
- **意义**：闭环一旦存在，后续硬约束才有**可验证**的载体（[ADR-0006](./adr/0006-domain-workstream-organization.md)）。
- **spec**：`.trae/specs/m0-walking-skeleton/`

### W2 · 仿真世界与模型
- **目标**：`agc_world` + 车 / UAV 模型迁 GZ Sim；`ros_gz` 桥；传感器链路。地面车为**真实物理**驱动（[ADR-0010](./adr/0010-ground-drive-real-physics.md)）。**分段交付**：W2-min（世界 + 车 + target 被动模型 + 桥最小集）→ W2-full（传感器链路 / costmap 输入数据 / 世界打磨）→ W2-interception（目标可控化 + 地面拦截载荷模型）。**拦截相关**：`intruder_uav` 升级为**可脚本化 / 可控**模型、`agc_car` 增**拦截载荷**（捕获网面 + 干扰载荷）模型（供 W6 干扰注入与捕获裁决，[ADR-0015](./adr/0015-interception-scope-physical-capture-and-behavioral-jamming.md)、[ADR-0016](./adr/0016-execution-scope-exit-w6.md)）。
- **产出**：可加载的世界与模型、桥配置（W2-min 供 M0；W2-full 供 W4/W5）。
- **验收**：无头 / GUI 均可起；`/clock` 同步；点云话题有数据（QoS 见 [风险](#risks)）。
- **挂载**：C3（非无头验收）、C7（出生间距，场景层）。
- **Nav2 吸收点**：TF / costmap 输入约定（`OccupancyGrid`、`PointCloud2`）、sensor QoS。
- **spec**：`.trae/specs/w2-sim-world/`

### W3 · 空中链路
- **目标**：PX4 uXRCE-DDS 链路 + 共享 LQR 轨迹跟踪 + **对目标常开 standoff 跟踪（含自避障）**（[ADR-0005](./adr/0005-px4-link-uxrce-dds.md)、[ADR-0017](./adr/0017-air-ground-collaboration-loop.md)、[ADR-0018](./adr/0018-shared-control-layer-and-lqr-plugin.md)、[ADR-0019](./adr/0019-uav-trajectory-and-tracking-parameter-calibration.md)）；PX4 本体以 [toolchain 外部依赖](./adr/0012-px4-body-as-toolchain-dependency.md) 形态使用。
- **分段**：**W3-a** 链路打通（PX4 以 `PX4_GZ_STANDALONE` 挂 `agc_world` + 起飞 / 悬停 / Offboard 接管）→ **W3-b** 共享 LQR + 真八字（C1）→ **W3-c** 常开 standoff 跟踪 + 自避障（C10）。
- **产出**：`agc_control`（通用控制层）+ `agc_air` 飞行闭环；前置 **W2 打补丁**（PX4 系统插件 + 作业区放大）。
- **验收**：悬停稳定、Offboard 可切；C1 真八字恒速 `v=2.0`；C10 跟踪常开 / 安全间距 `8/15 m` / 新鲜度冻结 / 自保。
- **挂载**：C1（真八字恒速）、C10（**跟踪常开** / 安全间距 / 新鲜度冻结）。
- **Nav2 吸收点**：**lifecycle managed node**、pluginlib（`agc_control`）、QoS 标定。
- **spec**：`.trae/specs/w3-air-link/`

### W4 · 感知与融合
- **目标**：odom / gt / perception / fusion / map 适配节点。
- **产出**：各话题与频率。
- **验收**：各话题频率与契约口径一致（±容差）。
- **Nav2 吸收点**：**costmap 分层语义**（static/obstacle/inflation/voxel）、map_server 约定。
- **spec**：`.trae/specs/w4-perception-fusion/`

### W5 · 地面导航与安全层
- **目标**：`RL → cmd_vel → collision_monitor → /agc/ground/cmd_vel`（[ADR-0004](./adr/0004-rl-primary-controller-nav2-safety-layer.md)；本次为 **RL 最小版**）。
- **分段**：**W5-a** 安全层真化（`collision_monitor` 点云源 + costmap 组装 + C2 三档承载）→ **W5-b** RL 最小版（`agc_rl` 接口 + 推理节点接入唯一执行链）→ **W5-c** 离线对照基线（Nav2 planner/controller + `agc_control` LQR 切换）+ 全链回归。
- **前置补丁**：**W2 打补丁**（地面平台保形换装为差速 UGV + 实测顶速，[ADR-0021](./adr/0021-ground-safety-thresholds-and-platform-recalibration.md)）。
- **产出**：RL 主控链路 + Nav2 `collision_monitor` 安全层 + 离线对照基线接入。
- **验收**：全程仅 [唯一执行口](./ROS2迁移交接文档.md#execution-port) 一个；**无第二命令者**；安全层触发可复现。
- **挂载**：C6（安全滤波器真实逃逸能力）；C2（三档阈值行为承载，[ADR-0021](./adr/0021-ground-safety-thresholds-and-platform-recalibration.md)）。
- **说明**：车体为真实物理驱动（[ADR-0010](./adr/0010-ground-drive-real-physics.md)），C6 的"真实逃逸能力"据此判定。
- **Nav2 吸收点**：直接复用 `collision_monitor`；planner/controller 的 **plugin 抽象**（用于离线基线切换）；costmap 分层语义组装（承接 W4 遗留）。
- **spec**：`.trae/specs/w5-ground-nav-safety/`

### W6 · 任务与评估
- **目标**：`task_manager`、`agc_run_logger`、12 个场景、评估口径；修复已知竞态（契约 [不可回退教训](./ROS2迁移交接文档.md#lessons)）。**拦截裁决**（物理捕获判据 + 行为级干扰注入）与"捕获 / 干扰"**终态语义扩展**（[ADR-0015](./adr/0015-interception-scope-physical-capture-and-behavioral-jamming.md)），并消解契约 [终态语义](./ROS2迁移交接文档.md#terminal-semantics) 的旧口径。
- **产出**：状态机 + 日志 + 场景 + 评估脚本。
- **验收**：终态语义正确；报告含 `shield_infeasible_rate` 与 `reset_failure_rate`。
- **挂载**：C4（超时算失败）、C5（相位复位 t=0）、C7（出生净空 / 复位重试，[ADR-0023](./adr/0023-spawn-clearance-recalibration.md)）、C8（终态语义）、C9（报告指标）。
- **Nav2 吸收点**：**行为树抽象**（`task_manager` 候选 → **已决议**：自研 C++ 状态机**吸收其模式**，不引入 `bt_navigator`）、恢复行为模式。
- **前置 P**：W2-interception 回退（`intruder_uav` **PX4 化**，[ADR-0015](./adr/0015-interception-scope-physical-capture-and-behavioral-jamming.md) 修订）+ 可行性闸门（双 PX4 同世界 + mode 注入）。
- **已决**：`task_manager` 架构 = 自研确定性 C++ 状态机；拦截 = 物理捕获（发射式网，[ADR-0022](./adr/0022-launched-net-capture-mechanism.md)）+ 行为级干扰（[ADR-0015](./adr/0015-interception-scope-physical-capture-and-behavioral-jamming.md) 修订）。
- **spec**：`.trae/specs/w6-task-eval/`

### W7 · RL 平台（**后续**，不在本次出口）
- **目标**：策略、训练 / 评估流水线、论文产线。
- **前置**：W2-full · W4 · W5 · W6 —— **逐项以已验收 run 证据为准**（每项指向 `run_id` + 验收命题，[ADR-0069](./adr/0069-w7-entry-and-exit-definition.md)）。
- **分段**：**W7-a** 最小平台（入口前置证据 + obs / action 版本化 + train / eval 协议 + 种子分解 + 单 worker 可重放 + **规则基线可达性**）→ **W7-b** 科研路线（选可证伪命题，[ADR-0070](./adr/0070-w7b-research-propositions-and-rl-role-adjudication.md)）→ **W7-c** 扩容（多 worker 隔离）。
- **产出**：可复现的训练与评估。
- **验收（W7-a）**：可信实验平台 —— 同 seed 可复现；train / eval 协议与评估一致；**不含**算法 / 论文（属 W7-b）。
- **未决**：算法 / 环境 / 指标 / 论文方向 —— 在 W7 spec 内定夺（W7-b）。
- **进度（2026-10-19）**：**W7-a 已落地** —— T1–T5 + 横切核查完成；证据 `experiments/w7-rl/{20261019-w7-core-final, 20261019-w7-replay-0001, 20261019-w7-baseline-0001}`。**W7-b 进行中** —— 科研裁定 [ADR-0070](./adr/0070-w7b-research-propositions-and-rl-role-adjudication.md) + 规格 `.trae/specs/w7-rl-b/`：**入口前置 E1–E5 已落地并取证**；臂与消融**可运行子集已跑**（基线锚 / P2 两臂 / 消融·安全干预），**P1 协同臂待整链**、确证判定未执行 —— 裁定记录 [ADR-0071](./adr/0071-w7b-adjudication-record-and-pending-arms.md)。W7-c **未开始**。
  - **补充（2026-10-19 晚）**：整链（`backend:=full`）**步 1「评价可用」已落地** —— 空中控制权交接（`task_manager` 在悬停后调 `/agc/air/release_control`，甲-1）+ `px4_adapter` 姿态四元数字段序修复（[ADR-0072](./adr/0072-px4-attitude-quaternion-field-order-and-full-chain-observation-fix.md)）：full `s08` 同 run 达真实终态 `TRACKING_SUCCESS`、`visibility_rate=0.982`（`run_id w7-rl-b/20261019-w7-rl-b-224947-6509`）。**P1 两臂配对 / 判定仍未执行**；依赖 `/agc/air/odom` 姿态的旧空中证据**须重验**。
  - **补充（2026-10-20）**：W7-b **步 2–3 推进** —— 依次落地三处前置并各自留痕：臂公平性（非协同臂改固定航迹，[ADR-0073](./adr/0073-p1-arm-fairness-noncoop-fixed-trajectory.md)）、DV 口径（可见 / 火控机会率改**固定观测窗口** `window_fixed_v1`，[ADR-0074](./adr/0074-visibility-metric-fixed-observation-window.md)）、整链 infra 处置（**运行前清场** + infra 重试协议，[ADR-0075](./adr/0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)：本机残留孤儿进程使 uXRCE 实体创建 > PX4 硬编码 1 s 预算）。随后 P1 两臂 **CRN 配对 N=20** 已跑（`run_id w7-rl-b/20261020-w7-rl-b-150408-8486`，40 episodes / 1.32 h / infra 0/40），判定 [ADR-0071](./adr/0071-w7b-adjudication-record-and-pending-arms.md)(h) 主 DV 成立（Δ`fire_control_opportunity_rate`=+0.2885、95% CI [+0.202, +0.375]、p=1e-06、n=20）。**但**该批次暴露 capture 场景终态语义缺陷（跟踪里程碑提前锁存 `TRACKING_SUCCESS`，捕获永不发生）→ 已由 [ADR-0076](./adr/0076-capture-scenario-tracking-milestone-not-terminal.md) 修正（`tracking_milestone_is_terminal`，仅 capture 场景）→ **旧批次数值不可作为最终判定**，须在新语义下重跑 N=20 后定论；协同臂为**显式 oracle 指向臂**（适用边界）。W7-c（多 worker）**未开始**；W8 **未开始**。
  - **补充（2026-10-21）**：**新终态语义下 P1 N=20 重跑完成并判定成立** —— 现行结论 `run_id w7-rl-b/20261021-w7-rl-b-175313-3256`（`backend:=full` / `s08` / seeds 0–19 / 40 episodes / 1.58 h / infra 终态 0/40 / 7 次 infra 重试 / **0 条事件拒收**）；判定产物 `metrics/p1_adjudication.json`，结论见 [ADR-0071](./adr/0071-w7b-adjudication-record-and-pending-arms.md)(h) 最终重跑子节：primary `fire_control_opportunity_rate` 均值配对差 **+0.2988**（95% CI [+0.149, +0.449]、p=5.1e-04、n=20）≥ MDE 0.10 → **成立**；secondary `min_clearance` 不显著（−0.1704、p=0.417，如实登记）。**过程留痕**：① 首个新语义批次 `20261021-w7-rl-b-104120-1456` 暴露启动双复位竞态（4 例共 **41 次 `CAPTURE_NET` 被静默拒收**）→ [ADR-0078](./adr/0078-capture-event-episode-identity-single-source.md) 修复；② 修复后批次暴露 B2 火控目标速度假噪声（静态目标被逐帧差分成 1–2 m/s，预测点偏 1–2.5 m）→ [ADR-0079](./adr/0079-b2-fire-control-target-motion-estimation.md) 修复（时间窗回归 + 静止阈值 + 发射解算门），本批仅剩 2 个非协同超时（固定航迹下几乎无开火机会，属臂行为）。旧批次 `150408` / `104120` / `133724` 数值不得引用。协同臂仍为**显式 oracle 指向臂**；成功率 20/20 vs 18/20 仅作画像。**另（B1 观测饥饿）**：诊断确认为空中 lifecycle 启动期 manager 卡死 + 节点 bond 在 configure 过早计时（`px4_adapter` 失效停机 → `no_pose` 全拒），已由 [ADR-0080](./adr/0080-air-lifecycle-startup-robustness.md) 修复（lifecycle 服务预热 + bond 改在 activate）并红-绿；**感知量程 12 m < 站距上限 15 m** 的几何盲区**已裁定保留为环境边界**（对两臂对称、已计入可见率；不修改参数、不重跑），批次产物 `metrics/perception_health.json` 自动报告 `rmax/no_pose/fov` 占比。W7-b 剩余（P2 判定、协同观测 / 目标预测消融）与 W7-c、W8 **未开始**。
- **spec**：`.trae/specs/w7-rl/`（W7-a）、`.trae/specs/w7-rl-b/`（W7-b）

### W8 · 对照与冻结（**后续**，不在本次出口）
- **目标**：新旧实现同场景、同种子各跑 N 次；出对比报告；ROS1 归档只读。
- **验收**：频率 / 数值差异在容差内；终态语义一致。
- **挂载**：C1–C10 全量容差复核。
- **spec**：`.trae/specs/w8-parity-freeze/`

---

<a id="hard-constraint-mounting"></a>
## 4. 硬约束挂载表

> 约束原文见契约 [硬约束](./ROS2迁移交接文档.md#hard-constraints)（C1–C10 即其 1–10 条）。**原则：挂到"首次使其可验证"的 workstream，未到该 workstream 前不强行回归。**

| ID | 约束摘要 | 挂载点 |
|---|---|---|
| C1 | 真八字轨迹恒速 `v=2.0` | W3 |
| C2 | 距离阈值顺序（紧急<碰撞<安全） | W0·W1（顺序）· W5（行为承载，[ADR-0021](./adr/0021-ground-safety-thresholds-and-platform-recalibration.md)） |
| C3 | 实证/验收用非无头（GUI） | W0（规范）· W2（实测） |
| C4 | 超时算失败、显式完成才算成功 | W6 |
| C5 | 每 episode 相位复位 t=0 | W6 |
| C6 | 安全滤波器用真实逃逸能力 | W5 |
| C7 | 出生净空 ≥ 可标定下限（[ADR-0023](./adr/0023-spawn-clearance-recalibration.md)）；复位失败 ≤3 次不占预算 | W2（场景层）· W6 |
| C8 | 终态"抵近并持续跟踪"；`SUCCESS` 不得称"拦截完成" | W6 |
| C9 | 报告含 `shield_infeasible_rate`、`reset_failure_rate` | W6 |
| C10 | 无人机跟踪常开；安全间距；`ref_max_age_s=0.5`、`confidence≥0.5` 冻结 | W3 |

---

<a id="risks"></a>
## 5. 风险清单

| 级别 | 风险 | 缓解 |
|---|---|---|
| 高 | Nav2 仅 2D，空中不可用 | 已定"模式全覆盖·进程按域选用"（[ADR-0003](./adr/0003-nav2-pattern-complete-absorption.md)）；空中保留 LQR + 自研避障 |
| 高 | PX4 由 MAVROS 换 uXRCE-DDS，控制时序/模式切换需重调 | W3 单列里程碑，先打通悬停再接管 |
| 高 | Gazebo Classic → GZ Sim 语义差异大 | W2 先迁世界与模型，再补传感器 |
| 中 | uORB 语义（微秒时间戳、NED/FRD↔ENU、QoS）易错 | W3 内建语义映射测试 |
| 中 | QoS 默认不兼容（sensor/reliable） | 逐话题标定 profile |
| 中 | RL 主控与安全层耦合 | 安全层独立可测，接口先行 |
| 中 | 残留孤儿进程（桥 / tf）占 DDS + shm → uXRCE 实体创建超 PX4 的 1 s 预算 → 整链 infra | 运行前清场 + infra 重试（[ADR-0075](./adr/0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)）；速查 [`conventions/整链排障速查.md`](./conventions/整链排障速查.md) |
| 中 | 时序 / executor 选择致频率数值漂移 | W8 同场景实跑对比，未验证不删旧实现 |
| 低 | `package.xml` / 依赖声明不完整 | `colcon` 逐包构建定位 |

---

<a id="rollback"></a>
## 6. 回滚与冻结策略

- 每 workstream 用 git 分支隔离（如 `ws/w0-conventions`），验收通过再合并。
- 旧树在 W8 通过前**保持可运行，不得删改**。
- 未完成新旧对比前，不得停用旧实现。
- 关键产物（对比报告、验收记录）落 `docs/`，实验产物落 `experiments/`（[ADR-0007](./adr/0007-experiment-artifacts-convention.md)）。

---

<a id="appendix-a"></a>
## 附录 A：ROS1 → ROS2 映射表

> **免责**：仅供迁移参考，**不作为代码或规范来源**；具体吸收点以各 workstream spec 的「Nav2 吸收点」节为准。

| ROS1 | ROS2 | 说明 |
|---|---|---|
| `catkin` / `catkin_tools` | `ament_cmake` / `ament_python` / colcon | `package.xml` 升 format 3 |
| `roscpp` | `rclcpp` | 逐节点改写 |
| `rospy` | `rclpy` | RL / 工具脚本 |
| `ros::init` / `NodeHandle` | `rclcpp::init` / `Node` | 生命周期与 executor 需重设 |
| `message_generation` / `message_runtime` | `rosidl_default_generators` / `rosidl_default_runtime` | `.msg`/`.srv` + rosidl |
| XML `.launch` + `rosparam` | `launch.py` + `parameters=[{}]` | 参数 YAML 结构重排 |
| `ros::param` / `getParam` | 声明式参数 `declare_parameter` | 需显式声明与类型 |
| `tf` | `tf2_ros` | 帧约定不变 |
| `/use_sim_time` | `use_sim_time` 参数 + `/clock` | ros_gz 桥接 |
| `gazebo_msgs` | `ros_gz_interfaces` / `ros_gz_sim` | `gt_adapter`、`scenario_reset` 重写 |
| `mavros_msgs` | `px4_msgs` + uXRCE-DDS | 空中链路重写 |
| Gazebo Classic 传感器插件 | gz-sim 插件 + `ros_gz_bridge` | Velodyne 链路重做 |
| `rosrun` / `roslaunch` | `ros2 run` / `ros2 launch` | 脚本全面改写 |
| `rostopic` / `rosservice` / `rosbag` | `ros2 topic` / `service` / `bag` | 评测与 CI 脚本改写 |

---

<a id="appendix-b"></a>
## 附录 B：环境就绪状态（已完成）

| 项 | 状态 |
|---|---|
| ROS 2 Jazzy + colcon + ros_gz | 已装；`gz sim` 为 Harmonic 8.15.0（来自 `ros_gz` vendor，**须先 `source /opt/ros/jazzy/setup.bash`**） |
| Nav2 | `ros-jazzy-nav2-bringup` 已装 |
| PX4-Autopilot | v1.15.4（含所需子模块），SITL 可执行文件已构建 |
| micro-XRCE-DDS Agent | **v2.4.3**（对系统 Fast DDS 2.14 构建，与 ROS 2 Jazzy 同源）；入口 `toolchain/bin/MicroXRCEAgent`。v2.4.2 已弃用（自建 DDS 2.11 跨版本不互通）。配方见 `toolchain/README.md` |
| `px4_msgs` | release/1.15，已在 `src/` 且通过 colcon build |
| 工作空间 | colcon 根 = `/home/chen/AGC`；`anti_drone_agc_workspace/`、`toolchain/` 各放 `COLCON_IGNORE` |

**环境坑（复现须知）**

1. 本机无需也无法单独 `apt install gz-harmonic`；Harmonic 随 `ros_gz` vendor 引入，**必须先 source** 否则 `libgz-sim8-gui.so.8` 找不到。
2. Micro-XRCE-DDS Agent 需 `-include` 一个 `fmt` 兼容头（fmt 9 移除了隐式 ostream formatter），并对系统 spdlog/fastcdr/fastrtps 构建、关闭 SuperBuild；**须用 v2.4.3**（v2.4.2 源码为 fastcdr 1.x camelCase API，无法对 fastcdr 2.x 编译）。
3. PX4 浅克隆若缺 tag，CMake 的 `git describe` 会失败 → 需 fetch 对应 tag。
4. 本机直连 GitHub 极慢（~25 KB/s）→ 外部仓库统一走 Gitee 镜像，pip 走清华源。

---

<a id="appendix-c"></a>
## 附录 C：目录速览

| 路径 | 作用 |
|---|---|
| `GLOSSARY.md` | 术语表 |
| `AGENTS.md` | 入口索引与铁律（每次会话自动加载） |
| `docs/ROS2迁移交接文档.md` | 契约真源（不变量） |
| `docs/ROS2迁移步骤.md` | 本 Runbook |
| `docs/adr/` | 架构决策记录 |
| `docs/conventions/` | 工程规范（W0 产出） |
| `.trae/specs/<ws>/` | per-workstream 规格三件套 |
| `src/` | ROS2 包（`agc_msgs`·`agc_sim`·`agc_control`·`agc_air`·`agc_perception`·`agc_ground`·`agc_task`·`agc_rl` + 第三方 `px4_msgs`） |
| `experiments/` | 实验产物唯一落点 |
| `reference/` | 归档旧树（只读） |
| `toolchain/` | 外部仓库 / 工具链 |
