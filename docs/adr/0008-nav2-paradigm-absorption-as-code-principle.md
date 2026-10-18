# Nav2 架构范式吸收：代码重构的交叉原则

## Status
accepted

## 背景与决策
"吸收 Nav2"（见 [GLOSSARY](../../GLOSSARY.md) 的**选择性吸收**）不只在进程与架构层生效（ADR-0003 / ADR-0004），而是**贯穿 W1–W6 代码重构的交叉原则**：每个 workstream 在重写原 ROS1 代码时，必须显式采纳 Nav2 的对应范式，而非沿用 ROS1 写法，并在自身 spec 中列明"吸收点 + 依据"。

## 吸收矩阵

| workstream | 从 Nav2 吸收什么 |
|---|---|
| W1 消息与接口 | 接口分层：topic / service / **action** 的职责划分（长时任务用 Action）；参数声明规范（`declare_parameter` + YAML 注入） |
| W2 仿真世界与模型 | TF / 坐标系与 costmap 输入约定（`OccupancyGrid`、`PointCloud2` 语义）；sensor 类 QoS（best-effort）标定 |
| W3 空中链路 | **lifecycle managed node**（configure/activate/deactivate/cleanup）；pluginlib 插件化；QoS 标定 |
| W4 感知与融合 | **costmap_2d 分层语义**（static / obstacle / inflation / voxel）；map_server 与占据栅格约定 |
| W5 地面导航与安全层 | 直接复用 `collision_monitor`（安全层）；planner/controller 的 **plugin 抽象**用于离线基线切换 |
| W6 任务与评估 | **行为树抽象**（BehaviorTree.CPP / nav2_behavior_tree）作为 `task_manager` 候选；恢复行为（recovery）模式 |

## Consequences
- 每个 workstream 的 spec 必须含 **"Nav2 吸收点"** 一节；未采纳某范式须写明理由。
- 评审口径：不只看"能跑通"，还看"是否按矩阵吸收"。
- 与 ADR-0003 的关系：0003 定"进程按域选用"，本条定"代码层怎么吸收"。

## 适用关系（S-04 校准）

> 本节由结构评审 [S-04](../../docs/temp/agc-review-2026-10-14/20-结构.md#s-04) 校准；口径见 [ADR-0060](./0060-nav2-absorption-applicability-and-command-source-trimming.md)。

- **与 [ADR-0003](./0003-nav2-pattern-complete-absorption.md)**：0003 定**进程 / 架构层范围**；本条定**代码层吸收**——本条 **amends（补充）** 0003 在代码层的落法，不改变其进程层结论。
- **与 [ADR-0004](./0004-rl-primary-controller-nav2-safety-layer.md)**：矩阵 **W5** 行的「直接复用 `collision_monitor`（安全层）」与「planner/controller 的 **plugin 抽象**用于**离线基线**切换」按 ADR-0004 的**在线角色**执行（RL 在线主控；Nav2 planner/controller 仅离线段）。
- **矩阵 W6 行「行为树抽象」已决议**：候选 `task_manager` = **自研确定性 C++ 状态机**（**吸收**行为树 / 恢复行为的**模式**，**不引入** `bt_navigator`）；见契约 [已决事项](../ROS2迁移交接文档.md#decisions) 与 [W6 spec](../../.trae/specs/w6-task-eval/spec.md)。
- **验收口径**：矩阵各行的落点按**已验证行为**（实际责任 / 运行必要性 / 失败验收）判定，**不**以空壳类 / 闲置进程计数（[ADR-0060](./0060-nav2-absorption-applicability-and-command-source-trimming.md)）。
