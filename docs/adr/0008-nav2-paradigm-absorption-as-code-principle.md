# Nav2 架构范式吸收：代码重构的交叉原则

## Status
accepted

## 背景与决策
"吸收 Nav2"（见 [GLOSSARY](../GLOSSARY.md) 的**选择性吸收**）不只在进程与架构层生效（ADR-0003 / ADR-0004），而是**贯穿 W1–W6 代码重构的交叉原则**：每个 workstream 在重写原 ROS1 代码时，必须显式采纳 Nav2 的对应范式，而非沿用 ROS1 写法，并在自身 spec 中列明"吸收点 + 依据"。

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
