# Nav2 吸收策略：模式全覆盖、进程按域选用

## Status
accepted

## 背景与决策
《交接文档》（契约真源）曾称"Nav2 无 3D 规划器、无人机不适用"，与"Nav2 覆盖所有部分"冲突。折中为：**地面域**真跑 Nav2 本体（costmap、全局/局部 planner、controller、行为树）；**空中域**不引入 Nav2 运行时进程，但重写时全面采纳其架构范式（costmap 分层语义、lifecycle 节点、planner/controller server 抽象、行为树），并保留/优化 LQR 与自研避障。唯一执行口 `/agc/ground/cmd_vel` 保持不变。

## Considered Options
- 空中也接 Nav2（需自研 3D planner/controller 插件）—— 被否：成本高、收益薄。
- 仅地面用 Nav2（维持契约原意）—— 被否：不满足"全范围吸收其方法"。

## Consequences
- 空中链路不依赖 Nav2 运行时；Nav2 升级/替换不阻塞空中域。
- "选择性吸收"成为空中域重写的默认判据（见 GLOSSARY）。

## 适用关系（S-04 校准）

> 本节由结构评审 [S-04](../../docs/reviews/agc-review-2026-10-14/20-结构.md#s-04) 校准；口径见 [ADR-0060](./0060-nav2-absorption-applicability-and-command-source-trimming.md)。**下游只链接现行适用**的决策。

- **范围分工**：本条定**进程 / 架构层范围**（"模式全覆盖·进程按域选用"）；[ADR-0008](./0008-nav2-paradigm-absorption-as-code-principle.md) 定**代码层怎么吸收**（W1–W6 吸收矩阵）。二者互补，ADR-0008 在本条基础上补充代码层落法。
- **在线角色归 [ADR-0004](./0004-rl-primary-controller-nav2-safety-layer.md)**：本条正文早期「地面域真跑 Nav2 本体（costmap、全局/局部 planner、controller、行为树）」中的**在线主控角色**已由 ADR-0004 取代——RL **直接出 `cmd_vel`** 为地面主控（c2），Nav2 planner/controller 降为**离线对照基线**；W6 的「行为树」在线候选已决议为**自研确定性 C++ 状态机**（**吸收**其模式，**不引入** `bt_navigator`，见契约 [已决事项](../ROS2迁移交接文档.md#decisions)）。故本条该句**仅**在"进程层真跑 Nav2 组件"意义上继续适用。
- **保持成立**：地面域真跑 Nav2 组件（`collision_monitor` 安全层、离线基线的 planner/controller 插件、costmap 分层语义）；空中域不引入 Nav2 运行时进程。
- **验收口径**：本条落点按 [ADR-0060](./0060-nav2-absorption-applicability-and-command-source-trimming.md) 的**已验证行为**（实际责任 / 运行必要性 / 失败验收）判定，**不以**空壳类 / 闲置进程计数。
