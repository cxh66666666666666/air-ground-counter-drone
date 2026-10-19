# Nav2 吸收的适用关系与按命令源进程裁剪

## Status
accepted

## 背景与决策

结构评审 [S-04](../../docs/reviews/agc-review-2026-10-14/20-结构.md#s-04)（**挑战既定约束 · 已裁定正式修订**）指出：「吸收 Nav2」不应以**空壳类数量 / 闲置进程数量**验收。两处须校准：

1. **决策适用关系不清**：[ADR-0003](./0003-nav2-pattern-complete-absorption.md) 早期正文列「地面域真跑 Nav2 本体（costmap、全局/局部 planner、controller、行为树）」，与 [ADR-0004](./0004-rl-primary-controller-nav2-safety-layer.md)（RL **直接出 `cmd_vel`** 为地面主控，Nav2 planner/controller 降为**离线对照基线**）及 [W6 spec](../../.trae/specs/w6-task-eval/spec.md) 已决的**自研确定性 C++ 状态机**冲突；[ADR-0008](./0008-nav2-paradigm-absorption-as-code-principle.md) 与两者的范围关系未显式标注。
2. **闲置进程**：`command_source=rl` 时 composition 仍起 RL **不消费**的独立 `nav2_costmap_2d` 进程。

**裁定**（**不**推翻「代码层吸收」原则）：

- **适用关系显式化**：ADR-0003 = **进程 / 架构层范围**（"模式全覆盖·进程按域选用"）；ADR-0008 = **代码层吸收**（W1–W6 吸收矩阵，**amends** 0003 的代码层落法）；**在线角色**归 ADR-0004 + W6 自研 FSM。三者适用关系在各自 ADR 的「适用关系」节显式标注，**下游只链接现行适用**的决策。
- **吸收验收口径 = 已验证行为**：按模式给**实际责任 / 运行必要性 / 失败验收**，**不以**类名 / 节点计数验收。**保留** Nav2 `collision_monitor`（安全层）与离线基线（`ground_baseline`）组件。
- **按 `command_source` 只开所需进程**：`rl` 主控**不消费** costmap（[inference_node.py](../../src/agc_rl/agc_rl/inference_node.py#L110-L114) 仅订阅 odom/scan/goal/state；[interface.py](../../src/agc_rl/agc_rl/interface.py) 的 obs = 扇区(scan)+位姿+距目标+上一动作，**无** costmap）→ `rl` 下**不**起独立 `nav2_costmap_2d` 进程；`baseline` 由 `ground_baseline` **自持**内建 `Costmap2DROS`（同一 `config/costmap.yaml` 参数源），本就不起独立进程。安全层与唯一执行口中继**不受影响**。

## Considered Options

- **保留独立 costmap 进程「以备将来 RL 观测」** —— 被否：当前无消费者，属闲置进程；评审口径明确**不以闲置进程计吸收**。若将来确实需要，届时按消费者出现再引入。
- **全面拆掉 Nav2 组件、回归纯自研** —— 被否：违背「代码层吸收」原则，且牺牲 `collision_monitor` 安全层与离线基线的复用价值。
- **不改，仅补文档** —— 被否：现象（闲置进程）仍在，验收口径仍未校准。

## Consequences

- 运行拓扑按命令源**如实裁剪**：`rl` 进程集与 `baseline` 进程集各自只含**实际责任**节点（`rl`：TF + `scan_self_filter` + `collision_monitor` + `ground_drive_relay` + RL 推理；`baseline`：TF + `scan_self_filter` + `collision_monitor` + `ground_drive_relay` + `ground_baseline`（自持 costmap））。
- 吸收**验收**改看**行为**（责任 / 必要性 / 失败验收），不靠类名 / 节点计数。
- [ADR-0003](./0003-nav2-pattern-complete-absorption.md) / [ADR-0008](./0008-nav2-paradigm-absorption-as-code-principle.md) 增「适用关系（S-04 校准）」节。
- [W5 spec](../../.trae/specs/w5-ground-nav-safety/spec.md) 的 **R3** 由「服务离线基线 **/ RL 观测**」校准为「**离线基线自持**」（RL **不消费** costmap）——声明回归真实；`/costmap` 判据在 `command_source=baseline` 路径核验。
- 冻结 9 消息 / 服务 ABI、唯一执行口语义、硬约束 C1–C10 均**不改**。
