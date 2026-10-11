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
