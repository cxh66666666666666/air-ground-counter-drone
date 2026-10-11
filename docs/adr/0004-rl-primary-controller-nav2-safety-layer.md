# RL 主控地面运动，Nav2 退居安全层与离线基线

## Status
accepted

## 背景与决策
地面域采用"分层接入 c2"：RL **直接输出 `cmd_vel`**，经 Nav2 `collision_monitor`（安全层：按多边形区域缩放/急停）收敛到唯一执行口 `/agc/ground/cmd_vel`。Nav2 的 planner/controller **不作为在线主控**，改作可切换的**离线对照基线**。空中域 Nav2 不适用（无 3D），保留/优化 LQR + 自研避障。

## Considered Options
- c1：RL 只出高层目标、Nav2 planner/controller 主控 —— 被否：RL 控制信号降级，削弱其作为一等公民的研究价值。

## Consequences
- 需实现 `RL → cmd_vel → collision_monitor → 唯一执行口` 链路。
- 论文基线取自 Nav2 控制/避障（可切换）；安全层复用既有 shield 口径（`shield_infeasible_rate`）。
- 空中域不受影响，仍按 ADR-0003 的"模式全覆盖"执行。
