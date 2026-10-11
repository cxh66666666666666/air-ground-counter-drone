# 本次执行范围出口到 W6；拦截落于 W2/W6，不新增 workstream

## Status
accepted

## 背景与决策
- 本次攻关范围**以 W6（任务与评估）为出口**：W3–W6 打通即"平台 / 软著"达标；**W7（RL 平台）、W8（新旧对照）归"项目结束后后续"**（与 [ADR-0014](./0014-research-mainline-mobile-interception-system.md) 一致）。
- 拦截（[ADR-0015](./0015-interception-scope-physical-capture-and-behavioral-jamming.md)）**不新增 workstream**，落点：
  - `intruder_uav` 升级为可控 / 可脚本化模型 → **W2**；
  - 拦截裁决（物理捕获判据 + 干扰注入）与"捕获 / 干扰"终态语义 → **W6**。

## Considered Options
- 新增独立 workstream（W9 拦截）—— 被否：会稀释 W3–W6 全链，与"系统论文为主"的工期冲突。
- 出口含 W7 / W8 —— 被否：RL 已归后续（ADR-0014）；W8 对照在"全新重写"下价值有限。

## Consequences
- Runbook 状态表与依赖图据此调整：**W7 / W8 标为后续**，W5 不再依赖 W7 提供策略。
- **已定**：地面主控**维持 [ADR-0004](./0004-rl-primary-controller-nav2-safety-layer.md)**——本次在 **W5 内做 RL 最小版**（`RL → cmd_vel → collision_monitor`），完整 RL 平台(W7)仍归后续；W5 前置不含 W7。
- **已定**：W4 取**最小口径**（odom / gt / perception 点云 + map adapter，够 W5 costmap 用）。
- W6 spec 撰写时须内含 [ADR-0015](./0015-interception-scope-physical-capture-and-behavioral-jamming.md) 的拦截裁决与终态语义扩展。