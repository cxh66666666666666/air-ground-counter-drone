# P1 度量口径修正：可见 / 火控机会率改用**固定观测时长窗口**

## Status

accepted

## Date

2026-10-20

## 背景

[ADR-0073](0073-p1-arm-fairness-noncoop-fixed-trajectory.md) 修正臂公平性后复验发现：`visibility_metrics`（口径 `window_active_v1`）的**分母 = episode active 窗口 = start → 终态**，而两臂都很快达 `TRACKING_SUCCESS`（≈16 s）→ 窗口**恰在"可见"时被截断** → `visibility_rate` 在"早成功"臂上**饱和**（协同臂 0.976–0.984、固定航迹非协同臂 0.986 → **无差**）。

**实测采样**（`atr=off` / `velocity` / s08 seed0，2026-10-20）：

```
t= 1.5s Δ=+21.5° obs_tgt=2 state=GROUND_APPROACH
t=16.5s Δ=-20.2° obs_tgt=0 state=TRACKING_SUCCESS   ← 任务在此刻终止（窗口截断）
t=25.5s Δ=-163.4° obs_tgt=0                        ← yaw 扫出视场后目标不可见
```

⇒ 该 DV 对"观测指向"自变量**不敏感**，且含"成功需可见"的**选择效应**；在其上判定 P1 会产生**假阴性**。

## 决策

`visibility_metrics`（P1 的 DV 来源）口径改为 **`window_fixed_v1`**：

- **分母** = **自首个 active 拍起固定 `obs_window_s` 秒内**的拍数（**不**随终态 / IDLE 截断）；
  - `run_logger` 新增参数 `obs_window_s`（缺省 60.0，单一注入 [`w6_base.yaml`](../../src/agc_task/config/w6_base.yaml)）；
  - `episode_runner` 在终态后**不立即拆除**，继续采到该固定时长（按**仿真钟**，上限仍受**墙钟看门狗**约束，超限如实早停并记 `obs_horizon_reached`）；CLI `--obs-horizon-s`（缺省 60.0）**须与 `obs_window_s` 同值**。
- 同一 `active` 窗口口径的其它指标（`episode_window_metrics`）**保持不变**（`window_active_v1` 仍在，供逐窗对照）。
- 阈值 / 样本量 / 显著性**均未改动**（[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)）；`visibility_metrics.definition_version` 升为 `window_fixed_v1` 并记 `window_s`（**版本化口径**，新旧数值**不可逐值对照**）。

## 备选

1. **保持 active 窗口口径、改用其它 DV**（如首获目标时延、最长失联）—— 保留备选：但这些量同样受"何时终止"影响；且 P1 的预注册 DV 是火控机会率 / 净空。
2. **换到"移动目标"场景以拉长窗口** —— 否决（本段）：支持的 capture 场景仅 `s08`（静态）；`s12`（线性 capture）为 `unsupported:obstacle`。
3. **让 episode 永远跑满墙钟看门狗** —— 否决：每回合固定 ~200 s，代价高且改变 R-024 的时长语义。

## 后果

- **正面**：DV 与"何时终止"解耦 → 消除饱和 / 选择效应，对"观测指向"敏感 → P1 可判。
- **代价 / 限制**：每 episode 至少观测 `obs_window_s`（60 s）→ run 墙钟变长；`obs_window_s` 与 `--obs-horizon-s` **须同值**（不同则窗口采不满）；口径变更使新数值与旧 `window_active_v1` **不可逐值对照**（版本化 + note 标注）。
- **未决 / 后续**：~~重跑 P1（N=20）→ 按预注册判定 → 补 [ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(f)~~ → 已跑（判定见 [ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(h)；其语义随后被 [ADR-0076](0076-capture-scenario-tracking-milestone-not-terminal.md) 取代，**待新语义重跑**）；full 链 infra 率已另治（[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)）。

## 参考

- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C9 必报指标）/ [#params](../ROS2迁移交接文档.md#params)（单一注入，节点不得覆写）
- 前序 ADR：[ADR-0043](0043-episode-window-metrics-and-valid-sample-mask.md)、[ADR-0046](0046-runner-three-layer-acceptance-and-integrity-gate.md)、[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)、[ADR-0073](0073-p1-arm-fairness-noncoop-fixed-trajectory.md)
- 预注册：[`src/agc_rl/config/w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)
