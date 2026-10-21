# B2 火控目标运动估计：时间窗回归替代逐帧差分（含发射解算门）

## Status

accepted

## Date

2026-10-21

## 背景

修复 ADR-0078 后的干净 P1 批次 `w7-rl-b/20261021-w7-rl-b-133724-7514` 仍有 **8 个 `TIMEOUT`**：其中 7 个回合**开过火但全部打空**（0 次 `CAPTURE_NET`），1 个回合全程 0 可见（`p1_coop/seed11`）。

日志证据（以 `p1_coop/seed2` 最典型）：

- 单回合连续 **28 发 0 中**；`SHOT_MISS` 的最近逼近距离多在 **0.5–1.5 m**，而网有效半径仅 **0.5 m**（`net_diameter_m=1.0`）；
- `SHOT_FIRED pred=(...)` 与静态目标真值 `(4.5, 3.0, ~2.0)` 的偏差可达 **1–2.5 m**，且逐发跳变（例如 `pred=(6.99,2.63,3.08)`、`(5.18,2.92,4.29)`）。

**根因（代码级）**：

1. `capture_adjudicator` 把 `twist≈0` 当作“速度字段缺失” → 退化为**逐帧位置差分**（`dt≈0.05 s`）；
2. 20 Hz 融合位置的测量噪声被 `1/dt` 放大成 **1–2 m/s 的假速度**；
3. 假速度再乘网飞行时间（约 1.3 s）→ 预测点偏离 1–2.5 m → 网半径 0.5 m 必然打空。

即：**不是弹道模型错，也不是包络错，而是把测量噪声当成了目标速度。**

## 先学（本机同版本 + 社区）

| 来源 | 做法 | 对本项目的含义 |
|---|---|---|
| PX4 `TargetEstimator`（`toolchain/PX4-Autopilot/src/modules/flight_mode_manager/tasks/AutoFollowTarget/follow_target_estimator/`） | α-β-γ 位置/速度/加速度估计；**仅当目标速度超过 `MINIMUM_SPEED_FOR_TARGET_MOVING=0.1 m/s` 才融合速度**；超时后**保留最后位置、速度/加速度清零**；加速度饱和 | 静止目标不得被差分成“运动目标”；速度估计需要门限与滤波 |
| 社区 `rm_gimbal_controllers`（ROS，RoboMaster 云台） | 检测数据**滑动平均滤波** + 弹道模型预测 | 先滤波再算提前量，顺序不能反 |
| 社区 `kf_hungarian_tracker` / α-β 标准资料 | Kalman / α-β 固定增益跟踪位置与速度 | 速度应由估计器给出，不由测量差分给出 |
| Nav2（本机 `/opt/ros/jazzy/share/nav2*`） | 无火控 / 目标指向同类实现（检索 0 命中） | 本问题不在 Nav2 吸收范围，按 PX4 + 社区跟踪实践落地 |

## 决策

### (a) 新增 `TargetMotionFilter`（纯标准库、可单测）

[`target_motion_filter.hpp`](../../src/agc_task/include/agc_task/target_motion_filter.hpp)：

- **位置**：一阶低通（α=0.5），抑制单帧噪声；
- **速度**：对最近 `target_fit_window_s`（默认 **1.5 s**）内的位置样本做**最小二乘直线拟合**；
  - 低于 `target_static_speed_eps_mps`（默认 **0.15 m/s**，对齐 PX4 0.1 先例）→ **速度置零**；
  - 速度硬上限 10 m/s；
- **新息门**：位置跳变 > `target_gate_m`（默认 **1.0 m**）→ 视为复位 / 鬼影 / 换目标，**重置滤波**，不把跳变融进速度；
- **预测**：`p + clamp(v·t_lead, target_max_lead_m)`（默认上限 **1.5 m**），即使速度短时异常也不会把瞄准点甩出网半径。

### (b) 发射解算门

- `capture_adjudicator` 只有在滤波器**已就绪**（窗口 ≥ `target_fit_min_s`=0.75 s 且样本 ≥ 8）时才允许开火；未就绪按冷却处理，不发弹；
- 交会时间改为**两轮迭代**（`t = 距离 / 网水平速度` ↔ 预测点），不再用单次近似；
- `SHOT_FIRED` 日志追加 `vel=(...)`，使“速度估计 → 瞄准点”可追溯。

### (c) 参数单一注入

[`w6_base.yaml`](../../src/agc_task/config/w6_base.yaml) 新增：`target_fit_window_s` / `target_fit_min_s` / `target_fit_min_samples` / `target_static_speed_eps_mps` / `target_gate_m` / `target_pos_alpha` / `target_max_lead_m`；删除已废弃的 `target_vel_max_age_s`。

### (d) B1 / B3 的处置（**不**盲改）

- **B1 观测饥饿**：`p1_coop/seed11` 全程 0 可见，但日志显示该回合 `target 臂指向生效`（GT 指向正常），同回合 `px4_adapter` 出现多次 EKF 复位 / 世界原点重锚。⇒ 更像是**空中观测链的位姿/原点问题**，不是指向策略；本轮**不**改指向，登记为待诊断。**已加诊断（2026-10-21）**：`air_perception` 每 5 s 输出目标门控拒绝原因计数（`no_pose/behind/rmin/rmax/fov_az/fov_el/blocked/dropped` + `target_present`），只观测、不放宽门。**诊断结论与修复**：主因为空中 lifecycle 启动期 manager 卡死 + 节点 bond 在 configure 过早计时 → `px4_adapter` 失效停机（`no_pose` 全拒）；已由 [ADR-0080](0080-air-lifecycle-startup-robustness.md) 修复（lifecycle 服务预热 + bond 改在 activate 建立）。**次因已裁定保留（2026-10-21）**：感知量程 12 m < 站距上限 15 m 的几何盲区对两臂对称且已计入可见率，不修改参数、不重跑；作为环境边界记录，并由批次产物 `metrics/perception_health.json` 自动报告。
- **B3 包络长期不满足**：非协同臂固定航迹带来低 `fco` 属自变量本身的行为，不是实现缺陷；不为它单独改代码。

## 验证

- **单测**（`build/agc_task/test_target_motion_filter`，4 项全过）：
  1. 静止目标 + 0.1 m 噪声（20 Hz）→ 速度 < 0.15 m/s，1.3 s 预测点误差 < 0.3 m；
  2. 匀速 0.5 m/s 目标 → 速度收敛（±0.12 m/s）；
  3. 5 m 位置跳变 → 滤波重置，不污染速度；
  4. 5 m/s 目标 × 2 s 提前量 → 被钳制在 1.5 m。
- **行为红-绿**（原超时种子，`backend:=full`）：

| 回合 | 修复前（干净 P1 批次） | 修复后 | 证据 |
|---|---|---|---|
| `p1_coop/seed2` | `TIMEOUT`，28 发 0 中，`pred` 偏 1–2.5 m | **`INTERCEPT_CONFIRMED`** | 2 发 1 中；`vel=(0,0,0)`、`pred=(4.32,3.02,1.88)`（真值 4.5/3.0/2.0） |
| `p1_coop/seed14` | `TIMEOUT`，26 发 0 中 | **`INTERCEPT_CONFIRMED`** | 同上（静态目标速度被正确置零） |
| `p1_noncoop/seed4` | `TIMEOUT`，4 发 0 中 | **`INTERCEPT_CONFIRMED`** | 低可见（0.355）下滤波器仍能就绪 |
| `p1_noncoop/seed18` | `TIMEOUT`，6 发 0 中 | **`INTERCEPT_CONFIRMED`** | — |
| `p1_noncoop/seed19` | `TIMEOUT`，3 发 0 中 | **`INTERCEPT_CONFIRMED`** | — |

承载 run：`w7-rl-b/20261021-w7-rl-b-173422-4460`（coop 2/14）、`w7-rl-b/20261021-w7-rl-b-174330-1130`（noncoop 4/18/19），均 `backend:=full`。

- **全量 N=20（现行结论）**：`w7-rl-b/20261021-w7-rl-b-175313-3256`（40 episodes / 1.58 h / infra 0/40）——
  - 协同臂 **20/20 `INTERCEPT_CONFIRMED`**；非协同臂 **18/20**，剩余 2 例均为固定航迹下几乎无开火机会（`fco≈0.001/0.002`，各 1 发），属 B3 臂行为；
  - P1 primary `fire_control_opportunity_rate` 均值配对差 **+0.2988**（95% CI [+0.1490, +0.4485]，p=5.1e-04）≥ MDE 0.10 → **成立**（[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(h) 最终重跑子节）。

## 后果

- **正面**：静态目标的假速度被消除；瞄准点从偏差 1–2.5 m 收敛到 < 0.35 m；发射前有明确“解算就绪”门。
- **代价 / 限制**：火控行为已变（发射时机 / 弹数 / 命中率）⇒ **本改动前后的 P1 数值不可混用**，须重跑 N=20；1.5 s 回归窗口对高机动目标的提前量有滞后（当前 capture 场景为静态 / 后续线性，可接受；机动场景需另评估）。
- **回归防线**：`ament_add_gtest(test_target_motion_filter)` 纳入 `colcon test`；`SHOT_FIRED` 的 `vel` 字段可用于后续日志审计。
- **未决 / 后续**：B1 观测链诊断（FOV / range / pose 拒绝计数 + EKF 原点一致性）；B3 若需提升非协同臂包络命中，应作为臂行为调参而非链路修复；P1 N=20 重跑与裁定刷新。

## 参考

- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)、[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)、[#params](../ROS2迁移交接文档.md#params)
- 前序 ADR：[ADR-0022](0022-launched-net-capture-mechanism.md)、[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)、[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)、[ADR-0078](0078-capture-event-episode-identity-single-source.md)
- 参照实现：PX4 `TargetEstimator`、PX4 `gimbal` / ROI、社区 `rm_gimbal_controllers`、α-β 滤波器标准资料
