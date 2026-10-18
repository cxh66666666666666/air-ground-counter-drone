# 拦截纳入范围：物理捕获判据 + 行为级电子干扰（真实 EMI 不建模）

## Status
accepted（W6 修订，见文末「修订」）

## 背景与决策
项目完全依托仿真后，**"干扰 / 拦截"纳入范围**，形态为 **物理拦截 + 电子干扰（行为级）**：

- **物理拦截**：以**距离 / 圆面阈值判据**判定"捕获"（通行值 **≤2 m**，或"穿越 2 m 圆网面"）。
- **电子干扰**：**行为级等价** —— 对入侵者施加干扰 → 强制 `RTL` / `LAND` / `HOLD` / 失联；**真实电磁场不建模**（gz-sim 无 EMI 传播模型，仅有 `RFComms` 路径损耗）。
- **执行者**：捕获 / 干扰**由地面移动平台承载**；无人机只做观测、standoff 跟踪与自保（[ADR-0017](./0017-air-ground-collaboration-loop.md)）。

**落地约束（先学后做核查所得，落地前须逐条在真机栈复验）**：

- PX4 v1.15 虽有故障注入机制（`SYS_FAILURE_EN` + `failure` 命令 / `MAV_CMD_INJECT_FAILURE=420`，失联动作由 `NAV_DLL_ACT`、`NAV_RCL_ACT` 决定），但 **v1.15 的 gz_bridge 无注入后端**（`SIM_GZ_EN_*` 为 v1.17 PR #25484）→ 注入须在 **ROS 层自研**（经 uXRCE 发 `px4_msgs/VehicleCommand`）。
- 现 `intruder_uav` 为**被动刚体**，须先升级为**可脚本化 / PX4 控制**模型，注入才有作用对象。
- 触发逻辑可**复用** Nav2 `collision_monitor`（`polygon` + `min_points` + BT `ToggleCollisionMonitor`）。

## Considered Options
- 仅跟踪、不做拦截 —— 被否：用户要求纳入（仿真可实现）。
- 真实 EMI 场仿真 —— 不可行：无原生模型，只能行为级等价。

## Consequences
- 新增工作：升级 `intruder_uav` 模型；拦截裁决与干扰注入节点；"捕获 / 干扰"终态语义。
- 契约真源 [终态语义](../ROS2迁移交接文档.md#terminal-semantics) 中悬空的"**阶段 A**"引用须据此消解；拦截相关终态（如 `INTERCEPT_CONFIRMED`）的启用条件在 W6 定夺。
- **待定**：拦截的 workstream 落点（是否新增 WS）——由后续 ADR 定夺。

---

## 修订（W6 定案，2026-10-13）

经 W6 spec 撰写与可行性核查，对本文「落地约束」作如下修订（**覆盖**原文相应表述）：

1. **入侵者形态改为 PX4 控制**（覆盖原文"可脚本化 / PX4 控制"的择一）：`intruder_uav` 由 W2 的失重 `VelocityControl` 刚体，升级为**第二个 PX4 SITL 实例**（各自独立 uXRCE-DDS agent + ROS 命名空间）；桥 `/agc/sim/target_cmd_vel` 退役。理由：行为级干扰需要"**被干扰后能自主 RTL / LAND / HOLD / 失联**"的**自主飞控对象**，脚本刚体无法真实承载该行为。
2. **注入路径**：经 uXRCE 发 `px4_msgs/VehicleCommand`（`DO_SET_MODE`）强制 **RTL / LAND / HOLD**；"**失联**"以**停发 Offboard 指令**触发 PX4 自身 failsafe（`NAV_DLL_ACT`）**等价**。**v1.15.4 无 gz 故障注入后端**（`SIM_GZ_EN_*` 为 v1.17 PR；本项目 vendored 源码核查 `SIM_GZ_EN_*` / `SYS_FAILURE_EN` **零命中**），故**不**使用 gz 注入。真实电磁场仍不建模。
3. **物理捕获判据改为发射式网**：原文"穿越 **2 m 圆网面**（固定网面）"的**判据作废**，改为**发射式网**——网由车载发射器射出并展开、交战包络由**网发射物理模型的可及集**推导（详见 [ADR-0022](./0022-launched-net-capture-mechanism.md)）。
4. **执行者不变**：捕获 / 干扰**由地面移动平台承载**（与 [ADR-0017](./0017-air-ground-collaboration-loop.md) 一致）；无人机只做观测、standoff 跟踪与自保。
5. **可行性闸门**：第二 PX4 实例与 mode 注入**须先经最小实验验证**（W6 前置 P R0）；**不通过则**撤销前置 P，干扰退回"目标 driver **行为剖面切换**"的脚本分支（语义与 `reason_code` 不变）。

## Consequences（修订补充）

- W2-interception 的 `VelocityControl` 路径**回退**（W6 前置 P）；`/agc/sim/target_cmd_vel` 退役，目标运动改由 W6 `intruder_driver` 经 `px4_msgs` Offboard 驱动。
- 契约 [终态语义](../ROS2迁移交接文档.md#terminal-semantics) 消解：**启用** `INTERCEPT_CONFIRMED`（**阶段 B**），`reason_code` 区分 `CAPTURE_NET` / `JAM_RTL|JAM_LAND|JAM_HOLD|JAM_LOST`；**阶段 A = 跟踪、阶段 B = 拦截**；不新增消息枚举（契约 [#messages](../ROS2迁移交接文档.md#messages) 字段 / 顺序不得变更）。
- 新增 [ADR-0022](./0022-launched-net-capture-mechanism.md)（发射式网捕获机制）。

---

## 落地记录（W6 前置 P 可行性核查，2026-10-13）

对本文修订第 1/2/5 条做**最小实验**核查 —— **闸门通过**：

- **双 PX4 实例共存可行**：v1.15.4 gz_bridge 支持多实例（模型名自动带 `_<instance>` 后缀，见 `GZBridge.cpp`）；`rcS` 对 `px4_instance != 0` 自动加命名空间 `px4_<n>`（话题 `/px4_<n>/fmu/*`），并 `MAV_SYS_ID = px4_instance+1`。实测：己方 `x500_0` 与入侵者实例同 `agc_world` 共存、各 **43** 个命名空间话题、**无串扰**。
- **命名不变量**：入侵者以**预 spawn 的 x500 机型 rename 为 `intruder_uav`** + PX4 `PX4_GZ_MODEL_NAME=intruder_uav` **attach**（不 spawn）→ 模型名保留 `intruder_uav`；PX4 相对话题（`/intruder_uav/command/motor_speed`）与 `/model/intruder_uav/odometry_with_covariance` 对齐，实测 `/px4_1/fmu/out/vehicle_odometry` **100 Hz**。
- **注入可行**：`px4_msgs/VehicleCommand` 的 `VEHICLE_CMD_SET_NAV_STATE`（100001）可直接强制 `nav_state`；实测注入 RTL(5) 使 `nav_state 4 → 5`。
- **关键前置（坑）**：入侵者 PX4 **必须设 `PX4_SIM_MODEL=gz_x500`** —— 它决定 airframe `4001_gz_x500`（含 `SENS_EN_MAGSIM=1` / `SENS_EN_GPSSIM=1`）；漏设则**无磁罗盘** → `Compass Sensor 0 missing / Yaw estimate error` → **无法 arm**。
- **落地**：`agc_sim` 的 `intruder_uav` 模型改**双模**（`controlled:=false` 被动 / 失重，供 M0/W5；`:=true` `merge model://x500`，供 W6）；`agc_task` 增 `intruder_driver`（Offboard arm + `static/linear/maneuver`）与 `jammer`。实跑：入侵者自地面**起飞至 z≈3.3 m Offboard 悬停**；注入 `LAND` → `nav_state 14 → 18`（AUTO_LAND），驱动侧**停发 Offboard（链路切断 → failsafe）**，与行为级干扰语义一致。
- **待续**：`drone`/`intruder` 双 PX4 的**整链 launch 集成**（与 `air_bringup` / `m0_skeleton` 组合）尚未完成；`target_system` 须显式设为本实例 sys id（`px4_instance+1`，即 2），`0`（广播）**不被接受**。