# 拦截纳入范围：物理捕获判据 + 行为级电子干扰（真实 EMI 不建模）

## Status
accepted

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