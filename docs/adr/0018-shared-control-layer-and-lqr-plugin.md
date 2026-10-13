# 通用控制层与共享 LQR 插件化（`agc_control`）

## Status
accepted

## 背景与决策

空中与地面两个域都需要**轨迹跟踪**能力：W3 空中需要 LQR 跟踪真八字与目标（standoff），W5 地面需要一个**离线对照基线**控制器（地面主控仍是 RL，[ADR-0004](./0004-rl-primary-controller-nav2-safety-layer.md)）。若各域各自实现，将重复造轮子并引入行为不一致。

据此定：

- **抽出通用控制层** `agc_control`：一份实现、多处调用。以 **pluginlib** 暴露两类插件：
  - **①轨迹生成器**（`linear`/`circular`/`helical`/`figure8`/`hover`/`step`）
  - **②控制器**（**LQR**）
- **通用接口**与执行器解耦：入参 = 当前 `state(pos, vel)` + 参考 `ref(pos, vel, acc)`，出参 = **加速度指令** `acc_cmd`（世界系 ENU/FLU）。各消费者自行转执行量——air 转 PX4 位置 / 速度 setpoint，ground 转 `cmd_vel`；控制频率由消费者 pump。
- 归属：**新建 `agc_control` 包**（不塞入现有包，避免 `agc_ground → agc_air` 之类错误依赖）；包边界约定同步更新。
- 该形态同时命中 [ADR-0008](./0008-nav2-paradigm-absorption-as-code-principle.md) 要求的 W3 **pluginlib 吸收点**。

## Considered Options

- **独立节点 + service/action 调用** —— 被否：引入通信时延，不适合高频控制内环。
- **各域各自实现** —— 被否：重复实现、行为漂移，违背"只实现一次"。
- **厚 `HardwareHooks` 抽象层** —— 被否：过度设计（[ADR-0009](./0009-sim-first-with-hardware-hooks.md) 只要求薄抽象）。
- **为"未来第三方（目标机）调用"预建框架** —— 被否（YAGNI）：接口做通用即可，不为未发生的需求建设施。

## Consequences

- 新增包 `agc_control`（`ament_cmake`）；更新 [包边界](../conventions/包边界.md) 与 Runbook 目录速览。
- 契约 [唯一执行口](../ROS2迁移交接文档.md#execution-port) 语义不变：地面须经安全层后由唯一执行口下发，LQR 仅作基线。
- 插件接口为通用 `state + ref → acc`，天然满足可替换边界（[ADR-0009](./0009-sim-first-with-hardware-hooks.md)）。