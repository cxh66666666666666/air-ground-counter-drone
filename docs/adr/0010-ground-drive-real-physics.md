# 地面驱动采用真实物理（GZ DiffDrive）

## Status
accepted

## 背景与决策
旧树地面车采用 `kinematic_sim`：`ground_drive_adapter` 对 `/agc/ground/cmd_vel` 做运动学积分（`x += vx·cos(yaw)·dt`、`y += vx·sin(yaw)·dt`、`yaw += wz·dt`），再经 `/gazebo/set_model_state` 直接写 `agc_car` 位姿。该方案无质量 / 惯性 / 轮速约束，**动力学不可信**（不体现打滑、加速受限、转弯半径），使 RL 训练与评估的物理意义打折，也使安全层失去对标对象（硬约束 C6 的"真实逃逸能力"）。

**决策**：地面车改为 GZ Sim 内**真实物理驱动**——

- 车模型 `agc_car` 重设为**差速两驱动轮 + 2 caster**，挂 `gz::sim::systems::DiffDrive` 插件；`wheel_separation`、`wheel_radius`、`max_wheel_torque`、`max_wheel_accel` 等参数落 `agc_sim/config`，由 launch 单一设置。
- `/agc/ground/cmd_vel`（`TwistStamped`，车体系）经 `agc_ground` 内的**中继节点**转为 GZ `/model/agc_car/cmd_vel`（`gz.msgs.Twist`，无戳、模型系）。
- 车体位姿由物理引擎产生；"唯一执行口"语义由"唯一写车体位姿者"重述为"**唯一命令车体运动者**"，**唯一例外**为场景复位的位姿写入（受运行期门控）。

## Considered Options
- 保留 `kinematic_sim`（数学积分 + `set_model_state`）—— 被否：物理仿真效果差、动力学不可信，与 C6 的真实逃逸口径不符。
- 用 Nav2 controller 在线主控车体 —— 被否：与 [ADR-0004](./0004-rl-primary-controller-nav2-safety-layer.md) 冲突（RL 为地面主控）。

## Consequences
- W2 重设车 SDF（底盘 + 轮 + 关节 + DiffDrive 插件）；契约 [唯一执行口](../ROS2迁移交接文档.md#execution-port) 与 [GLOSSARY](../../GLOSSARY.md) 的"唯一执行口"语义同步重述；契约 [已决事项](../ROS2迁移交接文档.md#decisions) 增列本 ADR。
- odom 改取 DiffDrive 插件输出（质量更好、带轮速），同时保留世界位姿**真值旁路**供 W4 与评估对照。
- 旧门禁判据 `drive_backend == kinematic_sim` 作废；M0 改判"命令口就绪 / odom 有效 / reset 完成"等。
- W5 安全层 `collision_monitor` 在真实动力学下才具备真实逃逸能力（C6）。
- W8 新旧对照中，车体轨迹数值差异按新基线**放宽**（旧数学仿真本就可疑）。