# ROS2 交接文档（契约真源）

> **角色**：本项目唯一的**契约真源（Contract Source of Truth）**（见 [GLOSSARY](../GLOSSARY.md)）。承载不可变约束：命名空间与坐标系、唯一执行口、参数入口、接口语义、硬约束、终态语义、不可回退教训。
> **不承载**：路线图、重构步骤、风险与排期 —— 见《步骤文档》(Runbook) 与 `.trae/specs/`。
> **引用约定**：跨文档引用一律用**稳定锚点**（`<文件>#<锚点>`），**禁止** §章节号（见 [ADR-0002](./adr/0002-docs-source-of-truth-layering.md)）。
> **基线**：ROS1(Noetic) 参考实现已冻结为只读归档（`reference/old_ros1/`），仅供 W8 行为对照，**不作代码来源**（见 [ADR-0001](./adr/0001-greenfield-ros2-rewrite.md)）。

---

<a id="namespace"></a>
## 1. 命名空间与坐标系

- 协同层统一命名空间 `/agc/`。
- **坐标系基准**：`agc_world`（Gazebo world，ENU：+x 东、+y 北、+z 上；位置 m、速度 m/s、角度 rad）为唯一基准。
- **机体系命名固定**：`drone/base_link`、`car/base_link`，**禁止裸 `base_link`**。
  > PX4 SITL 会一次性发布 `map` / `odom` / `base_link_frd` 等历史静态帧；协同层不发布裸 `base_link`。
- 含 Header 的协同位置/速度/地图/路径/状态消息 `frame_id=world` 或 Gazebo `world`；`/agc/ground/cmd_vel` 用车体系 `TwistStamped`。

<a id="execution-port"></a>
## 2. 唯一执行口

- **地面唯一执行口**：`/agc/ground/cmd_vel`；**唯一**命令车体运动的通道为地面执行中继（原 `ground_drive_adapter`），任何其它节点**不得**命令车体运动。车体位姿由 GZ Sim 物理引擎产生；**唯一例外**为场景复位的位姿写入（受运行期门控，见 [任务门控](#task-gating)）。
- **空中控制权**：起飞阶段归起飞控制器（原 `drone_launch_controller`）；`release_control` 后由 LQR 轨迹跟踪节点接管。

<a id="params"></a>
## 3. 参数入口

由 launch **单一设置**，节点**不得覆写**：

`/agc/target/source`、`/agc/obstacles/source`、`/agc/perception/sensors_file`、`/agc/air/takeoff_altitude_m`、`/agc/air/follow/*`、`/agc/scenario/dynamic_obstacles`、`/agc/world_file`、`/agc/log/seed`

<a id="task-gating"></a>
## 4. 任务门控与场景复位

- 任务门控 `/agc/task/start`；状态 `/agc/task/state`（`AgcTaskState`）。
- 场景复位 `/agc/scenario/reset`；`scenario_reset` 受**运行期门控**（非运行期不得写位姿）。

<a id="messages"></a>
## 5. 消息与服务

协同层消息（`air_ground_core`，迁移后独立为 `agc_msgs`）共 9 个，**字段与顺序不得变更**：

`AgcTaskState`、`AirObservationArray`、`ObstacleObservation`、`ObstacleTrack`、`ObstacleTrackArray`、`SensorObservation`、`TargetObservation`、`TargetTrack`、`TargetTrackArray`

> 其余包的消息/服务清单见附录 A。迁移后**接口语义不得变更**。

<a id="hard-constraints"></a>
## 6. 硬约束（逐条复验）

1. 8 字轨迹全程恒速、无加减速；角速度 0.1（线速度 0.3 m/s）。
2. 距离阈值顺序必须满足 紧急 < 碰撞 < 安全（默认 0.7m < 0.9m < 2.4m）。
3. 实证/验收**使用非无头（GUI）**；默认仍无头以向后兼容。
4. 训练与评估成功定义统一：**超时一律算失败**，只有显式完成才算成功。
5. 每个 episode 起始必须把轨迹相位复位到 t=0。
6. 安全滤波器参数须用**真实逃逸能力**（如 `lateral_speed = min(max_offset_xy, max_offset_change/dt)`）。
7. 场景生成须保证**出生间距 ≥2.4m**；复位失败最多重试 3 次且不占 episode 预算。
8. 状态机终态为"抵近并持续跟踪"，`SUCCESS` **不得**表述为"拦截完成"。
9. 评估报告须含 `shield_infeasible_rate` 与 `reset_failure_rate`。
10. 无人机跟随默认关闭（`--follow` 开启）；跟随要求目标新鲜（`ref_max_age_s=0.5`）且 `confidence≥0.5`，过期即冻结。

> **回归挂载**：每条约束挂到"首次使其可验证"的 workstream，映射表见 Runbook 的 `#hard-constraint-mounting`。

<a id="terminal-semantics"></a>
## 7. 终态语义

- 地面任务终态统一为 `TRACKING_SUCCESS`（"抵近并持续跟踪"）。
- 阶段 A **禁用** `INTERCEPT_CONFIRMED`；任何文档/报告**不得**把 `SUCCESS` 表述为"拦截完成"。

<a id="lessons"></a>
## 8. 不可回退教训

- **已知竞态（必须在 ROS2 版修复）**：`/agc/task/start` 门禁先校验 odom 再调 `/agc/scenario/reset`，导致 `odom_valid` 被无条件置 false、`evaluate()` 读到 `ODOM_INVALID`。**必须改为 reset 后再校验。**
- **已修复教训（必须保留门控）**：非运行期写车体位姿会覆盖 `car_start`，必须用 `/agc/task/state` 运行期门控。

<a id="decisions"></a>
## 9. 已决事项

| 议题 | 结论 | 记录 |
|---|---|---|
| PX4 链路 | uXRCE-DDS + `px4_msgs`，弃 MAVROS | [ADR-0005](./adr/0005-px4-link-uxrce-dds.md) |
| 工作空间形态 | 单一项目根 `/home/chen/AGC`；ROS1 只读归档 | [ADR-0001](./adr/0001-greenfield-ros2-rewrite.md) |
| Nav2 范围 | 模式全覆盖 · 进程按域选用 | [ADR-0003](./adr/0003-nav2-pattern-complete-absorption.md) |
| RL 角色 | 地面主控(c2)；Nav2 作安全层与离线基线 | [ADR-0004](./adr/0004-rl-primary-controller-nav2-safety-layer.md) |
| 地面驱动 | 真实物理（GZ DiffDrive），弃 `kinematic_sim` | [ADR-0010](./adr/0010-ground-drive-real-physics.md) |
| Nav2 吸收（代码层） | 交叉原则，W1–W6 吸收矩阵 | [ADR-0008](./adr/0008-nav2-paradigm-absorption-as-code-principle.md) |
| 实机 / 纯仿真 | 纯仿真优先，预留接口 | [ADR-0009](./adr/0009-sim-first-with-hardware-hooks.md) |
| 文档真源分层 | 每类信息一个家 + 稳定锚点引用 | [ADR-0002](./adr/0002-docs-source-of-truth-layering.md) |
| 执行组织 | 职责域 workstream（W0–W8） | [ADR-0006](./adr/0006-domain-workstream-organization.md) |
| 实验产物 | 唯一落点 `experiments/<campaign>/<run_id>/` | [ADR-0007](./adr/0007-experiment-artifacts-convention.md) |
| `task_manager` 架构 | **未决**，延后至 W6 spec | — |

---

<a id="appendix-a"></a>
## 附录 A：ROS1 基线盘点（非约束，供 W8 对照）

> **免责**：仅供 W8 行为对照，**不作为代码或规范来源**。

### A.1 运行环境（迁移前）

| 项 | 现状 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS (focal) |
| ROS 发行版 | ROS1 Noetic |
| 构建系统 | catkin_tools（产物 `build/`、`devel/`） |
| 仿真平台 | Gazebo Classic（仓库内 vendored `gazebo_ros_pkgs`） |
| 飞控仿真 | PX4 SITL（机型 `iris_3d_lidar`） |
| 中间件 | MAVROS（`/mavros/*`） |
| 时钟 | 全局 `/use_sim_time=true` |
| 语言分布 | C++ 约 140 个文件；Python 约 87 个文件 |

### A.2 包清单与职责

| 包 | 语言 | 职责 |
|---|---|---|
| `air_ground_core` | C++ | 空地协同核心：感知融合、任务状态机、地面规划/执行、日志与评估 |
| `lqr_trajectory_tracker` | C++ | LQR 轨迹跟踪（linear/circular/helical/8 字/step 悬停） |
| `drone_launch_controller` | C++ | 起飞/解锁/健康检查/模式切换 |
| `car_launch_controller` | C++ | 旧地面控制器（已废弃） |
| `rl_obstacle_avoidance` | Python | 无人机感知 + 规则避障 + A* + DQN 训练 |
| `rl_ground_avoidance` | Python | 地面车 DQN 避障（镜像） |
| `gazebo_ros_pkgs` | C++ | Gazebo Classic ROS 集成（vendored，删除） |
| `velodyne_gazebo_plugins` | C++ | Velodyne Gazebo Classic 插件（删除） |

### A.3 协同核心节点清单（`air_ground_core`，全部 C++）

`air_odom_adapter` / `ground_odom_adapter`、`gt_adapter`、`air_perception` / `air_fusion`、`map_adapter`、`task_manager`、`ground_planner`、`ground_commander` / `ground_drive_adapter`、`intruder_driver`、`scenario_reset`、`agc_run_logger`、`air_control_bridge`、`air_follow`、`car_commander`（已废弃）

### A.4 其余消息与服务

| 包 | 消息 | 服务 |
|---|---|---|
| `drone_launch_controller` | `DroneHealthStatus`、`FlightStatus` | `ArmMotors`、`ConnectionStatus`、`DroneHealthCheck`、`Takeoff` |
| `car_launch_controller` | `CarHealthStatus`、`CarStatus` | `CarHealthCheck`、`ConnectionStatus`、`Drive`、`EnableMotion` |
| `rl_obstacle_avoidance` | `ObstacleFeatures` | `PlanPath` |

### A.5 启动编排（迁移前）

- 唯一 run 入口：`src/air_ground_core/launch/agc_collab.launch`（XML）+ `scripts/start_agc_collab.sh`
- 单体实验入口：`scripts/start_sim.sh`（791 行，sim/takeoff/lqr/perception 四模式）
