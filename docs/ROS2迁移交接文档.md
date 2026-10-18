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

1. 8 字轨迹为**真八字**（Gerono，`rx=15` / `ry=9` / `h=2.5 m`），全程恒速、无加减速（线速度 `v=2.0 m/s`，弧长参数化）。
2. 距离阈值顺序必须满足 紧急 < 碰撞 < 安全。**顺序为不变量**；**绝对值为可标定量**，由地面安全层按 [ADR-0021](./adr/0021-ground-safety-thresholds-and-platform-recalibration.md) 的 Nav2 时间窗口径、以 W2 实测动力学标定为 `1.0 < 2.0 < 4.0 m`（ADR-0021(c) 已实测回填；旧值 `0.7/0.9/2.4 m` **作废**）。
3. 实证 / 验收**定性、关键场景**使用**非无头（GUI）**并留记录；**规模数值统计**允许 **headless**，但**须经 GUI ↔ headless 等价核验**（同参数 / 同 seed、关键指标一致），且产物**如实标注** `backend` / `mode`；默认仍无头以向后兼容（[ADR-0057](./adr/0057-c3-gui-acceptance-scope-authorization.md)）。
4. 训练与评估成功定义统一：**超时一律算失败**，只有显式完成才算成功。
5. 每个 episode 起始必须把轨迹相位复位到 t=0。
6. 安全滤波器参数须用**真实逃逸能力**（如 `lateral_speed = min(max_offset_xy, max_offset_change/dt)`）。
7. 场景生成须保证**出生净空 ≥ 可标定下限**（按新 C2 安全档 + 实体外伸推导，见 [ADR-0023](./adr/0023-spawn-clearance-recalibration.md)；旧值 `2.4 m` **作废**）；复位失败最多重试 3 次且不占 episode 预算。
8. 状态机终态为"抵近并持续跟踪"，`SUCCESS` **不得**表述为"拦截完成"。
9. 评估报告须含 `shield_infeasible_rate` 与 `reset_failure_rate`。
10. **无人机对目标的跟踪常开**（非可选；原"跟随默认关闭"作废）：保持**安全间距**（不紧贴），目的为**不丢失目标**；跟踪要求目标新鲜（`ref_max_age_s=0.5`）且 `confidence≥0.5`，过期即冻结。无人机同时负责**自身飞行安全**（自避障）。

> **回归挂载**：每条约束挂到"首次使其可验证"的 workstream，映射表见 Runbook 的 `#hard-constraint-mounting`。

<a id="terminal-semantics"></a>
## 7. 终态语义

- 阶段划分：**阶段 A = 跟踪**；**阶段 B = 拦截**（本轮启用阶段 B）。
- 地面任务**跟踪**终态统一为 `TRACKING_SUCCESS`（"抵近并持续跟踪"）；**不得**表述为"拦截完成"。
- **拦截达成**终态为 **`INTERCEPT_CONFIRMED`**（阶段 B 启用，见 [ADR-0015](./adr/0015-interception-scope-physical-capture-and-behavioral-jamming.md) 的 W6 修订），由 `reason_code` 区分方式：物理捕获 `CAPTURE_NET`（[ADR-0022](./adr/0022-launched-net-capture-mechanism.md)）/ 行为级干扰 `JAM_RTL|JAM_LAND|JAM_HOLD|JAM_LOST`；**不新增消息枚举**（[#messages](#messages) 字段 / 顺序不得变更）。
- 失败 / 中止终态：`MISSION_FAILED`（超时 / 未达成）、`ABORTED`（外部中止）、`INFRA_FAILURE`（门禁失败）。

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
| 节点语言 | C++ 实时域 / Python RL 域 | [ADR-0011](./adr/0011-node-language-policy.md) |
| PX4 本体形态 | `toolchain/` 外部依赖（pin v1.15.4）；`px4_msgs` 内联 `src/` | [ADR-0012](./adr/0012-px4-body-as-toolchain-dependency.md) |
| Nav2 吸收（代码层） | 交叉原则，W1–W6 吸收矩阵 | [ADR-0008](./adr/0008-nav2-paradigm-absorption-as-code-principle.md) |
| 实机 / 纯仿真 | 纯仿真优先，预留接口 | [ADR-0009](./adr/0009-sim-first-with-hardware-hooks.md) |
| 文档真源分层 | 每类信息一个家 + 稳定锚点引用 | [ADR-0002](./adr/0002-docs-source-of-truth-layering.md) |
| 执行组织 | 职责域 workstream（W0–W8） | [ADR-0006](./adr/0006-domain-workstream-organization.md) |
| 实验产物 | 唯一落点 `experiments/<campaign>/<run_id>/` | [ADR-0007](./adr/0007-experiment-artifacts-convention.md) |
| 定位（重写即交付物） | 软著 / 平台；论文产出于纯仿真 | [ADR-0013](./adr/0013-rewrite-as-deliverable-sim-only.md) |
| 研究主线 | 移动式主动拦截系统（D 主 / A 核）；RL·融合归后续 | [ADR-0014](./adr/0014-research-mainline-mobile-interception-system.md) |
| 拦截范围 | 物理捕获（**发射式网**）+ 行为级电子干扰（EMI 不建模）；入侵者 **PX4 化** | [ADR-0015](./adr/0015-interception-scope-physical-capture-and-behavioral-jamming.md)、[ADR-0022](./adr/0022-launched-net-capture-mechanism.md) |
| 执行范围出口 | 到 W6；W7 / W8 归后续；拦截落 W2 / W6 | [ADR-0016](./adr/0016-execution-scope-exit-w6.md) |
| 空地协同口径 | UAV 常开 standoff 跟踪 + 自保；地面自主规划 | [ADR-0017](./adr/0017-air-ground-collaboration-loop.md) |
| 通用控制层 / 共享 LQR | `agc_control` 插件化（pluginlib），air / ground 共用 | [ADR-0018](./adr/0018-shared-control-layer-and-lqr-plugin.md) |
| UAV 轨迹 / 跟踪参数 | 真八字 Gerono + 恒速 `v=2.0`；C1 修订 | [ADR-0019](./adr/0019-uav-trajectory-and-tracking-parameter-calibration.md) |
| 地面安全阈值 / 平台 | C2 厘清为"顺序不变量 + 可标定绝对值"；地面平台保形换装为差速 UGV（保 DiffDrive） | [ADR-0021](./adr/0021-ground-safety-thresholds-and-platform-recalibration.md) |
| 物理捕获机制 | 改为**发射式网**；交战包络由网发射物理模型推导；裁决器可插拔 | [ADR-0022](./adr/0022-launched-net-capture-mechanism.md) |
| 捕获裁决口径 | 现「入包络即命中」为 W6 **闭式占位**（仅产出终态 / 指标）；后续口径 = **火控窗口 + 瞄准提前量** | [ADR-0024](./adr/0024-capture-adjudication-fire-control-window.md) |
| C7 出生净空 | 语义拆分；下限改为可标定量（新 C2 + 实体外伸推导），旧值 `2.4 m` 作废 | [ADR-0023](./adr/0023-spawn-clearance-recalibration.md) |
| `task_manager` 架构 | **自研确定性 C++ 状态机**（吸收行为树模式，不引入 `bt_navigator`） | [W6 spec](../.trae/specs/w6-task-eval/spec.md) |
| C3 验收范围 | 定性 / 关键场景须 GUI；规模数值统计允许 headless（须 GUI ↔ headless 等价核验 + backend/mode 标注） | [ADR-0057](./adr/0057-c3-gui-acceptance-scope-authorization.md) |
| 捕获裁决器替换契约 | ADR-0024(c) 的「调用方不改」**范围澄清**：对外视图 / ABI 不变，内部允许 shot 生命周期与异步事件演进 | [ADR-0058](./adr/0058-capture-adjudicator-replacement-contract-and-event-semantics.md) |

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
