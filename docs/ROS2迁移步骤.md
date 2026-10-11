# ROS2 重构 Runbook（唯一执行入口）

> **角色**：本项目唯一的**执行入口（Runbook）**（见 [GLOSSARY](../GLOSSARY.md)）。只编排"现在做什么、如何验收"。
> **不承载**：任何约束 —— 约束一律**引用**契约真源 [ROS2迁移交接文档](./ROS2迁移交接文档.md)，不复述（[ADR-0002](./adr/0002-docs-source-of-truth-layering.md)）。
> **读者**：agent（按本文推进，每步须可判定）为主，人工以检查视角复核。
> **引用约定**：跨文档引用用稳定锚点（`<文件>#<锚点>`），禁止 §章节号。
> **基线**：ROS1 参考实现已冻结只读，**不得删改，不得复制代码**（[ADR-0001](./adr/0001-greenfield-ros2-rewrite.md)）。

---

<a id="usage"></a>
## 0. 用法

1. 按 workstream 推进；每个 workstream 各有规格 `.trae/specs/<ws>/{spec,tasks,checklist}.md`。
2. 卡片的"验收"必须**可命令化判定**；不通过的 workstream 不得进入其下游。
3. 完成一个 workstream，就在 [状态表](#status) 打勾。
4. 遇到约束问题，查契约真源对应锚点，**不改约束、改实现**。

<a id="principles"></a>
## 1. 原则

- **单一项目根** `/home/chen/AGC`（colcon 根即项目根）。
- **全新重写**：旧树只读归档，仅作 W8 行为对照。
- **Walking Skeleton 先行**（[M0](#m0)）：先打通一条最小端到端闭环，再逐域加厚。
- **Nav2 吸收**：按 [ADR-0008](./adr/0008-nav2-paradigm-absorption-as-code-principle.md) 的吸收矩阵，在 W1–W6 **代码层**落地，每个 workstream spec 必须含"Nav2 吸收点"一节。
- **纯仿真优先**，为实机预留接口（[ADR-0009](./adr/0009-sim-first-with-hardware-hooks.md)）。

<a id="status"></a>
## 2. Workstream 总览与状态

| WS | 名称 | 前置 | 挂载硬约束 | 状态 |
|---|---|---|---|---|
| 环境 | P0 工具链（Jazzy/ros_gz/Nav2/PX4/uXRCE） | — | — | ✅ 已就绪（附录 B） |
| **W0** | 仓库与工程规范 | 环境 | C2 | ✅ |
| **W1** | 消息与接口（`agc_msgs`） | W0 | C2 | ✅ |
| **M0** | 最小端到端闭环（Walking Skeleton） | W0·W1·W2(最小)·W5(桩) | — | ☐ |
| **W2** | 仿真世界与模型 | W0 | C3·C7(场景层) | ☐ |
| **W3** | 空中链路 | W2 | C1·C10 | ☐ |
| **W4** | 感知与融合 | W2 | — | ☐ |
| **W5** | 地面导航与安全层 | W2·W4·W7 | C6 | ☐ |
| **W6** | 任务与评估 | W5 | C4·C5·C7·C8·C9 | ☐ |
| **W7** | RL 平台 | W2 | — | ☐ |
| **W8** | 对照与冻结 | W6·W7 | C1–C10（容差） | ☐ |

### 依赖图

```
W0 ─┬─> W1 ─────────┐
    │               ├─> M0 骨架闭环 ──> 逐域加厚
    └─> W2 ─┬───────┘
            ├─> W3 空中链路
            └─> W4 感知融合 ─┐
                             ├─> W5 地面导航 ──> W6 任务评估 ──┬─> W8 对照冻结
W7 RL 平台 ─────────────────┘（提供策略）                      ┘
```

---

<a id="workstreams"></a>
## 3. Workstream 卡片

### W0 · 仓库与工程规范
- **目标**：确定目录骨架、包边界、命名与产物规范、git 与 `.gitignore`。
- **产出**：目录骨架 + 规范文档 + git 初始化 + `.gitignore` + 旧树归档至 `reference/old_ros1/`。
- **非目标**：CI/CD 流水线（另立 workstream）。
- **验收**：`colcon list` 只列出预期包；`experiments/**/config/` 可入库而其余被忽略（[ADR-0007](./adr/0007-experiment-artifacts-convention.md)）。
- **Nav2 吸收点**：参数声明规范、接口分层约定。
- **spec**：`.trae/specs/w0-repo-and-conventions/`

### W1 · 消息与接口
- **目标**：`agc_msgs` 独立包 + 9 个 rosidl 消息，字段级对齐契约。
- **产出**：`agc_msgs`。
- **验收**：`ros2 interface show agc_msgs/msg/AgcTaskState` 与契约 [消息](./ROS2迁移交接文档.md#messages) **逐字段一致**。
- **Nav2 吸收点**：topic/service/**action** 职责划分。
- **spec**：`.trae/specs/w1-msgs/`

<a id="m0"></a>
### M0 · 最小端到端闭环（Walking Skeleton）
- **目标**：起世界 → 出 odom → 收任务门控 → 下发 `cmd_vel`（占位/直通策略）→ 落一条评估。
- **前置**：W0、W1、W2（最小）、W5（桩）。
- **产出**：一条可复现的 run。
- **验收**：一条 `ros2 launch` 起全链；`/agc/ground/cmd_vel` 有输出；产物落 `experiments/<campaign>/<run_id>/`。
- **意义**：闭环一旦存在，后续硬约束才有**可验证**的载体（[ADR-0006](./adr/0006-domain-workstream-organization.md)）。
- **spec**：`.trae/specs/m0-walking-skeleton/`

### W2 · 仿真世界与模型
- **目标**：`agc_world` + 车 / UAV 模型迁 GZ Sim；`ros_gz` 桥；传感器链路。
- **产出**：可加载的世界与模型、桥配置。
- **验收**：无头 / GUI 均可起；`/clock` 同步；点云话题有数据（QoS 见 [风险](#risks)）。
- **挂载**：C3（非无头验收）、C7（出生间距，场景层）。
- **Nav2 吸收点**：TF / costmap 输入约定（`OccupancyGrid`、`PointCloud2`）、sensor QoS。
- **spec**：`.trae/specs/w2-sim-world/`

### W3 · 空中链路
- **目标**：PX4 uXRCE-DDS 链路 + LQR 轨迹跟踪 + 跟随（[ADR-0005](./adr/0005-px4-link-uxrce-dds.md)）。
- **产出**：起飞 / 悬停 / 接管闭环。
- **验收**：悬停稳定、Offboard 可切、8 字轨迹恒速。
- **挂载**：C1（8 字恒速）、C10（跟随默认关闭 / 新鲜度冻结）。
- **Nav2 吸收点**：**lifecycle managed node**、pluginlib、QoS 标定。
- **spec**：`.trae/specs/w3-air-link/`

### W4 · 感知与融合
- **目标**：odom / gt / perception / fusion / map 适配节点。
- **产出**：各话题与频率。
- **验收**：各话题频率与契约口径一致（±容差）。
- **Nav2 吸收点**：**costmap 分层语义**（static/obstacle/inflation/voxel）、map_server 约定。
- **spec**：`.trae/specs/w4-perception-fusion/`

### W5 · 地面导航与安全层
- **目标**：`RL → cmd_vel → collision_monitor → /agc/ground/cmd_vel`（[ADR-0004](./adr/0004-rl-primary-controller-nav2-safety-layer.md)）。
- **产出**：RL 主控链路 + Nav2 `collision_monitor` 安全层 + 离线对照基线接入。
- **验收**：全程仅 [唯一执行口](./ROS2迁移交接文档.md#execution-port) 一个；**无第二写入者**；安全层触发可复现。
- **挂载**：C6（安全滤波器真实逃逸能力）。
- **Nav2 吸收点**：直接复用 `collision_monitor`；planner/controller 的 **plugin 抽象**（用于离线基线切换）。
- **spec**：`.trae/specs/w5-ground-nav-safety/`

### W6 · 任务与评估
- **目标**：`task_manager`、`agc_run_logger`、12 个场景、评估口径；修复已知竞态（契约 [不可回退教训](./ROS2迁移交接文档.md#lessons)）。
- **产出**：状态机 + 日志 + 场景 + 评估脚本。
- **验收**：终态语义正确；报告含 `shield_infeasible_rate` 与 `reset_failure_rate`。
- **挂载**：C4（超时算失败）、C5（相位复位 t=0）、C7（出生间距 / 复位重试）、C8（终态语义）、C9（报告指标）。
- **Nav2 吸收点**：**行为树抽象**（`task_manager` 候选）、恢复行为模式。
- **未决**：`task_manager` 架构（自研状态机 vs Nav2 BT）—— 在 W6 spec 内定夺。
- **spec**：`.trae/specs/w6-task-eval/`

### W7 · RL 平台（重点优化）
- **目标**：策略、训练 / 评估流水线、论文产线。
- **产出**：可复现的训练与评估。
- **验收**：同 seed 可复现；指标齐全。
- **未决**：算法 / 环境 / 指标 / 论文方向 —— 在 W7 spec 内定夺。
- **spec**：`.trae/specs/w7-rl/`

### W8 · 对照与冻结
- **目标**：新旧实现同场景、同种子各跑 N 次；出对比报告；ROS1 归档只读。
- **验收**：频率 / 数值差异在容差内；终态语义一致。
- **挂载**：C1–C10 全量容差复核。
- **spec**：`.trae/specs/w8-parity-freeze/`

---

<a id="hard-constraint-mounting"></a>
## 4. 硬约束挂载表

> 约束原文见契约 [硬约束](./ROS2迁移交接文档.md#hard-constraints)（C1–C10 即其 1–10 条）。**原则：挂到"首次使其可验证"的 workstream，未到该 workstream 前不强行回归。**

| ID | 约束摘要 | 挂载点 |
|---|---|---|
| C1 | 8 字轨迹恒速 | W3 |
| C2 | 距离阈值顺序（紧急<碰撞<安全） | W0·W1 |
| C3 | 实证/验收用非无头（GUI） | W0（规范）· W2（实测） |
| C4 | 超时算失败、显式完成才算成功 | W6 |
| C5 | 每 episode 相位复位 t=0 | W6 |
| C6 | 安全滤波器用真实逃逸能力 | W5 |
| C7 | 出生间距 ≥2.4m；复位失败 ≤3 次不占预算 | W2（场景层）· W6 |
| C8 | 终态"抵近并持续跟踪"；`SUCCESS` 不得称"拦截完成" | W6 |
| C9 | 报告含 `shield_infeasible_rate`、`reset_failure_rate` | W6 |
| C10 | 跟随默认关闭；`ref_max_age_s=0.5`、`confidence≥0.5` 冻结 | W3 |

---

<a id="risks"></a>
## 5. 风险清单

| 级别 | 风险 | 缓解 |
|---|---|---|
| 高 | Nav2 仅 2D，空中不可用 | 已定"模式全覆盖·进程按域选用"（[ADR-0003](./adr/0003-nav2-pattern-complete-absorption.md)）；空中保留 LQR + 自研避障 |
| 高 | PX4 由 MAVROS 换 uXRCE-DDS，控制时序/模式切换需重调 | W3 单列里程碑，先打通悬停再接管 |
| 高 | Gazebo Classic → GZ Sim 语义差异大 | W2 先迁世界与模型，再补传感器 |
| 中 | uORB 语义（微秒时间戳、NED/FRD↔ENU、QoS）易错 | W3 内建语义映射测试 |
| 中 | QoS 默认不兼容（sensor/reliable） | 逐话题标定 profile |
| 中 | RL 主控与安全层耦合 | 安全层独立可测，接口先行 |
| 中 | 时序 / executor 选择致频率数值漂移 | W8 同场景实跑对比，未验证不删旧实现 |
| 低 | `package.xml` / 依赖声明不完整 | `colcon` 逐包构建定位 |

---

<a id="rollback"></a>
## 6. 回滚与冻结策略

- 每 workstream 用 git 分支隔离（如 `ws/w0-conventions`），验收通过再合并。
- 旧树在 W8 通过前**保持可运行，不得删改**。
- 未完成新旧对比前，不得停用旧实现。
- 关键产物（对比报告、验收记录）落 `docs/`，实验产物落 `experiments/`（[ADR-0007](./adr/0007-experiment-artifacts-convention.md)）。

---

<a id="appendix-a"></a>
## 附录 A：ROS1 → ROS2 映射表

> **免责**：仅供迁移参考，**不作为代码或规范来源**；具体吸收点以各 workstream spec 的「Nav2 吸收点」节为准。

| ROS1 | ROS2 | 说明 |
|---|---|---|
| `catkin` / `catkin_tools` | `ament_cmake` / `ament_python` / colcon | `package.xml` 升 format 3 |
| `roscpp` | `rclcpp` | 逐节点改写 |
| `rospy` | `rclpy` | RL / 工具脚本 |
| `ros::init` / `NodeHandle` | `rclcpp::init` / `Node` | 生命周期与 executor 需重设 |
| `message_generation` / `message_runtime` | `rosidl_default_generators` / `rosidl_default_runtime` | `.msg`/`.srv` + rosidl |
| XML `.launch` + `rosparam` | `launch.py` + `parameters=[{}]` | 参数 YAML 结构重排 |
| `ros::param` / `getParam` | 声明式参数 `declare_parameter` | 需显式声明与类型 |
| `tf` | `tf2_ros` | 帧约定不变 |
| `/use_sim_time` | `use_sim_time` 参数 + `/clock` | ros_gz 桥接 |
| `gazebo_msgs` | `ros_gz_interfaces` / `ros_gz_sim` | `gt_adapter`、`scenario_reset` 重写 |
| `mavros_msgs` | `px4_msgs` + uXRCE-DDS | 空中链路重写 |
| Gazebo Classic 传感器插件 | gz-sim 插件 + `ros_gz_bridge` | Velodyne 链路重做 |
| `rosrun` / `roslaunch` | `ros2 run` / `ros2 launch` | 脚本全面改写 |
| `rostopic` / `rosservice` / `rosbag` | `ros2 topic` / `service` / `bag` | 评测与 CI 脚本改写 |

---

<a id="appendix-b"></a>
## 附录 B：环境就绪状态（已完成）

| 项 | 状态 |
|---|---|
| ROS 2 Jazzy + colcon + ros_gz | 已装；`gz sim` 为 Harmonic 8.15.0（来自 `ros_gz` vendor，**须先 `source /opt/ros/jazzy/setup.bash`**） |
| Nav2 | `ros-jazzy-nav2-bringup` 已装 |
| PX4-Autopilot | v1.15.4（含所需子模块），SITL 可执行文件已构建 |
| micro-XRCE-DDS Agent | v2.4.2 已编译；入口 `toolchain/bin/MicroXRCEAgent` |
| `px4_msgs` | release/1.15，已在 `src/` 且通过 colcon build |
| 工作空间 | colcon 根 = `/home/chen/AGC`；`anti_drone_agc_workspace/`、`toolchain/` 各放 `COLCON_IGNORE` |

**环境坑（复现须知）**

1. 本机无需也无法单独 `apt install gz-harmonic`；Harmonic 随 `ros_gz` vendor 引入，**必须先 source** 否则 `libgz-sim8-gui.so.8` 找不到。
2. Micro-XRCE-DDS Agent v2.4.2 需 `-include` 一个 `fmt` 兼容头（fmt 9 移除了隐式 ostream formatter），并用系统 spdlog/fastcdr/fastrtps、关闭 SuperBuild。
3. PX4 浅克隆若缺 tag，CMake 的 `git describe` 会失败 → 需 fetch 对应 tag。
4. 本机直连 GitHub 极慢（~25 KB/s）→ 外部仓库统一走 Gitee 镜像，pip 走清华源。

---

<a id="appendix-c"></a>
## 附录 C：目录速览

| 路径 | 作用 |
|---|---|
| `GLOSSARY.md` | 术语表 |
| `AGENTS.md` | 入口索引与铁律（每次会话自动加载） |
| `docs/ROS2迁移交接文档.md` | 契约真源（不变量） |
| `docs/ROS2迁移步骤.md` | 本 Runbook |
| `docs/adr/` | 架构决策记录 |
| `docs/conventions/` | 工程规范（W0 产出） |
| `.trae/specs/<ws>/` | per-workstream 规格三件套 |
| `src/` | ROS2 包（`agc_msgs`·`agc_sim`·`agc_air`·`agc_perception`·`agc_ground`·`agc_task`·`agc_rl` + 第三方 `px4_msgs`） |
| `experiments/` | 实验产物唯一落点 |
| `reference/` | 归档旧树（只读） |
| `toolchain/` | 外部仓库 / 工具链 |
