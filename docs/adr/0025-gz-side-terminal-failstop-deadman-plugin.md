# GZ 侧末端失效停车：意图/执行分离 + System 插件死区守门人（R-009）

## Status
accepted

## Context

评审风险 **R-009（执行中继失效）**：地面唯一执行中继 `ground_drive_relay` 死亡时，末端（GZ `DiffDrive`）**无命令超时保证**——`DiffDrive` 保持最后一次命令。

**静态取证**（本机）：`libgz-sim8-diff-drive-system.so` 参数/token 清单仅含 `max_linear_velocity` / `max_angular_velocity` / `odom_topic` 等，**无 `timeout` / `deadman`**（见 `experiments/w5-ground-nav-safety/20261015-w5-r009-0/metrics/r009_relay.json` 的 `evidence`，及 `r009_evidence_dynamic.json` 的 `evidence_static`：`has_timeout_token=false`、`has_deadman_token=false`）。

**动态取证**（机制落地**前**采集，`experiments/w5-ground-nav-safety/20261015-w5-r009-evidence/metrics/r009_evidence_dynamic.json`）：车以真值 **1.439 m/s** 满速稳态、`SIGKILL` 中继后 **5 s 采样窗内持续运动**（`motion_duration_s = 5.001`，末端仍 **1.458 m/s**），累计位移 **7.45 m**，且 `cmd_out` 末值保持 1.5 —— **末端持续执行最后命令，危害确证**（评审原述「末端持续运动为中置信度风险、未实跑」由此转为已取证事实）。

## Decision

在**唯一执行通道内部**（GZ 服务器进程内）建立**末端失效停车**，采用「**双话题 + 死区看门狗**」：让守门人插件成为 `DiffDrive` 命令口的**唯一写入者**，而「意图」走另一条 gz 话题。

- **意图 / 执行分离**：ROS 侧话题名**不变**（`/model/agc_car/cmd_vel`，中继照旧发布），由 `ros_gz_bridge`（`src/agc_sim/config/bridge.yaml`）把其 **gz 侧**改桥到**意图话题** `/agc/sim/cmd_intent`。
- **守门人插件** `agc_cmd_deadman`（`src/agc_sim/src/agc_cmd_deadman.cpp`，`gz::sim::System` + `ISystemConfigure` + `ISystemPreUpdate`）：
  订阅意图话题；`PreUpdate` 中 **fresh（距上次收到 < timeout）→ 转发缓存值；stale → 发零速** 到 `DiffDrive` 默认订阅的 gz 话题 `/model/agc_car/cmd_vel`（**不改 SDF 的 `DiffDrive` 主题**）。
- **超时** `timeout` 由 SDF `<timeout>` 注入，值源自 `src/agc_sim/config/car.yaml`（`model.cmd_deadman_timeout_s = 0.3`）→ `world_bringup.launch.py` 的 xacro arg（**参数单一来源**，契约 [#params](../../docs/ROS2迁移交接文档.md#params)），与中继 `watchdog_s = 0.3` 同量级。
- 时钟用 `std::chrono::steady_clock`（与仿真暂停 / ROS 时钟冻结解耦）。

`DiffDrive` 默认订阅话题的核验：既有可运行链路——桥把 ROS `/model/agc_car/cmd_vel` 桥到 **gz `/model/agc_car/cmd_vel`** 可驱动车（W2 起生效），且 `libgz-sim8-diff-drive-system.so` token 含 `/model/`、`/cmd_vel`、`/odometry`、`DiffDrive subscribing to twist messages on [` —— 故 `DiffDrive` 订阅的是 gz `/model/<model>/cmd_vel`。

## 边界声明（非第二 ROS 命令者）

本插件 **SHALL NOT** 构成第二 ROS 命令者（契约 [#execution-port](../../docs/ROS2迁移交接文档.md#execution-port)）：

- **只「转发 ROS 已下达的值」或「归零」**，**永不自创运动**（无独立目标 / 轨迹 / 控制律）；
- **不产生任何 ROS 话题 / 节点**（仅在 gz-transport 内订阅意图、发布命令）；
- 因位于 **GZ 服务器进程内**（与物理引擎同生命周期），ROS 侧全部死亡时仍生效——这是它能作为"保证"的**唯一**原因；任何 ROS 节点 / 独立进程都可能同样被 `SIGKILL`，不能作为保证。

## Alternatives

- **在 `/model/agc_car/cmd_vel` 上自回环** —— 被否：插件会把自己的零 / 上次值当"心跳"，语义脆弱，易掩盖真实失源。
- **ROS 侧守门 / 进程信号处理（如捕获 `SIGTERM` 后发零）** —— 不足且不可靠：验收用 `SIGKILL` **不可捕获**；且节点死亡与末次零之间仍有竞态，节点本身亦可能是故障源。
- **依赖中继 `watchdog_s`** —— 不足：中继进程死亡则其看门狗逻辑随进程消失。
- **依赖 Nav2 `LifecycleNode` / bond** —— 未采纳：需额外 ROS 侧监督者，仍属"ROS 侧保证"，`SIGKILL` 下不成立（对应本 spec「Nav2 吸收点」中的候选，取证后弃用）。

## Consequences

- `src/agc_sim/config/bridge.yaml`：`/model/agc_car/cmd_vel` 的 `gz_topic_name` 改为 `/agc/sim/cmd_intent`（ROS 侧名不变）。
- `src/agc_sim/models/agc_car/agc_car.sdf.xacro`：新增插件元素与 `cmd_deadman_timeout_s` xacro arg。
- `src/agc_sim/config/car.yaml`：新增 `model.cmd_deadman_timeout_s`。
- `src/agc_sim/launch/world_bringup.launch.py`：注入 xacro arg + **追加** `GZ_SIM_SYSTEM_PLUGIN_PATH` 含 `<pkg prefix>/lib`（gz_sim.launch.py 会再并入 `LD_LIBRARY_PATH`，内置系统插件仍可加载）。
- `src/agc_sim/CMakeLists.txt`：新增共享库 `libagc_cmd_deadman.so`（`install(TARGETS ... DESTINATION lib)`），不破坏既有 world/models/config/launch 安装。
- 契约 / 冻结接口**未改**；`DiffDrive` 主题与 ROS 侧 `cmd_vel` 话题名**未改**（check 脚本 / 中继不受影响）。

## Evidence

- **危害（机制落地前）**：`experiments/w5-ground-nav-safety/20261015-w5-r009-evidence/metrics/r009_evidence_dynamic.json`（5 s 持续运动、位移 7.45 m、末端 1.458 m/s）。
- **机制验收（GUI，`scripts/run_w5_r009.sh gui 20261015-w5-r009-0`）**：`experiments/w5-ground-nav-safety/20261015-w5-r009-0/metrics/r009.json` —— K1 `SIGKILL` 中继 → 真值停稳 **0.849 s**（上界 2.5 s）、残余位移 **0.729 m**（上界 2.0 m）；K2 正常回归（0.8 m/s 稳态 + 零命令停住，守门人不误触发）；K3 重启中继恢复（0.795 m/s）。中继自身 S1–S6 见同目录 `r009_relay.json`（全 PASS）。
