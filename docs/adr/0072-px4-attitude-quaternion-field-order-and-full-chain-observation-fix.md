# PX4 姿态四元数字段序（Hamilton [w,x,y,z]）解析错误与整链观测链修复

## Status

accepted

## Date

2026-10-19

## 背景

W7-b 步 1 目标：让 `backend:=full` 整链「**评价可用**」——能产出**真实任务终态**，而非恒 `MISSION_FAILED(TIMEOUT)`。取证链：

1. **空中控制权从未交接**：`trajectory_tracker` 仅在被**显式授权**（`active_controller==trajectory_tracker`）时接管；授权由 `takeoff_controller` 的 `/agc/air/release_control` 移交，但**全仓无节点调用**（`task_manager` 的 start 事务只做 reset → reset_phase → 门禁）。→ 整场由 `takeoff_controller` 悬停飞行、航向冻结，目标落在机身固连前视锥外（见 [ADR-0040](0040-full-chain-coverage-matrix-and-same-run-causal-record.md) 已登记的 `release_control` 缺口）。
2. **甲-1 修复**：由 `task_manager`（唯一任务协调者）在 `reset_phase` 后、**等待「起飞完成（悬停）」**再调 `release_control`。**为何必须等悬停**：`takeoff_controller` 是**唯一**发 ARM / Offboard-mode 的节点，提前 release 会停其 tick → 飞机**永不 arm**（实测 start 时尚未 arm、悬停发生在 start 后 ~10–14 s）；且过早接管会让 tracker 以当前（近地）高度为基准而**不**继续爬升。
3. **修甲-1 后整链仍失败**：接管成功、`start OK`，但仍 `TIMEOUT`，`visibility_rate` 仅 **1.4%**。取证定位到**姿态口径**。

## 决策

### (a) 事实：PX4 姿态四元数字段序为 Hamilton **[w,x,y,z]**

- px4_msgs / uORB `VehicleOdometry.q`：`float32[4]`，注释「First value NaN if invalid/unknown」→ `q[0]` 即实部 `w`；ekf2 工具链（`src/modules/ekf2/test/sensor_simulator/convertULogToSensorData.py`）即按 `q[0]→qw` 解析。
- 实测（`backend:=full`，己方机已按 `observe_yaw_mode=target` 指向目标）：raw `q=[0.877,0.004,0.002,0.480]`
  - 按 **[w,x,y,z]** 解析 → ENU 偏航 **32.7°**（与 PX4 自身 `vehicle_attitude` 31.9°、被令偏航 32.7°、目标方位 ≈33° 一致）；
  - 按 **[x,y,z,w]** 解析 → ENU 偏航 **89.5°**（**即 `px4_adapter` 上报值**）。

### (b) 修复：`px4AttitudeToEnu` 按 **[w,x,y,z]** 索引

- [`px4_conversions.hpp`](../../src/agc_air/include/agc_air/px4_conversions.hpp)：`Quaternion q_px4(w=q[0], x=q[1], y=q[2], z=q[3])`（原为 `w=q[3], x=q[0], y=q[1], z=q[2]`）。
- 单测：`AttitudeIdentityIsLevelFacingNorth` / `VelocityFrameConversionAtNinetyDegreeHeading` 的 identity 字面由 `{0,0,0,1}`（[x,y,z,w] 约定）改 `{1,0,0,0}`；新增 [回归用例](../../src/agc_air/test/test_px4_conversions.cpp) `AttitudeFieldOrderIsPx4HamiltonWxyz`（58.1° NED → 31.9° ENU，容差 1e-3°），可拦下字段序回退。

### (c) 影响面（**如实登记**）

`px4AttitudeToEnu` 是 `px4_adapter` 唯一的姿态归一化入口，决定 `/agc/air/odom` 的 `pose.orientation`（世界 ENU / 机体 FLU）及由其推导的机体速度。解析错 ~58° 会：

- **感知**：`air_perception` 以该姿态旋转 boresight 做 **FOV 门**（`rotateVecByQuat(drone_quat_, boresight)`）→ 目标被判在视场外 → `visibility_rate≈1.4%` → 无持续获取 → `TIMEOUT`。
- **控制**：`trajectory_tracker` 的机体速度反馈（`worldVelFromOdom`）与接管初始 `last_yaw_` 被旋转错。
- **旧证据**：任何**依赖 `/agc/air/odom` 姿态 / 机体速度**的空中结论（W3 轨迹跟踪的航向 / 机体速度、R-036 机体速度 P95 等）建立在**错误口径**上，须按新解析**重验**。本 ADR **登记**该影响；**未**在旧 run 上重跑。

### (d) 修复后整链状态（复验）

`backend:=full`、`scene=s08_static_none_nominal_capture`、`seed=0`、`target_source=air_fusion`、`observe_yaw_mode=target`：

- 同 run 因果（`campaign/run_id` = `w7-rl-b/20261019-w7-rl-b-224947-6509`）：`相位复位(/agc/air/reset_phase)` → `起飞完成/悬停` → `release_control：控制权移交 trajectory_tracker` → `task_manager：已触发 /agc/air/release_control` → `tracker：接管控制权（唯一授权）` → `start OK`。
- run_logger summary（`experiments/w6-task-eval/20261019-s08_static_none_nominal_capture-0-225005-4480`）：终态 **`TRACKING_SUCCESS`**（`tracking_milestone`）、`visibility_rate=0.982`（335/341）、发射机会 79 拍。

### (e) 参数单一注入

`task_manager` 新增 `release_service` / `air_odom_topic` / `air_hover_climb_m`（默认 2.3，对齐 `air.yaml` 的 `takeoff_altitude_m=2.5` / `hover_tolerance_m=0.25`）/ `air_hover_timeout_s`（默认 18.0）；单一来源 = [`w6_base.yaml`](../../src/agc_task/config/w6_base.yaml)，节点不得覆写（契约 [#params](../ROS2迁移交接文档.md#params)）。

## 备选

1. **保留 [x,y,z,w] 解析并按差角补偿** —— 否决：错的是**字段序**（非恒定偏置），补偿无法在所有姿态成立，且污染速度换基。
2. **只在 `air_perception` 内修姿态** —— 否决：感知 / 控制两域共用 `/agc/air/odom`，须在**唯一归一化点**（`px4_adapter`）修正。
3. **由 `takeoff_controller` 自动释放控制权（免 `task_manager`）** —— 否决：控制权交接属**任务决策**（何时开始任务），不应耦合入 `takeoff_controller` 的飞行时序；由唯一任务协调者驱动。

## 后果

- **正面**：整链**评价可用**（真实终态 `TRACKING_SUCCESS`，非 `TIMEOUT`）；`visibility_rate` **0.014 → 0.982**；新增回归单测锁字段序；单一注入参数。
- **代价 / 限制**：旧空中姿态 / 机体速度证据须**重验**（(c)）；release_control 为**每进程一次**（`takeoff_controller.released_` 锁存）→ 单 run 单 episode（多 episode 需重启 launch，与 runner 现况一致）。
- **未决 / 后续**：`s08` 期望 `INTERCEPT_CONFIRMED` 而实测 `TRACKING_SUCCESS` → 捕获裁决 / `tracking_milestone` 的先后与 `interception_authorized` 须**另行裁定**（**不**静默改场景语义）；W7-b 步 2（P1 两臂配对）与步 3（P1 判定补入 [ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)）；旧证据重验清单。

## 参考

- 契约：[#namespace](../ROS2迁移交接文档.md#namespace)（世界 `world` ENU）/ [#params](../ROS2迁移交接文档.md#params)（单一注入）/ [#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一空中控制权）/ [#messages](../ROS2迁移交接文档.md#messages)（冻结 9 消息 ABI **未改**）
- 前序 ADR：[ADR-0005](0005-px4-link-uxrce-dds.md)、[ADR-0034](0034-failsafe-factor-separation-and-runtime-param-lock.md)、[ADR-0040](0040-full-chain-coverage-matrix-and-same-run-causal-record.md)、[ADR-0050](0050-air-control-single-authority-and-permit.md)、[ADR-0051](0051-c1-height-semantics-and-same-time-evidence.md)、[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)、[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)
- 规格：[`.trae/specs/w7-rl-b/`](../../.trae/specs/w7-rl-b/spec.md)
