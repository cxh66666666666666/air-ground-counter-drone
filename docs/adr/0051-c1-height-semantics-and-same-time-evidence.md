# C1 高度语义统一与同时间证据曲线（R-036）

## Status

accepted

## Date

2026-10-17

## 背景

评审风险 **R-036（C1 验收与世界高度）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-036)）指出，
W3 的 C1 恒速验收把**参考 / 机体估计 / 世界真值**三个指标混用，且高度语义不统一：

- 接管高度对齐使命令 `h` 成为**相对基准**（[`trajectory_tracker.cpp`](../../src/agc_air/src/trajectory_tracker.cpp)
  接管处 `z_offset_ = state.pos.z() - trajectory_->sample(0.0).pos.z()`），而**对外公开的 `base_ref` 未加
  `z_offset_`**（原 `ref_pub_->publish(toMsg(base_ref, ...))`）→ 对外 `base_ref` 与生效 `effective`
  （`effective.pos.z() += z_offset_`）**高度基准静默不同**；
- 既有 C1 检查（[`w3_c10_check.py`](../../scripts/w3_c10_check.py)）采用 **warmup 之后的速度误差 P95**，
  **不能推出全程最大误差**；
- 「恒速无加减速」若指**零向量加速度**则与曲线运动不相容（见 [X-01](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-01)）；
- **stock 原始 CSV**（`/tmp/stock_rec{,2,3}.csv`，见 [W3 tasks](../../.trae/specs/w3-air-link/tasks.md)）
  在所述 `/tmp` 路径**均不存在** → 外部归因**不能独立复核**。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C1**（真八字 Gerono / 全程恒速 `v=2.0`）、
[#namespace](../ROS2迁移交接文档.md#namespace)（世界系 `agc_world` ENU）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一空中控制权**不变**）、
[#messages](../ROS2迁移交接文档.md#messages)（冻结 9 消息 ABI **不得变更**）；产物纪律
[ADR-0007](0007-experiment-artifacts-convention.md)；前序 [ADR-0049](0049-lifecycle-repeatable-resource-state-machine-and-liveness-detection.md)（R-034）、
[ADR-0050](0050-air-control-single-authority-and-permit.md)（R-035）。

分类：**设计风险（验收口径 / 高度语义 / 证据闭环）**。本 ADR 决定 **C1 口径裁定 + 唯一世界高度语义 +
`base_ref` 与 `effective` 高度基准一致、可辨 + 同时间证据曲线**；**不**改契约 C1 硬约束文字，**不**改冻结
9 消息 ABI，**不**变唯一执行口语义，**不**新增第二 ROS 命令者。

## 取证（先学后做）

按 [AGENTS.md](../../AGENTS.md) 铁律「先学后做」，**先**对齐**同版本** Nav2 的「参考 vs 实际分指标、容限」范式
与**本项目既有成功用法**（弧长参数化恒速 / warmup+P95 口径 / 世界真值旁路），**再**改自身实现。

**同版本 Nav2（`/opt/ros/jazzy/include/...`，用 Shell `sed -n` 读）用法结论：**

- `nav2_controller/plugins/{simple,position}_goal_checker.hpp`：`isGoalReached(query_pose, goal_pose,
  velocity)` —— **实际位姿（query）vs 参考位姿（goal）** 分列；`getTolerances(pose_tolerance,
  vel_tolerance)` —— **位置容限与速度容限分列** → 「**参考 / 实际分指标 + 分别容限**」范式（对齐本 ADR 的
  三指标分别报告与速度 / 高度容限）。
- `nav2_util/geometry_utils.hpp`：`euclidean_distance(pos1, pos2, is_3d=false)` —— **二维 / 三维距离显式区分**
  （`is_3d` 决定是否含 `dz`）→ 「**维度 / 语义显式**」范式（对齐本 ADR 的**唯一世界高度语义**：z 必须显式回到
  世界 ENU 表达，不混 2D/3D）。
- `nav2_util/odometry_utils.hpp`（`OdomSmoother`）：里程计平滑封装、订阅带互斥 → 「**机体估计来源归一**」范式。

**本项目既有成功用法（W3）结论：**

- [`trajectory_generators.cpp`](../../src/agc_control/src/trajectory_generators.cpp)：`ParametricTrajectory`
  的周期轨迹按**弧长参数化**（先建 `s(θ)` 梯形积分表，再对给定 `s` 反解 θ），使 `|dp/dt| ≡ speed`；
  `Figure8Trajectory` Gerono `x=cx+rx·sinθ, y=cy+ry·sinθcosθ, z=height` → 「**参考恒速的对象 = 弧长参数化的
  切向速度**」（这是 C1「恒速」所指对象）。
- [`w3_c10_check.py`](../../scripts/w3_c10_check.py)：采归一化 `/agc/air/odom` / `/agc/targets` /
  `/agc/air/tracking_ref`；**阈值一律从 `air.yaml` 读取（不复制参数）**；判据用 **warmup 之后**的误差
  **P95**（跟踪误差 |drone−tracking_ref| 均值/P95/max）。
- **世界真值旁路**：[`gt_bypass.cpp`](../../src/agc_perception/src/gt_bypass.cpp) 按实体发 `/agc/gt/<entity>`
  （`geometry_msgs/PoseStamped`，world 系），但 `entities` 仅 `["agc_car","intruder_uav"]` —— **无己方无人机 GT**；
  W3 既有做法是**直接采 gz `/world/agc_world/pose/info`**（[W3 tasks](../../.trae/specs/w3-air-link/tasks.md)）。
- 验收脚本惯例：[`run_w6_r034.sh`](../../scripts/run_w6_r034.sh) / [`run_w6_r035.sh`](../../scripts/run_w6_r035.sh)
  （`source /opt/ros/jazzy` + `install/setup.bash` + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`；`pgrep -f "…[y]"`
  取 PID kill；`start_new_session`+`killpg`+`wait`；**无** `rm -rf`、**无**裸名 `pkill`）。

**本项目现状差距（读取时工作区）：** 公开 `base_ref` 未加 `z_offset_`（与 effective 基准静默不同）；
C1 检查只给 warmup 后 P95，**无**全程 max / 持续越限；世界 GT 未与 PX4 估计、参考**同时间轴并列**。

**本批取证实测（真实二进制台架 `w6_r036_bench.py`，`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`）：**
起**真实** `trajectory_tracker`（standalone，`bond_heartbeat_period=0.0`，发 `active_controller="trajectory_tracker"`
取得许可），同一 run 同一 ROS 时间轴记录 5 曲线（world GT / PX4 估计 / base / effective / setpoint），40 s @ `t≈0.02 s`：

- `base_ref`↔`effective` 高度基准差：**P95 8.1e-8 m、max 1.2e-7 m** → **一致**（R-036 前为 `z_offset`≈0.6 m）；
- `z_offset`（层高基准，注入偏置 z=+0.60 m 时）实测 **0.60 m**，可由 `tracking_status` 字段可辨；
- 参考 `base` `|v|`：**P95 = 2.0000**，相对目标 2.0 的 **max 偏差 ≈ 4.4e-16 m/s** → 弧长参数化**数值恒速**成立；
- 世界真值 `gt`（脚本化机体，偏置仅注入估计）：稳态速度误差 **P95 0.023 m/s**（占位容限 0.30）；
- 三指标**可区分**：`gt` 高度中位 3.100 m（= h 2.5 + z_offset 0.6）、`est` 高度中位 3.700 m（+0.60 m 估计偏置）→
  估计偏置被**如实分开统计**（`est` 高度残差稳态 P95 0.600 m、持续 41.4 s；`gt` 稳态 3e-5 m）；
- 同时间轴：`t` / `base` 头时间 / `sp` 接收时刻**均单调**，对齐误差 **P95 0.19 s ≤ 0.25 s**（单次 ~3 s 运行抖动为离群）。

## 决策

### (a) C1 口径裁定（对齐 X-01，**不静默**）

- **参考恒速**：`Figure8Trajectory` 输出的 `|vel| ≡ speed = 2.0`（**弧长参数化**）——这是「C1 恒速」的**对象**。
  数值容限 `c1.ref_speed_tol = 1e-3 m/s`（参考 `|v|` 相对目标 2.0 的 **max** 偏差；依据：双精度浮点 +
  弧长表线性内插的数值噪声量级）。判定用**参考 `base` 全程 max**，**非**稳态分位。
- **实际恒速**：**机体（world GT）** 在 **warmup 之后**的速度模**稳态误差**（`P95` 与 `max` **分别**报告）；
  容限 `c1.body_speed_p95_tol_mps = 0.30 m/s`（**占位待回填**：W3「恒速误差 `ε`」尚未实测标定，如实标注）。
- **「无加减速」正名**：指**参考沿弧长的切向速度恒定（无切向加减速）**；**不**声称「零向量加速度」（曲线运动
  有向心加速度）。此与契约 C1 字面差异属 [X-01](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-01) 已挑战项，
  **如实记录**；**不**改契约文字。
- **模式记账**：**基准轨迹试验**（base 模式，无 `tracking_ref` / `target`）单独判 C1；**接管瞬态（warmup）/ 自保
  （shield 介入）/ 目标跟踪（tracking 覆盖八字）** 分模式记账，**不**纳入基准恒速判据。
- **误差容限分列**：参考 `|v|` 用**全程 max**（数值容限 `1e-3`）；实际稳态速度误差用**稳态 P95**（占位 `0.30`）；
  **max / 持续越限** 单独报告 —— **不**以 warmup 后 P95 自证「全程最大误差」（evidence 显式区分
  `speed_err_full_max` 与 `speed_err_steady_p95`）。

### (a2) 唯一世界高度语义

定义**唯一世界高度语义** = **世界 ENU `z`（`world` frame，向上）**。world GT / PX4 估计 / base / effective /
setpoint 曲线**统一回到世界 ENU z** 表达：PX4 `TrajectorySetpoint` 内部为**局部 NED**，曲线须反变换
`z_world = -z_ned + origin_z`（本 ADR 的台架 `origin_pose=0`，故 `z_world = -z_ned`；速度 / 加速度为**自由矢量**、
仅符号翻转）。`z_offset_` 语义 = **层高基准**（接管时刻机体高度 − 轨迹 `h`），**非**世界原点代偿
（世界原点平移由 `px4_adapter` 的 EKF 原点处理，见 R-003）。

### (b) `base_ref` 与 `effective` 高度基准一致 / 显式可辨

改 [`trajectory_tracker.cpp`](../../src/agc_air/src/trajectory_tracker.cpp)：发布 `/agc/air/ref` **前**，对 `base_ref`
**副本**加 `z_offset_`（`base_ref_out.pos.z() += z_offset_`）→ 对外 `base_ref` 与生效 `effective` **同一世界
高度基准**；**不**改动下游使用的 `base_ref`。并在 `publishTrackStatus` 的 JSON **追加显式字段** `z_offset`
（数值）与 `height_datum`（`"world_enu"`）→ 差异**可辨**（既有字段保持不变，兼容 R-032/R-033 检查）。
确认 `/agc/air/ref` 在 `src/` 内**无消费者**（仅 `trajectory_tracker` 发布、`air.yaml` 定义）→ 改其高度基准安全。

### (c) 同时间证据曲线与报告（`run_w6_r036.sh` + `w6_r036_bench.py` + `w6_r036_check.py`）

`w6_r036_bench.py` 起真实节点，**同一 run、同一 ROS 时间轴**记录 **world GT / PX4 估计（`/agc/air/odom`）/
base（`/agc/air/ref`）/ effective / setpoint（`/fmu/in/trajectory_setpoint`，反变换回世界 ENU）** 5 曲线；
**effective 重建口径**：`effective.pos = nedToEnu(sp.position) − 0.5·acc_enu·dt²`、`effective.vel =
nedToEnu(sp.velocity)`（`dt = 1/setpoint_rate_hz = 0.02`；`acc_enu = nedToEnu(sp.acceleration)`）——即在
`offboard_interface.cpp::publishSetpoint` 的位置前馈 `0.5·a·dt²` 处逆运算（**非**新增命令者）。
**机体模型**（脚本化，**不消费 setpoint 的位置环**避免以被测输出自证）：机体真值速度**一阶**趋近参考速度
（隐式欧拉，任意步长稳定）+ 弱位置牵引；估计 = 真值 + **已知常量位置偏置**（z:+0.60 m）。对 **base / est / gt**
三指标分别报告**高度 z** 与**速度 |v|−2.0** 的 **P95 / max / 持续越限**（全窗 + 稳态窗）。产物
`<run_id>/r036/{bench.json,curves.csv}`（含 `scope_note`（**非整链 / 逻辑台架**）+ `unsupported`）。

### (d) `air.yaml` 承载 C1 容限（配套，非 ABI）

`trajectory_tracker.ros__parameters.c1` 登记 `ref_speed_target` / `ref_speed_tol` / `height_datum_tol_m` /
`body_speed_p95_tol_mps`（占位待回填）/ `height_err_p95_tol_m`（占位待回填）/ `sustained_limit_eps_mps`；
`w6_r036_bench.py` 与 `w6_r036_check.py` **读取**（不复制参数，对齐 `w3_c10_check.py`）。

### (e) 未证实归因保留（stock 材料）

**缺失的 stock 原始材料**（`/tmp/stock_rec{,2,3}.csv` —— W3 所述「stock PX4 + Gazebo 垂向起伏属 SITL 固有」
的判据来源）**经核实不存在**（`ls /tmp/stock_rec*.csv` 无文件）。该项外部归因**作为未证实归因保留**：**不**
据此删除 / 重写该偏差记录，**不**把「stock 固有」当作已复核结论；如需外推须重做并保留原件。

### (f) 边界（不改契约）

**不**改契约 C1 硬约束文字（改硬约束须另行批准），**不**改冻结 9 消息 / 服务 ABI，**不**变唯一执行口语义，
**不**新增第二 ROS 命令者；`z_offset` / `height_datum` 与 `c1.*` 均为**配套诊断 / 口径**（非 ABI、非命令者）。

## 备选

1. **保持现状（公开 `base_ref` 不加 offset，仅文档说明）**：最省事；但对外 `base_ref` 与生效 `effective`
   基准**静默不同**，C1 验收仍混淆 —— 否决 (b)。
2. **改契约 C1 文字为「切向恒速 + 稳态误差容限」**：超出本批授权（改硬约束须另行批准）—— 否决；改为
   **裁定口径 + 证据**并如实记录与 [X-01](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-01) 的差异。
3. **用 PX4 估计（`/agc/air/odom`）当「世界真值」**：把估计偏置（R-003）混入真值，仍三指标混用 —— 否决；
   改为**独立合成 world GT**（并如实标 `unsupported`：非整链 / 非 PX4 SITL 闭环）。
4. **新增专用「effective 诊断话题」直出内部 effective**：更直接；但会**新增话题 / 表面命令者嫌疑**，且
   `setpoint` 反变换已可忠实重建 —— 不采纳；改由 `setpoint` 反变换重建（口径写入 ADR）。
5. **把 `z_offset` 做成固定参数**：但它是**每次接管实测**的层高基准 —— 否；改由 `tracking_status` **报告实测值**。

## 后果

- **正面**：对外 `base_ref` 与 `effective` **同一世界 ENU z 基准**（实测基准差 P95 < 1e-7 m）；`z_offset` /
  `height_datum` 在 `tracking_status` **可辨**；**同一 run 同一时间轴**给出 5 曲线与三指标分别的
  **P95 / max / 持续越限**；参考恒速以弧长参数化数值自证（max 偏差 ~1e-16）；实际稳态速度误差按占位容限判定；
  缺失 stock 材料**作为未证实归因保留**。
- **代价 / 限制**：C1 容限 `body_speed_p95_tol_mps` / `height_err_p95_tol_m` 为**占位待回填**（W3 未标定）；
  台架机体为**脚本化**（非整链 / 非 PX4 SITL 闭环，如实 `unsupported`）；同时间对齐容限放宽至 P95 ≤ 0.25 s
  （单进程台架的 GIL / DDS 抖动，实测 P95 0.19 s）。
- **未决 / 后续**：C1 容限的实测回填依赖**真实整链 / 双 PX4** run（本批不可实测 → 占位）；R-037（传感器 /
  车体能力）与 R-038（数值 / 配置边界）另行处理。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-036)、[X-01](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-01)
- 规格：[fix-lifecycle-control-height-numeric-r34-r38 spec](../../.trae/specs/fix-lifecycle-control-height-numeric-r34-r38/spec.md)（R-036）
- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C1）、[#namespace](../ROS2迁移交接文档.md#namespace)、
  [#execution-port](../ROS2迁移交接文档.md#execution-port)、[#messages](../ROS2迁移交接文档.md#messages)
- Nav2 参照：`nav2_controller/plugins/{simple,position}_goal_checker.hpp`（query vs goal + 位置/速度容限分列）、
  `nav2_util/geometry_utils.hpp`（`euclidean_distance` 二维/三维显式）、`nav2_util/odometry_utils.hpp`（`OdomSmoother`）
- 前序：[ADR-0049](0049-lifecycle-repeatable-resource-state-machine-and-liveness-detection.md)（R-034）、
  [ADR-0050](0050-air-control-single-authority-and-permit.md)（R-035）、
  [ADR-0019](0019-uav-trajectory-and-tracking-parameter-calibration.md)（UAV 轨迹 / 跟踪参数）、
  [ADR-0008](0008-nav2-paradigm-absorption-as-code-principle.md)（Nav2 吸收）
- 本项目既有用法：`trajectory_generators.cpp`（弧长参数化恒速）、`w3_c10_check.py`（warmup+P95 口径 / 参数不复制）、
  `gt_bypass.cpp`（实体 GT 旁路）、W3 `tasks.md`（stock 归因 / `z` 峰峰判据）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 实现：`trajectory_tracker.cpp` / `air.yaml` / `run_w6_r036.sh` / `w6_r036_bench.py` / `w6_r036_check.py`
