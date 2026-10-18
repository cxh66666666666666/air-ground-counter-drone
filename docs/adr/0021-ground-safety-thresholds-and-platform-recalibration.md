# 地面安全阈值重标定（C2 修订）与地面平台换装

## Status
accepted

## 背景与决策

**问题一（C2 阈值过时）**。契约硬约束 **C2** 原为「距离阈值顺序 紧急 < 碰撞 < 安全（默认 `0.7m < 0.9m < 2.4m`）」。核对溯源：该组数值派生自旧树**无人机**规则避障配置（`reference/old_ros1/src/rl_obstacle_avoidance/config/rule_avoidance.yaml` 的归一化阈值 `emergency_dist_norm` / `collision_dist` / `safe_distance_norm`），是为 **40×40 m 小场地 + 慢速**设定的。而 [ADR-0019](./0019-uav-trajectory-and-tracking-parameter-calibration.md) 已将作业区放大到 **80×80 m**、目标巡航 `1.0 m/s`、跟踪速度上限 `2.0 m/s`；地面车却仍是 **TB3 级基座**（`agc_sim/config/car.yaml`：体长 `0.265 m`、`max_linear_speed 1.0`）。**旧阈值在新标度下无推导依据，且数值过小。**

**问题二（平台粗糙）**。`agc_car` 现为**基本几何体拼装**（灰色 box 车体 + 圆柱轮 + 球形 caster，无 mesh），惯量全为 `1e-3` 占位值，尺度/动力学均不匹配作业区与拦截场景（地面平台需载 2 m 捕获网面与干扰载荷）。

据此定：

### (a) C2 修订：顺序为不变量，绝对值为可标定量
C2 **保留为硬约束**的是**顺序不变量** `紧急 < 碰撞 < 安全`；其**绝对值为可标定量**，由**地面安全层**（W5）承载并按**实测动力学**标定。此为契约真源变更（先例：[ADR-0017](./0017-air-ground-collaboration-loop.md) 改写 C10、[ADR-0019](./0019-uav-trajectory-and-tracking-parameter-calibration.md) 改写 C1）。旧值 `0.7/0.9/2.4 m` **作废**。

### (b) 标定口径：Nav2 时间窗法
绝对值**不得**沿用或等比放大旧值，而按 Nav2 / 工业安全惯例以**时间窗 × 速度 + 制动距离**推导（Nav2 `collision_monitor` 默认 `time_before_collision: 2.0 s`；[ADR-0019](./0019-uav-trajectory-and-tracking-parameter-calibration.md) 已引 Nav2 `time_before_collision 1.2–2.0 s` 口径）：

- **安全档（slowdown 起始）** = `v_car_max · t_before + 裕度`，`t_before ∈ [1.2, 2.0] s`。
- **碰撞档（stop 起始）** = `v_car_max² / (2a) + 车体半长 + 裕度`，`a` = 车制动加速度。
- **紧急档（即时停）** = 车体半长 + 固定小裕度。
- 计**相对接近速度**（闭合速度上限 ≈ `v_car + v_target`）后取包络。

### (c) 三档绝对值（W2 实测动力学标定）
以 W2 补丁**实测** `v_car_max = 1.5 m/s`、制动加速度 `a ≈ 0.9 m/s²`、`t_before ∈ [1.5, 2.0] s`、车体半长 ≈ `0.5 m`、裕度 `0.3–0.5 m` 按 (b) 口径试算：碰撞档 `v²/(2a)+半长+裕度 ≈ 1.25+0.5+0.3 ≈ 2.0 m`，安全档 `v·t_before+裕度 ≈ 3.0+1.0 ≈ 4.0 m`，紧急档 `半长+小裕度 ≈ 1.0 m`。定为 **`紧急 1.0 m < 碰撞 2.0 m < 安全 4.0 m`**。三档由 [W5 spec](../../.trae/specs/w5-ground-nav-safety/spec.md) 的 `collision_monitor` zone 承载（`StopEmergency@1` / `StopCollision@2` / `SlowSafety@4`）；W5-a 实跑佐证（一条直行逼近：`1.00→0.30→0.00`，车停于立柱前 ~2.1 m 未接触）。**实测值已回填（W5-c，2026-10-13）。**

### (d) 平台换装：保形换装，不改驱动内核
地面平台改为**差速轮式 UGV（Husky 级尺度）**形态，**保留 [ADR-0010](./0010-ground-drive-real-physics.md) 的 `DiffDrive` 真实物理内核**（不改阿克曼、不动执行口语义）。模型取自 **Gazebo Fuel 官方 CC0 模型**（OpenRobotics `Husky A200` 等），**保形换装**：只替换**视觉 mesh + 碰撞体**，并按新尺度**回填质量 / 惯量 / 几何**；**保留**下列不变量：

`car/base_link`、`car/lidar_link` 及话题 `/agc/car_lidar`、`car/capture_net`（2 m 网面）+ `car/jammer`、GZ `DiffDrive`、桥入 `/model/agc_car/cmd_vel` / 桥出 `/agc/sim/ground_odom_raw`、`agc_sim/config/car.yaml` 为唯一参数源。

## Considered Options

- **保留旧值 `0.7/0.9/2.4`（或等比放大）** —— 被否：无新标度推导依据，且过小。
- **C2 保持字面绝对值不变、新值只落 W5 config** —— 被否：会让契约真源与实现长期不一致（真源说谎）。
- **换成阿克曼整车（SUV/Pickup/Prius）** —— 被否：乘用车形态与"地面无人机拦截平台"不符（现实中同类为 UGV 载网捕载荷），且须改 [ADR-0010](./0010-ground-drive-real-physics.md) 并波及执行口语义。
- **整包引入 ROS2 仿真包（Clearpath 等）** —— 被否：外部包进 `src/`，违 [ADR-0001](./0001-greenfield-ros2-rewrite.md) 与包边界。
- **自建 mesh** —— 被否：工作量大、收益低；Fuel CC0 模型已足够真实。

## Consequences

- 契约 [硬约束](../ROS2迁移交接文档.md#hard-constraints) **C2 改写**（顺序不变量 + 可标定绝对值，旧值作废）；契约 [已决事项](../ROS2迁移交接文档.md#decisions) 增列本 ADR。
- 触发 **W2 打补丁**（UGV 保形换装 + 动力学同步 + 实测顶速），与 [ADR-0019](./0019-uav-trajectory-and-tracking-parameter-calibration.md) 触发的 W3 补丁并列。
- [W5 spec](../../.trae/specs/w5-ground-nav-safety/spec.md) 的 C2 承载、`collision_monitor` zone 参数据此；**W5-a 首要验证**：`collision_monitor` 在本机 Jazzy 消费 `/agc/sim/car_points` 且 `car/base_link→car/lidar_link` TF 可解。
- **表述纪律**：文档 / 报告**不得**再引用 `0.7/0.9/2.4 m` 作为现行阈值；C2 数值一律引用本 ADR（暂定值须标注"待实测回填"）。

## 落地记录（W5-a 实测，2026-10-13）

W5-a 落地 `collision_monitor` 时实测到一处**模型级缺陷**并据此定案，补记如下（属本 ADR 的模型/参数决策范畴）：

- **根因**：`agc_car` 的 **2 m 捕获网面**（`car/capture_net`，W2-interception 载荷）竖立在车头正前方，而 `gpu_lidar` **对 visual 求交**——网面把激光前向视野**完全遮挡**（正前方恒返回 0.50 m），致安全层"看不见"任何障碍（车径直撞柱不停）。
- **决策**：将捕获网面**抬高至传感器扫描平面上方**（`car/capture_net` 位姿 → `x 0.55 / z 1.60`，网面下沿 base z 0.60 > 雷达 z 0.48）。既解开遮挡，又使网面更贴合目标机 ~2 m 高度（利于 W6 捕获判据）。
- **安全层感知源**：改用 **`scan`（2D `LaserScan`）** 而非点云——水平扫描在传感器高度一圈、天然不返回地面，避免点云高度带/地面滤除的调参脆弱性。**注**：`source_timeout` 取值**不**沿用 Nav2 官方示例（`5.0` 未经本车标定），改由本车实测动力学标定，见下文「追加记录：`source_timeout` 标定（R-008）」。
- **`stop_pub_timeout` 置极大**：使安全层**持续发布**（含零速）——唯一执行口始终有实时命令；否则 idle 时 `collision_monitor` 停发零速，导致任务门控的 `cmd_vel 回环` 确认**死锁**。
- **实测结论**：车自立柱前方 5 m 直行逼近 → `cmd_vel` `0.80 → 0.24`（SlowSafety@4 m，降速 30%）`→ 0.00`（StopCollision@2 m），车停于立柱前 ~1.6 m 未接触 —— **C2 顺序与三档触发、C6 真实逃逸均成立**；暂定值 `1.0/2.0/4.0 m` 由此**实测佐证**（绝对值的最终标定仍按 (b) 口径随实测动力学回填）。

## 追加记录：`source_timeout` 标定（R-008，2026-10-15）

<a id="source_timeout-calibration"></a>

**决策**：`collision_monitor` 的 `source_timeout` 由 **5.0 s 改为 0.3 s**（`src/agc_ground/config/ground.yaml` 单一注入）。

**问题**：原 `source_timeout = 5.0` 系照抄 Nav2 官方 `collision_monitor` 示例，**未经本车标定**；与本车顶速 `v_max = 1.5 m/s`（本 ADR(c) 实测）不匹配——失源后盲行尺度可达 `1.5 × 5.0 = 7.5 m`（尚未计制动），远超 4 m 安全档，安全层无法覆盖失源期间距离。

**判据（推导）**：失源后**总停距 = 失源盲行 `d_blind` + 制动距离 `s_brake`** 必须 ≤ **碰撞档 2.0 m**（安全层自身包络），且窗口 `T` 必须 **≫ 扫描周期**（激光 `update_rate = 10 Hz`，周期 0.1 s；取 `T ≥ 3 × 0.1 = 0.3 s` 避免单帧抖动误判）：

- `s_brake = v_max² / (2a)`；本车实测 `v_max = 1.5 m/s`、锁轮制动 `a ≈ 0.9 m/s²`（本 ADR(c)，2026-10-13）→ `s_brake = 1.25 m`。
- `d_blind = v_max · T`；取 `T = 0.3 s` → `d_blind = 0.45 m`。
- 总停距 `0.45 + 1.25 = 1.70 m < 2.0 m`（余量 `0.30 m`）→ 满足判据。
- `T = 0.3 s` 亦满足 `T ≥ 3×` 扫描周期，并与 `agc_perception/config/perception.yaml` 的 `source_timeout_s: 0.3` **同口径**。

**语义取证**：本机 Jazzy `nav2_collision_monitor` 的 `Source::getData` 对失效源**返回 false → 封锁机器人**（`/opt/ros/jazzy/include/nav2_collision_monitor/source.hpp`：`@return false if an invalid source should block the robot`）——即源超龄/变换失败时**停车**，**非静默透传**原命令；由 R-008 故障注入实测确认（`experiments/w5-ground-nav-safety/<run_id>/metrics/r008.json`）。

**实测来源**：`s_brake`/`v_max`/`a` 取自本 ADR(c)（W2 补丁实测）；端到端延迟与最小障碍检测距离（激光 `range_min = 0.2 m`）由 R-008 脚本实测记录（`r008.json` 的 `latency` / `scan_meta`）。

**表述纪律**：此后 `source_timeout` 一律引用本追加节；**不得**再以「对齐 Nav2 官方示例 `5.0`」作为取值依据。

## 追加记录：第三方资产许可适用关系（S-12，2026-10-18）

<a id="third-party-license-note"></a>

本文 (d) 与 Considered Options 中「Gazebo Fuel 官方 **CC0** 模型」的许可表述，系当时**未核实**的断言（来源材料**未**随资产保留）。据 [ADR-0068](0068-third-party-asset-provenance-license-and-hash-manifest.md)，上游 Clearpath Husky ROS 资产族（`husky_description` / `husky_gazebo` / `clearpath_gz`）在 ROS Wiki / ROS Index 标 **`BSD`**（要求保留版权 / 许可署名），故第三方 mesh 的许可表述**不再**由本文断言：来源 / 许可 / 署名 / `sha256` 一律引用 [第三方资产与许可](../conventions/第三方资产与许可.md)（许可**分列**，自有包 `Proprietary` **不外推**；未核实前登记为**未决**）。

本文其余决策（**保形换装**口径、C2 标定、`DiffDrive` 内核、插件弃用）**不**受本注影响。
