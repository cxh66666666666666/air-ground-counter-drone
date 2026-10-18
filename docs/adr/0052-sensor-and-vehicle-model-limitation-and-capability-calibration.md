# 传感器/车体限制声明与能力标定（R-037）

## Status

accepted

## Date

2026-10-17

## 背景

评审风险 **R-037（传感器/车体模型）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-037)）指出，
**「视觉上像真实平台」不等于「相同制动/转向能力」**，且地面安全结论被**未声明的模型限制**限定：

- **安全层用单水平 scan**：[`agc_car.sdf.xacro`](../../src/agc_sim/models/agc_car/agc_car.sdf.xacro) 的
  `car/lidar_link`（`z=0.48 m`、车顶水平面）经 `scan_self_filter` 产出
  `/agc/ground/scan_filtered`，供 **costmap / `collision_monitor`** 消费——**等价于 z≈0.48 m 的 2D 水平面扫描**。
  → **低障碍**（顶面低于扫描面）、**悬空结构**（在 z=0.48 之上非扫掠高度）、**薄障碍**（受束间距限制）**未必可见**；
  **立柱 world 的成功不能外推任意障碍**。
- **载荷轻量、无碰撞**：`car/capture_net`（@ `(0.55,0,1.60)` 半径 1.0）与 `car/jammer`（@ `(-0.20,0,0.30)`）
  均**无 `<collision>`**、`mass 0.05` → **不能用于真实网接触验证**。
- **四轮外观、后两轮驱动 + 前自由从动 ≠ 原 Husky 滑移动力学**：仍需从**换装后模型**单独标定制动 / 转向能力。
- **口径不一致病灶**：[ADR-0023](0023-spawn-clearance-recalibration.md) 出生净空**计入载荷（网面/发射器）外伸 `1.0 m`**，
  但载荷**无 collision** 且位于 `z=1.60`（**高于** 0.48 扫描面）→ **一侧（净空）计入、另一侧（碰撞/安全层）不计**。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C2**（阈值顺序 `紧急 < 碰撞 < 安全` 为
不变量、绝对值为可标定量，见 [ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md)）、
**C6**（真实逃逸能力）、**C7**（出生净空为可标定量，见 [ADR-0023](0023-spawn-clearance-recalibration.md)）；
[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一执行口语义**不变**）；产物纪律
[ADR-0007](0007-experiment-artifacts-convention.md)。

分类：**设计风险（模型能力限制 / 标定）**。本 ADR 决定**传感器高度与障碍族限制的如实登记、安全声明收窄、
载荷外伸/碰撞口径一致化、换装后车体的制动/转向能力标定**；**不**改冻结 9 消息 ABI，**不**变唯一执行口语义，
**不**新增第二 ROS 命令者，**保留** [ADR-0010](0010-ground-drive-real-physics.md) 的 **DiffDrive 真实物理路线**
（**不**回退 `kinematic_sim`），**不**改车体物理 / 惯性 / 驱动插件。

## 取证（先学后做）

按 [AGENTS.md](../../AGENTS.md) 铁律「先学后做」，**先**对齐**同版本** Nav2（`nav2_collision_monitor`）的
**footprint 与传感器覆盖声明**惯用法，再改自身实现。

**Nav2 `nav2_collision_monitor`（同版本 `params/collision_monitor_params.yaml` + 头文件
`source.hpp` / `polygon.hpp` / `velocity_polygon.hpp`）用法结论：**

- **polygon 语义分两类**（`polygon.hpp` doc）：`STOP/SLOWDOWN/LIMIT` 的 polygon 表示**机器人周围区域（zone）**；
  `APPROACH` 的 polygon 表示**机器人 footprint**（由 `footprint_topic`（如 `/local_costmap/published_footprint`）动态设定，
  `time_before_collision` 体现「按速度留时间」）。
- **传感器覆盖以 source 声明**：`observation_sources` 逐源声明类型与覆盖域——`pointcloud` 源**显式声明
  `min_height` / `max_height` / `use_global_height`**（在高度带内求交）；而 `scan`（2D `LaserScan`）源**没有**高度带参数，
  即**隐含只覆盖扫描平面那一层**。→ 本项目用 `scan` 源，则**覆盖域 = 单一水平面**，**低/悬空障碍天然不在覆盖域**；
  这正是本项须**如实登记并收窄声明**之处。
- **`min_points`**（`polygon.hpp`）：进入 zone 触发动作所需的**最少数据点数**——细/薄障碍在给定距离的命中束数受限。
- **失效源语义**：`Source::getData` 对失效源 `return false`（**封锁机器人**，非静默透传）——R-008 已据此标定 `source_timeout`。
- **footprint 与 zone 分离**（velocity_polygon 的 `holonomic=false`、`rotation/translation_forward/backward/stopped`
  子区，`stopped` 兜底覆盖全速度域）：本项目 W5 已按 [ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md)
  承载三档 `velocity_polygon`（R-007）。

**本项目现状（读取时工作区）：**

- `agc_car.sdf.xacro`：`car/lidar_link` `z=0.48`；`gpu_lidar` 3D（H 360 束/360°、V 16 束 `±0.261799 rad ≈ ±15°`、
  `range [0.2,20]`、`update_rate 10 Hz`）；`car/capture_net` @ `(0.55,0,1.60)` 半径 1.0 **无 collision**；
  `car/jammer` @ `(-0.20,0,0.30)` **无 collision**；`payload_mass 0.05`；后两轮 DiffDrive 驱动、前两轮自由从动。
- `ground.yaml`：`collision_monitor` 三档 `velocity_polygon` + `scan_self_filter`（`/agc/ground/scan_filtered`）
  单水平面口径；**注释已含** z=0.48、束间距 `≈0.01745·D`、`min_points=2`——但**未**显式声明**低/悬空/薄障碍可见性边界**，
  亦**未**显式**收窄**「立柱 world 达标 ≠ 任意障碍通用」。
- `car.yaml`：`max_linear_velocity 1.5` / `max_angular_velocity 2.0` / `max_wheel_acceleration 5.0`；
  载荷注释「无碰撞、不改动 DiffDrive 标定」——但**未**与净空外伸口径交叉声明。

**Nav2 参照「不可得则如实标注」**：本项目**未**在 Nav2 中找到「能力标定（制动距离/转弯扫掠实测）」的直接组件——
Nav2 侧为**参数校验 + footprint/覆盖声明**，制动能力标定属本系统责任（同 [ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md)
的「绝对值由实测动力学标定」）。故本项**先取 Nav2 的「覆盖声明 / zone-footprint 分离」惯用法**，
**能力标定以本系统真跑 gz 取真值/接触证据**（见决策 (c)）。

## 决策

### (a) 如实登记传感器高度 + 障碍族限制，并收窄安全声明

- **传感器高度**：`car/lidar_link` **`z=0.48 m`**（车顶水平面）。模型为 `gpu_lidar` 3D，但**安全层 / costmap 消费的
  `/agc/ground/scan_filtered` 等价于 z≈0.48 的 2D 水平面**扫描 → **安全层可见性 = 单一 z=0.48 平面**。
- **障碍族可见性边界（如实）**：① **低障碍**（顶面低于扫描面 / 不入水平光束）→ **不可见**（2D 安全层口径）；
  ② **悬空结构**（在 z=0.48 之上、非扫掠高度）→ **不可见**；③ **薄障碍**（细柱/薄板）→ 受**束间距**限制
  （距离 `D` 处相邻束间距 `≈ 0.01745·D m`；三档 `min_points=2`）；④ **自体遮挡 / 自滤**后仅未被车体遮挡方向可见。
- **收窄声明**：把「立柱 world 达标」**限定**为「在 `z≈0.48 m` 处有**足够水平截面**且**进入光束**」的障碍族；
  **明确不得外推**为任意障碍（含低 / 悬空 / 薄障碍）的**通用**保证。
- **落点**：本 ADR（决策 + 边界）+ `agc_car.sdf.xacro` / `ground.yaml` 的**注释登记**（**不改物理**）。

### (b) 载荷外伸与碰撞口径一致化（显式声明各自适用域）

现状：净空（[ADR-0023](0023-spawn-clearance-recalibration.md)）**计入**载荷外伸 `1.0 m`；载荷**无 collision**、
位于 `z=1.60`（高于扫描面）→ 碰撞 / 安全层**不计**。**裁定（采纳，最小改动）**：**显式声明各自适用域**，
消除「一侧计入另侧不计」的**隐含**不一致：

- **净空（布局约束）**：载荷外伸 `1.0 m` **仅**作为**出生净空推导**的外伸登记（[ADR-0023](0023-spawn-clearance-recalibration.md) 现行，
  保持），约束场景随机化不得使其侵入他实体初始包络；
- **安全层（2D 视野）**：载荷位于 `z=1.60`、**不**在 z=0.48 扫描面 → **不**进安全层视野（**显式声明**）；
- **物理碰撞**：载荷**无 collision**（当前**不**参与真实接触）→ 捕获 / 干扰**行为级**以外的**物理接触验证不适用**
  （`unsupported` 登记）；
- **重标定义务**：载荷**物理启用（加 collision）后**，C2 / C6 与本外伸 / 碰撞口径**必须重新标定**，**不**借旧标定维持。

**可选路线（本批不改，仅记录）**：若把 `capture_net` 降至扫描面或加 collision，可令「净空外伸」与「碰撞/安全层」**物理一致**；
但会使网面重新遮挡激光前向视野（[ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md) W5-a 已实测：
网面在正前方恒返回 `0.50 m`）并改变捕获几何——**本批不采用**，留待载荷物理启用时统一重标。

### (c) 能力标定（优先真跑 gz 取真值 / 接触证据）

从**换装后模型**测**满速制动距离**与**转弯扫掠 / 摩擦敏感性**：

- **真值证据**：gz 真值 `/agc/gt/agc_car`（由 `gt_bypass` 直读 gz `/world/agc_world/dynamic_pose/info`）；
  **接触证据**：`/agc/ground/contact`（`gz.msgs.Contacts` → `ros_gz_interfaces/Contacts`）。
- **满速制动**：经**唯一执行口** `/agc/ground/cmd_vel` 命令至 `v_max=1.5` **稳态**（真值速度判稳态），再命令零，
  测 gz 真值**制动距离**与**制动耗时**（**不**仅看「发布零命令」）；与 [ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md) 的
  `a≈0.9 m/s²` 预测 `s=v²/(2a)=1.25 m` 对拍。
- **转弯扫掠 / 摩擦敏感性**：命令 `ω_max=2.0`（原地转向）测实际**偏航率**；命令 `(v,ω)` 组合测实际**转向半径 / 扫掠净空**；
  以真值速度 vs 命令速度差记**滑移（摩擦）敏感性**。
- **若本环境无法稳定实跑 gz**（GUI 库缺失 / DDS 发现 / lifecycle 卡死）→ **如实标 `unsupported`** + 失败证据（日志），
  **不**伪造、**不**借 [ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md) 旧标定维持 C2 / C6。

### (d) 验收（实跑台架 + 自包含核查）

新增 [`run_w6_r037.sh`](../../scripts/archive/run_w6_r037.sh) + [`w6_r037_bench.py`](../../scripts/archive/w6_r037_bench.py)
（**Phase A 实跑**：起最小 gz 场景 = `world_bringup`（世界 + 桥 + spawn 换装后 `agc_car`）+ `ground_drive_relay`（唯一执行口）
+ `gt_bypass`（真值），经唯一执行口驱动并采真值 / 接触）+ [`w6_r037_check.py`](../../tests/w6_r037_check.py)
（**Phase B 自包含核查** (a)…(j)）。覆盖：a 传感器高度如实登记；b 安全声明已收窄（含「不外推任意障碍」）；
c 载荷外伸/碰撞口径一致；d 满速制动真值/接触证据 或 如实 `unsupported`；e 转弯扫掠/摩擦敏感性同上；
f 未改物理/惯性/驱动插件（**对 R-037 之前的模型基线**核验）；g 保留 DiffDrive、未回退 `kinematic_sim`；
h ABI / 单命令者 / 跨文档章节号引用为空；i ADR-0052 存在 + `accepted` + 取证节 + 含 R-037 + 无章节号引用；
j 脚本卫生。证据落
`experiments/w6-task-eval/<run_id>/r037/{bench.json,check.json}` + `metrics/r037.json`。

### (e) 边界（不改契约）

**不**改冻结 9 消息 / 服务 ABI，**不**变唯一执行口语义（仍为 `/agc/ground/cmd_vel` 唯一命令车体通道），
**不**新增第二 ROS 命令者；**保留** [ADR-0010](0010-ground-drive-real-physics.md) 的 **DiffDrive 真实物理路线**，
**不**回退 `kinematic_sim`；**不**改车体的**物理 / 惯性 / 驱动插件**（本项仅注释 / 参数登记 + 测量）。

## 落地记录（R-037 实跑回填，2026-10-17）

最小 gz 场景（世界 + 唯一执行口中继 + 真值旁路）**实跑**换装后 `agc_car`（`experiments/w6-task-eval/20261017-w6-r037-0/r037/bench.json`）：

- **满速制动**（经唯一执行口 `/agc/ground/cmd_vel`）：稳态 `v=1.5 m/s` → 命令零 → gz 真值**制动距离 `0.281 m`**、
  **制动耗时 `0.384 s`** → 等效减速度 `a = v²/(2s) ≈ 4.0 m/s²`，**显著大于** [ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md)
  的假设 `a ≈ 0.9 m/s²`（故 C2 碰撞档 `2.0 m` 对本模型**偏保守**；C2 绝对值不变，另行重标须批准）。
- **转弯扫掠 / 摩擦敏感性**：原地 `ω=2.0 rad/s` → 实测偏航率 **`1.734 rad/s`**（比值 0.867）；
  弧线 `(v=1.0, ω=1.0)` → 实测**转向半径 `1.126 m`**（理想 `1.0 m`）、换算**外扫掠半径 ≈ `1.705 m`**、
  实测前向速度 `1.0 m/s` → **滑移比 ≈ 0**（此速度域无明显打滑）。
- **接触真值**：满速逼近 `pole_1`（world `(0,-6)`）→ **1 次接触事件**（`t≈2.98 s`，`/agc/ground/contact`），
  接触位置 gt ≈ `(−0.03, −5.31)`。
- **载荷物理接触验证**：**`unsupported`**（`capture_net`/`jammer` 无 collision、不在 z=0.48 扫描面）——
  本批不启用载荷物理，其结果**不**进本标定。

## 备选

1. **不加声明、仅在报告文字里提限制**：则限制不落真源、下游仍可能外推——否决 (a)。
2. **把 `capture_net` 降至扫描面 / 加 collision 以令口径物理一致**：会重新遮挡激光前向视野、改变捕获几何且需重标 C2/C6
   ——本批**不取**（记为可选路线，见 (b)）；改**显式声明各自适用域**。
3. **仅凭 [ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md) 旧标定（`a≈0.9`）断言制动能力**：
   换装后模型未单独验证 → 违 R-037「从换装后模型测」——否决；改**真跑 gz 取真值/接触证据**（(c)）。
4. **实跑困难时伪造数值**：违「不伪造」红线——否决；改如实 `unsupported` + 失败证据。
5. **回退 `kinematic_sim` 以简化标定**：违 [ADR-0010](0010-ground-drive-real-physics.md)（C6 真实逃逸口径）——否决。

## 后果

- **正面**：传感器安装高度（z=0.48）与**障碍族可见性边界**（低 / 悬空 / 薄）**落真源**；安全声明**收窄**为
  「z≈0.48 平面有足够水平截面且进入光束」的障碍族，**禁止外推**任意障碍；载荷**外伸（净空）**与
  **碰撞（物理/安全层）**口径**显式分层**，消除隐含不一致，并登记**载荷物理启用后重标定义务**；
  换装后车体的**满速制动 / 转弯扫掠 / 摩擦敏感性**以**真值 / 接触证据**标定（或如实 `unsupported`），
  支撑 C2 / C6 的可复核性。
- **代价 / 限制**：载荷**物理接触验证**明确为 `unsupported`（无 collision）——本批**不**启用载荷物理；
  「立柱 world 达标」**仅**对限定障碍族成立，安全结论的适用范围**变窄**（如实）。
- **未决 / 后续**：载荷物理启用后**必须重标** C2 / C6 与外伸 / 碰撞口径（(b) 重标定义务）；低障碍 / 悬空的
  **实测可见性**（ray / scan 记录）为后续可加的证据；完整车体重建 / 悬架 / 滑移动力学差异**不在本批**。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-037)
- 规格：[fix-lifecycle-control-height-numeric-r34-r38 spec](../../.trae/specs/fix-lifecycle-control-height-numeric-r34-r38/spec.md)（R-037）
- Nav2 参照：`nav2_collision_monitor` 的 **zone vs footprint**（`polygon.hpp`）、**`velocity_polygon` 子区**、
  **`source` 覆盖声明**（`scan` 无高度带 vs `pointcloud` 的 `min_height`/`max_height`）、**`min_points`**、
  **失效源封锁**（`source.hpp` 的 `getData`）
- 前序：[ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md)（C2 重标定 + 平台换装）、
  [ADR-0023](0023-spawn-clearance-recalibration.md)（出生净空计入载荷外伸）、
  [ADR-0010](0010-ground-drive-real-physics.md)（DiffDrive 真实物理）、
  [ADR-0042](0042-safety-infeasibility-honest-metric-and-contact-evidence.md)（接触真值）、
  [ADR-0008](0008-nav2-paradigm-absorption-as-code-principle.md)（Nav2 吸收）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 实现：`src/agc_sim/models/agc_car/agc_car.sdf.xacro` / `src/agc_sim/config/car.yaml` /
  `src/agc_ground/config/ground.yaml` / `scripts/run_w6_r037.sh` / `scripts/w6_r037_bench.py` /
  `scripts/w6_r037_check.py`
