# UAV 联合安全屏障与显式不可行路径（R-033）

## Status

accepted

## Date

2026-10-17

## 背景

评审风险 **R-033（UAV 自保证明）**（[10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-033)）指出
[`safety_shield`](../../src/agc_air/src/safety_shield.cpp) 的**安全参考 ≠ 安全轨迹**：

- **逐个障碍推出参考点**，**再**按目标间距推出 → 后投影**可重新落入前约束**（投影**不可交换**）；
- **仅检参考端点**，无 **swept path**（当前位置 → 参考的连线是否穿障）/ 实际跟踪误差 / 加速度可达性联合检验；
- 静态列表**不含车 / 动态障碍状态**；
- **无「不可行」显式输出**——无可行参考时仍静默返回可能不可达参考。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C10**（无人机对目标常开跟踪
**且负责自身飞行安全**，即自避障）、**C6**（安全滤波器参数须用**真实逃逸能力**）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一命令者语义不变）、
[#messages](../ROS2迁移交接文档.md#messages)（9 消息 ABI 不得变更）；产物纪律
[ADR-0007](0007-experiment-artifacts-convention.md)。

分类：**设计风险**。本 ADR 决定**空中安全层的联合复检与显式不可行路径**；**不**回退已验收组件，
**不**改冻结 ABI，**不**新增第二 ROS 命令者。评审亦明确：**二维静态屏障可作限定实验模型**，
**不可**默认为三维通用安全。

## 取证（先学后做）

按 [AGENTS.md](../../AGENTS.md) 铁律「先学后做」，**先**对齐同版本 Nav2（`nav2_collision_monitor` /
`nav2_behaviors`）的成功用法：

- **footprint / 速度多边形与**扫掠**几何检查**：Nav2 `nav2_collision_monitor` 的
  `class VelocityPolygon : public Polygon`（[`velocity_polygon.hpp`](/opt/ros/jazzy/include/nav2_collision_monitor/velocity_polygon.hpp)）
  把几何拆为**随速度变化的子多边形**（`SubPolygonParameter` 按 `linear_min/max`、`theta_min/max`、
  方向角分段）——即**判据依赖运动状态而非单点端点**；`polygon.hpp` 注释指明 `APPROACH` 模型下
  多边形**代表机体 footprint**。→ 本批据此把判据从「**参考端点**」升级为「**当前位置→参考的扫掠段 +
  端点**的联合复检」，并把修正边界扩为**真实安全半径 + 跟踪误差 + 反应 + 可达性余量**。
- **显式安全动作（不静默放行）**：`nav2_collision_monitor/types.hpp` 的
  `enum ActionType { DO_NOTHING, STOP, SLOWDOWN, APPROACH, LIMIT }`——冲突时给**显式动作**。
  → 本批据此给**显式「不可行」输出**（`infeasible` + `reason` + `action`），无可行参考时走
  **简单可验证的 hold / 逃逸**（**不**静默返回不可达参考）。
- **恢复（recovery）/ hold 语义**：Nav2 `nav2_behaviors/plugins/wait.cpp`（`Wait` 恢复行为）在
  无有效进展时进入**显式等待 / 保持**动作。→ 本批据此外的不可行路径 = **hold**（保持当前位置安全）
  或 **escape**（自当前位置逃逸到最近可行点）。
- **传感器新鲜度以源时间 + 有效窗口**：`collision_detector_node` 的 `source_timeout` 语义
  （「Maximum time interval in which data is considered valid」，同 [ADR-0047](0047-c10-freeze-downstream-closure-source-time-and-offset-lifetime.md)）。

**本项目现状差距**（读取时工作区）：`safety_shield.cpp` 的障碍循环在约 `:62`、目标后投影在约 `:79`；
两者**顺序单遍**（后投影可落回前约束）；**仅**比较参考端点（`if (d < r_eff)`），无扫掠段检验；
静态表为 `obstacle_*` 平行数组（**无**车 / 动态障碍状态）；`apply()` **无**不可行返回（恒返回
`Reference`）。`trajectory_tracker.cpp` 在约 `:347` 以 `shield_.apply(effective, target_pos_, tgt_fresh)`
调用并据位置 / 速度差算 `shield_active`。

## 决策

### (a) 联合约束复检（**不**让后投影重新落入前约束）

把「静态障碍约束 + 目标间距约束」建为**同一条联合约束集**（XY 圆盘，修正后半径 = 硬半径 + 余量），
`apply()` 分三步：

1. 候选端点 = 期望参考；端点不可行时先做**联合投影**（对所有违约盘反复径向推出，直至**同时**满足
   或耗尽迭代预算，见 `projectJointFeasible`）——**不**做「逐障碍 + 目标」的**单遍**投影即返回
   （单遍后投影可重新落入前约束，R-033 病灶）。
2. **终检**：候选端点**同时**满足全部约束 **且** 当前位置→候选端点 的直线段**不穿约束**
   （`feasibleXY` ∧ `sweptClear`）。
3. 终检通过 → 采用（`reason="adjusted"`）；否则 → 显式不可行（见 (b)）。

### (b) **扫掠复检**（不只检端点）

`sweptClear`：对每个**起点在外**的约束盘，检验 当前位置→参考端点 的**线段最短距离** ≥ 修正后半径
（起点已在盘内的约束**跳过**——那属于「自当前位置逃出」，由不可行路径处理）。参考端点虽安全但**连线
穿障** → 检出（`reason="infeasible_swept"`）。

### (c) **显式不可行路径**（无可行参考 → 简单可验证 hold / 逃逸）

联合复检未通过 → **显式不可行**（`infeasible=true` + `reason` ∈ `{infeasible_joint, infeasible_swept}`
+ `action` ∈ `{hold, escape}`）：

- 当前机体位置**本身安全**（在所有修正后圆盘之外）→ **hold**（参考 = 当前位置，速度 / 加速度前馈清零）；
- 否则 → **escape**（用联合投影把当前位置逃到最近可行点）；
- 连联合投影亦不收敛 → 对**最严重违约**约束做**单约束径向推出**（保证该约束净空）。

**速度 / 加速度前馈清零**——不向 LQR 给**可能不可达 / 会侵入约束**的参考。本批**不**引入
QP / CBF（列为后续更优路线）。

### (d) 跟踪误差 / 反应时间 / 可达性余量（配置项承载）

修正后半径 = 硬半径 **+** `obstacle_margin`（跟踪误差 / LQR 滞后）**+** `reaction_time_s × max_speed_mps`
（反应时间）**+** `reachability_margin`（可达性 / 制动）。三者**分别**由 `air.yaml` 的
`trajectory_tracker.safety` 段承载。绝对值为**可标定量**（对齐 C2「顺序不变量 + 可标定绝对值」精神）。

### (e) **二维静态限定模型**如实标注（动态障碍 = `unsupported`）

本层是**二维静态圆盘屏障**，仅作**限定实验模型**：只处理 XY 静态障碍圆盘 + 目标间距圆盘；
**不**含车 / 动态障碍状态（`/agc/obstacles/source` 动态障碍、他机运动状态 → 记账 `unsupported`）、
**不**建模三维机体外形 / 载荷外伸 / 网几何。ADR 与台架均**显式标注**，**不得**把二维立柱世界结论
外推为三维通用安全（对齐 C10「自避障」的范围限定声明）。

### (f) 配套诊断（**非** ABI / **非**命令者）

扩展已有 `std_msgs/String` 诊断 `/agc/air/tracking_status`（[ADR-0047](0047-c10-freeze-downstream-closure-source-time-and-offset-lifetime.md)），
每条 tick 追加 `shield_infeasible` / `shield_reason` / `shield_action`。**不**改冻结 9 消息 ABI；
**不**新增第二 ROS 命令者；仅供验收 / 观测。

### (g) 验收（实跑逻辑台架 + 自包含核查）

新增 [`run_w6_r033.sh`](../../scripts/archive/run_w6_r033.sh) + [`w6_r033_bench.py`](../../scripts/archive/w6_r033_bench.py)
（Phase A **实跑**真实 `trajectory_tracker`，`safety.enabled=true` + **每场景专用**静态障碍 / 目标几何参数，
合成驱动发布 odom / active_controller / tracking_ref / targets，订阅 `/fmu/in/trajectory_setpoint`
重建**修正后有效参考** + `/agc/air/tracking_status`）+ [`w6_r033_check.py`](../../tests/w6_r033_check.py)
（读台架证据 + 静态核查）。四场景：

- **a 两障碍交叠**（`:62` 后投影落回前约束的复现）：障碍 (0,0)/(10,0)、修正半径 5.2 → 交叠；
  期望参考 (5,0) 深入重叠区 → **联合复检不可行**（`infeasible_joint`）→ hold（保持当前位置）。
- **b 参考安全但连线穿障**：障碍 (0,0)；当前位置 (0,−8) 与参考 (0,8) **端点均安全**，但连线穿障 →
  **扫掠**检出（`infeasible_swept`）→ hold。
- **c 目标快速闭合**：目标 (10.4,0) 的目标间距盘与障碍 (0,0) 盘冲突，期望参考 (5.2,0) 落冲突区 →
  不可行 → hold（**不**给不可达参考）。
- **d 真值最小净空**：汇总 a/b/c 的**真值最小净空**（对**硬半径**计，**不含**余量）≥ 0，且
  **不可行事件计数** ≥ 3。

产物落 `experiments/w6-task-eval/<run_id>/{r033/bench.json,metrics/r033.json,log/}`。
**范围标注**：本台架为**逻辑台架**（真实 tracker + 合成驱动），**非**整链 / 非 GUI / 非 C3。

## 备选

1. **保持「逐障碍 + 目标」顺序单遍投影**：实现最简；但后投影可重新落入前约束（R-033 病灶）——否决 (a)。
2. **仅检参考端点**：免扫掠计算；但「参考安全 ≠ 轨迹安全」（连线穿障）——否决 (b)。
3. **无可行参考仍返回投影结果**：可保持"飞机继续飞"；但可能给 LQR **不可达 / 侵入约束**参考——否决 (c)。
4. **不可行时直接大范围复位到基础八字**：与 R-032 的「获后不回落八字」冲突，且可能远离任务区——不采纳；
   优先**保持当前位置 / 距冲突障碍逃逸**的简单可验证动作。
5. **引入 QP / CBF 联合安全滤波**：更优但**超出本批**（列为**后续**），且非 R-033 病灶——不采纳。
6. **新建动态障碍 / 车状态执行节点**：越界（属 W4/W5），且 R-033 **不**为自保证明强行扩接口——
   如实**记账 `unsupported`**（见 (e)）。

## 后果

- **正面**：参考修正从「端点单遍投影」升级为「**联合复检 + 扫掠复检**」——后投影**不**再落回前约束；
  无可行参考时进入**显式不可行**（`infeasible` + `reason` + `action`）并走**简单可验证 hold / 逃逸**
  （**不**静默返回不可达参考）；余量按**跟踪误差 / 反应时间 / 可达性**分层由配置项承载；
  二维静态限定模型与动态障碍 `unsupported` **如实标注**。
- **代价 / 限制**：新增参数 `reaction_time_s` / `max_speed_mps` / `reachability_margin`（承载者登记于
  `air.yaml`）；`/agc/air/tracking_status` 追加 3 个诊断字段（**未**改冻结 ABI）；修正后半径较原
  「安全半径 + `obstacle_margin`」更保守（增加反应 + 可达性余量），障碍外沿更远。
- **未决 / 后续**：动态障碍 / 车状态、三维机体外形、载荷外伸、网几何 = `unsupported`（本批不建模）；
  QP / CBF 联合安全滤波属**后续**；基准（W8）三维通用安全保真需另立项；真值最小净空由**硬半径**口径
  计（余量不参与真值净空）。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-033)
- 规格：[fix-runner-green-freeze-uav-safety-r31-r33 spec](../../.trae/specs/fix-runner-green-freeze-uav-safety-r31-r33/spec.md)（R-033）
- Nav2 参照：`nav2_collision_monitor` 的 `velocity_polygon.hpp`（速度相关子多边形）/ `types.hpp`
  （`ActionType` 显式安全动作）/ `collision_detector_node.hpp`（`source_timeout`）；
  `nav2_behaviors/plugins/wait.cpp`（recovery / hold）
- 前序：[ADR-0047](0047-c10-freeze-downstream-closure-source-time-and-offset-lifetime.md)（C10 冻结下游封闭）、
  [ADR-0046](0046-runner-three-layer-acceptance-and-integrity-gate.md)（runner 三层验收）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 实现：`safety_shield.{hpp,cpp}` / `trajectory_tracker.cpp` / `air.yaml` /
  `run_w6_r033.sh` / `w6_r033_bench.py` / `w6_r033_check.py`
