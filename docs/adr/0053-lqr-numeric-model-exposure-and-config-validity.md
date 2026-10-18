# LQR 数值模型暴露与配置合法性启动校验（R-038）

## Status

accepted

## Date

2026-10-17

## 背景

评审风险 **R-038（数值模型与配置边界）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-038)）指出，
共享 LQR 的**模型 / 收敛假设未暴露**，配置错误可**静默变成控制错误**：

- [`lqr_controller.cpp`](../../src/agc_control/src/lqr_controller.cpp) 连续双积分 `A=[[0,I],[0,0]]`（**幂零**）、`B=[[0],[I]]`；
  `Ad=I+A·dt`（此 A 幂零 → Ad **恰为精确**），但 `Bd=B·dt` **漏掉位置的 `0.5·dt²` 项**（精确应为
  `Bd=[[0.5·dt²·I],[dt·I]]`）——注释却称「精确离散化」（**不实**）；DARE 值迭代到 `max_iter_`
  **未报告未收敛**、**未检查增益有限性**；`dt_/r_` 仅**粗略静默兜底**（`dt<=0→0.02`、`r<=0→1.0`），
  `q_pos_/q_vel_` **无校验**；
- [`ground_baseline.cpp`](../../src/agc_ground/src/ground_baseline.cpp) 把**世界系加速度**积分成 `vx_des/vy_des`，
  再转车体系得 `lin=clamp(bx,0,v_max)` / `ang=k_yaw·atan2(by,max(bx,0.1))` —— **非**对应差速动力学
  （丢弃 vy 的横向自由度），**不代表 air 与 ground 同等闭环**；
- [`safety_shield.cpp`](../../src/agc_air/src/safety_shield.cpp) 障碍参数**平行数组**
  `obstacle_names/obstacle_x/obstacle_y/obstacle_safety_radius` 用 `n=min({...})` **静默按最短长度截断**
  → 配置漏一个数组即**静默漏障碍**；半径 / 坐标**无正数 / 有限性校验**；
- [`map_adapter.cpp`](../../src/agc_perception/src/map_adapter.cpp) 已有部分几何守卫（`res_<=0||width<=0||height<=0`），
  但**角度 / 有限性**未在**启动期明确拒绝**；[`air_perception.cpp`](../../src/agc_perception/src/air_perception.cpp)
  的 `seed` 经 `strtol` **静默退回**默认基种子（非数值输入被静默接受）。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C1 恒速 / C2 阈值顺序 / C6 真实逃逸能力 /
C10 常开跟踪与自保）、[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一执行口语义**不变**）、
[#messages](../ROS2迁移交接文档.md#messages)（冻结 9 消息 ABI **不得变更**）；产物纪律
[ADR-0007](0007-experiment-artifacts-convention.md)。

分类：**设计风险（数值健全 / 配置合法性）**。本 ADR 决定 **LQR 精确离散化 + 数值健全失败路径、
ground 映射适用性标注 / 可达性对拍、安全层平行数组长度一致校验、地图 / seed 启动期校验**；
**不**改冻结 9 消息 / 服务 ABI，**不**变唯一执行口语义，**不**新增第二 ROS 命令者。

## 取证（先学后做）

按 [AGENTS.md](../../AGENTS.md) 铁律「先学后做」，**先**对齐**同版本** Nav2（Jazzy）的**启动期参数 / 数值校验与
合法性拒绝**惯用法，再改自身实现。

**Nav2 同版本启动期校验惯例（读 `/opt/ros/jazzy/include/...` 可得者）：**

- `nav2_util::validateMsg`（`nav2_util/validate_messages.hpp`）：显式 **值检查**（`isinf` / `isnan` → 判无效）+
  **逻辑检查**（如 `map` 尺寸须等于 `height*width`）；注释写明「otherwise, we regard it as an invalid message」
  ——**非法即拒**，不静默接受。
- `nav2_util::node_utils`（`node_utils.hpp`）：`get_plugin_type_param` 在缺参 / 未初始化时
  **`RCLCPP_FATAL` + `throw std::runtime_error`**（"No 'plugin' param for param ns!"）；参数声明用
  `declare_parameter_if_not_declared`（**声明只一次**）。
- `nav2_util::twist_subscriber`（`twist_subscriber.hpp`）：无效 ROS 参数 → **`throw std::invalid_argument`**。
- `nav2_costmap_2d` / `nav2_navfn_planner`：在 **构造 / configure 期**声明参数并校验，非法 → 抛错 / FATAL
  （`@throw std::logic_error|runtime_error` 见各头文件）。
- **惯例结论**：Nav2 的守法是「**启动期声明 → 校验 → 非法即 FATAL / 抛错（节点启动失败）**」，而非运行期
  静默纠正或按最短长度截断。

**本项目现状差距（读取时工作区）：** 见「背景」；关键在 `LqrController` 的 `Bd` 缺项 + DARE 不报告收敛 +
静默兜底，`safety_shield` 的 `min(...)` 截断，`map_adapter` 缺角度 / 启动期拒绝，`air_perception` 的 `seed`
静默 `strtol` 兜底。

**本批取证实测（本机真实二进制 / 实跑）：**

- **DARE 收敛预算实测**（解析复算新旧两模型，Q=R=diag(4I,4I)/I、dt=0.02）：达 `tol=1e-10` 需**约 559 次**
  迭代（旧 `Bd=B·dt` 与精确 `Bd` 所需迭代数**相近**，均 ~560）。**故旧默认 `max_iter=200` 实为「未收敛」**
  （第 200 次残差 ≈ **0.0088**）——旧实现**静默使用未收敛增益**（评审病灶被实测证实）。本批将默认预算提为
  **2000**（留裕度），并对**不足预算**明确失败。
- **精确离散化前后差异**：`Bd=B·dt` → 位置项为 0；精确 `Bd` 位置项为 `0.5·dt²`（dt=0.02 → **2×10⁻⁴**），
  `v_{k+1}` 行不变（`dt`）。两组增益数值近似（差异 ~1e-3 量级），但**模型语义**由「近似」变为「精确」；
  适用频率 = `1/dt`（air `dt=0.02`→**50 Hz**；ground `dt=0.05`→**20 Hz**）。
- **节点启动期拒绝实测**（`experiments/w6-task-eval/<run_id>/r038/bench.json`）：非法地图几何（`size_x=0`）、
  非法障碍几何（`size` 含 0）、非法 `seed`（`"abc"`）→ 真实 `map_adapter` / `air_perception` **非零退出 +
  「拒绝启动」日志**；合法配置**存活不被误拒**。

## 决策

### (a) LQR：精确离散化 + DARE 收敛 / 有限性失败路径 + 集中合法性校验

- **精确离散化**：`Ad = I + A·dt + 0.5·A²·dt²`（本 A 幂零 → `A²=0` → 高阶项为零 → **恰为精确**，如实注释）；
  `Bd = B·dt + 0.5·A·B·dt²`（`A·B=[[I],[0]]`）→ **`Bd=[[0.5·dt²·I],[dt·I]]`**（**补上位置 `0.5·dt²` 项**；
  原文「精确离散化」表述**改正**：Ad 精确、Bd 现也已精确）。
- **DARE 收敛 / 残差 / 有限性**：记录迭代次数与**残差** `max|P_next−P|`；迭代结束若**残差 ≥ `tol_`（未收敛）**
  或 `P` / `S` / `K_` 含**非有限**（NaN/Inf）→ **抛 `std::runtime_error`**（使 `configure` 失败 → 由 R-034 的
  error 路径接管；**不静默使用**）。默认 `max_iter` 由 **200 提为 2000**（实测 `tol=1e-10` 需 ~559 次）。
  暴露**只读诊断** `converged()` / `residual()` / `iterations()` / `dt()` / `appliedFrequencyHz()`（**非 ABI**）。
- **集中合法性校验**（`validateParameters()`）：`q_pos_>0`、`q_vel_>0`、`r_>0`、`dt_>0`、`tol_>0` 且全为**有限**，
  `max_iter_>0`，`acc_max_` 有限；可行域说明（`Q=diag(q_pos·I₃,q_vel·I₃)` 半正定、`R=r·I₃` 正定 → 由标量正性承载）。
  **删除**旧静默兜底（`if (dt_<=0) dt_=0.02;` / `if (r_<=0) r_=1.0;`）→ 改为**拒绝**并报出字段与值。
- 新增诊断日志：Ad/Bd 是否精确式、`dt`、**适用频率 `1/dt`**、迭代次数、残差、是否收敛。

### (b) ground_baseline：世界加速度→diff-drive 的适用性标注 + 单独对拍

- **不改**控制律为完整差速动力学（本批不做）；在 [`ground_baseline.cpp`](../../src/agc_ground/src/ground_baseline.cpp)
  增加**适用性注释标注**（锚本 ADR）：共享 LQR 输出为 3 自由度 **holonomic** 世界加速度；映射仅取前进分量
  `lin=clamp(bx,0,v_max)`（**丢弃倒车**）+ `ang=k_yaw·atan2(by,max(bx,0.1))`（**丢弃纯横向自由度**），
  无轮加速度动力学约束 → 可达集 ⊊ 世界系 holonomic 参考集。
- **单独对拍**：验收脚本内做**独立解析可达性 / 跟踪比较**（命令映射可达性；**非整链仿真、非复制 DARE 逻辑**），
  如实报告 ground 与 air（holonomic、无障碍）差异（见下「后果」数值），**不**假定 air 与 ground 同等闭环。

### (c) safety_shield：平行数组长度一致 + 几何合法性（启动期）

- **删除** `n=min({...})` 静默截断 → **长度一致校验**：`obstacle_names/x/y/safety_radius` 四数组长度**必须相等**，
  否则 **抛错**（报出各数组长度）→ `configure` 失败 → 生命周期 error 路径接管。
- **几何合法性**：`obstacle_safety_radius > 0` 且有限、`obstacle_x/y` 有限；非法 → 拒绝。
- 保持既有「磁盘联合约束」语义**不变**（仅加校验，不改算法）。

### (d) 地图几何 / 角度 / `seed` 启动期校验

- [`map_adapter.cpp`](../../src/agc_perception/src/map_adapter.cpp)：新增 `validateStartupOrThrow()`（构造末调用），
  校验 `resolution/size_x/size_y > 0` 且有限、`origin_x/y` 有限、障碍 `x/y` 有限、障碍 `size`（`sx/sy>0`）有限、
  `yaw` 有限；非法 → **抛错**（**拒绝启动**）。既有 `rebuild()` 运行期守卫保留。
- `seed` 消费方：`air_perception.seed` **补启动期校验**（显式 `"unseeded"` 保留默认基种子，其余必须为**整数字面量**，
  否则拒绝；**删除** `strtol` 静默兜底）；`scenario_reset.spawn_seed` / `run_logger.seed` 为**强类型 `int` 参数**
  （由 launch `int()` 生成），非法值由 ROS 参数类型系统在**启动期拒绝** → **如实登记为「已覆盖（类型安全）」**。

### (e) 验收（失败型 gtest + 真实节点台架 + 自包含核查）

- **只增失败型验证，不写镜像实现测试**：`agc_control` gtest [`test_lqr_validation.cpp`](../../src/agc_control/test/test_lqr_validation.cpp)
  （非法 `Q/R/dt` 拒绝、DARE 未收敛报告、只读诊断）；`agc_air` gtest
  [`test_safety_shield_validation.cpp`](../../src/agc_air/test/test_safety_shield_validation.cpp)（平行数组不一致 /
  非法几何拒绝 + 合法几何通过）。
- [`run_w6_r038.sh`](../../scripts/run_w6_r038.sh) + [`w6_r038_bench.py`](../../scripts/w6_r038_bench.py)（**实跑**真实节点
  启动期拒绝）+ [`w6_r038_check.py`](../../scripts/w6_r038_check.py)（自包含核查，判 (a)…(k)）。

### (f) 边界（不改契约）

**不**改冻结 9 消息 / 服务 ABI，**不**变唯一执行口语义，**不**新增第二 ROS 命令者；新增均为诊断 / 日志 / run config
（`LqrController` 只读 getter、ground 对拍报告均为**非 ABI**）。

## 备选

1. **保持 `Bd=B·dt` 近似 + 不报告未收敛**：模型语义不实、数值错误难从 topic 活性发现——否决。
2. **未收敛时仅记 WARN 并沿用末次 P**：仍静默使用不可靠增益——否决；改**明确失败**。
3. **`max_iter` 不足时仅告警不失败**：与「不静默使用」冲突——否决；改失败（默认预算提为 2000 使合法配置可达）。
4. **ground 本批重写为完整差速动力学**：超出本批范围（spec 非目标）——否决；仅**标注 + 对拍**。
5. **地图 / seed 非法仅记 ERROR 继续运行**：配置漏项可静默漏障碍 / 静默退回种子——否决；改**启动期拒绝**。

## 后果

- **正面**：`Bd` 补上位置 `0.5·dt²` 项（模型**精确**、适用频率可见）；DARE **未收敛 / 非有限增益明确失败**
  （旧默认预算实为未收敛的病灶被暴露并修正）；非法 `Q/R/dt` **拒绝**（非静默兜底）；`safety_shield` 漏配一数组
  **即报错**（不再静默漏障碍）；地图几何 / 角度非法、`seed` 非法**启动期拒绝**；ground 映射适用性**如实标注**
  并**单独对拍**，**不**外推 air 结论。
- **ground vs air 对拍（解析，独立分析）**：`ground_frac_desired_tracked_within_10pct ≈ 0.042`（72 个 360° 期望
  方向中仅前向窄扇区 3 个可跟踪；其余因**横向被丢弃**（70/72）与**倒车被截断**（36/72）不可跟踪），
  `air_frac_desired_tracked_within_10pct = 1.0`（holonomic、无障碍可跟踪任意期望速度）；
  ground 可达集 ⊊ air 参考集。
- **代价 / 限制**：默认 `max_iter` 由 200 提为 2000（每次求解 ~560 次 6×6 迭代，**微秒级**，可忽略）；
  旧配置若显式设 `max_iter` 过小将**报错**（行为变更，属**预期**的显式失败）；ground 控制律**未**换为完整差速
  动力学（本批非目标）；对拍为**解析分析**（非整链仿真）——已如实登记。
- **未决 / 后续**：ground 完整差速动力学 / 可达集实测（真值对拍）后续批次；DARE 更稳健求解（如 Schur / 迭代
  加速）非本批；实机独立急停不在 R-038 范围。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-038)
- 规格：[fix-lifecycle-control-height-numeric-r34-r38 spec](../../.trae/specs/fix-lifecycle-control-height-numeric-r34-r38/spec.md)（R-038）
- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) / [#execution-port](../ROS2迁移交接文档.md#execution-port) /
  [#messages](../ROS2迁移交接文档.md#messages)
- Nav2 参照：`nav2_util::validateMsg`（值 + 逻辑检查）/ `nav2_util::node_utils`（`get_plugin_type_param` 抛 `std::runtime_error`）/
  `nav2_util::twist_subscriber`（`std::invalid_argument`）/ `nav2_costmap_2d`（构造期校验）
- 前序：[ADR-0049](0049-lifecycle-repeatable-resource-state-machine-and-liveness-detection.md)（R-034 error 路径）/
  [ADR-0018](0018-shared-control-layer-and-lqr-plugin.md)（共享控制接口）/ [ADR-0008](0008-nav2-paradigm-absorption-as-code-principle.md)（Nav2 吸收）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 实现：`lqr_controller.{hpp,cpp}` / `ground_baseline.cpp` / `safety_shield.cpp` / `map_adapter.cpp` /
  `air_perception.cpp` / `test_lqr_validation.cpp` / `test_safety_shield_validation.cpp` /
  `run_w6_r038.sh` / `w6_r038_bench.py` / `w6_r038_check.py`
