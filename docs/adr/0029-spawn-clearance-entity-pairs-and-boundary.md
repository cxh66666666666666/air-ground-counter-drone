# 出生净空统一候选校验：实体对 + 边界（不再用单一标量代理）

## Status
accepted

> 注（S-05 / [ADR-0061](./0061-w6-runtime-topology-backend-identity-and-single-parameterized-entry.md)）：W6 顶层入口已统一为 [`w6.launch.py`](../../src/agc_task/launch/w6.launch.py)（`backend` 参数 ∈ lightweight | full），原 `w6_task_eval.launch.py` / `w6_full.launch.py` 退役；本 ADR 历史结论正文不改，下文的旧入口名按现入口读作同一拓扑。

## 背景

评审 R-014 指出：`scenario_reset` 的出生净空校验**退化**为单一标量代理——只检查
`hypot(car, target) ≥ spawn_clearance_min_m`（6.0 m）。其后果是：己方 UAV、静态立柱
（`pole_1`/`pole_2`）、作业区边界、以及各实体的**几何外伸**均不参与判断，随机搜索
（`spawn_randomize`）也不复核上述约束。于是出现"目标远但车压立柱 / 车与己方机近邻 / 车出界"
仍可通过的漏洞。

[ADR-0023](./0023-spawn-clearance-recalibration.md) 已把 C7 出生净空拆为**可标定下限**，但其
承载实现只落到"车↔目标"一对。R-014 将其扩展为**多实体对 + 边界**的统一候选校验。

## 决策

### (a) 统一候选校验函数
`scenario_reset` 新增统一校验 `validateCandidate(cx, cy, tx, ty, reason)`，对**每个候选**
（含随机搜索的每一轮）按序检查，任一不满足即拒并给出 `reason`：

1. **边界**：车、目标各自中心须落在"作业区边界内缩自身几何外伸"区间内
   （`x_min + r ≤ x ≤ x_max - r`）→ 否则 `boundary_out_car` / `boundary_out_target`。
2. **车↔目标**：`≥ spawn_clearance_min_m`（沿用 C7 标定值 6.0）→ 否则 `car_target_clearance`。
3. **己方 UAV 对**（仅当 `own_uav_present`）：车↔己方、目标↔己方 → `car_own_uav_clearance` /
   `target_own_uav_clearance`。
4. **静态障碍**（逐障碍）：车↔障碍、目标↔障碍 → `car_obstacle_clearance:<name>` /
   `target_obstacle_clearance:<name>`。

全部候选非法 → 返回 false（保留最后一次 reason）。

### (b) 几何外伸 vs 安全档的适用关系
出生净空下限按**实体对类型**取不同口径：

- **静态障碍对（车/目标↔立柱）**：用**几何不重叠**口径——下限 = 两实体**几何外伸之和**
  （车半长 `0.5037` + 立柱半宽 `0.15`；UAV 半宽 `0.25` + 立柱半宽 `0.15`）。立柱是刚性几何体，
  净空要求是"不嵌入"，故取几何外伸而非安全档。
- **车↔目标**：**沿用 C7 标定值 6.0 m**（[ADR-0023](./0023-spawn-clearance-recalibration.md) 的
  地面安全包络口径：新 C2 安全档 4.0 + 车体半长 0.5 + 载荷外伸 1.0 + 裕度 ≈ 6.0）。二者为
  动态实体且车携网，须以安全包络而非几何不重叠为准。
- **己方 UAV 对（车/目标↔己方机）**：用**几何外伸**口径（两 UAV 半宽之和 / 车半长 + UAV 半宽）。

各实体对下限允许**显式覆盖**参数（`clearance_min_car_own_uav_m` / `clearance_min_target_own_uav_m`
/ `clearance_min_car_obstacle_m` / `clearance_min_target_obstacle_m`）；当覆盖值 `< 0` 时**按两实体
外伸之和推导**（`r_a + r_b`），即默认走几何口径。

> 数值纪律：`0.5037` / `0.25` / `0.15` 与上述对下限均为**暂定值，待实测回填**，不得据作
> 长期标定（同 [ADR-0023](./0023-spawn-clearance-recalibration.md) 表述纪律）。

### (c) 显式运行模式
`own_uav_present` 为**显式运行模式**开关，语义对齐 R-001 `air_present` 原则：**不得由实体缺席
推定**。仅当该参数为 true 才校验己方 UAV 对。`w6_full.launch.py` 起真实空中链路时置 true 并透传
己方机 spawn 位（`px4_own_pose` → `own_uav_x/own_uav_y`，world ENU）；`w6_task_eval.launch.py`
复用 `m0_skeleton`（无空中链路），保持基参默认 false。

### (d) 配置自洽
`obstacle_names` / `obstacle_x` / `obstacle_y` 三数组按索引对应；**长度不一致视为配置错误**，
返回 `obstacle_config_invalid` 并**拒全部候选**（不静默截断）。

## Considered Options

- **延续单一标量代理（仅车↔目标）** —— 被否：即 R-014 所指退化，己方机 / 障碍 / 边界 / 随机
  复核均缺失。
- **全部实体对生硬套 6 m** —— 被否：6 m 是地面安全包络口径；对刚性立柱、对 UAV-UAV 用 6 m
  过紧，会破坏默认场景（默认 spawn 距 `pole_1` 仅 5.0 m）与既有场景登记表。
- **实体对 + 边界统一校验，按对类型区分几何/安全口径** —— **采纳**。

## Consequences

- `scenario_reset` 服务应答失败改为 `reset rejected: <reason>`（reason 见上，可含 `:<障碍名>`），
  日志同步打印；成功 `reset ok`。
- 参数单一注入（契约 [#params](../ROS2迁移交接文档.md#params)）：`config/w6_base.yaml` 增补
  实体外伸 / 实体对下限 / 边界 / 己方机 / 静态障碍清单；`w6_full.launch.py` 透传己方机 spawn 位；
  `w6_task_eval.launch.py` 不改。
- C7 语义落地更完整（实体对 + 边界）；门控逻辑（`isResettable` / bypass）不在本项范围，由
  Task 3（R-015）承担。
- 本决策不新增 / 不修改任何冻结消息或服务 ABI（沿用 `std_srvs/Trigger`）。
