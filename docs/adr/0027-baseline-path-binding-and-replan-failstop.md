# 离线基线路径绑定与重规划失效安全（R-012）

## Status
accepted

## 背景与决策

评审风险 **R-012（规划失败）**：`agc_ground` 的离线对照基线 `ground_baseline::plan()` 对新路径为空或抛异常
**仅打印 warning、不清旧 `path_`**；`tick` 随后继续 `lookaheadRef` **旧路径**。路径**未绑定地图版本 / 目标版本 /
有效时限**——目标不可达或地图增障后，车辆仍可能驶向**旧目标 / 旧通道**；安全层停车不等于规划基线正确。

本 ADR 记录三项决策：**(a) 路径绑定（目标身份 + 地图签名 + 有效时限）**、**(b) 地图签名口径**、
**(c) 规划失败 → 失效停车**；均已按 [ADR-0026](./0026-rl-action-conventions-and-ground-escape-calibration.md)
同等的"先学后做"流程对齐同版本 Nav2 / ROS 约定后落地。

### (a) 路径绑定：目标身份 + 地图签名 + 有效时限

**决策**：每次**成功**规划即记录该路径的绑定三元组，并仅在绑定仍有效时才继续使用既有路径：

- **目标身份**：规划时刻的 `goal`（`last_plan_goal_`）。当前 `goal` 与其位移 > `replan_displacement_m` → 判为**目标变**。
- **地图签名**：规划时刻的 costmap 内容签名（`last_plan_map_sig_`，见 (b)）。当前签名与之不等 → 判为**地图变**。
- **有效时限**：`max_path_age_s`（config）——路径自规划起超过该时长 → 判为**超时**，强制重规划。

`needReplan` 按 **目标变 → 地图变 → 超时 → （否则）周期 `replan_period_s`** 的顺序判定，任一绑定失效即**立即**
重规划（受 `kReplanGuardS = 0.2 s` 限流防止同 tick 抖动）；正常无变化时仅按周期刷新。

### (b) 地图签名口径：**硬障碍栅格**内容签名

**先学后做（取证）**：Nav2 planner/controller **不以 costmap 内容变化驱动重规划**（仅按目标更新 / 显式请求），
故"地图增障 → 重规划"为本项目自定语义，需自行定义"地图变"的可判定口径。复用 Nav2 组件
`Costmap2DROS::getCostmap()` + `Costmap2D::getCharMap()`（本机 Jazzy 头文件已核验：
`costmap_2d_ros.hpp` `getCostmap()`、`costmap_2d.hpp` `getCharMap()` / `getSizeInCellsX/Y` / `getOriginX/Y`）。

**决策**：签名 = 对 costmap **尺寸 + 原点 + `cost ≥ 253`（`INSCRIBED_INFLATED_OBSTACLE`，即 lethal / inscribed）
栅格索引**做 FNV-1a 64 位哈希。

**为何不用全栅格内容哈希（实测依据，r012 GUI 排障）**：全栅格哈希在车辆行驶中会因 obstacle/voxel 层逐帧
`raytrace` **clearing**（FREE / UNKNOWN 翻转）与 inflation 传播而**瞬态抖动**——实测触发高频重规划
（路径发布率由周期 0.5 Hz 升至 ~0.6–0.9 Hz）。改为**只计硬障碍栅格**后：无增障时签名在行驶中**稳定**
（路径发布率 ≈ 周期）、增障后签名**即偏离**，与噪声解耦。

### (c) 规划失败 → 失效停车（fail-stop）

**决策**：`plan()` 在**空路径**或**抛异常**时置 `path_valid_ = false` 并记录失效日志；`tick` 对**失效路径**
默认**发布零速**（fail-stop），**不再** `lookaheadRef` 旧路径；仅当绑定有效（目标未变 / 地图未变 / 未超时）时
才继续既有路径。既有参数 **`stop_on_invalid`**（原未被消费）**接线生效**：为 `true`（默认）时失效即停车；
为 `false` 为**显式 opt-out**（回落到旧路径，仅用于对照实验，默认配置不启用）。

**先学后做（取证）**：本机 Jazzy `nav2_navfn_planner::NavfnPlanner::createPlan` 在 `makePlan` 失败
（目标不可达 / 出界 / 全部被占）时 **throw `nav2_core::PlannerException`**（`strings libnav2_navfn_planner.so`：
`"Failed to create plan with tolerance of:"`）——故"目标不可达"实际落到本节点 `catch` **异常**分支；
空路径（`poses.size() < 2`）分支为对**其它** planner（返回空而不抛）的防御分支。

## 证据（GUI 实跑，硬约束 C3）

`scripts/run_w5_r012.sh gui` → `experiments/w5-ground-nav-safety/20261015-w5-r012-0/metrics/r012.json`
（`overall_pass: true`）。在 `w5_safety_scenario.launch.py command_source:=baseline` 下，冻结 `task_manager`
后由脚本直接注入 `/agc/ground/goal`：

| 子测 | 判据 | 结果 |
|---|---|---|
| d 正常不抖动 | 目标/地图未变、未超时 → 路径发布率 ≈ 周期（<< 控制 20 Hz） | PASS（0.375–0.75 Hz） |
| a 地图增障 | 沿路径注入静态障碍 → 节点日志「重规划触发：地图变化」+ 迅速产出**改变**的新路径 | PASS（latency 0.002 s，path_changed） |
| b 目标跳变 | 改可达新目标 → 新路径端点 ≈ 新目标（目标身份绑定） | PASS（end == 新目标） |
| c 不可达目标 | `createPlan` 抛异常 → 路径失效 + 零速停车 + 不再驶向旧目标 | PASS（branch=exception，停车，失效后 0 新路径） |

## Nav2 吸收点

| 吸收点 | 来源 | 落点 |
|---|---|---|
| planner 失败即**不沿用旧路径**（失效当前路径） | `对齐 Nav2` | `path_valid_ = false` + fail-stop |
| 路径 / 目标 / 地图版本关联与有效时限 | `全新重写` | `last_plan_goal_` / `last_plan_map_sig_` / `max_path_age_s` |
| `Costmap2DROS::getCostmap()` + `Costmap2D::getCharMap()` 取栅格做**内容签名** | `复用 Nav2 组件`（本机 Jazzy 头文件已核验） | `mapSignature()`（硬障碍栅格 FNV-1a） |

**未采纳**：不引入 `bt_navigator` / 行为树；**不新增第二 ROS 命令者**（契约 [#execution-port](../../docs/ROS2迁移交接文档.md#execution-port)）；
不改 Nav2 planner 插件本体。

## 参数（`src/agc_ground/config/baseline.yaml` 单一注入）

- 新增 `max_path_age_s`（路径有效时限）。
- 既有 `stop_on_invalid` 接线生效（默认 `True`）。
