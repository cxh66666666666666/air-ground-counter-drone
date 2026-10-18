# 场景几何单一真源与按域派生

## Status

accepted

## 背景与决策

结构评审 [S-06](../../docs/temp/agc-review-2026-10-14/20-结构.md#s-06)（**用户裁定 · 单一真源 + 派生**）指出：同一几何事实存在**五份独立副本**——物理世界（[`agc_world.sdf`](../../src/agc_sim/worlds/agc_world.sdf) 的 `pole_1/pole_2` pose / size）、空中安全列表（[`air.yaml`](../../src/agc_air/config/air.yaml) `safety.obstacle_names/x/y`）、感知障碍先验（[`obstacles.yaml`](../../src/agc_perception/config/obstacles.yaml)）、感知地图栅格（[`map.yaml`](../../src/agc_perception/config/map.yaml)）、`scenario_reset` 出生校验（[`w6_base.yaml`](../../src/agc_task/config/w6_base.yaml) 障碍 / 边界 + [`scenario_reset.cpp`](../../src/agc_task/src/scenario_reset.cpp) 默认值）。无单一真源、无版本哈希 → 移动一根立柱可能只改某一域，造成「地图安全但物理碰撞」；生成与评估真源不同。契约真源：[#params](../ROS2迁移交接文档.md#params)（launch 单一注入 / 节点不得覆写）、[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C1–C10）。相关上游决策：[ADR-0020](./0020-perception-as-truth-derived-simulation.md)（感知真值派生）/ [ADR-0041](./0041-scenario-axis-carriers-and-coverage.md)（轴承载者）/ [ADR-0023](./0023-spawn-clearance-recalibration.md) / [ADR-0029](./0029-spawn-clearance-entity-pairs-and-boundary.md)（出生净空）。

**先学后做（取证依据）**：核验本机既有实现——① `ros_gz_sim` / gz 的**世界加载**语义（`gz sim <flags> <sdf>`，世界由**单一参数**指定）；② Nav2 `bringup` 的**参数分层 + 缺失即拒启动**惯例（参数 schema 校验）。结论：应把几何真源**上移**为独立文件，各域由其**派生**（launch 生成 / composition 注入），并把真源**内容哈希**记入证据。

**决策**（**不**改冻结 ABI / 唯一执行口语义 / [Runbook #status](../ROS2迁移步骤.md#status)；**不**改世界物理几何**逐值**）：

1. **单一几何真源**：新增 [`agc_sim/config/world_geometry.yaml`](../../src/agc_sim/config/world_geometry.yaml)——静态障碍实体 `id/x/y/z/yaw/size`（+ `color`，仅视觉保真，**非几何**）与作业区 `resolution/size_x/size_y/origin_x/origin_y`。几何值**逐值**与历史一致（`pole_1` @ (0,-6,1.5)、`pole_2` @ (6,6,1.5)，`0.3×0.3×3`）。
2. **物理世界派生**：[`agc_world.sdf`](../../src/agc_sim/worlds/agc_world.sdf) **移除**硬编码 `pole` 块为 **base world**；[`world_bringup.launch.py`](../../src/agc_sim/launch/world_bringup.launch.py) 删除 `_strip_static_obstacles`，改 `_inject_static_obstacles(src, dst, geometry)`——按 `world_obstacles`（`static` 注入 pole 生成临时世界 / `none` 直接用 base world）由真源生成运行期世界。生成世界几何 **== 真源** 逐值一致；**不**改世界其它物理。
3. **按域派生（composition 集中，不内联重复）**：[`w6_composition.py`](../../src/agc_task/launch/w6_composition.py) 读真源（`load_geometry`）并派生——感知障碍先验（`write_derived_obstacles`：`static`=真源 / `none`=空）、感知地图几何（`write_derived_map`：`work_area`）、`scenario_reset` 障碍 / 边界（`compose_params` 注入）、空中安全列表（`apply_air_geometry` 注入 `trajectory_tracker.safety.obstacle_names/x/y`）。`air.yaml` / `w6_base.yaml` / `scenario_reset.cpp` 默认值中的**障碍坐标与边界改派生入口 + 标注**，**不**保留第二份硬编码几何（`obstacle_safety_radius` / margin 等**安全余量**仍留 `air.yaml`，属安全参数非几何）。
4. **边界（standalone 回退）**：[`obstacles.yaml`](../../src/agc_perception/config/obstacles.yaml) / `obstacles_none.yaml` / [`map.yaml`](../../src/agc_perception/config/map.yaml) **保留**为 standalone（不经 composition 的 `perception_bringup` / `air_bringup`）回退副本，文件头注明「几何真源 = `world_geometry.yaml`，本文件须与其一致」；standalone `air_bringup` 亦由真源注入空中安全几何。回退副本由检查器保一致（未决：长期或统一改派生）。
5. **几何版本身份**：`geometry_sha256`（真源文件**字节** sha256）+ 真源路径记入 `scene_manifest`（manifest `geometry` 块：`source` / `sha256` / `work_area` / 障碍 `id/pose/size`）。`assertions()` 增「派生几何 == 真源」断言——不符即**拒绝运行**。
6. **权限成文**：真源为**物理 / 真值**事实，**不等于**「所有算法都可读真值」；感知静态先验仍受 [ADR-0020](./0020-perception-as-truth-derived-simulation.md) 成文约束（不受空观测清除）。
7. **验收**：新增 [`scripts/w6_geometry_check.py`](../../scripts/w6_geometry_check.py)（只读）——①真源 schema；②`static` 世界 pole == 真源、`none` 世界无 pole；③回退副本与真源一致；④`air.yaml` 无硬编码障碍几何；⑤`w6_base.yaml` 去硬编码且 compose 派生 == 真源；⑥`geometry_sha256` 稳定 + manifest `geometry` 块 == 真源；⑦静态先验成文假设仍在。

## Considered Options

- **仅文档标注、仍各自硬编码** —— 被否：未消解「多份独立几何事实」的域间不一致，移动立柱仍可能只改一域。
- **让各域运行时直读物理世界** —— 被否：感知 / 空中安全不应从仿真真值现取几何（会**静默引入真值优势**、破坏 [ADR-0020](./0020-perception-as-truth-derived-simulation.md) 的通道建模口径）。
- **真源上移 + 按域派生（launch 生成 / composition 注入）+ 内容哈希** —— **采纳**：单点修改只改真源，各域随之派生，几何版本可复核。
- **真源放 `air.yaml` / `w6_base.yaml` 内** —— 被否：真源须为**物理世界**服务的独立实体，落在某消费域会加剧「谁是真源」的歧义。

## Consequences

- **代码**：新增 `world_geometry.yaml`（真源）；`agc_world.sdf` → base world；`world_bringup.launch.py` 按 `world_obstacles` 由真源注入 / 不注入生成世界；`w6_composition.py` 集中承载读真源 + 派生 + 断言 + `geometry_sha256`；`air.yaml` / `w6_base.yaml` / `scenario_reset.cpp` 去硬编码几何（改派生入口 + 标注）；`perception_bringup.launch.py` 增 `map_file` 参数；`air_bringup.launch.py` / `m0_skeleton.launch.py` / `w6.launch.py` 注入派生文件；新增 `scripts/w6_geometry_check.py`。
- **行为**：除真源位置外**行为等价**——`static` 世界仍含 `pole_1/pole_2`（逐值不变）、`none` 世界仍无 pole；`scenario_reset` 净空校验语义与 `safety_shield` 校验语义**不变**。
- **未决**：①standalone 回退副本（`obstacles.yaml` / `map.yaml`）现由 `scripts/w6_geometry_check.py` 保一致，**长期**可考虑统一改为派生以彻底消除副本；②`obstacles.yaml` 的 `obstacle_safety_radius` 为**按障碍数**的长度数组（真源障碍数变化时须同步，由 R-038 四数组等长校验在启动期拒绝）——本批真源仍为 2 根立柱，不触发；③run `config/toolchain_world.json` 的几何哈希槽与 S-07（运行身份）协同，本批先落 `scene_manifest`。
- **同步更新**：引用几何 / 世界生成的下游注释就近加中性注；本 ADR 与 [ADR-0041](./0041-scenario-axis-carriers-and-coverage.md)（真源上移注）协同。
- **冻结不变**：9 消息 / 服务 ABI、唯一执行口语义（**无**第二 ROS 命令者）、硬约束 C1–C10、世界物理几何逐值、[Runbook #status](../ROS2迁移步骤.md#status) 均**不改**。

## 只读旧树边界

- 对齐旧树**行为事实**（WHAT）：「旧静止障碍几何真值由场景文件单一给定、`none` 由单一真源去除障碍生成」——本 ADR 把真源**上移**为独立文件并扩到全几何派生，**未**复制旧代码 / 资产 / 目录命名（HOW 全新重写）。
