# 地图内容身份与单调更新序号（R-040）

## Status

accepted

## Date

2026-10-18

## 背景

评审风险 **R-040（地图身份）**（[10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-040)）指出，
[`map_adapter.cpp`](../../src/agc_perception/src/map_adapter.cpp) 的 `/agc/map/version_hash` **仅**由
`hashBytes(data)`（对栅格 `data` 字节串做 FNV-1a）生成，**不含** `frame_id` / `resolution` / `width` /
`height` / `origin` / **场景身份**：

- **相同 `data` 可代表不同地图**——两份网格若栅格字节相同但 `origin` / `resolution`（或 `frame_id`）不同，
  却是**不同的世界几何**，却被判为「地图未变」；据 hash 证明复现**只能证明一串字节，不能证明空间相同**；
- 等价判定 `sameObstacles` 按**分辨率分箱**比较障碍位置 / 尺寸、yaw 用**固定 `0.1` 容差**（不随尺寸缩放）
  → 障碍边缘**跨格**后栅格可能已变，却因**同分箱**被判「未变」而**漏报**（抑制噪声重建时静默遗漏占据边界变化）；
- 无**单调更新序号**表达「实质更新」，消费者无法区分「内容等价」与「发生了一次实质重建」。

契约真源：[#namespace](../ROS2迁移交接文档.md#namespace)（`world` 系）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（地面唯一命令通道，本项只改感知端地图发布与诊断）、
[#params](../ROS2迁移交接文档.md#params)（launch 单一设置）、
[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C9 指标口径 / C10 语义）、
[#messages](../ROS2迁移交接文档.md#messages)（冻结 9 消息）、[#lessons](../ROS2迁移交接文档.md#lessons)（reset 后须校验新数据）。

分类：**设计风险（内容身份 / 更新语义）**。本 ADR 决定**内容身份 hash 折入几何元数据与源版本、区分内容 hash
与单调更新序号、等价判定改为真实 raster（不欠包）、保持 R-006 三分语义**；**不**改冻结 9 消息 / 服务 ABI，
**不**变唯一执行口语义，**不**新增第二 ROS 命令者，**不**重写地图算法，**不**改 `/agc/map` 的 `OccupancyGrid` 契约。

## 取证（先学后做）

按 [AGENTS.md](../../AGENTS.md) 铁律「先学后做」，**先**对齐**同版本**（ROS 2 Jazzy）Nav2 的
**栅格内容签名 / 更新序号 / 同数据不同几何的装载校验**成功用法，再改自身实现。

**同版本 Nav2 取证（本机 `/opt/ros/jazzy/include/...` 实读的源文件与符号）：**

- **几何元数据是地图身份的组成部分**：`nav2_costmap_2d::Costmap2D`
  （`include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d.hpp`）显式暴露几何访问器
  `getSizeInCellsX()`（:248）/ `getSizeInCellsY()`（:254）/ `getOriginX()`（:272）/ `getOriginY()`（:278）/
  `getResolution()`（:284），内部成员 `size_x_` / `size_y_` / `resolution_` / `origin_x_`（:530-533）；
  `Costmap2DPublisher`（`include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d_publisher.hpp`）缓存
  `saved_origin_x_` / `saved_origin_y_`（:161-162）并在 `updateGridParams()`（:143）时按几何更新发布参数——
  **origin / resolution 变化被视为「发布的地图变了」**，而非无关元数据。
- **内容（full）与更新（delta）分离，而非以「一串字节」代表身份**：`Costmap2DPublisher` 以
  `updateBounds()`（:116）维护**变化矩形** `x0_/xn_/y0_/yn_`（:157-160），`publishCostmap()`（:128）发布
  **完整** `nav_msgs/OccupancyGrid` / `nav2_msgs/Costmap` 与**增量** `createGridUpdateMsg()` /
  `createCostmapUpdateMsg()`（:135-138）；`always_send_full_costmap_`（:75）与 `costmap_published_once_`（:164）
  表达「完整内容 vs 增量更新」两种语义。对应消息 `nav2_msgs/msg/CostmapUpdate.msg` 与
  `map_msgs/msg/OccupancyGridUpdate.msg`（字段 `header` / `x` / `y` / `width` / `height` / `data`）是**独立的
  更新通道**——Nav2 **不**用单条 hash 同时承担「内容身份」与「更新计数」。
- **同数据不同几何 / 模式在装载期被显式校验**：`nav2_map_server::LoadParameters`
  （`include/nav2_map_server/map_io.hpp` :31-40）以 `resolution` / `origin` / `mode` / `free_thresh` /
  `occupied_thresh` / `negate` 为**装载参数**；`LOAD_MAP_STATUS`（:42-48）区分
  `INVALID_MAP_METADATA` 与 `INVALID_MAP_DATA`——**同样的像素数据在不同 `mode` / `resolution` / `origin` 下
  语义不同**，非法元数据是**显式拒绝**，非静默接受。装载入口 `loadMapYaml()`（:56）/ `loadMapFromFile()`（:64）/
  `loadMapFromYaml()`（:75）。
- **结论（本版本惯例）**：① 地图身份**须含几何元数据**（frame / resolution / size / origin）；
  ② 「内容等价」与「更新序列」应**分别表达**（full 内容 vs 增量更新）；
  ③ 同 `data` 不同几何 / 模式**语义不同**，须能**区分并校验**。

**本项目现状差距（读取时工作区）：** `hashBytes(data)` 仅对栅格字节串哈希、版本赋值即该串；`sameObstacles`
按分辨率分箱 + 固定 yaw `0.1` 容差；无单调更新序号。

**本批取证实测（本机真实二进制 / 实跑）：** 见
`experiments/w6-task-eval/<run_id>/r040/bench.json` —— 修复后：同 `data` 字节、不同 `origin_x` 的两 run
**data-only hash 相同**而**内容身份 hash 不同**；同分箱（`floor(x/res+0.5)` 均 1）跨格 → `data` 变化且
`update_seq` **递增**；同内容反复输入 → `update_seq` / `version_hash` **不变**，实质变化（移动 / 动态轨过期）
→ **严格递增**；从 `/agc/map` 消息自身重算的内容身份 hash 与发布的 `version_hash` **逐条一致**。

## 决策

### (a) 内容身份 hash 折入几何元数据与源版本

- `/agc/map/version_hash`（`std_msgs/String`，latched）**话题名保持**，但其内容改为**内容身份 hash**：
  `FNV-1a( frame_id, obstacles_source, scenario_version, resolution, width, height, origin_x, origin_y, data )`。
  组装顺序（**须与核查脚本逐字节一致**）：
  `frame_id`+NUL, `obstacles_source`+NUL, `scenario_version`+NUL, `resolution`(8B), `width`(8B),
  `height`(8B), `origin_x`(8B), `origin_y`(8B), `data` 字节。
- **确定性字节化**：`double` / `uint64` 以**本机 IEEE-754 binary64 / little-endian 内存字节**（`memcpy` 到
  `uint8_t[8]` 后喂入）——本项目全链**同机同架构**，字节序稳定；**不**用文本格式化（避免精度 / 舍入歧义）。
  `hashBytes` **保留**为底层字节哈希工具（内容身份 hash 由它 + 元数据组装后喂入）。
- 新增参数 `scenario_version`（string，default `""`）与既有 `obstacles_source` 作为**场景 / 源版本**组成；
  由 launch 单一注入（[#params](../ROS2迁移交接文档.md#params)）。

### (b) 区分内容 hash 与单调更新序号

- 新增成员 `update_seq_`（`uint64_t`，初始 `0`）与 latched 诊断话题 `/agc/map/update_seq`
  （`std_msgs/UInt64`；参数 `update_seq_topic`，default `/agc/map/update_seq`）。
  `update_seq_` **仅**在**实质重建**（栅格内容或身份改变）时 **+1**；与内容 hash **分别**可查询。
- `/agc/map/version_hash` 仅当**内容身份变化**时才发布（保持原去重惯例）。

### (c) 等价判定改为真实 raster（精确等价，不欠包）

- **废弃** `sameObstacles` 的分辨率分箱 + 固定 yaw `0.1` 容差（及 `last_unknown_` / `effective_` 相关逻辑）。
- `rebuild()` 对**候选有效障碍集合**（含 static 覆盖 / 动态新鲜 / unknown 覆盖逻辑，**保持 R-006**）**实际
  栅格化**出候选 `data`，与当前 `grid_.data`（若 `built_`）**逐字节比较**：相同 → 内容等价，**不重建、不发布、
  序号不变**（`return false`）；不同 → 采用、`update_seq_++`、重算内容 hash、返回 true。
- 真实 raster 相等是**精确等价**、天然**不欠包**——障碍边缘**跨格**必被捕获；**不**用未验证分箱 / 固定 yaw
  容差替代等价性证明。

### (d) 保持 R-006 三分语义与数据契约

- `0=free / 100=occupied / -1=unknown`；`/agc/map/valid = geometry_ok_ && knowledge_current()`；
  静态先验 `static_prior_` **永不被空观测清除**；动态轨按 `dynamic_max_age_s_` 过期 → 覆盖格标 unknown；
  `validateStartupOrThrow()`（R-038）保留。`/agc/map` 仍 latched、`frame_id=world`、`info` 五字段正确。
- **不**重写地图算法，**不**改 `OccupancyGrid` 契约。

### (e) 验收

- `scripts/run_w6_r040.sh` + `scripts/w6_r040_bench.py`（真实二进制 + 合成障碍流 / 临时 map YAML：
  同字节不同 origin / 同分箱跨边界 / 单调序号 / R-006）+ `scripts/w6_r040_check.py`（自包含核查 (a)…(j)，
  含：从 `/agc/map` 消息自身重算内容身份 hash 与发布版一致）。

### (f) 边界（不改契约）

**不**改冻结 9 消息 / 服务 ABI，**不**变唯一执行口语义，**不**新增第二 ROS 命令者；新增均为诊断 / run config
（`/agc/map/update_seq` 为 `std_msgs/UInt64`；`scenario_version` / `update_seq_topic` 为 run config）；
`/agc/map` 的 `OccupancyGrid`（三态 / `info` 五字段 / `frame_id=world`）语义**不变**；
`/agc/map/version_hash`（`String`）与 `/agc/map/valid`（`Bool`）话题与类型**不变**（仅 `version_hash` 内容语义
由「{data} 字节串」改为「含几何元数据 + 源版本的内容身份」）。

## 备选

1. **保留仅 `hashBytes(data)`，只加注释说明**——**否决**：相同 `data` 不同几何仍判「未变」，正是 R-040 要消除的
   身份混淆。
2. **内容 hash 里用文本格式化 double（如 `"%.17g"`）**——**否决**：跨格式化库存在精度 / 舍入歧义；改用固定宽度
   `memcpy` 内存字节（同机同架构稳定，且核查脚本可用 `struct.pack('<d')` 逐字节镜像）。
3. **保留分箱 + 固定 yaw 容差，仅调大阈值**——**否决**：分箱 / 固定容差**无法证明不欠包**（边缘跨格必漏报）；
   改用真实 raster 逐字节比较（精确等价）。
4. **用内容 hash 同时充当更新序号**——**否决**：hash 表**内容等价**（同内容同 hash），序号表**实质更新计数**；
   二者语义不同，须分别表达（对齐 Nav2 full 内容 / 增量更新分离）。

## 后果

- **正面**：`/agc/map/version_hash` 如实表达**含几何元数据 + 源版本的内容身份**（同 `data` 不同几何**不再**误判
  相同）；`/agc/map/update_seq` 单调递增使「内容等价」与「实质重建」**分别**可查；等价判定改为真实 raster
  **不欠包**（边缘跨格不再漏报）；R-006 三分语义与 `OccupancyGrid` 契约保持。
- **代价 / 限制**：内容身份 hash 的字节序依赖**同机同架构**（本项目全链满足；跨架构须复核）；`rebuild()` 每次
  候选都做一次完整栅格化（较分箱略贵，但本项目地图规模小、可接受）。
- **未决**：若未来引入**跨架构**回放 / 异构节点，需将 double 字节序显式规范化为固定（如强制 little-endian
  `static_cast`）或改用带版本的规范化文本——本批不做，按需另议。[R-040](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-040)
  属 [00-总览](../../docs/reviews/agc-review-2026-10-14/00-总览.md#roadmap) 的「可延后但需登记」层，**发布 / 干净重建前
  必须完成**。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/reviews/agc-review-2026-10-14/10-风险.md#r-040)
- 规格：[fix-ground-odom-map-identity-goal-r39-r41 spec](../../.trae/specs/fix-ground-odom-map-identity-goal-r39-r41/spec.md)（R-040）
- 契约：[#namespace](../ROS2迁移交接文档.md#namespace) / [#execution-port](../ROS2迁移交接文档.md#execution-port) /
  [#params](../ROS2迁移交接文档.md#params) / [#messages](../ROS2迁移交接文档.md#messages) /
  [#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)
- Nav2 参照（实读）：`nav2_costmap_2d::Costmap2D` 几何访问器 `getSizeInCellsX/Y` / `getOriginX/Y` /
  `getResolution`（`costmap_2d.hpp`）；`Costmap2DPublisher` 的 `updateBounds` / `publishCostmap` /
  `createGridUpdateMsg` / `createCostmapUpdateMsg` 与 `saved_origin_x_/y_` / `always_send_full_costmap_`
  （`costmap_2d_publisher.hpp`）；`nav2_map_server::LoadParameters` / `LOAD_MAP_STATUS` 的
  `INVALID_MAP_METADATA` / `INVALID_MAP_DATA`（`map_io.hpp`）；`nav2_msgs/CostmapUpdate` /
  `map_msgs/OccupancyGridUpdate`
- 前序：[ADR-0020](0020-perception-as-truth-derived-simulation.md)（真值派生感知）/
  [ADR-0008](0008-nav2-paradigm-absorption-as-code-principle.md)（Nav2 吸收）/
  [ADR-0053](0053-lqr-numeric-model-exposure-and-config-validity.md)（R-038 含地图几何启动期校验）/
  [ADR-0054](0054-ground-odom-source-time-and-reset-epoch-velocity.md)（R-039）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 实现：`map_adapter.cpp` / `config/perception.yaml` / `launch/perception_bringup.launch.py` /
  `run_w6_r040.sh` / `w6_r040_bench.py` / `w6_r040_check.py`
