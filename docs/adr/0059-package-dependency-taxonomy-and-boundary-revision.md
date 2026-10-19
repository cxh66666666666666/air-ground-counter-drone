# 包依赖按类别分层（修订包边界）

## Status
accepted

## 背景与决策

W0 产出的 [包边界](../conventions/包边界.md) 把**消息通信 / 库链接 / launch 依赖混成同一规则**：原文规则 1「`agc_msgs` 是唯一公共依赖」与规则 2「其余包之间不得互相依赖对方的库」照字面会**禁止必要的运行依赖**（如 `agc_task` 在 launch 中起 `agc_air` / `agc_ground` / `agc_perception`），或促使这些真实运行依赖被隐藏、未在 manifest 声明；依赖方向图也无法解释现状（`agc_task` 承载 PX4 驱动 / 干扰注入、跨域合理使用 `nav_msgs` / `geometry_msgs`）。此为结构评审 [S-02](../reviews/agc-review-2026-10-14/20-结构.md#s-02)「挑战既定约束」项，**经用户裁定正式修订**（改规范 + 记 ADR）。

**先学后做（取证）**：对照同版本 Nav2 成功用法——`nav2_bringup` 对**仅被 launch 起进程**的包（`navigation2` / `nav2_minimal_tb3_sim` / `ros_gz_sim` / `ros_gz_bridge`）使用 `<exec_depend>` 而**不** `find_package` 链接其库；对**编译期使用的库**（`nav2_costmap_2d` / `nav2_util`）使用 `<depend>`。即 Nav2 自身已把「链接库」与「起进程」分开声明。

据此修订 [包边界](../conventions/包边界.md)，把依赖**按类别分层**：

- **①消息 / 接口依赖**：跨域一律经 `agc_msgs`（叶依赖）。
- **②库链接依赖**：仅 `agc_control` 共享库允许 `agc_air` / `agc_ground` 链接同一实现；除此禁止业务库互链。
- **③运行组合 / launch 依赖**：编排者以 `exec_depend` 声明所起进程之包，**不**据此链接其实现库。
- **④外部设备 / 第三方边界**：`px4_msgs` / uXRCE-DDS、Nav2、`ros_gz` / gz vendor。

**保留**：禁止环形依赖；禁止包间业务库互链。

## 附：内部库的公共性裁定（S-03）

**先学后做（取证）**：对照同版本 Nav2——公共库包 `nav2_util` / `nav2_costmap_2d` **同时** `ament_export_include_directories`（`nav2_util` 另导出库目标 `nav2_util_core`）**与** `install(DIRECTORY include/ ...)`，导出 == 产物（`/opt/ros/jazzy/include/<pkg>/` 确实存在）；而纯 launch / config 包 `nav2_bringup` 的生成 CMake **无任何** `ament_cmake_export_*`（其 `Config.cmake` 的 `_extras` 为空）。结论：**「对外承诺」必须与「安装产物」一致**——承诺须落地，纯内部则**零承诺**。

**本仓事实（取证）**：`agc_air` 的 `agc_air_lib` 为 `STATIC`、仅本包节点 / 测试使用；`agc_air` 只 `install(TARGETS <node>)` + `install(DIRECTORY launch config)`，**不**安装 `include/`；却曾 `ament_export_include_directories(include)` + `ament_export_dependencies(${DEPS} Eigen3)` + `agc_air_lib` 的 `$<INSTALL_INTERFACE:include>` → 生成的 `ament_cmake_export_include_directories-extras.cmake` 指向**不存在**的 `install/include`（下游 `find_package(agc_air)` 会报 "exports the include directory … which doesn't exist" 警告）；全仓**无** `find_package(agc_air)` 消费者。

**裁定**：`agc_air_lib` 为**纯内部实现**，`agc_air` **不**对外承诺公共接口——移除 `ament_export_include_directories(include)`、`agc_air_lib` 的 `$<INSTALL_INTERFACE:include>` 与 `ament_export_dependencies`；`agc_air` **不**安装 `include/`（对齐 `nav2_bringup` 的「内部 / 仅 launch 包零导出」）。共享通用控制继续以 `agc_control`（`INTERFACE` + 安装 `include/` + `ament_export_*`）为**唯一**公共库家（[ADR-0018](./0018-shared-control-layer-and-lqr-plugin.md)）。**未**为修 export 新建任何跨域链接。若将来某 helper 需**真正公共**，须经包边界批准并**同时**安装头文件 / 正确导出。

## Considered Options

- **维持单一规则**（「唯一公共依赖」）—— 被否：与真实运行拓扑不符，会误禁必要运行依赖或催生隐藏依赖。
- **允许任意包互依赖**（无类别区分）—— 被否：丧失边界约束，退化为无架构。
- **只按 build/exec/test 三分**（不区分消息 / 库 / 组合）—— 被否：不解释「起进程 vs 链接库」的本质差异。

## Consequences

- [包边界](../conventions/包边界.md) 依赖方向（第 2 节）与依赖检查（第 3 节）按四类改写；依赖方向图区分「链接」与「运行组合」。
- 现状（`agc_task` 起各域 + 承载 PX4 驱动 / 干扰注入）在新规则下**合规**，**不**迁包。
- **未决（显式登记）**：`intruder_driver` / `jammer` 是否应归入独立「设备域」（而非留在 `agc_task`）——本轮**不**迁移；后续若迁，按③关系调整 `exec_depend`，**不**引入库链接。
- 契约（[唯一执行口](../ROS2迁移交接文档.md#execution-port) / [硬约束](../ROS2迁移交接文档.md#hard-constraints)）**不受影响**。
