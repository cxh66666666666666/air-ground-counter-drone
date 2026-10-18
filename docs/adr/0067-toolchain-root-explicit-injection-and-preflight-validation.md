# 工具链根显式注入与启动前验证（S-11）

## Status

accepted

## Date

2026-10-18

## 背景

结构评审 [S-11](../../docs/temp/agc-review-2026-10-14/20-结构.md#s-11)（**路径与安装假设**，中·挑战）指出：
两处 launch 入口 [`w6.launch.py`](../../src/agc_task/launch/w6.launch.py) 与
[`air_bringup.launch.py`](../../src/agc_air/launch/air_bringup.launch.py) 原以
`Path(get_package_share_directory(...)).parents[3]` **由包 share 目录深度上溯**推断项目根，
再从根拼 `toolchain/`（PX4 源码 / 构建 / `bin/MicroXRCEAgent`）。

该推断**依赖 isolated install 的目录层级**：merge-install / overlay 等布局会得到**不同的根**
→ 合法**固定根**项目（[AGENTS.md](../../AGENTS.md#L20) 明确 `/home/chen/AGC`）
反而因与业务无关的布局假设**找不到 PX4 / Agent**；且解析结果**不校验存在性**
→ 缺失时**静默拼路径**、后段才崩。评审建议：在现固定根约束内**显式注入**并**验证**工具链根与
binary，**不**由 package share 目录深度猜；ROS 资产仍通过 **ament index**；若未来要可搬迁，另提
根路径约束挑战及成本，本轮**不**静默推荐全仓相对化。

契约真源：[#params](../ROS2迁移交接文档.md#params)（**参数入口单一注入**）/
[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一执行口语义**不**改）；
上游决策 [ADR-0065](0065-toolchain-rebuildable-pin-manifest.md)（工具链 pin 清单）/
[ADR-0061](0061-w6-runtime-topology-backend-identity-and-single-parameterized-entry.md)（单参数化入口）。

## 决策

### (a) 项目根由 launch 参数显式注入

两入口**声明** launch 参数 `agc_root`（默认 = **固定项目根** `/home/chen/AGC`，可覆盖）；
工具链路径（`<agc_root>/toolchain/...`）由该参数**派生**；**移除** `parents[3]`-based 上溯。
[`air_bringup.launch.py`](../../src/agc_air/launch/air_bringup.launch.py) 的
`generate_launch_description()` 经 `OpaqueFunction` 读取参数（**所有** `DeclareLaunchArgument`
仍在**顶层**声明，保证 `ros2 launch ... --show-args` 可内省）。

### (b) 启动前校验（缺失即拒启动）

两入口在校验 `toolchain/bin/MicroXRCEAgent` 与
`toolchain/PX4-Autopilot/build/px4_sitl_default/bin/px4` 存在；任一缺失即 `raise RuntimeError`
（含缺失路径与 `agc_root:=<path>` 提示），**不**静默。

### (c) ROS 资产仍经 ament index

世界 / 邻包配置仍由 `get_package_share_directory` 取得（**不**改为全仓相对化）。

### (d) 固定根约束维持

维持 [AGENTS.md](../../AGENTS.md#L20) 的**固定根**约束；**不**引入任意搬迁假设
（如需可搬迁，另提根路径约束挑战与成本，**不**静默推荐全仓相对化）。

### (e) 校验脚本

新增 [`scripts/toolchain_root_check.py`](../../scripts/toolchain_root_check.py)（**只读**、退出码语义）：
静态断言两入口**无** `parents[`-based 根上溯且**均**声明 `agc_root`（默认固定根），
功能断言两工具链二进制存在；`ok=false → 退出码 1`。实测 **6 项全 PASS**。

## 备选

1. **保持 `parents[3]` 深度上溯**：写作最省；但**依赖 install 布局**，merge-install / overlay 下
   取到**不同根**，且**无**存在性校验 —— **否决**（改为显式参数 + 启动前校验）。
2. **全仓相对化 / 支持任意搬迁**：看似通用；但本轮**维持固定根**，评审明确「本轮**不**静默推荐
   全仓相对化」，搬迁需求应**另立挑战** —— **否决**（维护固定根约束）。
3. **只硬编码绝对路径、不做参数注入**：最直接；但**不可覆盖**、不便测试与隔离环境验证，
   与评审「**显式注入**」要求不符 —— **否决**（`agc_root` 声明 + 默认固定根 + 可覆盖）。
4. **改用 `AGC` 环境变量解析根**：免参数传递；但本机实测该变量**未设置**（`AGC=<unset>`），
   仍属**隐式**来源 —— **否决**（改用显式 launch 参数，默认固定根，更可审查、可覆盖）。

## 后果

- **正面**：根来源**唯一显式**；缺失即**拒启动**（不静默）；可 `agc_root:=` 覆盖；
  [`toolchain_root_check.py`](../../scripts/toolchain_root_check.py) 可复核（实测 6 项全 PASS）；
  `--show-args` 可内省参数。
- **代价 / 限制**：默认值写死 `/home/chen/AGC`，与固定根约束**耦合**；变更固定根需**同步默认值**。
- **未决 / 后续**：若未来要求**可搬迁**，另提**根路径约束挑战**（含迁移成本）；W7 / W8 **不**改。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[20-结构](../../docs/temp/agc-review-2026-10-14/20-结构.md#s-11)（路径与安装假设）
- 规格：[fix-structure-toolchain-root-asset-provenance-s11-s12 spec](../../.trae/specs/fix-structure-toolchain-root-asset-provenance-s11-s12/spec.md)
- 前序：[ADR-0065](0065-toolchain-rebuildable-pin-manifest.md)（工具链 pin 清单）/
  [ADR-0061](0061-w6-runtime-topology-backend-identity-and-single-parameterized-entry.md)（单参数化入口）
- 契约：[#params](../ROS2迁移交接文档.md#params) / [#execution-port](../ROS2迁移交接文档.md#execution-port)
- 实现：[w6.launch.py](../../src/agc_task/launch/w6.launch.py) /
  [air_bringup.launch.py](../../src/agc_air/launch/air_bringup.launch.py) /
  [toolchain_root_check.py](../../scripts/toolchain_root_check.py)
