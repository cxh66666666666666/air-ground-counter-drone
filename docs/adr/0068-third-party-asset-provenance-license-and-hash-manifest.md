# 外部资产来源、许可与哈希清单（S-12）

## Status

accepted

## Date

2026-10-18

## 背景

结构评审 [S-12](../../docs/temp/agc-review-2026-10-14/20-结构.md#s-12)（**外部资产与发布元数据**，中·挑战）指出：
[`agc_car.sdf.xacro`](../../src/agc_sim/models/agc_car/agc_car.sdf.xacro#L5) 仅在**注释**声明 5 个 mesh 取自
Gazebo Fuel `Clearpath Robotics Husky A200`（自称 owner OpenRobotics、`CC0 v1.0`）；
[`meshes/`](../../src/agc_sim/models/agc_car/meshes) 目录**无** `LICENSE` / `NOTICE` / 来源版本 / 文件哈希；
自有包 `package.xml` 多为 `0.0.0` + `Proprietary`（如 [agc_air](../../src/agc_air/package.xml)）；
`px4_msgs` 有自身 `LICENSE`。→ 发布 / 软著交付时**难以说明**哪些资产原创 / 来源 / 修改 / 许可；
工具链 pin（[ADR-0065](0065-toolchain-rebuildable-pin-manifest.md)）**替代不了 mesh 身份**。

**先学后做核查（本轮）**：上游 Clearpath Husky **ROS 资产族**（`husky_description` / `husky_gazebo` /
`clearpath_gz`）在 ROS Wiki / ROS Index 标 **`BSD`**（要求**保留版权 / 许可署名**），与仓内注释自称
「**CC0 v1.0**」**不一致**；Fuel 模型页的**确切资源 id / 下载版本 / 随附 LICENSE 文本**本轮**未能取回**。
→ 现行「CC0」属**未经验证的断言**，**必须**据实登记（候选许可 + 来源材料未取回 → **未决**）。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C1–C10 **不**改）/
[#decisions](../ROS2迁移交接文档.md#decisions)；上游决策 [ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md#third-party-license-note)（保形换装 / 许可适用关系注）/
[ADR-0065](0065-toolchain-rebuildable-pin-manifest.md)（工具链 pin 清单）/
[ADR-0007](0007-experiment-artifacts-convention.md)（实验产物唯一落点）。

## 决策

### (a) 第三方资产独立清单（人读 + 机器可读）

新增 [`docs/conventions/第三方资产与许可.md`](../conventions/第三方资产与许可.md)：政策 + 人读表 +
```yaml 块（`agc_third_party_assets:`，`schema: 1`），逐资产记 **id / 上游来源（模型名 / owner / 资源 id /
取得方式）/ 下载版本 / 上游许可 / 署名 / `sha256`**。

### (b) 许可分列、不外推

自有包（`agc_*`）许可 `Proprietary` **仅**适用自有代码，**不得**外推到第三方资产；第三方资产许可
**逐资产独立**登记，且**不得**标 `Proprietary`。**不**改包版本号（`0.0.0` / `Proprietary` 保留）。

### (c) 据实登记，未核实即未决

许可**无法当场核实**时，登记**候选许可 + 状态 `unresolved`** 并写明理由（`license_note`）；**禁止**
未经验证地**断言**许可。据实：候选 `BSD`（上游资产族口径）+「Fuel 资源材料未取回」→ 未决。

### (d) 署名随资产入库 + 修正注释断言

[`meshes/NOTICE`](../../src/agc_sim/models/agc_car/meshes/NOTICE) 承载来源 / 署名 / 许可状态（含显式未决）；
[`agc_car.sdf.xacro`](../../src/agc_sim/models/agc_car/agc_car.sdf.xacro#L5) 顶部注释**去**未验证 `CC0`
断言，改**引用**清单；**不改** mesh 文件内容、**不改**物理 / 惯性 / 传感器 / 几何参数。

### (e) 校验脚本

新增 [`scripts/third_party_asset_check.py`](../../scripts/third_party_asset_check.py)（**只读**、退出码语义）：
清单 == 实测 `sha256`、无未列出 mesh（**双向**集合）、第三方**未**标 `Proprietary`、`unresolved` 需理由；
`ok=false → 退出码 1`。实测 **26 项全 PASS / 退出码 0**。

### (f) 正式分发政策

对外分发以**版本 / tag** + **artifact manifest**（逐文件哈希）为准；本清单为其**资产级**子集，
工具链 pin 由 [ADR-0065](0065-toolchain-rebuildable-pin-manifest.md) 覆盖。**本轮不**实建 tag、**不**改包版本号。

## 备选

1. **沿用注释断言 `CC0`**：最省；但**未核实**且与上游 `BSD` 口径**不一致**，致交付可能不合规 ——
   **否决**（改为据实登记）。
2. **直接把 `Proprietary` 覆盖到所有资产**：统一省事；但**错置**第三方资产许可、**违**上游署名要求 ——
   **否决**（分列不外推）。
3. **仅引入第三方 ROS 仿真包（Clearpath）**：省去自维资产；但外部包进 `src/` 违
   [ADR-0001](0001-greenfield-ros2-rewrite.md) 与包边界 —— **否决**（沿用 [ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md) 既有裁定）。
4. **许可取回前不予登记 / 静默留空**：看似「不犯错」；但违「**未决不静默假设**」纪律 ——
   **否决**（显式 `unresolved` + 候选）。

## 后果

- **正面**：资产来源 / 许可 / 署名 / 哈希**独立可核**；许可**分列**（不外推）；未决**显式**（不静默断言）；
  [`third_party_asset_check.py`](../../scripts/third_party_asset_check.py) 可复核（实测 26 项全 PASS）。
- **代价 / 限制**：清单需随**新增资产**同步维护（**先登记后入库**）；`unresolved` 字段需**后续回填**。
- **未决 / 后续**：Fuel 资源 id / 下载版本 / 随附 LICENSE 文本**待取回**；取回后回填并置
  `license_status: resolved`。W7 / W8 **不**改。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[20-结构](../../docs/temp/agc-review-2026-10-14/20-结构.md#s-12)（外部资产与发布元数据）
- 规格：[fix-structure-toolchain-root-asset-provenance-s11-s12 spec](../../.trae/specs/fix-structure-toolchain-root-asset-provenance-s11-s12/spec.md)
- 前序：[ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md#third-party-license-note)（换装 / 许可适用关系注）/
  [ADR-0065](0065-toolchain-rebuildable-pin-manifest.md)（工具链 pin 清单）/
  [ADR-0007](0007-experiment-artifacts-convention.md)（实验产物唯一落点）
- 实现：[第三方资产与许可](../conventions/第三方资产与许可.md) /
  [meshes/NOTICE](../../src/agc_sim/models/agc_car/meshes/NOTICE) /
  [agc_car.sdf.xacro](../../src/agc_sim/models/agc_car/agc_car.sdf.xacro#L5) /
  [third_party_asset_check.py](../../scripts/third_party_asset_check.py)
