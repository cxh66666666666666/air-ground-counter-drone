# PX4 本体以 toolchain 外部依赖形式纳入

## Status
accepted

## 背景与决策
[ADR-0005](./0005-px4-link-uxrce-dds.md) 只决了**链路协议**（uXRCE-DDS + `px4_msgs`），
未决"PX4 本体是否纳入项目、以何形态"。P0 已实际落地：`toolchain/PX4-Autopilot`（v1.15.4，
SITL 已构建）与 `toolchain/Micro-XRCE-DDS-Agent`（v2.4.2，入口 `toolchain/bin/MicroXRCEAgent`），
`toolchain/` 含 `COLCON_IGNORE` 且不入库（`.gitignore`）；接口包 `px4_msgs` 内联在 `src/`，
pin 于 `release/1.15`。

**决策**：显式锁定分工——

- **PX4 本体（固件 / SITL）**：作为 **`toolchain/` 外部依赖**，**不入 colcon、不入 git**；
  以 **v1.15.4 + 子模块 SHA** pin 住，复现步骤记录于 `toolchain/README.md` 与 Runbook 附录 B。
- **`px4_msgs`**：**内联入库**于 `src/`——接口包必须在工作空间内编译才能被 `ros2 interface show`
  与其它包依赖发现（对照：Nav2 为运行时栈，走 apt + `<depend>`，见 ADR-0003）。

## Considered Options
- PX4 源码 vendor 进 `src/` —— 被否：体积大、非 ROS 包、构建重，且污染 colcon 工作空间。
- 系统级 apt PX4 —— 被否：无合适发行包，且需自定义 SITL 机型 / 世界。

## Consequences
- W3 空中链路的复现依赖 `toolchain/` 按 README 重建；新环境须先完成 Runbook 附录 B。
- **PX4 版本与 `px4_msgs` 必须同步**（本仓 pin v1.15.4 ↔ `release/1.15`）；升级须连同本 ADR 复核。
- PX4 的 gz 世界 / 机型与 `agc_sim` 的 `agc_world` 的对接方式在 W3 定夺。