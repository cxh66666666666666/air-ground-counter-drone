# toolchain/ — 项目本地仿真工具链

> 本目录**不纳入版本控制**（见根 `.gitignore`），仅本 README 入库。用于索引工具链内容，使环境可重建。
> 复现步骤与环境坑见 Runbook [附录 B](../docs/ROS2迁移步骤.md#appendix-b)，本文只索引、不复述。

## 组成

| 路径 | 内容 |
|---|---|
| `PX4-Autopilot/` | PX4 v1.15.4 源码 + SITL 构建 |
| `Micro-XRCE-DDS-Agent/` | uXRCE-DDS Agent v2.4.2 |
| `bin/` | 工具入口（如 `MicroXRCEAgent`） |
| `localdeps/` | 离线依赖 |
| `microxrce_compat/` | fmt 9 兼容头 |