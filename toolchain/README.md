# toolchain/ — 项目本地仿真工具链

> 本目录**不纳入版本控制**（见根 `.gitignore`），仅本 README 入库。用于索引工具链内容，使环境可重建。
> 复现步骤与环境坑见 Runbook [附录 B](../docs/ROS2迁移步骤.md#appendix-b)，本文只索引、不复述。

## 组成

| 路径 | 内容 |
|---|---|
| `PX4-Autopilot/` | PX4 v1.15.4 源码 + SITL 构建 |
| `Micro-XRCE-DDS-Agent/` | uXRCE-DDS Agent v2.4.2（**已弃用**，见下） |
| `Micro-XRCE-DDS-Agent-2.4.3/` | uXRCE-DDS Agent v2.4.3（**现行**，对系统 Fast DDS 构建） |
| `bin/` | 工具入口（如 `MicroXRCEAgent` → v2.4.3） |
| `localdeps/` | 离线依赖（agent 2.4.3 起仅用于 spdlog 等非 DDS 依赖） |
| `microxrce_compat/` | fmt 9 兼容头 |

## uXRCE-DDS Agent 版本（必须与 ROS 2 的 DDS 栈同版本）

**结论（2026-10-12）**：agent 必须与 ROS 2 Jazzy 使用**同一 Fast DDS / fastcdr 版本**，否则会出现
「PX4↔agent 的 XRCE 链路一切正常（会话建立、time sync converged、`/fmu/out/*` datawriter 创建成功），
但 agent 的 ROS 侧 DDS 端点对 ROS 节点不可见（`ros2 topic info /fmu/out/*` 报 `Publisher count: 0`）」。

| 版本 | 链接 | 与 ROS 2 Jazzy（Fast DDS 2.14 / fastcdr 2.2） |
|---|---|---|
| v2.4.2 | 自建 Fast DDS **2.11** + fastcdr **1.1** | ❌ 跨版本不互通 |
| **v2.4.3** | CMake 依赖即 **fastdds 2.14 + fastcdr 2.2** | ✅ 同源 |

**重建配方**（在 `Micro-XRCE-DDS-Agent-2.4.3/`，先 `source /opt/ros/jazzy/setup.bash`）：

```bash
cmake -S . -B build_ros \
  -DCMAKE_PREFIX_PATH="/opt/ros/jazzy;$AGC/toolchain/localdeps/sysroot/usr" \
  -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_LOGGER=ON \
  -DUAGENT_P2P_PROFILE=OFF -DUAGENT_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-I/opt/ros/jazzy/include/fastrtps -I/opt/ros/jazzy/include/fastcdr \
    -I$AGC/toolchain/localdeps/sysroot/usr/include -I$AGC/toolchain/localdeps/sysroot/usr/include/x86_64-linux-gnu \
    -I$AGC/toolchain/Micro-XRCE-DDS-Agent-2.4.3/include \
    -include $AGC/toolchain/microxrce_compat/fmt_compat.hpp"
make -C build_ros -j4     # 产出 build_ros/MicroXRCEAgent
```

要点：①`-I/opt/ros/jazzy/include/fastrtps`、`.../fastcdr` 是 ROS 侧的**真正头文件根**（Fast DDS 头在
`include/fastrtps/fastdds/...`）；②`fmt_compat.hpp` 仍必需（sysroot 的 spdlog 1.9.2 + fmt 9）；
③v2.4.2 源码用的是 fastcdr 1.x 的 camelCase API，**无法**对 fastcdr 2.x 编译，故必须用 v2.4.3。
