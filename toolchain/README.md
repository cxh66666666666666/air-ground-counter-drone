# toolchain/ — 项目本地仿真工具链

> 本目录**不纳入版本控制**（见根 `.gitignore` 仅放行本文件），**仅本 README 入库**。本 README 是工具链**唯一入库的可重建 pin 清单**：源 SHA / submodule / 兼容输入来源 / 依赖版本 / 构建选项 / 产物二进制哈希，全部只写在这里（`toolchain/` 其余内容被 git 忽略，pin 值写到别处即丢失）。
> 复现步骤与环境坑见 Runbook [附录 B](../docs/ROS2迁移步骤.md#appendix-b)；本文**只索引、不复述**约束类信息（约束真源见契约文档与 Runbook）。

## 组成

| 路径 | 内容 |
|---|---|
| `PX4-Autopilot/` | PX4 v1.15.4 源码 + SITL 构建 |
| `Micro-XRCE-DDS-Agent/` | uXRCE-DDS Agent v2.4.2（**已弃用**，见下） |
| `Micro-XRCE-DDS-Agent-2.4.3/` | uXRCE-DDS Agent v2.4.3（**现行**，对系统 Fast DDS 构建） |
| `bin/` | 工具入口（如 `MicroXRCEAgent` → exec v2.4.3 真实二进制） |
| `localdeps/` | 离线依赖（agent 2.4.3 起仅用于 spdlog / fmt 等非 DDS 依赖；DDS 侧来自 `/opt/ros/jazzy`） |
| `microxrce_compat/` | AGC 自研 fmt 兼容头（`fmt_compat.hpp`） |
| `COLCON_IGNORE` | 使 colcon 不递归扫入本目录 |

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
`include/fastrtps/fastdds/...`）；②`fmt_compat.hpp` 仍必需（sysroot spdlog **1.12** / fmt **9.1**；fmt 9 移除
隐式 ostream formatter，上游 agent 代码需该兼容头才能编译）；③v2.4.2 源码用的是 fastcdr 1.x 的 camelCase API，
**无法**对 fastcdr 2.x 编译，故必须用 v2.4.3。

## 可重建 pin 清单

下表为人类可读摘要；紧随其后的 ```yaml 块是**机器可读真源**（`scripts/toolchain_pin_check.py` 按此解析并比对实测）。

| 类别 | 关键值 |
|---|---|
| PX4 源 | 仓库 tag `v1.15.4`，HEAD `99c40407…561ef`；9 个 SITL(gz) 必需子模块逐条锁定 SHA |
| Agent 源 | `https://github.com/eProsima/Micro-XRCE-DDS-Agent` tag `v2.4.3`，HEAD `73622810…222` |
| 产物 | `bin/MicroXRCEAgent`（包装脚本）、`build_ros/MicroXRCEAgent`、`build/px4_sitl_default/bin/px4`、`fmt_compat.hpp` 逐条 sha256 |
| 构建选项 | PX4 `px4_sitl_default` / RelWithDebInfo；Agent `build_ros` + 系统 FastDDS/FastCDR/Logger，Release |
| DDS 依赖 | ROS 侧 fastrtps `2.14.7` / fastcdr `2.2.8`（来自 `/opt/ros/jazzy`）；host spdlog `1:1.12.0+ds-2build1` / fmt `9.1.0+ds1-2`；sysroot spdlog `1.12.0` / fmt `9.1.0` |
| 离线包 | `localdeps/debs/` 21 个 `.deb`（Ubuntu 24.04 noble amd64）逐包 sha256 |

```yaml
# agc_toolchain_pins —— 工具链唯一入库的可重建 pin 清单；由 scripts/toolchain_pin_check.py 解析并与实测比对
agc_toolchain_pins:
  schema: 1
  git_repos:
    - id: px4
      path: toolchain/PX4-Autopilot
      remote: https://gitee.com/mirrors/PX4-Autopilot   # 本机使用的镜像；上游 https://github.com/PX4/PX4-Autopilot（本机直连 GitHub 极慢）
      tag: v1.15.4
      head: 99c40407ffd7ac184e2d7b4b293f36f10fe561ef
      submodules:           # 仅列 SITL(gz) 必需且已初始化的子模块
        Tools/simulation/gz: d754381a1cecdd7f17050acd72bf5bf1327bced6
        src/drivers/gps/devices: ad769ff53d683d06937c92bae58377d65893967b
        src/lib/crypto/libtomcrypt: 673f5ce29015a9bba3c96792920a10601b5b0718
        src/lib/crypto/libtommath: fd73d7630b9d3ed5a79d613ff680a549e9780de7
        src/lib/crypto/monocypher: baca5d31259c598540e4d1284bc8d8f793abf83a
        src/lib/events/libevents: 59f7f5c0ec2e76fadbc1dc40cc0705d614472edc
        src/lib/heatshrink/heatshrink: 052e6de72f67f1777198bce98f3de62f7f3c16a0
        src/modules/mavlink/mavlink: a3558d6b335d930fc01816fd168d16b3f38ed434
        src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client: 711aef423edd1820347b866d1e4164832df35d04
    - id: uxrce_agent
      path: toolchain/Micro-XRCE-DDS-Agent-2.4.3
      remote: https://github.com/eProsima/Micro-XRCE-DDS-Agent
      tag: v2.4.3
      head: 73622810d984349b80bbac0ef55fc0b694d62222
  artifacts:
    - path: toolchain/bin/MicroXRCEAgent
      sha256: 91d9b1a30431b43f55d14abe36bb7319d3394f10c141a41c0c7ee9717f489359
      note: 入口包装脚本（exec 真实二进制）
    - path: toolchain/Micro-XRCE-DDS-Agent-2.4.3/build_ros/MicroXRCEAgent
      sha256: adb91e47bd8c2b0db61c590fd9e5ea36e898535564bd3e88e7b62658255bd483
      note: agent 真实二进制（构建产物）
    - path: toolchain/PX4-Autopilot/build/px4_sitl_default/bin/px4
      sha256: 1c990470f5735282e85a59c342d20bfa34f566764f7fe1ac062ce9387c12dac2
      note: PX4 SITL 可执行文件（构建产物）
    - path: toolchain/microxrce_compat/fmt_compat.hpp
      sha256: 727f13ffb43bc2d93ccd80bf4ec74f263dfaa1123bd40dca76b2a0c31a8669bd
      note: AGC 自研兼容头（非上游文件，来源=本项目排障产出）
  build_options:
    px4:
      config: px4_sitl_default
      config_file: boards/px4/sitl/default.px4board
      cmake_build_type: RelWithDebInfo
      build_dir: build/px4_sitl_default
      compiler: /usr/bin/cc, /usr/bin/c++
    uxrce_agent:
      build_dir: build_ros
      cmake_prefix_path: /opt/ros/jazzy;toolchain/localdeps/sysroot/usr
      definitions: UAGENT_USE_SYSTEM_FASTDDS=ON, UAGENT_USE_SYSTEM_FASTCDR=ON, UAGENT_USE_SYSTEM_LOGGER=ON, UAGENT_P2P_PROFILE=OFF, UAGENT_BUILD_TESTS=OFF, CMAKE_BUILD_TYPE=Release
      forced_include: toolchain/microxrce_compat/fmt_compat.hpp
  dds_versions:
    ros_fastrtps: 2.14.7
    ros_fastcdr: 2.2.8
    host_spdlog: 1:1.12.0+ds-2build1
    host_fmt: 9.1.0+ds1-2
    sysroot_spdlog: 1.12.0
    sysroot_fmt: 9.1.0
  localdeps_debs:               # 逐包 sha256（Ubuntu 24.04 noble amd64）
    catch2_3.4.0-1build1_amd64.deb: c5ea160f1e349c8b58a422b231872d066387177aeb751c0e0465ca71a3031656
    gcc-14-base_14.2.0-4ubuntu2~24.04.1_amd64.deb: b95c172411a7fdae70307cf33a9f5320ba5e056b556454543dd5b679d5ce1c4f
    libasio-dev_1%3a1.28.1-0.2_all.deb: 658e0aa368003d2e7a89e228f652fe7a530de3883f7acd14ec7151470720e8e0
    libc6_2.39-0ubuntu8.9_amd64.deb: ff5557d99b51f761c4b7c92368b9cc45565eda17df9bf9eb4b134d09825008be
    libfastcdr-dev_1.1.0-1.2build1_amd64.deb: 68324de4a29f5e8d23f20977907e2859e80d1b3597ed41a0a963446a19bde42e
    libfastcdr1t64_1.1.0-1.2build1_amd64.deb: 40b53118491a16db5afaa333d9c90bd2f8fd67f2a671bb0f6d475a691ecb829f
    libfastrtps-dev_2.11.2+ds-6.1build3_amd64.deb: 5765d8512de8ff91c395e05edf7b2a1118fd6358e78f5e5620859b85212beb89
    libfastrtps2.11t64_2.11.2+ds-6.1build3_amd64.deb: 8f82688f1ff49e754ada4cd0fa983d21f2ec020439a15114c7cedb3cd4c74711
    libfmt-dev_9.1.0+ds1-2_amd64.deb: cc05cae4b7c6e541b6871e3333329605c0d90312a852f2c3b507ad5c30dd9914
    libfmt9_9.1.0+ds1-2_amd64.deb: 0a9dd337aea7ae59aba44994a92dd2780365b534a714518477c4e906f0efd297
    libfoonathan-memory-dev_0.7.3-2.1build1_amd64.deb: f91b898b7d52dad20cace526b155c406bcfc3168f49869850dca8ce2a2334668
    libfoonathan-memory0.7.3t64_0.7.3-2.1build1_amd64.deb: 3a05a16656d5c4d35f272fc4526cb7459bcfb3b0f8aa52c2a1f1d34e6a159888
    libgcc-s1_14.2.0-4ubuntu2~24.04.1_amd64.deb: aa7fadbe33b78bcf99885318040601c550c208929565b179891d9a3cc2aa68cd
    libspdlog-dev_1%3a1.12.0+ds-2build1_amd64.deb: 850b97a93e252c1d2f9d8e9f59cac919015efb56241138098637f84a188ae3a0
    libspdlog1.12_1%3a1.12.0+ds-2build1_amd64.deb: e69e315e1596b6cf97a714ae797e55d42765664a739a0b30beac35164a4edaff
    libsqlite3-0_3.45.1-1ubuntu2.8_amd64.deb: b1190bb72359f5fcc47406aa46065eaf4f1ca208085c51224a52b04bedc0b4bb
    libssl-dev_3.0.13-0ubuntu3.16_amd64.deb: 2b4734df0475e305ac03abb0a2cc5659651ed37ef02d967a1c4033a154cf714b
    libssl3t64_3.0.13-0ubuntu3.16_amd64.deb: 219f43b1cd836a4da550938db5fda93160d269d39a4edcf1f1ce698a470db797
    libstdc++6_14.2.0-4ubuntu2~24.04.1_amd64.deb: a51f8de7829211db961a31f02158058ad1a95f92ac6d0a5dff6350e2821c54c0
    libtinyxml2-10_10.0.0+dfsg-2_amd64.deb: 7444576728d0886cc58cee1749dc2ab5a680c1756f0a8b49c973fa0cfb180973
    libtinyxml2-dev_10.0.0+dfsg-2_amd64.deb: cbdab1d3cb31a306f485c4158cc9072763647eb75090d0a55b70778a927d9387
```

## 从空目录恢复（配方）

> **验证缺口（必须显式声明）**：本清单**本轮未在真正空目录端到端实跑**——不复制现有 `build/` 当重建证明。
> 因此「已有机器构建成功」**不**等于环境可从空目录独立重建。该验证缺口已在 `docs/adr/0065-*.md` 登记为未决事项。

1. **PX4 源码**：从 Gitee 镜像克隆指定 tag 并初始化所需子模块（pin 块 `git_repos[px4]` 锁定 HEAD 与 9 个子模块 SHA）：
   ```bash
   git clone -b v1.15.4 https://gitee.com/mirrors/PX4-Autopilot toolchain/PX4-Autopilot
   git -C toolchain/PX4-Autopilot submodule update --init --recursive \
     Tools/simulation/gz src/drivers/gps/devices \
     src/lib/crypto/libtomcrypt src/lib/crypto/libtommath src/lib/crypto/monocypher \
     src/lib/events/libevents src/lib/heatshrink/heatshrink \
     src/modules/mavlink/mavlink src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client
   ```
2. **PX4 SITL 构建**：
   ```bash
   make -C toolchain/PX4-Autopilot px4_sitl_default   # 等价 PX4_CONFIG=px4_sitl_default
   ```
   产出 `toolchain/PX4-Autopilot/build/px4_sitl_default/bin/px4`，用脚本核对 `sha256`（pin 块 `artifacts`）。
3. **uXRCE-DDS Agent v2.4.3**：
   ```bash
   git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent toolchain/Micro-XRCE-DDS-Agent-2.4.3
   source /opt/ros/jazzy/setup.bash
   # 按上文「uXRCE-DDS Agent 版本」节的 cmake 配方构建 -> build_ros/MicroXRCEAgent
   ```
   产出 `toolchain/Micro-XRCE-DDS-Agent-2.4.3/build_ros/MicroXRCEAgent`，核对 `sha256`（pin 块 `artifacts`）。
4. **入口包装脚本**：写入 `toolchain/bin/MicroXRCEAgent`（内容即 `exec <AGC>/toolchain/Micro-XRCE-DDS-Agent-2.4.3/build_ros/MicroXRCEAgent "$@"`），核对 `sha256`。
5. **离线依赖**：`localdeps/debs/` 的 21 个 `.deb` 来源与逐包 `sha256` 见 pin 块 `localdeps_debs`（安装后供 sysroot 使用）；`localdeps/sysroot/` 由这些包解包构成。
6. **运行期环境**：PX4 以 `cwd=build/px4_sitl_default`、`-s etc/init.d-posix/rcS` 启动；需设
   `PX4_GZ_STANDALONE=1`、`PX4_GZ_WORLD=agc_world`、`PX4_SIM_MODEL=gz_x500`、
   `GZ_SIM_RESOURCE_PATH=<px4>/Tools/simulation/gz/models:<px4>/Tools/simulation/gz/worlds`。
7. **校验**：`python3 scripts/toolchain_pin_check.py`。

## 离线包访问方式

- **本机直连 GitHub 极慢（~25 KB/s）**：外部仓库（PX4）走 **Gitee 镜像**；`Micro-XRCE-DDS-Agent` 走上游 Git（如需亦可换镜像）。
- **pip**：走**清华源**。
- **`localdeps/debs/`**：离线 `.deb`（Ubuntu 24.04 noble amd64），来源为 **Ubuntu 官方 archive / ports 对应版本**，逐包 `sha256` 见 pin 块 `localdeps_debs`。
- **ROS 侧 Fast DDS / fastcdr**：直接来自已安装的 `/opt/ros/jazzy`（`ros-jazzy-fastrtps` / `ros-jazzy-fastcdr`），**无需自建**。

## 校验

```bash
python3 scripts/toolchain_pin_check.py
```

比对 README 记录值 vs 实测（产物哈希、仓库 HEAD/tag、子模块、依赖版本、离线包哈希），不一致即报并**退出码 1**。
