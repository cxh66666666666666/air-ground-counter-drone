# failsafe 因子分离与运行时参数锁定（三因子 / 「停流 ≠ `NAV_DLL_ACT`」更正）

## Status

accepted

> 注（S-05 / [ADR-0061](./0061-w6-runtime-topology-backend-identity-and-single-parameterized-entry.md)）：W6 顶层入口已统一为 [`w6.launch.py`](../../src/agc_task/launch/w6.launch.py)（`backend` 参数 ∈ lightweight | full），原 `w6_task_eval.launch.py` / `w6_full.launch.py` 退役；本 ADR 历史结论正文不改，下文的旧入口名按现入口读作同一拓扑。

## 背景

评审风险 **R-019（failsafe 因果混淆）**：`jammer` 把「停发 Offboard」写成 `NAV_DLL_ACT` 等价，且
`LOST` 同时镜像强制 `RTL`（`src/agc_task/scripts/jammer.py` 旧 `JAM_MAP['LOST'] →
(VehicleStatus.NAVIGATION_STATE_AUTO_RTL, 'JAM_LOST')`）。这使「LOST 因子」的观测结果（RTL）无法
区分是**强制模式**、**native offboard-loss** 还是**data-link-loss**——多机制 / 参数 / 重入差异令
「等价」叙述不可复核，干扰因果结论失真。

同版本取证（[ADR-0012](./0012-px4-body-as-toolchain-dependency.md) pin PX4 v1.15.4，本机源码
`toolchain/PX4-Autopilot`）：

- **Offboard 丢失 ≠ 数据链丢失**：offboard-loss 由 `COM_OF_LOSS_T`（默认 `1.0 s`，
  `src/modules/commander/commander_params.c`）判定超时、`COM_OBL_RC_ACT`（默认 `0` = Position；
  enum `0 Position / 1 Altitude / 2 Manual / 3 Return / 4 Land / 5 Hold / 6 Terminate / 7 Disarm`）
  决定动作；**data-link-loss** 才是 `NAV_DLL_ACT`（默认 `0`）。二者为**不同机制**。
- `COM_OBL_RC_ACT = 0`（Position）映射到 nav_state `POSCTL = 2`（`px4_msgs/msg/VehicleStatus.msg`：
  `MANUAL=0, ALTCTL=1, POSCTL=2, AUTO_LOITER=4, AUTO_RTL=5, OFFBOARD=14, AUTO_LAND=18`）。

**运行时参数锁定机制（本机来源证据）**：`w6_full.launch.py` 以
`cwd=<toolchain>/PX4-Autopilot/build/px4_sitl_default`、`-s etc/init.d-posix/rcS` 起 PX4 实例；
rcS（`build/px4_sitl_default/etc/init.d-posix/rcS`）末段 `. px4-rc.params`，其 `PATH` 含
`${R}etc/init.d-posix` 且 `R=<pwd>/`（见 `build/px4_sitl_default/bin/px4-alias.sh`）→ 实际 source
`build/px4_sitl_default/etc/init.d-posix/px4-rc.params`；`param` 别名已由 `px4-alias.sh` 定义、rcS 既用
`param select/import/compare`，故 `param set NAME VALUE` 机制可用（`src/systemcmds/param/param.cpp`
提供 `set` 子命令）。把三参数写入该文件即可令**每次 SITL 启动**确定化。

契约约束（[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) /
[#params](../ROS2迁移交接文档.md#params) / [#execution-port](../ROS2迁移交接文档.md#execution-port) /
[#messages](../ROS2迁移交接文档.md#messages)）：不新增第二 ROS 命令者、不改冻结 9 消息 / 服务 ABI、
参数须经 `config/w6_base.yaml` **单一注入**；`/agc/task/jam_confirm` 取值须仍**恰为**
`JAM_RTL|JAM_LAND|JAM_HOLD|JAM_LOST`（`task_manager.cpp` 精确匹配，**不得**追加 factor）。

## 决策

### (a) 三类独立实验因子
`jammer` 以**因子标签**（`factor`）区分三类可独立观测 / 标注的干扰因子，修正「停流 ≡ `NAV_DLL_ACT`」：

1. **强制模式**（`forced_mode`）：`RTL / LAND / HOLD` 经 uXRCE 发 `px4_msgs/VehicleCommand`
   （`VEHICLE_CMD_SET_NAV_STATE`）强制 nav_state `5 / 18 / 4`。
2. **仅断 Offboard**（`offboard_only`）：`LOST` **只**停发 Offboard 流，**不**发 `SET_NAV_STATE`
   （`JAM_MAP` 的 `nav_state` 为 `None`）；随后行为由运行时锁定的 `COM_OF_LOSS_T` / `COM_OBL_RC_ACT`
   解释。其 R-018 确认期望 nav_state = 参数 `lost_expected_nav_state`（默认 `2` = POSCTL）=
   运行时锁定 `COM_OBL_RC_ACT` 的映射值。
3. **数据链失联**（`NAV_DLL_ACT`）：**本轮未实现**（见 (d)）。

因子标签随证据串落 `/agc/task/jam_evidence`：`JAM_SENT/JAM_ACK/JAM_CONFIRMED/JAM_FAIL_* <reason>
factor=<factor>`；`/agc/task/jam_confirm` 取值仍恰为 `JAM_*`（**不**追加 factor，保 `task_manager`
精确匹配）。

### (b) 「停流 ≠ `NAV_DLL_ACT`」事实更正
停发 Offboard 流触发的是 **offboard-loss**（`COM_OF_LOSS_T` + `COM_OBL_RC_ACT`），**不是**
data-link-loss（`NAV_DLL_ACT`）。`jammer` / `intruder_driver` 的 docstring 与日志据此更正（原
「停流触发起自身 failsafe 等价」「模拟链路被切断」「LOST 以 RTL 近似」等叙述作废）。**不**改契约
真源文字；本次更正记录于本 ADR。

### (c) 运行时参数锁定与记录
- **单一真源**：`src/agc_task/config/px4_failsafe_lock.yaml`（非 ROS 节点参数，顶层平铺），声明
  `com_of_loss_t` / `com_obl_rc_act` / `nav_dll_act`。
- **注入**：`scripts/lock_px4_failsafe_params.sh` 读该真源，生成 SITL 启动参数文件
  `build/px4_sitl_default/etc/init.d-posix/px4-rc.params`（`param set COM_OF_LOSS_T/COM_OBL_RC_ACT/
  NAV_DLL_ACT <v>`）；写前若 `.agc-orig` 不存在则备份一次原文件；SITL 目录不存在则告警并 `exit 0`
  （不阻断 launch）；末尾把 `NAME=VALUE` 三行打印到 stdout。**只写入 build 产物，不改 PX4 源码**。
- **接线**：`w6_full.launch.py` 的 `actions` **开头**插入 `ExecuteProcess` 调用该脚本，确保在 PX4
  （12 s / 25 s 定时器）**之前**执行。
- **记录**：run 证据（`r019.json`）写 `runtime_px4_params` + `runtime_px4_params_source`（lock 文件
  路径 + 注入机制），值取自**实际注入**的 `px4-rc.params`，**不以源码默认代填**。

### (d) 因子③未实现的如实限制
本轮**仅**实现因子 (1)(2)。**未**隔离 native data-link-loss：本项目未切断真正的数据链
（uXRCE / MAVLink），`NAV_DLL_ACT` 保持默认 `0`（Disabled），仅**如实标注**、**不**声称已隔离。
报告与 `r019.json` 于 `factor_3_data_link_loss = {implemented: false, note: ...}` 如实记录。

### (e) 参数 / 不改 ABI
`jammer` 新增参数 `lost_expected_nav_state`（经 `config/w6_base.yaml` 单一注入，默认 `2`；节点不覆写）。
新通道均为既有 `std_msgs/String`（证据 / 确认），**未**改冻结 9 消息 ABI，**未**新增第二 ROS 命令者，
**未**改唯一执行口语义。

## Considered Options

- **保持现状（`LOST` 镜像 `RTL` + 停流写成 `NAV_DLL_ACT` 等价）** —— 被否：即 R-019 缺口
  （因果混淆 / 不可复核）。
- **`LOST` 强制 `LAND` / `HOLD` 近似 offboard-loss** —— 被否：仍是强制模式，掩盖 offboard-loss
  与 data-link-loss 的机制差异。
- **运行期读取 PX4 参数（如 `param get`）代替启动锁定** —— 被否：读取依赖运行中的 PX4 实例与
  uXRCE 通道，且 `COM_OF_LOSS_T` 等在 offboard-loss 施加时刻已生效；启动注入 + 写入证据文件更确定。
- **以源码默认值代填运行值** —— 被否：违反「不以源码默认替代运行值」。
- **(a)–(e) 组合** —— **采纳**。

## Consequences

- 干扰因子可独立观测 / 标注：`RTL/LAND/HOLD`（强制模式，发 `SET_NAV_STATE`）与 `LOST`（仅断
  Offboard，**不**发命令）在**时序日志**上可区分（命令有无 + `factor` 标签），不再混淆。
- 「停流 ≡ `NAV_DLL_ACT`」事实表述已更正；offboard-loss 的因果归因 = 运行时锁定参数
  `COM_OF_LOSS_T` / `COM_OBL_RC_ACT`。
- 因子③（data-link-loss）**未实现**，如实限制标注，供后续 workstream 单列。
- **风险标注 / 置信度**：确认闭环以**假 PX4 回读状态流**与**注入的 SITL 启动参数文件**验证
  （见 [Evidence](#evidence)），未含真实 PX4 Offboard 动力学与真实数据链；真实 SITL 整链对拍留待后续
  （与本仓既有 harness 同口径）。
- 只读 `toolchain/PX4-Autopilot` 源码未改；仅运行时写入其 build 产物 `px4-rc.params`（备份 `.agc-orig`）。

## Evidence

命令化验收 `scripts/run_w6_r019.sh headless 20261016-w6-r019-0`（自包含 harness，无 gz / launch），
结果 `experiments/w6-task-eval/20261016-w6-r019-0/metrics/r019.json`（`overall_pass: true`）：

| 用例 | 注入 | 观测 |
|---|---|---|
| a `LOST` 仅断 Offboard | 合法 + 包络新鲜 | 授权 `JAM_LOST`；**无**任何 `SET_NAV_STATE`（`command==100001`）；证据含 `factor=offboard_only`；断流经 `COM_OF_LOSS_T` 后进入锁定映射模式（POSCTL=2）→ 确认 `JAM_LOST` |
| b `RTL` 强制模式 | 合法 + 包络新鲜 | 发 `SET_NAV_STATE`（`param1==5`）；授权 / 确认 `JAM_RTL`；证据含 `factor=forced_mode`；记录命令到达相对授权事件时序（与 a 的「无命令」可区分） |
| c 运行时参数 | 调 `lock_px4_failsafe_params.sh` | `px4-rc.params` 注入三参数，值 == `config/px4_failsafe_lock.yaml` 声明值；`jammer.lost_expected_nav_state` == `COM_OBL_RC_ACT` 映射（跨源一致） |
| d 因子③限制 | 读 `docs/adr/0034-*.md` | ADR 存在且含 `NAV_DLL_ACT` 与「未实现 / 限制」表述；`r019.json` 顶层 `factor_3_data_link_loss.implemented = false` |

## 修订（W7-b 步 1，2026-10-19）：仿真唯一时间源 `UXRCE_DDS_SYNCT=0` + PX4→ROS 时间映射加固

**背景（实证）**：W7-b「整链评价可用」取证发现 `air_perception` 崩溃 `cannot store a negative
time point in rclcpp::Time`（exit -6，连续两次复现）。**一手依据**（rclcpp `jazzy` `time.cpp`）：
`Time(const builtin_interfaces::msg::Time &)` 在 `sec < 0` 时**构造即抛**，而 `Time(int64_t ns)`
不设防——故**任何节点**只要构造出负的 `rclcpp::Time` 并写入消息，**所有消费该消息的节点都会崩**。
`PX4→ROS` 时间映射即该生产者：`mapper_.toRosNanoseconds(px4_us) = (px4_us + offset_us)·1000`，
而 `offset_us`（`ros_us − px4_us` 的 EMA）在**时间源尺度跳变**后失真。

实测脏值（内省探针，`/agc/air/odom`）：`stamp[min = −1702794509.698, max = +85139797.395, neg* = 1]`。
数值可解释：若 uXRCE-DDS 桥时间同步把 PX4 时间戳推到**墙上时钟尺度**（µs ≈ 1.79e15）而 ROS 侧是
**仿真时钟**（≈50 s），则 `offset ≈ −1.79e15 µs`；再映射仿真尺度的 `px4_us` 得 `≈ −1.79e18 ns`
→ `sec ≈ −1.79e9`，与实测 −1.7028e9 同量级同符号；反向即 `+8.5e7 s`。

**决策**：

1. **仿真以 `/clock` 为唯一时间源**：把 `UXRCE_DDS_SYNCT: 0` 纳入同一**单一真源**
   [`src/agc_task/config/px4_failsafe_lock.yaml`](../../src/agc_task/config/px4_failsafe_lock.yaml)，
   由同一 [`lock_px4_failsafe_params.sh`](../../scripts/lock_px4_failsafe_params.sh) 注入 SITL 启动参数
   文件（`param set UXRCE_DDS_SYNCT 0`）。依据 PX4 官方文档：该桥时间同步「**在仿真中可/应禁用**」，
   配合各 ROS 节点 `use_sim_time=true` 使用 `/clock`。
2. **生产者侧加固（代码，与上条成对）**：
   - [`px4_conversions.hpp`](../../src/agc_air/include/agc_air/px4_conversions.hpp) 的 `Px4TimeMapper`
     新增 `reset()`，并在 `observe()` 中**偏移跳变 > 0.1 s 即重锚**（不再 EMA 混合，避免旧估计把新
     尺度"稀释"成离奇时间戳）；
   - [`px4_adapter.cpp`](../../src/agc_air/src/px4_adapter.cpp) 对映射结果做**合理性校验**：非负且与
     `now()` 偏差 ≤ `stamp_max_skew_s`（单一真源 [`air.yaml`](../../src/agc_air/config/air.yaml)，默认 5.0 s）；
     **超界即拒发该帧 + 一次性如实告警 + 重锚**（**不**静默替换时间戳，R-004 源采样时刻语义不许伪装）；
     **EKF `reset_counter` 变化**亦触发重锚（局部原点与时间基准同源跳变）。
3. **消费端不替代生产者**：[`air_perception.cpp`](../../src/agc_perception/src/air_perception.cpp) 对入站
   `Header.stamp` 增加 `sec < 0` 前置守卫（拒收 + 一次告警）作为**纵深防御**——它只防崩溃，
   **不是**根因修复；根因修复是上面的 1 / 2。

**证据**（本仓 run，`/tmp` 内省探针复跑，full 整链 `s08`）：`UXRCE_DDS_SYNCT: curr: 1 -> new: 0`
（PX4 自证生效）；修后 `/agc/air/odom` `neg* = 0`、stamp 全落在仿真区间（43.5→107 s）；
运行期 `negative time point` 崩溃 **0**、`air_perception` 守卫拒收 **0**、`px4_adapter` 映射告警 **0**；
`px4_adapter` 如实记录了 `EKF 复位计数 0→3→5→6→9→10` 并逐次重锚。

**未决（如实登记）**：修后**观测链仍间歇**——内省显示 `early` 窗口 `/agc/air/obs` 目标条数
`max = 1`、`/agc/targets` `len = 1`、阶段推进到 `GROUND_APPROACH`，而 `late` 窗口两者**全为 0**
（`standoff_follower` 记 `目标过期/置信度不足 → 冻结跟踪`）。几何上己方机悬停于 `(≈0, ≈0, 3.3 m)`、
目标 `(4.6, 3.0, 2.1–4.7 m)`、斜距 ≈5.4 m 在量程 `[0.30, 12.0]` 内，故**非量程问题**；疑似
**机体固连前视**（`boresight_body = +X`、`hfov ±50°` / `vfov ±35°`）在己方机偏航变化时**丢目标**
——正是 [F-10](../../docs/reviews/agc-review-2026-10-14/30-未来.md#f-10)「可见性协同」与 ADR-0070 P1
命题所指现象。**该现象本轮不修**，留作后续（属研究问题，不是"修复"）。

