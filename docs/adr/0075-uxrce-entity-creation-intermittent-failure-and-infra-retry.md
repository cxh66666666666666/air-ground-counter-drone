# 整链 uXRCE 实体创建**间歇**失败（PX4 1 s 硬预算）与 runner 级 infra 重试协议

## Status

accepted

## Date

2026-10-20

## 背景

W7-b 步 2（`backend:=full` 整链、P1 两臂配对）被同一类 infra 挡住：own PX4 反复

`task_manager: 等待起飞完成超时（45.0s 内未见爬升达 2.30 m：air_odom=0 z=0.00 base=inf）` → `start rejected: air_hover_timeout` → `INFRA_FAILURE`；PX4 侧 grep 到 `ERROR [uxrce_dds_client] create entities failed: participant: 255`，此后**无任何** `created rt/fmu/out/... data writer`。

取证 run：`w7-rl-b/20261020-w7-rl-b-140610-1087`（`full` / `s08` / seed 0 / `p1_coop`，跑前已清 `/dev/shm/fastrtps_*`、无残留进程、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、`ROS_DOMAIN_ID=0`）。

## 决策

### (a) 根因：agent 侧 participant 创建**间歇**超时 + PX4 客户端**硬编码 1 s** 预算

- **PX4 侧**（[`uxrce_dds_client.cpp`](../../toolchain/PX4-Autopilot/src/modules/uxrce_dds_client/uxrce_dds_client.cpp)）：`uxr_run_session_until_all_status(&session, 1000, &participant_req, ...)` —— 等 participant 创建的预算**固定 1000 ms**；失败即 `PX4_ERR("create entities failed: participant: %i")` 并 `return`（**不建 pubs、不重试**）→ 无 `rt/fmu/out/*` writer → 无 `/agc/air/odom` → 不 arm → `air_hover_timeout`。
- **agent 侧**：同一 agent 进程内 `establish_session → participant created` 的耗时**间歇**：良好 0.67–0.73 s，异常 1.69–2.72 s（> 1 s 预算即必然失败）。

| run | 客户端 1（own PX4） | 客户端 2（入侵者 PX4） | own 结果 |
|---|---|---|---|
| `w7-rl-b/20261020-w7-rl-b-093626-2907` | 0.67 s | 0.67 s | 成功（该 run 整链可用） |
| `w7-rl-b/20261020-w7-rl-b-115346-1825` | 0.73 s | 0.72 s | 成功 |
| `w7-rl-b/20261020-w7-rl-b-124006-2585` | **1.80 s** | 0.79 s | `create entities failed` |
| `w7-rl-b/20261020-w7-rl-b-134048-5265` | **2.72 s** | 0.83 s | `create entities failed` |
| `w7-rl-b/20261020-w7-rl-b-140610-1087` | **1.69 s** | 0.84 s | `create entities failed` |

### (b) 已排除的假设（每条有实测反例）

| 假设 | 反例 |
|---|---|
| "第一个连接 agent 的客户端必中招" | 093626 / 115346 的**第一**客户端 0.67 / 0.73 s 成功 |
| DDS 发现范围（SUBNET + 本机 `eth3`） | 本轮 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 仍慢（1.69 s） |
| `/dev/shm` 残留 `fastrtps_*` 段 | 跑前已清（152 → 0），仍慢 |
| agent 端口单点拥塞 | 已按实例分 agent（own 8888 / 入侵者 8889），仍慢 |
| 游离 ROS 节点占图 | 跑前确认 0 |

**性质**：本机 Fast DDS / agent 侧的**间歇时延尖峰**（本轮取样 3/5 命中），**不是**确定性配置错误；因此不能靠"改一个配置"消除。

### (c) 处置：runner 级 **infra 重试协议**（机器噪声处理，非实验处理）

- **判据唯一来源** = [`agc_rl.termination.classify_episode_outcome`](../../src/agc_rl/agc_rl/termination.py) 的 `classification`：
  - `classification == 'infra'`（`INFRA_FAILURE` / 未达终态的墙钟截断）→ 按上限**重跑**该 (arm, seed)；
  - `classification == 'mission'`（含 `MISSION_FAILED(TIMEOUT)` —— 契约 [#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) 的 C4 超时属**任务终态**）→ **一律不重试**（不得以重试掩盖 mission 结果）。
- **实现**：[`episode_runner.run_episode_with_infra_retry`](../../scripts/episode_runner.py)（`run_episode` 的包装，`infra_retries=0` = 关闭、既有行为不变）；[`w7b_arms_runner.py`](../../scripts/w7b_arms_runner.py) 新增 `--infra-retries`（缺省 3）。
- **证据纪律**：每次尝试**各留产物**——首试沿用原 `log_tag`，第 k 次重试追加 `.r<k>` 后缀（日志不覆盖），逐次 `run_id` / 终态 / 原因记入 `infra_attempts`（落 `metrics/arms.json`）；上报结果 = **最后一次尝试**。
- **公平性**：两臂施加**同一**协议（同 K、同判据、同触发条件）→ 不构成自变量差异。

### (d) 与预注册的关系（**不**静默改判据）

[`w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml) 的 `keep_all_failures_and_infra` **不变**：

- 重试耗尽的 infra **仍**计入分母（其 primary DV 记**失败**），**不**以成功替补；
- 阈值 / 样本量 / 自变量（`observe_yaw_mode`、`target_source`、`air_tracking_ref`）**均未改动**；
- infra 率与重试次数须在报告中**如实报出**（本机特性，**不得**被读作臂效应）。

## 备选

1. **PX4 本地补丁把 1000 ms 放宽**（如 5 s）—— 根治、近 100% 成功，但改动命中"PX4-Autopilot 精确 pin v1.15.4 / 子模块 SHA"红线 → 需 patch 文件 + 构建脚本应用 + 独立 ADR 裁量 + 重建 PX4；**留作后续**（见后果·未决）。
2. **watchdog 重启 own PX4 进程** —— 省时，但 PX4 被 SIGKILL 后 gz 模型失去控制会失稳 / 坠落，重挂很可能接上已失稳模型，并牵连 arm / 起飞时序 → 否决（本段）。
3. **环境侧试探**（限制 agent 侧发现 / 错峰启动 own PX4） —— 成本低，但现有证据（(b) 表）不支持其有效，且会引入新变量 → 不作首要手段。
4. **重试至成功为止、或把 infra 记为成功** —— 否决：改写任务终态语义，违反契约 C4 与 [ADR-0046](0046-runner-three-layer-acceptance-and-integrity-gate.md)（infra 单列、不吞入成功率）。

## 后果

- **正面**：间歇 infra 由"整轮作废 / 需人工重跑"降级为**自动恢复**，且逐次证据可审计；P1 配对 run 得以开展。
- **代价 / 限制**：单 episode 墙钟最坏 ×(K+1)（缺省 K=3）；报出的成功率含"重试后成功"，故 **infra 率与重试分布必须随报告一并报出**；本 ADR **不**消除根因。
- **未决 / 后续**：① 根治路线（PX4 客户端预算放宽）另行裁定与留痕；② ~~判定脚本剔除 infra 配对~~ → **已修（2026-10-20）**：[`w7b_p1_adjudicate.py`](../../scripts/w7b_p1_adjudicate.py) 现按预注册把**失败 / infra / 缺值**的 primary DV 记 **0 并计入分母**（`integrity.…primary_honored` 自检），并在既有数据集上复验（20/20 计入、剔除 0 对）；③ 判定结论见[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(h)（其语义随后被 [ADR-0076](0076-capture-scenario-tracking-milestone-not-terminal.md) 取代，待重跑）。

### (i) 风险与限制（**显式登记**）

1. **残留孤儿的来源尚未定性**：已确证"有孤儿 → 时延恶化 → infra"与"清场 → 恢复"，但**未**能用实验区分「外部终止（断网 / 工具宿主只杀 leader）」与「其它异常退出路径」。当前由**运行前清场**兜底；正常收尾的 run 实测 0 孤儿（故组回收对正常路径有效）。
2. **清场与并行 worker 冲突（W7-c 前置风险）**：`sweep_stale_processes` 按**本项目特征**匹配并 SIGKILL，**不区分进程属于哪个 worker 实例** → 一旦 W7-c 同机并行跑多个 worker，后启动的 worker 会杀掉先启动者的进程。**W7-c 必须先落地隔离手段**（独立 `ROS_DOMAIN_ID` 前缀 + 进程命名/标记 + 或每 worker 独占容器/网络命名空间），再启用清场；在此之前**只允许单 worker 串行**。
3. **清场不覆盖 `/dev/shm`**：段清理仍为**人工 ops 前置**（(h)），避免误删机器上其它 DDS 系统的段。

## 更新（2026-10-20 晚）—— 方案 B **实测证伪**；真根因＝**残留孤儿进程**；运行前清场落地

### (e) 方案 B 实测证伪（**污染环境下**）

同一条命令连试 4 次（`--infra-retries 3`，run `w7-rl-b/20261020-w7-rl-b-143239-0508`）→ **4/4 全失败**，且每次都是同一处：own 客户端 participant 创建 **1.47 / 1.79 / 2.14 / 2.53 s**（全部 > 1 s 预算），入侵者恒 0.83 s。⇒ **重试对"持续偏高的时延"无效**（B 解决的是**间歇**尖峰，不是持续污染）。

### (f) 真根因：**上次 run 逃逸的孤儿进程**（占 DDS + `/dev/shm`）

排查发现 **6 个孤儿进程**（`ros_gz_bridge/parameter_bridge`×2、`set_pose_bridge`、`tf2_ros/static_transform_publisher`×3），**存活 2.5–3.3 h**，`ppid` 已变为子收割者：

- 它们是 `ros2 launch` 的子进程，但本体在 **`/opt/ros/jazzy/lib/...`**、命令行**不含** `install/agc_` 字样 → **早先的清场模式（只匹配 `install/agc_*`）漏掉了它们**；
- 来源：上一批 run 被**外部终止**（网络中断 / 工具宿主只杀到 leader）→ 子进程逃逸为孤儿（正常收尾的 run 实测 **0 孤儿**，故 `_reap_process_group` 的组回收对正常路径有效）。

**一次性验证（同一命令、同一参数）**：清掉 6 个孤儿 + 清 `/dev/shm/fastrtps_*` 后单跑 → own 客户端 **0.672 s**（< 1 s）→ `created rt/fmu/out/vehicle_odometry data writer` → **`TRACKING_SUCCESS`**、`vis_rate=0.9726`、`fco_rate=0.1642`、`infra_retries_used=0`。⇒ 根因在**环境残留**，**不在** PX4 的 1 s 预算（故**不动** PX4 源码 / 不触 pin 红线）。

### (g) 落地：**运行前清场**（`sweep_stale_processes`）

`episode_runner` 在 `Popen` **之前**调用（本 run 进程尚未创建 → 命中者必为上次残留）：

- 匹配口径 = **`argv[0]` / `/proc/<pid>/exe`**，**不**扫整条命令行子串（诱饵测试实测：扫子串会把"命令行里提到这些字符串"的调试父 shell 误杀）；守卫（`agc_world` / `agc_car` / `agc_world.sdf`）取自**参数**；`gz sim` 因 `argv[0]` 为 `ruby` 按命令行匹配但**排除 shell**。
- 证据入产物：`stale_swept_pids` / `stale_swept_cmds`（结果体内）+ 该次 run 日志追加"清场"行（审计）。
- **诱饵/负例测试**（2026-10-20）：`install/agc_*` 诱饵与"桥 + `agc_world`"诱饵**被清**；无 `agc` 守卫的桥、非 AGC 进程、以及**仅命令行提及字符串的 shell** 均**存活**。
- **方案 B 的 `run_episode_with_infra_retry` 保留**作二级保险（K=3；低负载下的偶发尖峰仍可能 < 5 s 内恢复），但**不再**作为主要手段。

### (h) ops 前置（**不**纳入 runner 自动步骤）

- 批跑前：确认**无 ROS 进程**后清 `/dev/shm/fastrtps_*`（本机已知段累积效应：单 run 约余 100 段，累积数百段曾致 SHM 端口初始化失败）。
  **不**由 runner 自动清：清场只保证本项目的进程已退出，**无法**保证机器上不存在其它 DDS 系统 → 自动删段有误伤风险。

## 参考

- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)（C4 超时属任务终态 / 必报指标）、[#execution-port](../ROS2迁移交接文档.md#execution-port)、[#params](../ROS2迁移交接文档.md#params)
- 前序 ADR：[ADR-0005](0005-px4-link-uxrce-dds.md)、[ADR-0007](0007-experiment-artifact-layout.md)、[ADR-0046](0046-runner-three-layer-acceptance-and-integrity-gate.md)、[ADR-0069](0069-w7-entry-and-exit-definition.md)、[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)、[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)、[ADR-0074](0074-visibility-metric-fixed-observation-window.md)
- 预注册：[`src/agc_rl/config/w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)
- 规格：[`.trae/specs/w7-rl-b/`](../../.trae/specs/w7-rl-b/spec.md)
