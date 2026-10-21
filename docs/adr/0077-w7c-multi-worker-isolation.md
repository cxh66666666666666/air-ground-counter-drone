# W7-c 多 worker 隔离设计（**草案 / 未实施**）

## Status

proposed（草稿；**未**实施、**未**准入执行）

## Date

2026-10-20

## 背景

W7-c 的目标是把 W7-a/b 的**单 worker 串行**评估扩为**同机多 worker 并行**（[ADR-0069](0069-w7-entry-and-exit-definition.md)「单 worker 可重放」→ 多 worker 隔离）。

现状约束（均为**实测**结论，不是推测）：

1. **同机两个整链实例会互相污染**：`/dev/shm/fastrtps_*` 段、DDS 发现域、PX4 实例命名空间、uXRCE agent 端口、gz 世界名、实验产物目录**全部共享**。
2. **`/dev/shm` 段会累积**（单 run 约余 100 段）；累积数百段即出现 SHM 端口初始化失败（[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)）。
3. **uXRCE 客户端实体创建预算仅 1 s（PX4 硬编码）**：任何额外负载/拥塞都会直接变 infra；多实例共用一个 agent 更易超时（已改为按实例分端口）。
4. **本机 `ROS_DOMAIN_ID` 非 0 时 lifecycle 服务数据面不通**（实测 77/81：`ChangeState` 一律超时，话题仍正常）→ **不能**用"每 worker 一个 domain id"作为隔离手段（除非先解决该缺陷）。
5. **`sweep_stale_processes`（[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)(i)）不区分 worker 归属**，并行时后启动者会杀掉先启动者的进程。

## 决策（草案，待评审后转 accepted）

### (a) 隔离单元 = **网络命名空间**（每 worker 一个），而非 ROS domain

理由：约束 4 封死了"按 domain 隔离"的常规做法；而**独立 netns 下每 worker 拥有独立 loopback**，于是：

- 所有 worker 可**统一使用 `ROS_DOMAIN_ID=0`** → 规避缺陷 4；
- DDS 发现（`LOCALHOST`）天然不出 netns → 无跨 worker 串扰；
- PX4 MAVLink / agent UDP 端口（8888/8889/14550…）在各 netns 内**互不冲突**，无需改端口分配；
- `/dev/shm` 仍为**全局共享**（POSIX shm 不随 netns 隔离）→ 见 (c)。

候选实现（按落地成本排序，择一）：
1. **容器**（每 worker 一容器，`--network=none` 或默认 bridge + 独立 IPC）——最干净，但需镜像/挂载规划；
2. `unshare -n` + 显式挂载/回环配置 —— 轻量，但需要 root 或 user-ns 支持，且要与 `ros2 launch` 进程组回收配合；
3. 若 1/2 都不成立 → **退回单 worker**（明确不做 W7-c 并行）。

### (b) 每 worker 独占的资源

| 资源 | 方案 |
|---|---|
| ROS 域 | 统一 `0`（依赖 netns 隔离） |
| uXRCE agent 端口 | 沿用"按 PX4 实例分端口"（own 8888 / 入侵者 8889，[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)）；netns 内不冲突 |
| PX4 实例 | `px4 -i N` → 命名空间 `px4_N`（各 worker 内部编号可相同，因 netns 隔离） |
| gz 世界 | 每 worker **独立 gz 进程**（世界名可同名）；不建议共享一个 gz 多模型（会破坏"同世界同时钟"的 episode 语义） |
| 产物目录 | `experiments/<campaign>/<run_id>/worker_<id>/`；或 campaign 名带 worker 后缀——**须单一约定**，避免混批 |
| 进程组 | 每 episode 一个 session（已有 `start_new_session=True`）；**worker 级**再加一层可识别标记（环境变量 / 参数前缀）供清场按归属过滤 |

### (c) `/dev/shm` 与清场

- **段清理**：仍为 worker 启动前的**人工 / 编排层**动作；**禁止**由 worker 自动 `rm -f /dev/shm/fastrtps_*`（会误删同机其它 worker 的段）。
- **清场**：`sweep_stale_processes` 必须增加**归属判据**（例如进程环境变量 `AGC_WORKER_ID`，或 `ROS_DOMAIN_ID`/netns inode），**只清本 worker**；否则禁止启用并行。

### (d) 准入判据（并行前必须全部满足）

1. 单 worker 串行跑出**可复现**基线（同一 seed 两次 run 的终态与 DV 一致）；
2. 两 worker 并行跑**同一 (scene, seed)** 各一次 → 终态一致、DV 差异在**预先声明**的容差内、`infra` 不高于单 worker 基线；
3. 并行跑完 `ps` 无跨 worker 残留、`/dev/shm` 段增长与 episode 数线性（可解释）；
4. RTF 未因并行劣化到影响 `--obs-horizon-s` 采满（否则窗口采不满 → DV 失真）。

### (e) 明确**不**做的事

- **不**改实验语义：每 worker 仍跑完整的 `w6.launch.py backend:=full`（不做"共享一个仿真、多策略"的省资源变体）；
- **不**用"多线程跑多个 episode 共享一个 gz"——违反"同世界同时钟 + episode 独立复位"的前提；
- **不**在 W7-c 内改判定阈值 / 样本量（预注册纪律不变）。

## 备选

1. **每 worker 一个 `ROS_DOMAIN_ID`** —— 否决（本机实测缺陷 4：非 0 域 lifecycle 服务面不通）；若将来修好该缺陷，本方案可复活且更轻量。
2. **共享一个 gz 世界 + 多套感知/节点** —— 否决：破坏 episode 隔离与复位语义。
3. **串行但流水线化（预启动下一个 gz）** —— 保留备选：收益小、复杂度不低；若并行隔离成本过高可退此方案。

## 后果（若采纳）

- **正面**：获得真正的并行扩容；隔离边界清晰（netns），不依赖脆弱的 domain 切换。
- **代价 / 限制**：需要容器或 netns 的运维能力；`/dev/shm` 与清场归属仍需显式处理；单机 CPU/内存成为首要瓶颈（每 worker = 1 gz + 2 PX4 + 全链路节点）。
- **未决**：① netns 方案（容器 vs `unshare`）的落地选型；② 清场归属标记的具体字段；③ 并行容差与准入判据的数值（须**先于**并行 run 写定）；④ 产物目录命名约定；⑤ 是否需要在 `w6.launch.py` 增加 `worker_id` 参数（单一注入）。

## 参考

- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)、[#params](../ROS2迁移交接文档.md#params)、[#namespace](../ROS2迁移交接文档.md#namespace)、[#execution-port](../ROS2迁移交接文档.md#execution-port)
- 前序 ADR：[ADR-0005](0005-px4-link-uxrce-dds.md)、[ADR-0007](0007-experiment-artifacts-convention.md)、[ADR-0069](0069-w7-entry-and-exit-definition.md)、[ADR-0075](0075-uxrce-entity-creation-intermittent-failure-and-infra-retry.md)
- 运维速查：[`docs/conventions/整链排障速查.md`](../conventions/整链排障速查.md)
