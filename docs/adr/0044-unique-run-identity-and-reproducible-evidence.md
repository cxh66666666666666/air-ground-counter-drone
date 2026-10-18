# 唯一运行身份与可复现证据（R-029）

## Status

accepted

## Date

2026-10-16

## 背景

评审风险 **R-029（运行身份与复现）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-029)）指出：
[`run_logger`](../../src/agc_task/src/run_logger.cpp) 的 `run_id = <YYYYMMDD>-<scenario>-<seed>`
在**同日同场景同 seed** 下落到**固定路径**，且以 `ofstream` **重写**已有产物；config 只保存 logger
自身几项参数与 `git HEAD`，**未存**所有节点展开参数 / **dirty diff** / 未跟踪源码；当前工作区存在大量
**未提交实现**；[`episode_runner`](../../scripts/episode_runner.py) **不验证** summary 创建时间或
**同一 run 关联**，并以固定 `time.sleep(18)` 当作就绪。后果：重复失败可**覆盖成功**或反之；旧 summary
可被新回合引用；历史验证**无法绑定运行代码**，科研复现与问题归因均不可靠。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C9**（必报指标）、
[#params](../ROS2迁移交接文档.md#params)（单一注入，节点不得覆写）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一执行口语义不变，
`/agc/ground/cmd_vel` 仍是唯一执行口）、[#messages](../ROS2迁移交接文档.md#messages)（9 消息 ABI
不得变更）；产物纪律 [ADR-0007](0007-experiment-artifacts-convention.md)（唯一落点
`experiments/<campaign>/<run_id>/`）。R-028 已引入 episode 窗口统计与有效样本掩码，见
[ADR-0043](0043-episode-window-metrics-and-valid-sample-mask.md)。

分类：**设计风险与平台级复现问题**。本 ADR 只对**运行身份与证据保全**做决定，**不**回退任何已验收组件，
**不**改契约冻结 9 消息 ABI，**不**新增第二 ROS 命令者。

## 决策

### (a) run_id 唯一不可覆盖（保留 `-<scenario>-<seed>` 子串）

`run_id` 生成改为 `<YYYYMMDD>-<scenario>-<seed>-<HHMMSS>-<rand4>`（时刻 + 随机后缀），并**保留**
`-<scenario>-<seed>` 子串以兼容既有 `*-<scenario>-<seed>-...` glob 定位。run 目录以
`fs::create_directory` **独占创建**（已存在即失败）：先建 campaign 父目录，再对 run 目录独占创建；
失败则置 `dir_ok_=false` 并报错。主程序 `main()` 在 `dirOk()==false` 时**退出码 2**，**不重写**已有
`summary.json`。这样「同日同场景同 seed 连跑两次」落到**两个不同** run 目录，前次产物仍可复核。

### (b) `episode_id` 贯穿日志 / config / summary

`run_logger` 订 `/agc/task/episode`（`std_msgs/Int32`，`transient_local`；由 `task_manager` 在
start 成功后发布，**非命令者，诊断类**）。收到后：日志打印、`config/episode_id.txt` 覆写为实际值、
summary 增 `episode_id` 键。初值 `config/episode_id.txt` = `-1`；summary 在未收到时记 `null`
（**不**以 0 冒充）。该字段使一次 run 的**episode 身份**可核对。

### (c) 保存完整展开参数 + 代码身份 + 工具链与世界模型哈希

- **展开参数**：`params_applied.yaml`（由 launch **单一注入**的合并后参数文件，含全部节点
  `ros__parameters`）拷入 `config/`；另拷 `scene_manifest.json`。
- **代码身份**：`git_head.txt` / `git_status_porcelain.txt` / `git_diff.patch` /
  `git_untracked.txt`。`workspace_fingerprint.json` 含
  `fingerprint_sha256 = sha256(git HEAD + status --porcelain + diff)`、`diff_sha256`、
  `status_entries`、`dirty`。**dirty 工作区非禁跑**，但 run 可据此重建（能回答「用的是哪份代码 /
  参数 / 世界」）。
- **工具链与世界模型哈希**：`toolchain_world.json` 含 `ROS_DISTRO`、`gz sim --versions`、code root、
  `git_head`、可选 PX4 `px4_root` 及其 `px4_head`、世界模型文件路径与 `world_sha256`
  （空路径 / 文件缺失记 `null`）。

新增均为 run **config 产物**（只落 `experiments/<campaign>/<run_id>/`，ADR-0007）与**诊断订阅**，
**不改**物理 / 驱动，**非** ROS 命令者。

### (d) runner 以唯一 run_id / 创建时间校验同一 run 关联，去掉固定 sleep 就绪依赖

`episode_runner` 收集 summary 改为 glob `*-<scenario>-<seed>*/metrics/summary.json`，并以
`mtime >= 本 episode 墙钟起点 - 1.0` 过滤，取最新的候选；再以 `run_id` 含
`-<scenario>-<seed>-` 记 `same_run_verified`（**同一 run 关联**）。就绪改为**有界轮询**
`wait_ready()`（以 `/agc/task/start` 服务就绪为门槛）替代固定 `time.sleep(18)`，随后仅有界稳定窗口
收集 `/clock` 起点。

### (e) `run_id_override` 仅作确定性验证 / 审计钩子

新增 `run_id_override` 参数**只替换 run 目录**名**（供 Phase B 复现「已有目录 → 拒写」）；**独占创建
仍拒绝已有目录**，因此**不削弱**「唯一不可覆盖」。默认空 = 走 `makeRunId()` 生成唯一 id。

### (f) 验收脚本与产物

新增 [`run_w6_r029.sh`](../../scripts/archive/run_w6_r029.sh) + [`w6_r029_check.py`](../../tests/w6_r029_check.py)：
Phase A 唯一不覆盖（同场景同 seed 连跑两次 → 两个不同 run 目录）；Phase B 拒写已有目录（预建目录 +
哨兵 → rc=2 且哨兵**逐字节不变**）；Phase C `episode_id` 贯穿；Phase D 身份齐备；Phase E runner 同一
run 关联；Phase F 核查 (a)…(f)。产物落 `experiments/w6-task-eval/<run_id>/{dup,reject,episode,identity,runner,metrics/r029.json,log}`。

## 备选

1. **继续用固定路径 `<YYYYMMDD>-<scenario>-<seed>`**：最简、glob 稳定；但会**覆盖**前次产物
   （评审病灶）——否决（改为唯一 id + 独占创建，同时保留 `-<scenario>-<seed>` 子串兼容 glob）。
2. **不存 diff（只存 `git HEAD`）**：省空间、避免大 patch；但 dirty 工作区下**无法绑定运行代码**
   （HEAD 不标识未提交实现）——否决（存 `git_diff.patch` + 内容指纹，dirty 非禁跑但可重建）。
3. **无 `run_id_override`，Phase B 用真实唯一 id 撞目录**：免新增参数；但无法**确定性**复现「已有目录 →
   拒写」，验证不可复制——否决（保留 override 钩子，独占创建语义不变）。
4. **runner 继续固定 `sleep` 当就绪 + 精确路径猜 summary**：简单；但**固定等待**既非就绪协议又易误取旧
   summary（缺数据假绿）——否决（有界轮询就绪 + glob 时间窗口关联）。

## 后果

- **正面**：同日同场景同 seed **不再覆盖**；已有目录**拒写并退出 2**；`episode_id` 与展开参数 / 代码身份 /
  工具链与世界哈希齐备，dirty 亦可重建；runner 以唯一 run_id 与创建时间**校验同一 run 关联**，
  去掉固定 `sleep` 就绪依赖。Phase A…F 实跑对照。
- **代价 / 限制**：`git_diff.patch` 与 `git_status_porcelain.txt` 在工作区**大量未提交**时**可能较大**
  （实测 diff ~ 数百 KB，属如实保全）；run_id 现含时刻 + 随机后缀，依赖 `-<scenario>-<seed>` 子串的
  既有 glob 仍可匹配（**glob 兼容后缀**），但**精确路径**定位（旧写法无 `*` 后缀）会失配，已在
  `run_w6_r027.sh` 同步改为后缀容忍。
- **未决 / 后续**：`fingerprint_sha256` 目前只覆盖 `HEAD + status + diff`，未纳入未跟踪文件**内容**
  （仅列名）；多后端（含 PX4）工具链身份待 `px4_root` 实跑回填；campaign 级「校验过的不可变产物」引用
  （R-030 配套）属后续项。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。

## 参考

- 评审：[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-029)
- 规格：[fix-scenario-axis-metrics-identity-r26-r30 spec](../../.trae/specs/fix-scenario-axis-metrics-identity-r26-r30/spec.md)（R-029）
- 产物纪律：[ADR-0007](0007-experiment-artifacts-convention.md)
- 前序：[ADR-0043](0043-episode-window-metrics-and-valid-sample-mask.md)
- 实现：`run_logger.cpp` / `episode_runner.py` / `run_w6_r029.sh` / `w6_r029_check.py`
