# 安全不可行率诚实口径与真实接触证据（R-027：proxy_v1 保留 + gz_contact_v1 真实碰撞）

## Status

accepted

## 背景

评审风险 **R-027（不可行率定义）**（[10-风险](../../docs/temp/agc-review-2026-10-14/10-风险.md#r-027)）指出：
[`run_logger`](../../src/agc_task/src/run_logger.cpp) 的 `shield_infeasible_rate` 把「安全层触发
（`collision_monitor` STOP/SLOWDOWN）且前向扫描 ≤ `contact_dist_m`」直接当作**接触 / 不可行**，
**不要求实际接触 / 制动失败**；TTC = `range/|v_body|` 忽略目标闭合、车体边界与转弯；拍分母受 `odom`
发布频率与停止驻留时长影响。后果：**正常提前停车可得到高不可行率**，`753/753` 不能证明每拍发生物理接触；
不同策略停车时长改变统计；C9 字段存在 ≠ 统计可解释（另见 [X-06](../../docs/temp/agc-review-2026-10-14/10-风险.md#x-06)）。

契约真源：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) **C9**（必报指标）、
[#params](../ROS2迁移交接文档.md#params)（单一注入，节点不得覆写）、
[#execution-port](../ROS2迁移交接文档.md#execution-port)（唯一执行口语义不变）、
[#messages](../ROS2迁移交接文档.md#messages)（9 消息 ABI 不得变更）；产物纪律
[ADR-0007](0007-experiment-artifacts-convention.md)（唯一落点 `experiments/<campaign>/<run_id>/`）。

**先学后做（取证依据）**：核验本机同版本既有实现——① **GZ Sim 官方 contact sensor 教程**（父 link 内
`<sensor type="contact">` + 同 link 内 `<collision>` name；检测接触发布 `gz.msgs.Contacts`）；
② **`ros_gz_bridge` 的 `gz.msgs.Contacts` ↔ `ros_gz_interfaces/msg/Contacts` 映射**（已用
`strings libros_gz_bridge.so` 确认符号存在；`Contacts = Header + Contact[]`，`Contact` 含
`collision1/collision2`（`Entity.name`）、`geometry_msgs/Vector3[] positions`、`float64[] depths`）；
③ 世界 [`agc_world.sdf`](../../src/agc_sim/worlds/agc_world.sdf) 已加载 `gz-sim-contact-system`，无需再加世界插件。
**实测**（`gz topic -l`）：contact sensor 的 gz 话题由**作用域名**决定（`/world/agc_world/model/agc_car/
link/car/base_link/sensor/contact/contact`），**不**取 SDF `<topic>` 值 —— 桥条目据实采用该名。

分类：**事实错误与设计风险**。本 ADR 给出诚实口径与真实接触证据的明确决定，**不**否定 C9 必报要求，
**不**回退任何已验收组件。

## 决策

### (a) 保留历史字段 proxy_v1 + 口径版本标注

`shield_infeasible_rate` / `ticks_total` / `ticks_triggered` / `ticks_infeasible` 等历史键**保留不动**
（**不**静默重算旧历史）；新增 `definition_version`（`{"shield_infeasible_rate": "proxy_v1",
"collision_truth": "gz_contact_v1", "true_shield_infeasible": "gz_contact_v1"}`）与
`metrics_comparability` 文字，使「同一字段在不同版本下含义不同」**可判定、可复核**。

### (b) `gz_contact_v1` 真实碰撞（GZ contact sensor 为真值）

车模 [`agc_car.sdf.xacro`](../../src/agc_sim/models/agc_car/agc_car.sdf.xacro) 的 `car/base_link` 增
contact sensor（监测同 link 内 `base_collision`），由世界已加载的 `gz-sim-contact-system` 发布
`gz.msgs.Contacts`，经 [`bridge.yaml`](../../src/agc_sim/config/bridge.yaml) 桥为 ROS
`/agc/ground/contact`（`ros_gz_interfaces/Contacts`，`GZ_TO_ROS`）。
[`run_logger`](../../src/agc_task/src/run_logger.cpp) 订阅该话题，逐帧判定**车↔非车**接触对
（`collision1`/`collision2` 的 `name` 含 `agc_car` 为车侧；另一侧 name 非空且不含 `agc_car`），
**排除世界地面**（`ground_plane`；车体 `base_collision` 在复位瞬间因 `car_z` 轻微没入地面属**建模产物**，
非障碍 / 实体碰撞），按**上升沿**（该对上一帧无接触）去重，记 `collision_count` / `collision_events`
（`{t_sim, other, depth, position}` —— 含**时刻**与**接触几何**：最小穿透深度 + 首个接触点 world 坐标）。**真实碰撞 = GZ 接触（对场景实体）**，独立于 proxy_v1「触发拍代理」计数。
新增传感器 / 话题均为**诊断类**，**不改**物理 / 惯性 / 驱动插件，**非** ROS 命令者。

### (c) 触发事件 / 干预时长 / 剩余停车裕度

- `trigger_events`：`evaluateTick()` 中 `triggered && !prev_triggered_` 的**上升沿**计数；
- `intervention_duration_s`：触发期间用**仿真时钟**增量（`dt = now().seconds() - last_tick_sim_s_`，
  `>0` 才累加）累计；
- `stopping_margin_min_m` / `stopping_margin_negative_ticks`：触发拍且 `have_scan_` 时，
  锁轮制动距离 `brake_d = v^2/(2·a_brake)`（`a_brake` 取自
  [ADR-0021](0021-ground-safety-thresholds-and-platform-recalibration.md)(C) 实测锁轮制动），
  `margin = scan_min_forward - brake_d`；维护最小值与负值触发拍数；
- `ticks_triggered_no_scan`：无 `scan` 的触发拍（缺数据证据，不计入裕度）。

### (d) 真实安全不可行（`true_shield_infeasible`）

触发结束（`!triggered && prev_triggered_`）时，若该触发期内发生过**真实碰撞事件**（`onContacts` 置
`current_trigger_has_contact_`）→ `infeasible_trigger_events_++`；若 episode 在**仍处于触发期**时结束
（无下降沿）且该触发期内已发生真实接触，则在 `finalize()` 补记一次（**honest 优先，避免漏报**）。
`true_shield_infeasible` = 是否有此事件（含补记）；`true_shield_infeasible_rate` =
`infeasible_events_total/trigger_events_`，`trigger_events_==0` 时输出 **`null`**（**不**以 0 冒充）。

### (e) 零速停于阈值外 SHALL NOT 计为碰撞

车在阈值（如 2 m）外缓停（`v≈0`、未发生接触）→ **无 GZ 接触** → `collision_count==0`。
此由「碰撞 = GZ 真实接触」**天然满足**；**不**用 `scan ≤ contact` 反推 collision（proxy_v1 的
`ticks_infeasible` 与真实碰撞**分离**，互不冒充）。

### (f) 缺数据 → `not_applicable` 掩码

新增 `not_applicable`：`{"scan_derived": <total_ticks_==0 || scan_samples_==0>,
"trigger_derived": <trigger_events_==0>, "collision_truth": <contact_topic 参数为空>}`。
缺数据段**如实标注**（**不**默认记 0 冒充有效统计）。`contact_topic` 为空 = 接触真值不可用。

### (g) 不与旧 RL 安全集指标直接对照 + 边界 / 红线

`proxy_v1` 与 `gz_contact_v1` **均不与旧 RL 安全集指标直接对照**（口径 / 分母不同；真实碰撞以 GZ 接触
为真值），此约束写入 `metrics_comparability`。**不**改契约冻结 9 消息 / 服务 ABI；**不**新增第二 ROS
命令者（新增均为诊断类话题 / run config / GZ 传感器桥）；**不**改唯一执行口语义；参数**单一注入**
（契约 [#params](../ROS2迁移交接文档.md#params)），节点不得覆写。

### (h) 验收脚本与产物

新增 [`run_w6_r027.sh`](../../scripts/run_w6_r027.sh) + [`w6_r027_check.py`](../../scripts/w6_r027_check.py)：
Phase A 负对照（`s11` 车朝 `pole_1` 直行、安全层阈值外提前停车 → `collision_count==0`）；
Phase B 正例（`agc_car` spawn 与 `pole_1` 几何重叠 → GZ 真实接触 → `collision_count>=1` 且 `other` 含 `pole`）；
Phase C 核查 (a)…(f)。产物落 `experiments/w6-task-eval/<run_id>/{summaries,metrics/r027.json,log}`。

## 备选

1. **直接改 `shield_infeasible_rate` 为真实接触口径**：单字段最简；但**破坏历史可比**、旧 run 无法解释
   ——否决（保留历史字段 + 版本标注更诚实）。
2. **仅用几何（`scan≤contact`）当接触真值**：无需传感器；但**几何不产生物理接触**（提前停车也会命中）
   ——正是 R-027 病灶——否决（采用 GZ contact **物理真值**）。
3. **用 `gz topic -p` 或额外 ROS 发布者造接触**：可复现但**新增第二命令者 / 改世界** → 触红线——否决
   （Phase B 以 spawn 几何重叠经 GZ 物理产生接触）。
4. **`trigger_events_==0` 时输出 `0`**：便于画表；但**以 0 冒充有效统计**——否决（输出 `null`）。

## 后果

- **正面**：`bridge.yaml` / `agc_car.sdf.xacro` 提供**真实接触真值**；`run_logger` 双口径并行且
  `definition_version` / `metrics_comparability` 显式标注；真实碰撞 / 触发事件 / 干预时长 / 剩余停车裕度 /
  真实安全不可行可核对；阈值外缓停**不计**碰撞；缺数据 `not_applicable` 如实标注；Phase A/B 实跑对照。
- **代价 / 限制**：GZ contact sensor 的 gz 话题名由**作用域名**决定（实测采用，非 SDF `<topic>`），
  换世界 / 换模型名需同步核对；`contact_msgs_received` 频率随物理步长（实测 ~250 Hz）；
  `intervention_duration_s` 依赖仿真时钟（`use_sim_time`）；接触证据仅覆盖**车体 base_collision**
  与其它碰撞体，未覆盖轮 / 载荷几何。
- **未决 / 后续**：真实安全不可行的**判定阈值**（如裕度门限）、多接触体聚合、以及 R-028（episode 窗口
  统计）对拍，属后续项。
- 与旧树：**只读**对齐**行为事实**（WHAT），**不**复制旧代码 / 资产（HOW）。
