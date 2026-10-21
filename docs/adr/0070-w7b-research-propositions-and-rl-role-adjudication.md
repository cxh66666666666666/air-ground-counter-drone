# W7-b 科研命题与 RL 角色裁定框架：两条单一自变量可证伪命题、最小火控与归因前置、预注册判定

## Status

accepted

## Date

2026-10-19

## 背景

[ADR-0069](./0069-w7-entry-and-exit-definition.md) 已把 W7 第一出口定为**可信实验平台**（W7-a，已落地并闭合），并明确 **RL 算法、网络、奖励、训练步数、论文结论属 W7-b**。评审 [F-02](../reviews/agc-review-2026-10-14/30-未来.md#f-02) 要求"**研究问题先于算法**"，[F-03](../reviews/agc-review-2026-10-14/30-未来.md#f-03) 显式**挑战** [ADR-0004](./0004-rl-primary-controller-nav2-safety-layer.md) 的 RL 主控角色，[F-06](../reviews/agc-review-2026-10-14/30-未来.md#f-06)/[F-10](../reviews/agc-review-2026-10-14/30-未来.md#f-10)/[F-11](../reviews/agc-review-2026-10-14/30-未来.md#f-11)/[F-14](../reviews/agc-review-2026-10-14/30-未来.md#f-14) 分别要求安全层归因、可见性协同、火控升级与分层基线消融，[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13) 要求统计冻结与保留测试集。

研究主线为**移动式主动拦截系统**、空地协同路径/制导为核、RL 为手段（[ADR-0014](./0014-research-mainline-mobile-interception-system.md)）。W7-b 现存三处**未决**、若不裁定即会成为**隐式 ADR**（[F-02](../reviews/agc-review-2026-10-14/30-未来.md#f-02) 警告）：

1. **以什么为科研自变量**（协同程度 α 抑或 RL 职责分层 β）——若不二选一，则"改善捕获机会"**无法归因**；
2. **捕获奖励的前置**——现捕获裁决是闭式**占位包络**（[ADR-0024](./0024-capture-adjudication-fire-control-window.md)、[R-020](../reviews/agc-review-2026-10-14/10-风险.md#r-020)），直接据此训练等于**学占位边界**；
3. **判定规则**——若不在实验前预注册，即为事后拟合（[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)）。

本 ADR 只做**科研设计裁定**（命题 / 变量 / 对照 / 前置能力 / 判定规则），**不**定算法、奖励、网络、训练步数——那些由命题倒推、在 [`.trae/specs/w7-rl-b/`](../../.trae/specs/w7-rl-b/spec.md) 承载。

## 决策

### (a) 两条可证伪命题，各自**单一自变量**

W7-b **SHALL** 落两条命题，**每条只有一个自变量**（多变量不可归因）：

| 命题 | 自变量（IV） | 因变量（DV，见 (b)） | 固定条件 |
|---|---|---|---|
| **P1 协同收益（主线）** | **协同程度 α** = {空地协同策略, 非协同（地面单体 / 空中固定航迹）} | 任务出口 + 捕获机会代理 + 净空 | 同传感权限 / 速度约束 / 安全层 / 终态 |
| **P2 RL 职责分层（副线）** | **职责分层 β** = {臂 A: c2（RL 直接出 `cmd_vel`）, 臂 B: 高层 RL（拦截点 / 火控窗口 / 局部偏移 + 成熟低层控制）} | 同上 | 固定为"**协同**"；其余同 P1 |

理由：主线是**协同**（[ADR-0014](./0014-research-mainline-mobile-interception-system.md) 的"协同路径规划/制导为核"），[F-02](../reviews/agc-review-2026-10-14/30-未来.md#f-02) 原型命题字面即"**协同策略**"；RL 职责是"手段性"裁定（[ADR-0014](./0014-research-mainline-mobile-interception-system.md)、[F-03](../reviews/agc-review-2026-10-14/30-未来.md#f-03)），故为其在**固定协同条件下**的内部对照。

### (b) 因变量：**分层多指标**，非单一成功率

- **主量**：任务出口（`TRACKING_SUCCESS` / `CAPTURE_NET`）；**超时一律失败**（契约 [#hard-constraints](../ROS2迁移交接文档.md#hard-constraints) C4）。
- **辅量·捕获机会代理**：进入捕获包络的**停留时长**、**有效火控窗口时长**（包络内 ∧ 净空达标 ∧ 目标可见，[F-10](../reviews/agc-review-2026-10-14/30-未来.md#f-10)）。
- **辅量·净空**：最小安全距离、碰撞率、**shield 干预量/时长**（契约 C9 代理率，[F-06](../reviews/agc-review-2026-10-14/30-未来.md#f-06)）。
- **infra / 复位失败单列**，不吞入成功率（[ADR-0046](./0046-runner-three-layer-acceptance-and-integrity-gate.md)）。

禁止把"单一成功率"当唯一判据（[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)）。

### (c) 捕获奖励前置：**最小动态火控裁决**

含"捕获机会/命中"的因变量**SHALL** 先由**最小动态火控闭环**承载（阶段：相对高度与可及集 → 确定射击时刻 / 目标预测 / 展开-飞行 / 命中-失手-冷却 → 与 GZ 逐帧弹道交叉核验，[F-11](../reviews/agc-review-2026-10-14/30-未来.md#f-11)），替换现 `EnvelopeAdjudicator`（可插拔抽象的 I/O 契约不变）；网的**弹道在本节点逐帧积分**（**非** GZ 物理刚体），「与 GZ 交叉核验」= **逐帧与 GZ 目标/车真值位姿**比对。**替换后先冻结任务难度再训练**，对齐 [ADR-0069](./0069-w7-entry-and-exit-definition.md) 未决段"火控升级先于捕获奖励"。**禁止**以占位包络训练后宣称"移动式主动拦截"。

### (d) W7-b 入口**前置能力**（先建、后实验）

| # | 能力 | 依据 |
|---|---|---|
| **E1** | **raw / effective 命令同拍记录**：在同一采样瞬时取安全层**输入**与**输出**快照（并**如实标注** `header.stamp` 是否被安全层保持）+ 干预量 / 干预时长 + 实际净空 + 任务结果，并支持 **with-shield / 等预算对照臂**（纯仿真无 shield 臂须明标范围） | [F-06](../reviews/agc-review-2026-10-14/30-未来.md#f-06)、[ADR-0069](./0069-w7-entry-and-exit-definition.md)（原 D8 移出项） |
| **E2** | **最小动态火控裁决**（见 (c)） | [F-11](../reviews/agc-review-2026-10-14/30-未来.md#f-11) |
| **E3** | **可见性度量口径修复**：按 active / 稳定任务窗口重算可见率、最长失联、恢复时长、有效火控机会（现 `tgt_age` 未按任务窗口切分，分母失真） | [F-10](../reviews/agc-review-2026-10-14/30-未来.md#f-10) |
| **E4** | **`obs_version` v2**（新增目标 **bearing / 相对速度 / 观测年龄**；yaw / 朝向交由确定控制器）+ **v1 与旧权重回放** | [F-04](../reviews/agc-review-2026-10-14/30-未来.md#f-04)；机制见 W7-a T2 |
| **E5** | **离线数据集 / 回放库**（由 W7-a 可复现 run 归档的轨迹构建） | [F-16](../reviews/agc-review-2026-10-14/30-未来.md#f-16) |

**协同通道**复用现 W3 / W4，**不另建**（[ADR-0069](./0069-w7-entry-and-exit-definition.md)(b)：复用当前空地仿真栈）。

### (e) RL 角色裁定：**两臂对照，裁定前遵守 c2**

[F-03](../reviews/agc-review-2026-10-14/30-未来.md#f-03) 之挑战**在本段正式裁定**，以**对照臂**形式（**不**直接改在线主控）：臂 A 与臂 B **同安全层、同唯一执行口**（契约 [#execution-port](../ROS2迁移交接文档.md#execution-port)）、**同预算**；**裁定前默认遵守 c2**（[ADR-0004](./0004-rl-primary-controller-nav2-safety-layer.md)），不预断 B 胜出。任何臂**不得**在同一任务引入**第二命令者**。

### (f) 臂与消融清单

1. **基线锚**：规则基线「**Navfn + 自研 LQR**」（复现 W7-a T5 证据；[F-14](../reviews/agc-review-2026-10-14/30-未来.md#f-14)，[R-012](../reviews/agc-review-2026-10-14/10-风险.md#r-012) **准确命名**，**不**称"Nav2 controller 完整配置"）；
2. **P1 两臂**：协同 / 非协同；
3. **P2 两臂**：c2 / 高层 RL；
4. **消融**：协同观测、目标预测、安全干预（with / without shield，等预算）。

每臂**同**传感权限 / 速度约束 / 安全层 / 终态语义（[F-14](../reviews/agc-review-2026-10-14/30-未来.md#f-14)）。**本段不新增**"成熟 Nav2 controller 臂"（留 W8 对照冻结，[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)）。

### (g) 统计与配对协议

沿用 W7-a **三级种子**（scene / episode / attempt）+ 六流，配对比较用 **CRN**（外生五流不随 attempt 变）；预算复用 [ADR-0039](./0039-time-budget-kinds-and-rtf-record.md) 三类口径；**超时一律失败**（C4）、**infra 单列**；报告**有效 episode 数 + 保留全部失败与 infra 总数 + 置信区间**（[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)）；**不承诺** bitwise 物理一致，RTF 差异边界成文（W7-a 已知边界）。

### (h) 预注册判定规则

判定**SHALL** 在实验前预注册，禁止事后拟合：

- **P1**：协同臂在**预设有效样本 + 置信区间**下显著优于非协同（有效火控机会 / 最小净空）→ **协同收益成立**；否则**证伪**。
- **P2**：臂 B 在**等预算**下显著优于臂 A → **RL 职责升级成立**；否则**保留 c2**（**不**推翻 [ADR-0004](./0004-rl-primary-controller-nav2-safety-layer.md)）。

样本量与显著性阈值**依效应大小在实验前设定**（[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)）。

### (i) 结论域边界

W7-b 结论**仅对已验证通道族**（在 独立噪声 / 共同偏置 / 突发丢包 / 延迟 / 乱序 / 假阳性 / 身份交换 中**实际建模并验收者**）成立；**不得**据此声称真实检测或 sim-to-real；**GT 不进普通策略观测**，仅评估 / 显式 oracle 臂（[F-09](../reviews/agc-review-2026-10-14/30-未来.md#f-09)）。

### (j) 算法路径：**离线优先，规则基线不达出口才上在线 RL**

① 先建**离线数据集 / 回放库**（E5）与**规则臂**；② **仅当**规则基线在各消融下**不达出口**时，才引入在线 RL（受预算约束：CPU / 单卡小网络）。**本段不锁**网络细节、奖励与训练步数。对齐 [F-02](../reviews/agc-review-2026-10-14/30-未来.md#f-02)"先证明任务非平凡且可达，再决定 RL 是否有必要"。

### (k) W7-b 出口

**出口 = 命题卡 + 臂可运行证据 + 裁定文档**，**不含**论文级统计结论（后者属 W8 冻结，[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)）。逐条判据见 [`.trae/specs/w7-rl-b/`](../../.trae/specs/w7-rl-b/spec.md)。

## 备选

1. **以轴 β（RL 职责分层）为主线命题** —— 否决：系统论文主线是**协同**（[ADR-0014](./0014-research-mainline-mobile-interception-system.md)），把"用了哪种 RL"当主线会**以手段替代贡献**（[F-02](../reviews/agc-review-2026-10-14/30-未来.md#f-02)）。
2. **F-02"三选一"并列三候选** —— 否决：可见性 / 安全干预更适为 P1 的**受控子指标**；并列会稀释主线承重。
3. **以占位包络训练捕获奖励** —— 否决：学占位边界（[R-020](../reviews/agc-review-2026-10-14/10-风险.md#r-020)），结论不可用于主动拦截主线。
4. **直接训 DQN/PPO 比成功率** —— 否决：[F-02](../reviews/agc-review-2026-10-14/30-未来.md#f-02) 明示此为"优化占位裁决"的陷阱；算法须由命题倒推。
5. **本段新增成熟 Nav2 controller 对照臂** —— 否决（本段）：自变量为协同 / 职责，非导航栈；留 [F-14](../reviews/agc-review-2026-10-14/30-未来.md#f-14) 的 W8 对照。

## 后果

- **正面**：W7-b 的科研变量、对照、前置与判定**显式可判定**；[F-03](../reviews/agc-review-2026-10-14/30-未来.md#f-03) 挑战被**正式承接**而非静默；[F-06](../reviews/agc-review-2026-10-14/30-未来.md#f-06) / [F-10](../reviews/agc-review-2026-10-14/30-未来.md#f-10) / [F-11](../reviews/agc-review-2026-10-14/30-未来.md#f-11) / [F-14](../reviews/agc-review-2026-10-14/30-未来.md#f-14) 各有落点。
- **代价 / 限制**：W7-b **先出证据与裁定**、不出论文结论；最小火控 / 归因 / 可见性口径是**新增工程量**（E1–E5），先于任何策略对照。
- **未决 / 后续**：算法、奖励、网络、训练步数、样本规模（W7-b 后段）；多 worker 隔离（W7-c，[F-08](../reviews/agc-review-2026-10-14/30-未来.md#f-08)）；保留测试集 / 置信区间 / 原件归档（W8，[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)）；实机（[F-17](../reviews/agc-review-2026-10-14/30-未来.md#f-17)）。
- **不改**：契约 C1–C10、冻结 9 消息 / 服务 ABI、唯一执行口语义、W7 / W8"后续"地位（[ADR-0016](./0016-execution-scope-exit-w6.md)）。归档旧树 `reference/` **只读**（仅对齐行为事实，[ADR-0066](./0066-decision-applicability-and-reference-discipline.md)）。

## 参考

- 评审：[F-02](../reviews/agc-review-2026-10-14/30-未来.md#f-02)、[F-03](../reviews/agc-review-2026-10-14/30-未来.md#f-03)、[F-04](../reviews/agc-review-2026-10-14/30-未来.md#f-04)、[F-06](../reviews/agc-review-2026-10-14/30-未来.md#f-06)、[F-08](../reviews/agc-review-2026-10-14/30-未来.md#f-08)、[F-09](../reviews/agc-review-2026-10-14/30-未来.md#f-09)、[F-10](../reviews/agc-review-2026-10-14/30-未来.md#f-10)、[F-11](../reviews/agc-review-2026-10-14/30-未来.md#f-11)、[F-13](../reviews/agc-review-2026-10-14/30-未来.md#f-13)、[F-14](../reviews/agc-review-2026-10-14/30-未来.md#f-14)、[F-16](../reviews/agc-review-2026-10-14/30-未来.md#f-16)（[30-未来](../reviews/agc-review-2026-10-14/30-未来.md)）；[R-012](../reviews/agc-review-2026-10-14/10-风险.md#r-012)、[R-020](../reviews/agc-review-2026-10-14/10-风险.md#r-020)
- 契约：[#hard-constraints](../ROS2迁移交接文档.md#hard-constraints)、[#execution-port](../ROS2迁移交接文档.md#execution-port)、[#params](../ROS2迁移交接文档.md#params)、[#terminal-semantics](../ROS2迁移交接文档.md#terminal-semantics)
- 前序 ADR：[ADR-0004](./0004-rl-primary-controller-nav2-safety-layer.md)、[ADR-0014](./0014-research-mainline-mobile-interception-system.md)、[ADR-0016](./0016-execution-scope-exit-w6.md)、[ADR-0024](./0024-capture-adjudication-fire-control-window.md)、[ADR-0039](./0039-time-budget-kinds-and-rtf-record.md)、[ADR-0044](./0044-unique-run-identity-and-reproducible-evidence.md)、[ADR-0046](./0046-runner-three-layer-acceptance-and-integrity-gate.md)、[ADR-0069](./0069-w7-entry-and-exit-definition.md)
- 规格：[`.trae/specs/w7-rl-b/`](../../.trae/specs/w7-rl-b/spec.md)；上游 W7-a：[`.trae/specs/w7-rl/`](../../.trae/specs/w7-rl/spec.md)
