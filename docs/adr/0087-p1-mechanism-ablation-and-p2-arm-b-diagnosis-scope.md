# P1 机制消融（指向 × 跟踪参考 2×2）与 P2 臂 B 负向诊断的范围裁定

## Status

accepted

## Date

2026-10-23

## 背景

P1 v5（[ADR-0085](0085-w7b-p1-mechanism-activating-scenario-pilot.md)(h)）在机制激活场景（`s15`，static + 窄视场）得到 active FCO 配对差 **+0.0804**（CI [+0.0281, +0.1328]、p=0.0046；按 [ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md) 三态为 `inconclusive`）。但 α 在实现上是**两项开关的组合**：

- `observe_yaw_mode ∈ {target, velocity}`（机头**指向**：对准目标 / 随速度）；
- `air_tracking_ref ∈ {on, off}`（空中**跟踪参考**：跟随目标航迹 / 固定 figure-8）。

现有两臂同时改变二者（协同 = target+on；非协同 = velocity+off）⇒ **无法归因**：收益来自**指向**、**跟踪位置**，还是**两者交互**。同时，地面仍用**固定任务目标**（`goal_follow_target=false`，[ADR-0085](0085-w7b-p1-mechanism-activating-scenario-pilot.md) 固定条件）——消融在该条件下进行，不得顺手改地面侧。

P2 v3（[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(j)）判定 `falsified`（B 臂 active FCO 低于 A：Δ=−0.0561）——**负向结果的原因未诊断**（重规划、抵近过程、预测、安全层干预均未拆开）。

**用户裁定（2026-10-23）**：①优先补机制消融（P1 拆两项贡献）；②P2 先诊断 B 臂再决定是否改进高层；③**现有证据不足以进入在线 RL 训练**。

## 决策

### (a) P1：2×2 析因消融（**先定规则**，独立批次）

| cell | `observe_yaw_mode` | `air_tracking_ref` | 臂名 | 状态 |
|---|---|---|---|---|
| ① 指向 + 跟踪 | `target` | `on` | `p1_coop` | 已有（v5） |
| ② 无指向 + 无跟踪 | `velocity` | `off` | `p1_noncoop` | 已有（v5） |
| ③ **仅指向** | `target` | `off` | `p1_point_only` | **新增** |
| ④ **仅跟踪** | `velocity` | `on` | `p1_track_only` | **新增** |

- **开关边界（已授权）**：两个开关均为既有参数，**不新增任何旋钮**；`observe_yaw_mode` 的 `target` 仍是**显式 oracle 指向**（[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)(i)）；`air_tracking_ref` 的两种取值由 [ADR-0073](0073-p1-arm-fairness-noncoop-fixed-trajectory.md) 定义（其"备选 2"已预留"只留指向差异"的用法）。
- **固定条件**：与 v5 相同（`s15_static_none_fov_capture` / full 链 / `goal_follow_target=false` / 同安全层 / 同终态 / 同 seed 集）。
- **先定规则（写入早于运行）**：
  1. **同批四 cell**（单 run、每 cell N=20、seeds 0–19、CRN 逐 seed 配对），**不**跨 run 拼 cell；
  2. 分析为**估计型**（析因对比 + 置信区间），**不设门槛判定**（消融回答"贡献来自哪一项"，不是"是否达标"）——避免与 [ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md) 的门槛语义混用；
  3. 对比量（逐 seed 配对）：**指向主效应**（tracking=on：①−④；tracking=off：③−②）、**跟踪主效应**（pointing=target：①−③；pointing=velocity：④−②）、**交互**（(①−④)−(③−②)）；DV = active-window FCO（primary 口径同 v5），辅报 `vis_rate` / `env_occ|vis` 的分解；
  4. **预登记的已知风险**：cell ④（`velocity`+`on`）在 [ADR-0073](0073-p1-arm-fairness-noncoop-fixed-trajectory.md) 中已被实证会触发 **C10 hold 自锁**（获目标后丢失 → 冻结、机头随速度 → 目标出视场不可恢复）⇒ 该 cell **可能退化**；若退化，照实报告并把"跟踪主效应"只按 cell ①/③ 一侧（pointing=target）解读，**不得**把退化 cell 当作公平基线。
  5. **pilot 先行**：先 3 seeds × 4 cell（同批 12 episodes）验证四 cell 可运行 / 可解释；pilot **不入判定**，仅用于确认设计可承载；随后按 (a)3 的规则跑 N=20。
- **产物**：`experiments/w7-rl-b/<run_id>/metrics/{arms.json, p1_ablation.json}`；分析脚本 [`scripts/w7b_p1_ablation.py`](../../scripts/w7b_p1_ablation.py)。
- **不改判定**：本消融**不**改 P1 v1/v4/v5 的 verdict；若消融后要立"某单项即足够"的新判定，须**新预注册版本（v7）**。

### (b) P2：臂 B 负向结果的**诊断先行**（不改 verdict、不改臂）

用 v3 run（`w7-rl-b/20261022-w7-rl-b-v3-n20r`）**既有产物**拆 B 的负向：

1. **重规划**：B 的 `重规划触发` 频次（[ADR-0083](0083-w7b-p2-arm-information-equivalence-and-active-window-fco.md)(g) 低通修复后的**残留**）；
2. **抵近过程**：active 窗口时长 / 进入包络耗时 / 终态前的逼近行为（对照 A）；
3. **预测**：高层外推（`lead`）与实际目标运动的一致性（静止目标下应为零提前量）；
4. **安全层干预**：`intervened_ticks` / 干预时长（两臂对照）。

- **判据（先定）**：诊断报告**只**做归因（哪一组件解释 B 的 active FCO 劣势），**不得**以"调到 B 赢"为目标（[ADR-0082](0082-w7b-p2-arm-b-rule-high-level-first.md) 的预注册纪律不变）；若诊断指向可修的实现缺陷（如重规划振荡），先修**缺陷**并留 ADR，**再**决定是否需要**新预注册**重判 P2。
- **产物**：`experiments/w7-rl-b/20261022-w7-rl-b-v3-n20r/metrics/p2_arm_b_diagnosis.{json,md}`（描述性；verdict 不改）。

### (c) 不进在线 RL

现有证据（P1 `inconclusive`、P2 `falsified`、机制消融未完成、B 臂负向未归因）**不足以**支撑"训练 RL 高层"的决定；[ADR-0082](0082-w7b-p2-arm-b-rule-high-level-first.md)(d) 的触发条件**未满足**，维持"规则高层优先"。

### (d) P2 臂 B 诊断结果（2026-10-23；用 v3 run 既有产物）

产物：`experiments/w7-rl-b/20261022-w7-rl-b-v3-n20r/metrics/p2_arm_b_diagnosis.{json,md}`（[`scripts/w7b_p2_arm_b_diagnose.py`](../../scripts/w7b_p2_arm_b_diagnose.py)）。

| 量 | A `p2_c2_low_level` | B `p2_rl_high` | 差 |
|---|---|---|---|
| active FCO 均值 | 0.1108 | 0.0571 | **B 低 0.054** |
| 可见拍中包络占比 | 0.1135 | **0.0587** | **B 腰斩** |
| active 窗口时长均值 | 16.4 s | **41.2 s** | **B 2.5×** |
| 重规划次数/回合 | **0** | **23.7** | 目标身份变 190 ｜ 地图变化 282 ｜ 路径超时 2 |
| 开火次数（总 / 有开火回合） | 24 / 20 | 38 / 19 | B 开火**更多**但窗口更长 |
| 首射相对窗口开始延迟 | 13.4 s | 18.7 s | B 慢 5.3 s |
| 安全层干预拍 / 干预时长 | 279 / 12.5 s | **901 / 34.2 s** | **B 3.2×** |
| 发射时 `pred` 与目标真值水平偏差（均值 / 中位 / p90 / max） | 0.213 / 0.196 / 0.335 / 0.577 m | **1.282** / 0.260 / **5.582** / **8.741 m** | 中位相近，**B 重尾** |
| `pred` 偏差 > 0.5 m（网半径）的发射占比 | 8%（2/24） | **26%（10/38）** | B 在估计劣化时仍开火 |

**归因（B 负向的可解释组件，按量级）**：

1. **抵近过慢 + 重规划退化**：B 的 active 窗口 **2.5×** 于 A，每回合 **23.7** 次重规划（A 为 0）——低层消费高层拦截点后反复重规划，路径跟踪退化 ⇒ 窗口变长、包络占用被摊薄（`env_occ|vis` 0.114 → 0.059）。
2. **安全层持续纠偏**：B 的干预拍 **3.2×**（901 vs 279）、干预时长 34.2 s vs 12.5 s ⇒ 相当比例的指令被安全层改写，机动效率进一步下降。
3. **发射质量重尾**：B 的中位射击偏差与 A 相当（0.26 vs 0.20 m），但 **26% 的发射 `pred` 偏差 > 网半径**（最大 8.7 m）——在目标估计劣化时仍开火；A 仅 8%（最大 0.58 m）。
4. **净空的诚实登记**：两臂 `min_clearance` 相当（4.64 vs 4.73 m），**不**支持"B 更危险"的说法。

**结论与后续（不改进程判定）**：B 的负向**可归因**（重规划退化 → 窗口变长/包络占用摊薄 + 安全层纠偏 + 发射质量重尾），**不是**"分层架构必然更差"。是否改进高层取决于：①重规划振荡是否属**实现缺陷**（若是 → 先修缺陷并留 ADR，再以**新预注册**重判 P2）；②发射解算是否应加"`pred` 偏差 / 一致性"门（当前 [ADR-0079](0079-b2-fire-control-target-motion-estimation.md) 已有静止/上限门，未覆盖本项）。**v3 判定不改写**。

### (e) P1 消融 pilot（3 seeds × 4 cell）与随之启动的 N=20

**pilot** run `w7-rl-b/20261023-w7-rl-b-p1abl-pilot`（12 episodes + 重试；[`scripts/w7b_p1_ablation.py`](../../scripts/w7b_p1_ablation.py) → `metrics/p1_ablation.{json,md}`）：

| cell | 指向 | 跟踪 | 终态（3 seeds） | active FCO 均值 | active 可见率 | 可见拍中包络占比 |
|---|---|---|---|---|---|---|
| ① `p1_coop` | target | on | 3× 捕获 | 0.051 | 0.416 | 0.203 |
| ② `p1_noncoop` | velocity | off | 1 捕获 / 2 失败 | 0.018 | 0.068 | 0.282 |
| ③ `p1_point_only` | target | off | 3× 捕获 | **0.123** | 0.589 | 0.224 |
| ④ `p1_track_only` | velocity | on | 1 捕获 / 2 失败 | 0.026 | 0.097 | 0.162 |

- **设计可承载** ✔（四 cell 均可运行、可比）；cell ④ 如预登记出现 **C10 hold** 记录（seed2 日志 3 条 `显式 hold`）但未完全失能（1/3 捕获）。
- **pilot 级信号（n=3，不入判定）**：**指向**方向的对比为正（tracking=off 时 ③−② = **+0.105**），而 **跟踪**方向在 pointing=target 时为 **负**（①−③ = −0.072，即"仅指向"优于"指向+跟踪"）；交互 −0.080。⇒ 与 v5 的机制边界一致（该场景为**静止目标**，跟踪位置的价值有限），**但 n=3 的 CI 极宽，不得据此下结论**。
- **决定**：按 (a)5 进入 N=20 全批（四 cell 单 run，`w7-rl-b/20261023-w7-rl-b-p1abl-n20`，80 episodes）；pilot 数值**不**与 N=20 混用。

#### N=20 结果

见 (f)（同一 run `20261023-w7-rl-b-p1abl-n20`；本节不重复数值，避免两处漂移）。

### (f) P1 消融 N=20 结果（2026-10-23；**估计型**，不改 verdict）

承载 run `w7-rl-b/20261023-w7-rl-b-p1abl-n20`（4 cell × seeds 0–19 = 80 episodes / 墙钟 5.0 h / infra 重试 45 次）；分析产物 `metrics/p1_ablation.{json,md}`。

| cell | 指向 | 跟踪 | 终态 | active FCO（真实） | 成功回合 FCO | active 窗口均值 | 可见率 | 可见拍中包络占比 |
|---|---|---|---|---|---|---|---|---|
| ① `p1_coop` | target | on | **19/20 捕获** | 0.083 | 0.087 | 22.8 s | 0.576 | 0.187 |
| ② `p1_noncoop` | velocity | off | 14/20 | 0.020 | 0.024 | 79.7 s | 0.063 | 0.325 |
| ③ `p1_point_only` | target | off | 15/20 | **0.155** | **0.184** | 42.0 s | 0.555 | 0.265 |
| ④ `p1_track_only` | velocity | on | **4/20**（14 失败 + 2 infra ACTIVE） | 0.021 | 0.094（n=4） | 102.1 s | 0.077 | 0.097 |

> **注（窗口口径）**：`active 窗口均值` 为**全回合**（含失败的长超时）；③ 的 5 个超时拉高其均值。**成功回合**窗口：① **17.7 s** vs ③ **16.0 s**（相近），但 FCO 拍数 **51 vs 107**、率 **0.087 vs 0.184** ⇒ ③ 的优势**不是**"窗口更短/更长"，而是**同等窗口内机会拍更多**。

**析因对比**（逐 seed 配对差；DV = active FCO）：

| 对比 | n | 均值 | 95% CI | p(双侧) | 读法 |
|---|---|---|---|---|---|
| **指向主效应**（tracking=on）①−④ | 18 | **+0.0658** | [+0.0280, +0.1035] | **0.0019** | 指向带来收益 |
| **指向主效应**（tracking=off）③−② | 20 | **+0.1347** | [+0.0801, +0.1894] | **5.6e-05** | 指向带来收益（更强） |
| **跟踪主效应**（pointing=target）①−③ | 20 | **−0.0719** | [−0.1253, −0.0185] | **0.011** | **跟踪参考无收益、反而降低 FCO** |
| 跟踪主效应（pointing=velocity）④−② | 18 | +0.0006 | [−0.0201, +0.0214] | 0.949 | 无差异（该侧退化严重） |
| 交互 (①−④)−(③−②) | 18 | **−0.0667** | [−0.1300, −0.0034] | — | **负交互**：两项同时开时增益被抵消 |
| 对角总差 ①−② | 20 | +0.0628 | [+0.0333, +0.0924] | 0.0003 | 与 v5（+0.0804）同向 |

**结论（归因）**：

1. **收益主要来自"指向"**：无论是否跟踪，`target` 相对 `velocity` 都显著为正（+0.066 与 +0.135）——与机制预期一致：窄视场下**机头是否对准**决定目标能否被持续观测。
2. **"跟踪位置"在本场景无收益，且在指向开启时有害**：①−③ = **−0.072**（显著）。成功回合的窗口长度相近（17.7 vs 16.0 s），差异在**机会拍数**（51 vs 107）与**可见拍中包络占比**（0.187 vs 0.265）——跟踪改变了空中几何，使"可见"更少转化为"包络内"；**具体因果链（空中位置 → 融合目标估计 → 地面抵近/停留）未单独测量**，留作后续诊断。该结论**限 s15（静止目标）**，不能外推到移动目标。
3. **cell ④ 如预登记退化**：`velocity+on` 的 20 回合中 14 失败 + 2 infra ACTIVE——**C10 hold 自锁**再次被实证；其**成功回合** FCO 0.094（n=4）说明该组合并非"无能力"，而是**任务成功率崩塌**。⇒ 该 cell **不得**当作公平基线；"跟踪主效应"只按 pointing=target 一侧解读（本 ADR 已如此）。
4. **交互为负**：两项同时开启时，跟踪的负向与指向的正向部分抵消（−0.067）。
5. **成功率权衡（如实登记）**：① 19/20 捕获 vs ③ 15/20——**③ 的 FCO 优势伴随更低的任务成功率**；本消融的 DV 只有 FCO，**不** adjudicate 成功率。若以成功率为决策终点，须**新预注册**（[ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md) 纪律）。

**口径与限制（如实登记）**：
- `可见拍中包络占比` 是**条件于可见**的比率：两 cell 的可见时刻不同 ⇒ 该量的差异含**时间选择效应**，乘式恒等式成立但"包络项"只有**描述**意义，不作因果解读；
- 本分析为**估计型**（无门槛、无 verdict 变更）；若要把"**仅指向即可**"立为命题，须**新预注册（v7）**并重新规划 N；
- 全部 cell 仍为**显式 oracle 指向臂**（`target` 用真值方位）；结论限已验证通道族（s15 窄视场）。

## 备选

1. **两臂合并臂（同时改变两项）继续比较** —— 否决：不可归因（本 ADR 背景）。
2. **跳过 pilot 直接 N=20** —— 否决：cell ④ 有已知退化风险，且新臂未冒烟；pilot 成本低（≈30 min）。
3. **把地面也改为跟随感知目标** —— 本 ADR 不做：会改变 v5 起的固定条件，属**新命题**，须独立预注册。
4. **给 P2 B 臂加新机制（如更激进的高层策略）** —— 推迟：先诊断，再决定（用户裁定）。

## 后果

- **正面**：α 的两项贡献可归因；cell ④ 的退化风险被**预先登记**而非事后解释；P2 负向有归因路径；"不进 RL"成为显式裁定而非默认。
- **代价 / 限制**：新增 2 臂 ⇒ 每批 80 episodes（本机实测 **5.0 h**）；cell ④ 如预期退化（4/20 捕获），使"跟踪主效应"只能单侧解读。
- **未决 / 后续**：是否把"**仅指向即可**"立为新命题（须**预注册 v7** + 重新规划 N，且须处理成功率权衡）；P2 B 臂的重规划振荡是否属实现缺陷、是否加"pred 一致性"门；W7-c / W8 仍未开始。

## 参考

- 命题与纪律：[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)、[ADR-0082](0082-w7b-p2-arm-b-rule-high-level-first.md)、[ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md)
- 臂定义：[ADR-0073](0073-p1-arm-fairness-noncoop-fixed-trajectory.md)（`air_tracking_ref` + C10 hold 自锁实证）
- 上游结果：[ADR-0085](0085-w7b-p1-mechanism-activating-scenario-pilot.md)(h)(i)、[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(j)(l)
- 实现：[`w7b_arms_runner.py`](../../scripts/w7b_arms_runner.py)、[`w7b_p1_ablation.py`](../../scripts/w7b_p1_ablation.py)、[`run_logger.cpp`](../../src/agc_task/src/run_logger.cpp)