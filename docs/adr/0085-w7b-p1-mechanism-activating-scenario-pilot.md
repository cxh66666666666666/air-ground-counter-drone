# W7-b P1 机制激活重测：pilot 候选场景与**先定**的选场景规则

## Status

accepted

## Date

2026-10-23

## 背景

P1 v4 在 `s08_static_none_nominal_capture`（静态目标 / 无障碍 / 标称感知 / 固定目标）下判定为**证伪**（Δ=+0.0278、p=0.153；[ADR-0084](0084-w7b-p1-active-window-dv-recheck.md)(e)）。同日补记的**机制边界**（[ADR-0084](0084-w7b-p1-active-window-dv-recheck.md)(f)）给出机制级证据：该场景下 α（观测指向 + 跟踪参考）的因果链几乎被旁路——

- FCO 门 = **包络内 ∧ 目标可见**（[`run_logger.cpp`](../../src/agc_task/src/run_logger.cpp)）；包络占用由**地面车路径**决定，而 s08 的 `goal_follow_target=false` 使 goal = 目标真值 ⇒ **地面路径与感知无关**，两臂同源；
- 唯一受 α 影响的因子只剩"可见"，而 active 窗口可见率仅差 **6 pt**（0.932 vs 0.869）——因为目标 static、无遮挡、标称 FOV（100°/70° 与 70°/50°）。

⇒ 真正的待答问题是：**在具备机制激活条件的场景下，α 是否产生收益**。这**不是**"再跑一次同一个命题"（那是改 primary 之外的事后修补），而是**新命题、新固定条件**，须新预注册承载。

现存 12 个场景中**无**可跑的"更难捕获场景"：`s12`（linear + dynamic obstacle + capture）因 `unsupported:obstacle` 被 launch 拒绝；其余 capture 场景仅 `s08`。故需新增场景。

## 决策

### (a) 先 pilot 选场景，再写预注册 v5

新增两个**候选**捕获场景（轴组合均有承载者，见 [scenario_carriers.yaml](../../src/agc_task/config/scenario_carriers.yaml)），先以 **pilot（3 seeds × 2 臂）** 观测"机制是否激活 + 任务是否可解"，再据此**只选一个**场景进入 v5 的 N=20 确证。

| 候选 | 场景 | 相对 s08 的改动 | 预期机制激活点 |
|---|---|---|---|
| **A** | `s13_linear_none_nominal_capture` | 目标 **linear**（0.5 m/s）+ `goal_follow_target=true` | 目标运动 ⇒ 空中**跟随**有内容；地面 goal 由感知生成 ⇒ 地面路径依赖观测 |
| **B** | `s14_linear_none_fov_capture` | A + 感知 **fov**（hfov 30°/vfov 20°） | 再加**指向**决定可观测性（窄视场下机头是否对准目标成为主导因子） |

两候选**均**保留 capture 语义（`interception_authorized=true` + `tracking_milestone_is_terminal=false`，配置不变量由 [`test_scene_config_invariants.py`](../../src/agc_task/test/test_scene_config_invariants.py) 守护）。

### (b) **先定**的选场景规则（写入 **早于** pilot 执行；防止"挑一个出效果的"）

pilot = `p1_coop` / `p1_noncoop` 两臂 × seeds {0,1,2} × 候选 A、B（共 12 episodes）；**pilot 结果只用于选场景与估计 SD，不作为任何判定证据**（对齐 v3 的 `power_basis` 仅作规划的先例）。

按下列**机制层**判据选场景（**不**按 DV 效应量排序）：

1. **可解性**：协同臂 ≥2/3 seeds 达 `INTERCEPT_CONFIRMED`；且非协同臂 ≥1/3 seeds 达终态（避免"非协同臂完全失明"的**退化场景**——那种场景把 α 退化成"能否看见"的二元门，不再是"协同程度"的对照）。
2. **机制响应**：候选场景下，两臂的 **active 窗口可见率之差 ≥ 10 pt**（α 的作用路径确被激活），或非协同臂 active FCO 相对协同臂**下降 ≥ 30%**。
3. **优先级**：两候选都满足 → **取改动更小的 A**（少动一个轴）；仅 B 满足 → 取 B；都不满足 → **不**进入 v5，写"现场景族无法激活 α"的边界结论。

### (c) 之后的路径（规则先定）

- 选中场景 ⇒ 写 **预注册 v5**：`scene=<选中>`、两臂定义 / N=20 / α=0.05 / MDE（依 pilot 的 active-window 配对差 SD 规划，**不**因效应小而下调）在 run 前写定；P1 v1 / v4 结论**均不回溯改写**（它们是各自的固定条件结论）。
- v5 判定 = 双侧配对 t `p < α` **且** Δ ≥ MDE；证伪即"该机制激活场景下亦无收益"，一并入 W8。
- pilot 产物（含未选中候选）**全部保留**，作为场景选择过程的证据。

### (d) pilot 第 1 轮结果（2026-10-23）：**A / B 均不满足规则，且原因是可诊断的几何崩塌**

承载 run：`w7-rl-b/20261023-w7b-pilot-s13`、`w7-rl-b/20261023-w7b-pilot-s14`（各 2 臂 × seeds 0–2 = 6 episodes；headless / full 链）。

| 候选 | 协同臂终态 | 非协同臂终态 | active 可见率（协同 / 非协同） | 是否满足规则 |
|---|---|---|---|---|
| **A** `s13`（linear·nominal） | 1/3 `INTERCEPT_CONFIRMED` | 2/3 | 0.18–0.23 / 0.10–0.25 | **否**（可解性 1/3；无可见率差） |
| **B** `s14`（linear·fov） | **0/3**（全 `MISSION_FAILED`） | **0/3** | 0.11–0.13 / 0.02–0.04 | **否**（完全退化） |

**诊断（哪一环崩了）**：`perception_health` 显示两臂的感知拒绝**绝大多数是 `rmax`**（A 协同臂 seed0：`rmax=4198 / gate_total=5204 ≈ 81%`）——即飞机与目标距离长期 **> 12 m 量程**，与站距带 `safe_max = 15 m`（[`air.yaml`](../../src/agc_air/config/air.yaml)）上限**不匹配**。叠加 C10 的 **hold 语义**（`已获目标后参考 / 目标失效 → 显式 hold（不回落八字）`，日志逐条可见）：协同臂一旦在 >12 m 处丢失目标即**冻结**，而目标**持续直线远离** ⇒ 不可恢复 ⇒ 观察崩溃。非协同臂（`atr=off`，不订阅跟踪参考、不触发 hold）反而能按固定航迹飞 ⇒ **A 里非协同臂反而更常成功**。

⇒ 本轮 pilot 测到的**不是"α 无收益"**，而是"**在直线远离 + 量程/站距带不匹配 + C10 hold 自锁**下，两臂被同一个几何缺陷一起打盲"。该缺陷在 s08（静态目标）不显现（飞机能在带内稳定 ≤12 m），在**追随型**场景则具有**不对称性**（协同臂的参考本身就依赖 `/agc/targets`）。

### (e) 修订：第 2 轮 pilot（同规则，**最后一轮**）

为把"指向 / 跟踪"与"追不上 + 自锁"**解耦**，追加两个**静态目标**候选（无追随，规避 `rmax` 崩塌；难度只落在观测通道上）：

| 候选 | 场景 | 相对 s08 的改动 | 预期机制激活点 |
|---|---|---|---|
| **C** | `s15_static_none_fov_capture` | 感知 **fov**（hfov 30°/vfov 20°） | 目标出视场 ⇒ **机头是否对准**决定可观测性（窄视场 ≈ ±15°） |
| **D** | `s16_static_none_dropout_capture` | 感知 **dropout**（`drop_prob=0.3` + 成簇丢失） | 观测稀疏 ⇒ **持续跟踪**恢复更快、有效观测更多 |

**规则不变**（可解性 / 机制响应 / 优先级），仍**不**按 DV 效应量挑选；第 2 轮后无论结果如何**不再加候选**：满足 → 进 v5；不满足 → 写"现场景族（含静态窄视场与丢包）无法激活 α"的边界结论入 W8。

### (f) 第 2 轮结果与选场景（2026-10-23）：**选 C（`s15`）**

承载 run：`w7-rl-b/20261023-w7b-pilot-s15`、`w7-rl-b/20261023-w7b-pilot-s16`（各 2 臂 × seeds 0–2）。

| 候选 | 协同臂终态 | 非协同臂终态 | active 可见率均值（协同 / 非协同） | active FCO 均值（协同 / 非协同） | 规则判定 |
|---|---|---|---|---|---|
| **C** `s15`（static·fov） | **2/3** | **2/3** | **0.359 / 0.173（Δ≈+19 pt）** | 0.109 / 0.015 | **满足**（可解性 ✔；机制响应 ≥10 pt ✔） |
| **D** `s16`（static·dropout） | 2/3 | 3/3 | 0.979 / 0.979（Δ≈0） | 0.093 / 0.133 | **不满足**（active 可见率饱和，机制未激活） |

- **选 C**（唯一满足者；无需启用 tie-break）。D 的丢包在 **active 窗口**内几乎不降低可见率——`drop_prob=0.3` 被任务窗口内的多次重观测与 `visibility_max_age_s=0.5` 平滑，故 active 窗口的可见率与 FCO 均无差异。
- **如实登记（C 的启动 fragility）**：`s15` 协同臂 seed0 先经历 **2 次 infra（ACTIVE 超时）**，第 3 次为 mission `TIMEOUT`（active 可见率 0.003）——属启动期 infra/几何自锁，**不是**机制效应；pilot 的可解性按 2/3 计。该回合在 v5 中按预注册 `keep_all_failures_and_infra` **记 0 计入分母**（对协同臂不利，不会夸大效应）。
- **pilot 的定位**：仅用于选场景与规划（SD=0.141，n=3），**不**构成任何判定。

### (g) 下一步：预注册 v5（`s15`）

在 v5 run **之前**写定：`scene=s15_static_none_fov_capture`、两臂定义不变、primary=`fire_control_opportunity_rate_active`、N=20、α=0.05、**MDE=0.10**（沿用；pilot SD 仅用于把握度规划，不因 pilot 均值 ≈0.094 而下调）。v1 / v4 结论均保留为各自固定条件的历史，不回溯改写。

### (h) v5 执行结果（2026-10-23）：**未达预注册 MDE → 判证伪；但机制确已激活且效应方向正确、统计显著**

承载 run `w7-rl-b/20261023-w7-rl-b-p1v5-n20`（`backend:=full` / `s15_static_none_fov_capture` / seeds 0–19 / CRN 同 seed 配对 / 40 episodes）；判定产物同 run `metrics/p1_adjudication.json`（`prereg_version: 5`）。

| DV | 角色 | n 配对 | 剔除 | 均值配对差（协同 − 非协同） | 95% CI | 配对 t p | Wilcoxon p | 判定 |
|---|---|---|---|---|---|---|---|---|
| `fire_control_opportunity_rate_active` | **primary** | 20 | 0 | **+0.0804** | [+0.0281, +0.1328] | **0.0046** | 0.0022 | **Δ < MDE 0.10 ⇒ 按预注册判据不成立（证伪）** |
| `min_clearance_m` | secondary | 20 | 0 | −0.4125 | [−1.1180, +0.2929] | 0.236 | 0.330 | 不显著（仅报告） |
| `fire_control_opportunity_rate`（legacy fixed） | 描述 | 20 | 0 | +0.3319 | [+0.1919, +0.4719] | 8.6e-05 | 8.5e-04 | 仅描述，不参与判定 |

- **机制激活（实测强于 pilot）**：active 窗口可见率 **协同 0.473 vs 非协同 0.066（Δ≈41 pt）**；active FCO 均值 0.099 vs 0.019。→ [ADR-0084](0084-w7b-p1-active-window-dv-recheck.md)(f) 指认的"场景不足以激活 α"在 s15 已被解除：α 的作用路径（指向 / 跟踪 → 可观测性）确被激活。
- **臂画像**：协同 **15/20 `INTERCEPT_CONFIRMED`**（4 `MISSION_FAILED` + 1 重试耗尽的 infra）；非协同 **13/20**（7 `MISSION_FAILED`）。infra 重试 14 次（协同 9 / 非协同 5），终态 infra 1。
- **口径纪律**：primary 的失败 / infra / 缺值按 0 计入分母（`primary_honored=true`、剔除 0、零填 **12** = 协同 5 + 非协同 7）；N=20 / α / MDE / 臂定义**未改**。
- **结论（现行 P1 结论）**：在**机制已激活**的固定条件下，协同观测相对非协同观测的 **FCO 提升为 +0.080（绝对值）、CI 不含 0、p=0.005**——**方向与假设一致**；但按**当日口径**（v5：`p<α AND Δ≥MDE`）"未通过"。**三态读法（[ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md)，2026-10-23 补记）**：CI 覆盖 MME 0.10 ⇒ **`inconclusive`（未通过门槛）**——未支持"收益 ≥ 10 pt"，亦未排除之。两条读法都须如实并列，不得只报其一：**不得**称"协同无效"（效应显著且方向正确），**也不得**称"协同收益成立"（未达门槛）。
- **与 v4 的关系**：v4（s08，Δ=+0.028、p=0.153）与 v5（s15，Δ=+0.080、p=0.005）**不矛盾**——前者场景未激活机制（[ADR-0084](0084-w7b-p1-active-window-dv-recheck.md)(f)），后者激活后效应量级上升且显著；固定条件不同，两者各自独立成立。
- **后续（[ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md)(d)；须**新**预注册）**：要取得决断性结论（`supported`/`falsified`）须按"门槛 − 效应余量"规划样本量——MME=0.10、σ≈0.112 时：余量 0.03 → N≈109、余量 0.02 → N≈246；本 ADR 不代做该决定，也**不得**事后改 v5 阈值。**若考虑降低 MME**：版本顺延为 **v7**（ADR-0085 原文所称"v6"已被 [ADR-0086](0086-decision-rule-three-state-ci-versus-mme.md) 用作三态规则），且须满足 [ADR-0088](0088-mme-change-governance-independent-justification-and-seed-isolation.md)——**独立工程依据**（不得以本次观测 0.0804 倒推）、**新种子 seeds ≥20**、**按实际判定规则重算 N**；"N=20 可检测 ≈0.070"**不**保证能通过"均值 ≥0.08"的规则。

### (i) FCO **描述性拆解**（2026-10-23；**不**参与判定、**不**改 verdict）

**动机**：判定口径把失败 / infra 回合的 primary 记 0（`keep_all_failures_and_infra`，本 run 共 **12** 个零填值）⇒ 判定量混合了"**任务成功**"与"**过程中的火控机会**"。**超时 (=失败) 不等于此前没有火控机会**，故须把两者拆开，并解释"可见率 +41 pt 而计分只 +8 pt"。

**产物**：[`scripts/w7b_p1_decompose_fco.py`](../../scripts/w7b_p1_decompose_fco.py) 读既有产物（`arms.json` + 各 episode `summary.json` + 逐 episode 日志），输出 run 目录下 `metrics/p1_fco_decomposition.{json,md}`（逐回合表 + 聚合表 + 分解）。

**乘法分解**（active 窗口；`FCO_rate = vis_rate × env_occ|vis`，`env_occ|vis` = **可见拍中落入交战包络**的比例）：

| 量（池化，按拍） | 协同 | 非协同 | Δ |
|---|---|---|---|
| 可见率 `vis_rate` | 0.327 | 0.065 | **+0.262** |
| 可见拍中包络占比 `env_occ\|vis` | 0.134 | **0.227** | **−0.094** |
| FCO 率（池化） | 0.044 | 0.015 | +0.029 |

贡献分解（恒等式 `Δfco = env_n·Δvis + vis_n·Δenv + Δvis·Δenv`）：**可见项 +0.0597 ｜ 包络项 −0.0061 ｜ 交互项 −0.0246**。

**配对差三口径**（同 seed，协同 − 非协同）：

| 口径 | n 配对 | 均值配对差 | 读法 |
|---|---|---|---|
| **scored**（判定用，失败记 0） | 20 | **+0.0804** | v5 判定值（[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(l)） |
| real（全回合真实 FCO） | 19 | +0.0846 | 描述：**零填压缩 ≈ 0.004**（很小） |
| success_only（两臂皆成功） | 9 | +0.1297 | 描述；**有选择偏差**（条件于成功） |

**逐回合（失败回合真实值，节选）**：协同 seed5 可见率 **0.964** 而 FCO 仅 0.012（可见拍中包络占比 1.3%）；协同 seeds 9/11 与 infra 回合作真"失明"（可见率 ≤0.003）；非协同失败回合可见率 0.014–0.089、真实 FCO 0.000–0.008。

**结论（为何 +41 pt 可见率只换来 +8 pt 计分）**：

1. **包络占用是约束项**：协同臂可见拍更多，但**可见拍中落在包络内的比例更低**（13.4% vs 22.7%）⇒ 可见性**不等比例**转化为火控机会（交互项 −0.0246 即此项的放大效应）；
2. **池化稀释**：失败回合时长最长（130–165 s）且机会极少（真实 FCO ≤0.012），按拍池化后 Δ 降到 +0.029；
3. **零填影响很小**（+0.0804 vs +0.0846）⇒ 判定值与"全回合真实"几乎同量级，**判定未被零填显著扭曲**（此前疑虑经数据排除）；
4. **仅成功回合**的差 +0.130 说明：在"都成功"的回合里协同的机会优势更大，但该口径有选择偏差，**不得**当判定证据。

**新登记的候选缺陷（不静默）**：协同 seed5 在该 run 中 **2 次开火**（t=32.53 首发 `SHOT_MISS`，miss 0.543 m ≈ 半径 0.5；t=36.03 第二发 `SHOT_FIRED`）后，**第二发无 `SHOT_HIT`/`SHOT_MISS` 结果行**，此后火控窗口未再开启，最终 `MISSION_FAILED(TIMEOUT)`（日志：`..._p1v5-n20/log/ep_p1_coop_s15_static_none_fov_capture_5.log`）。**待查**（发射-结局配对缺失 / 冷却或窗口状态机卡住），复核前不改动任何口径。

**既有产物缺口（如实标注）**：①"**首次机会拍**时刻"（tick 级）与②"**无条件包络占用率**"（不与可见取交）均**未记录**（本表给"可见拍中包络占比"与"首次**开火**时刻"作代理）；如需，须给 [`run_logger.cpp`](../../src/agc_task/src/run_logger.cpp) 增诊断计数（**仅新增字段**，不改既有定义；后续 run 生效）。

## 备选

1. **不 pilot，直接对 A、B 各跑 N=20** —— 否决：成本翻倍（约 4.5 h）；且盲选场景可能落到退化场景（非协同臂失明）。
2. **不新增场景，用轻量栈或改 s08 参数制造难度** —— 否决：轻量栈无空中域（无 α 可言）；改 s08 参数等于改既有场景的语义，会破坏 W6 回归证据。
3. **只加 B（一次到位加窄视场）** —— 否决：无法区分"目标运动"与"指向"两因素的激活贡献，且 B 可能退化；A 的存在是必要的对照。

## 后果

- **正面**：选场景依据**机制层**判据且**先于** pilot 写定，可避免"挑出效果"的嫌疑；pilot 成本低（约 30–40 min）；无论结论正负，W7-b 都获得"α 在机制激活场景下是否有效"的可判定答案。
- **代价 / 限制**：新增 2 个场景属 W6 配置扩面（轴组合已声明、承载者齐备，需本 ADR 留痕）；pilot 的 3 seeds 不构成任何统计结论。
- **未决 / 后续**：pilot 结果与选中场景（本节待填）；v5 预注册；P2 消融与 W7-c 仍未执行。

## 参考

- 前序：[ADR-0084](0084-w7b-p1-active-window-dv-recheck.md)（P1 v4 复核 + 机制边界）、[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(k)、[ADR-0073](0073-p1-arm-fairness-noncoop-fixed-trajectory.md)（α 的两项差异）、[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)（命题 / 判定纪律）
- 场景与承载：[`scenarios.yaml`](../../src/agc_task/config/scenarios.yaml)、[`scenario_carriers.yaml`](../../src/agc_task/config/scenario_carriers.yaml)、[`perception_sensors*.yaml`](../../src/agc_perception/config/)
- 预注册：[`w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)