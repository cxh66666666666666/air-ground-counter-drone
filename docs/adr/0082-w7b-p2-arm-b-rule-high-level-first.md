# W7-b P2 臂 B 先以规则高层承载（职责分层优先，训练 RL 后置）

## Status

accepted

## Date

2026-10-21

## 背景

[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)(e)(f)(h) 把 P2 定义为「RL 职责分层」对照：
臂 A = c2（RL 直接出 `cmd_vel`），臂 B = 高层决策 + 成熟低层，等预算下臂 B 显著优于臂 A 则职责升级成立。
同 ADR (j) 又规定**离线 / 规则优先**：先建离线数据集与规则臂，**仅当**规则基线在各消融下**不达出口**时才引入在线 RL。

现状（[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(c)(f)）：

- 两臂均已达 `TRACKING_SUCCESS`，但**判定未执行**；
- 臂 B 的 [`high_level_node.py`](../../src/agc_rl/agc_rl/high_level_node.py) 高层为**规则实现**（目标线性外推 + `lead_s`），非已训练 RL；
- 臂 A 的 `command_source=rl` 同样是确定性 JSON 规则策略，项目尚无训练管线 / 奖励 / 训练预算。

因此必须先裁定：P2 v1 用现有规则高层，还是先训练一个高层 RL。

## 决策

### (a) P2 v1 使用规则高层

P2 确证对比的臂 B **使用现行规则高层**（目标预测拦截点 + 成熟低层 `Navfn + 自研 LQR`），
**不先训练 RL**。理由：P2 的自变量是**职责分层**（命令权在高层还是端到端），不是学习算法；
两臂训练 / 调参预算均为 0 时，该自变量才可干净识别。

### (b) 结论域收紧

P2 结论**只**覆盖「职责分层 vs c2」，**不得**表述为「训练 RL 优于 c2」或「高层 RL 成立」。
产物 `arm` 标识沿用 `p2_rl_high`（保持 run 身份与既有证据可对齐），但报告 / 图表 / 裁定文档
**必须**标注「规则高层（ADR-0082）」。

### (c) 「等预算」口径

- **训练 / 调参预算**：两臂均为 0（无学习权重、无逐臂调参）。
- **评估预算**：同场景 / 同 seed 集合 / 同 timeout / 同 episode 数（沿用预注册）。
- **推理算力**：臂 B 多一层规划 + LQR，属**架构成本**，如实单列报告；**不**声称等算力。

### (d) 训练 RL 的后置触发条件

仅在以下任一条件满足时，才启动高层 RL 训练，并须**新预注册版本** + 两臂同等训练 / 调参预算：

1. 规则高层在 P2 或消融中**不达任务出口**（对齐 ADR-0070(j)）；
2. 后续 workstream 明确新的「学习 vs 规则」命题。

**禁止**静默把臂 B 替换为训练策略；替换即新臂、新预注册、新证据。

## 备选

1. **先训练高层 RL** —— 否决：违反 ADR-0070(j) 的规则优先顺序；无训练管线；「训练 vs 规则」使等预算无法定义；会把学习效应混入职责分层自变量；延迟 P2 判定。
2. **两臂都训练 RL** —— 否决：成本更高，训练方差 / 收敛差异会污染职责分层自变量，且超出 W7-b 范围（ADR-0070(j) 不锁网络 / 奖励 / 步数）。
3. **直接把在线主控改为分层** —— 否决：裁定前须遵守 c2（[ADR-0004](0004-rl-primary-controller-nav2-safety-layer.md)），只能在对照臂上做。

## 后果

- **正面**：P2 的臂 B 身份已定，并已按预注册 v2 执行 N=20 判定（**证伪、保留 c2**，[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(i)）；P2 确证走 **full 链 + 固定协同**（轻量栈无空中域，不能承载确证）；结论边界清晰；训练 RL 作为独立后续命题不被静默夹带。
- **代价 / 限制**：本轮**不能**回答「训练 RL 是否优于 c2」；该问题显式后置到 (d) 的触发条件。
- **未决**：P2 的固定条件 / DV 操作化 / 等预算口径 / 阈值已由**预注册 v2** 写定，见 [`w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)（`version: 2`；P1 仍按 v1 口径，不回写）。为避免复述，具体字段与数值以该文件为单一来源。
- **文档**：[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md) 的「高层策略由规则升级为 RL」未决项由本 ADR 收敛为「P2 v1 不升级」。

## 参考

- 命题与判定：[ADR-0070](0070-w7b-research-propositions-and-rl-role-adjudication.md)(e)(f)(h)(j)
- 臂证据与未决项：[ADR-0071](0071-w7b-adjudication-record-and-pending-arms.md)(c)(f)
- 现行主控：[ADR-0004](0004-rl-primary-controller-nav2-safety-layer.md)
- 预注册：[`src/agc_rl/config/w7b_prereg.yaml`](../../src/agc_rl/config/w7b_prereg.yaml)
- 臂实现：[`high_level_node.py`](../../src/agc_rl/agc_rl/high_level_node.py)
