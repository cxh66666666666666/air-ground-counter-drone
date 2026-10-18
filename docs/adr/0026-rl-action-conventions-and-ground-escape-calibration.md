# RL 动作方向 / 量纲约定与地面逃逸能力标定（C6 · R-011）

## Status
accepted

## 背景与决策

评审风险 **R-011（动作与能力）**：`agc_rl` 的动作映射 `interface.action_to_twist` 存在**符号反转**与**量纲混用**，
且偏移参数（`min_offset` / `max_offset`）**未形成对差速车真实横向逃逸的实测**——C6（契约 [#hard-constraints](../../docs/ROS2迁移交接文档.md#hard-constraints)「安全滤波器参数须用真实逃逸能力」）不能由参数名或理想横移推出。
本 ADR 记录三项决策：**(a) 转向符号约定**、**(b) 净空量纲一致化**、**(c) C6 逃逸能力标定**。

### (a) 转向符号：对齐 ROS 2 约定，left → 正 `angular.z`

**先学后做（取证）**：ROS 2 的转向约定为车体系 **+ 左（逆时针）= 正 `angular.z`**（z 向上右手定则，REP-103）——
本机 `sensor_msgs/LaserScan.msg` 明确「the positive Z axis (counterclockwise, if Z is up)」；契约「Nav2 吸收点」亦
已定 `twist 的唯一转向约定（正 angular.z = 逆时针 = 左）`。

**现状缺陷（静态求值复核）**：旧 `action_to_twist(action, heading_err, forward_min, clearance_norm, params)` 在
`heading_err=0`、正前净空充足时，`left` 得 `(v=1.0, w=-0.36)`、`right` 得 `(v=1.0, w=+0.36)`——**与命名相反**
（`w = k_heading·heading_err - k_lateral·lateral`，而 `left` 取 `lateral=+magnitude`）。

**决策**：`left` / `left_fast` → **正 `angular.z`（左转）**；`right` / `right_fast` → **负 `angular.z`（右转）**。
航向项与横向项**同号**：

```
w = k_heading*heading_err + k_lateral*lateral     # 正 lateral = 左 = 正 angular.z
```

### (b) 量纲一致：净空与偏移同为米，且与 `lidar_max_range` 解耦

**缺陷**：旧实现把无量纲的 `forward_min`（`range / lidar_max_range`，∈[0,1]）与以米为单位的
`clearance_norm`（旧取 `max_offset = 1.2`）混算 `ratio=(clearance_norm-forward_min)/clearance_norm`——
改变 `lidar_max_range` 会**无理由**改变动作强度。

**决策**：净空改为**米**并引入参考值 `clearance_ref_m`（config，米），二者**同量纲**：

```
ratio = clamp((clearance_ref_m - forward_clearance_m) / clearance_ref_m, 0, 1)
magnitude = min_offset + ratio*(max_offset - min_offset)
forward_clearance_m = forward_clearance(sectors, ..., half) * lidar_max_range   # 归一值乘量程还原为米
```

参数经 config 单一注入（契约 [#params](../../docs/ROS2迁移交接文档.md#params)）；`action_to_twist` **不再**接触任何量程参数。

### (c) C6 逃逸能力标定：实测差速车在反应窗口内的横向可达

**口径**：反应窗口 `T = 2.0 s` 取 Nav2 `time_before_collision` 上界（[ADR-0021](./0021-ground-safety-thresholds-and-platform-recalibration.md)
的 `t_before ∈ [1.2, 2.0] s`）。理论界（常 yaw 率圆弧）：`lateral(T) ≈ (v/ω)*(1 - cos(ω*T))`。

**实测（GUI，`agc_world`，硬约束 C3）**：车（Husky 级 4 轮滑移差速，`v_max = 1.5`、`ω_max = 2.0`）沿 +x 达稳态速后，
持续给 `action_to_twist` 输出（车前净空置 0 → `magnitude = max_offset` 满偏），测 `T` 内 odom 世界 y 位移与 yaw：

| 配置 | 动作 | 世界 yaw Δ | 横向位移 (m) |
|---|---|---|---|
| `max_offset = 1.2`（标定前） | left(v=1.0) | +1.56 ~ +1.65 | +0.99（3 次一致） |
| | left_fast(v=1.3) | +1.23 ~ +1.64 | +0.95 ~ +1.29 |
| | right / right_fast | 负 | -0.96 ~ -1.07 |
| `max_offset = 1.0`（标定后，最终） | left / right | ±1.24 ~ ±1.31 | ±0.88 ~ ±1.08 |
| | left_fast / right_fast | ±1.17 ~ ±1.23 | ±0.89 ~ ±1.29 |

- **`max_offset = 1.2` 不可达**：3 次采样每轮最大横向可达 `1.065 / 0.997 / 1.293 m`，多数 `< 1.2`；常速左转稳定 ~0.99 m。
- **`max_offset = 1.0` 可达**：4 次采样（含终跑）每轮最大横向可达 `1.043 / 1.287 / 1.083 / 1.017 m`，**均 ≥ 1.0 m**。
- **决策**：**`max_offset` 由 `1.2` 下调至 `1.0 m`**（`rl.yaml` 单一注入）；`min_offset = 0.3 m` 不变。
- **物理解释**：实测横向 ≈ 理想圆弧界的 ~0.72×——4 轮滑移底盘（横向 μ2=0.2，摩擦面允许擦地滑移）转弯时**侧滑削刮**，
  实际横向位移小于理想滚动圆弧；故**不得**用理想弧模型推定。`max_yaw_rate = 1.0` 未构成瓶颈（本测 `w = k_lateral*max_offset = 0.8 < 1.0`，未截）。
- **方向闭环（R-011 Scenario 3）**：所有采样中 `left`/`left_fast` → 世界 **yaw 增**且横向 **y 为正**；`right`/`right_fast` → 反向——
  ROS 约定经**安全层 + 唯一执行中继**端到端成立、可复现。

## Considered Options

- **保留 `max_offset = 1.2`（或按理想弧/名称推定）** —— 被否：实测多数不可达，属「命名 1.2 m 横移但差速车实际不可达」的失配，违 C6。
- **提高 `k_lateral`（不降 `max_offset`）以榨取更大横向** —— 被否：超 R-011 范围（本项只修动作语义与标定），且侧滑削刮下增益有限、易致失稳。
- **把净空参考继续取无量纲值** —— 被否：量纲混用，动作强度随雷达量程无理由变化（R-011 缺陷本体）。
- **动作符号交由下游纠正** —— 被否：学习接口（W7 训练）一旦冻结会学会补偿错误标签，须在接口层修。

## Consequences

- `src/agc_rl/agc_rl/interface.py`：`action_to_twist` 改签名为 `(action, heading_err, forward_clearance_m, params)`；
  符号改正（`+ k_lateral*lateral`）；量纲一致（`clearance_ref_m`，米）。
- `src/agc_rl/agc_rl/inference_node.py`：移除 `self.clearance_norm = self.control['max_offset']` 误用；
  新增 `control.clearance_ref_m`（默认 1.0）；`_tick` 计算 `forward_clearance_m = forward_clearance(...) * lidar_max_range` 后传入。
- `src/agc_rl/config/rl.yaml`：新增 `control.clearance_ref_m: 1.0`（米）；`max_offset: 1.2 -> 1.0`（C6 标定值）。
- `src/agc_rl/test/test_interface.py`：新增最小单元断言（符号 / 符号一致 / 量纲解耦）。
- `scripts/run_w5_r011.sh`、`scripts/w5_r011_check.py`：新增动作方向 / 逃逸标定 GUI 验收脚本。
- **契约 / 冻结消息 / 服务 ABI 未改**；唯一执行口语义未改（无第二 ROS 命令者）；RL 节点仍未写 `/agc/ground/cmd_vel`。

## Evidence

- **单元（R-011 符号 + 量纲解耦）**：`python3 -m pytest src/agc_rl/test/ -q` → **11 passed**（`test_interface.py` 新增 7 例）。
- **C6 逃逸 / 方向（GUI 实跑）**：`scripts/run_w5_r011.sh gui <run_id>` →
  - 规范产物（最终配置 `max_offset=1.0`，PASS）：`experiments/w5-ground-nav-safety/20261015-w5-r011-0/metrics/r011.json`；
  - 标定前（`max_offset=1.2`，FAIL）：`experiments/w5-ground-nav-safety/20261015-w5-r011-precalib-0/`；
  - 追加采样：`experiments/w5-ground-nav-safety/20261015-w5-r011-1|2|3/metrics/r011.json`。
- **静态取证（ROS 约定）**：`/opt/ros/jazzy/share/sensor_msgs/msg/LaserScan.msg`（positive Z = counterclockwise）；REP-103。

## 表述纪律

此后 RL 动作的左右方向与净空参考一律引用本节（ADR-0026）；`max_offset` 一律引用本 ADR 的标定值（1.0 m），
**不得**再以「名称 / 理想横移」推定横向逃逸，亦**不得**在文档中复述契约硬约束（只引用 [#hard-constraints](../../docs/ROS2迁移交接文档.md#hard-constraints)）。
