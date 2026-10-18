"""RL 接口契约冻结（W5-b · spec R6）。

本模块是 `agc_rl` 的**接口真源**：观测（obs）空间与动作（action）空间的唯一冻结定义。
节点（`inference_node`）与策略（`policy`）**只**经本模块构造 obs / 解释 action，
不得各自另立维度或语义（契约 [参数入口](../../docs/ROS2迁移交接文档.md#params)：由 config 单一注入）。

对齐旧树行为事实（只读对齐 WHAT，不复制代码）：
`reference/old_ros1/src/rl_ground_avoidance/scripts/ground_env.py` 的
`get_observation()`（`num_sectors + 2 + 1 + N_ACTIONS` 维）与 `ACTION_NAMES`（5 离散动作）。
本项目为 ROS2 + 真实物理重写：obs 的扇区改为**由 `/agc/ground/scan` 实时激光**解析（旧树用虚拟激光），
action 改为在节点内直接映射为 `cmd_vel`（RL 直接出 `cmd_vel`，见 ADR-0004）。

obs 维度（总 `obs_dim = num_sectors + 3 + n_actions`）：
  [0, num_sectors)          扇区最小量程归一（range / lidar_max_range，clamp [0,1]，1.0 = 无遮挡；
                            R-010：NaN / 未知 → 0.0 保守按遮挡，+inf 合法无回波 → 无遮挡）
  [num_sectors + 0]         车体 world x / pose_norm_m，clamp [-2, 2]
  [num_sectors + 1]         车体 world y / pose_norm_m，clamp [-2, 2]
  [num_sectors + 2]         到目标距离归一（min(dist / track_norm_m, 1)）
  [num_sectors + 3, ...)    上一动作 one-hot（n_actions 维）

action 空间：`n_actions = 5` 离散
  0 沿原路径 · 1 左移 · 2 右移 · 3 左移并加速 · 4 右移并加速
"""

import math

N_ACTIONS = 5
ACTION_NAMES = {
    0: "along_path",
    1: "left",
    2: "right",
    3: "left_fast",
    4: "right_fast",
}


def obs_dim(num_sectors):
    """观测维度：扇区 + 位姿(2) + 跟踪误差(1) + 上一动作 one-hot。"""
    return int(num_sectors) + 3 + N_ACTIONS


def sectors_from_angles(angles, ranges, num_sectors, max_range):
    """按逐束角度（rad，车体系，0=正前、逆时针为正）归并扇区最小量程归一值。

    扇区覆盖整圈（每扇区 `2π / num_sectors`），索引 0 为车头正前方。

    读数分类（R-010 失效语义；方向一律取**保守**）：
      * **合法回波**（有限且 `> 0`）→ 归一量程（`1.0` = 无遮挡）。
      * **合法无回波**（`+inf`；契约内自滤把自返回置 inf，见 `scan_self_filter`）→ 该束**确证无遮挡**，
        不改变扇区下限。
      * **未知 / 无效**（`NaN`、`-inf`、`<= 0`、`None`）→ **不得**当作无遮挡，按「可能有遮挡」计（归一 `0.0`）。
    """
    out = [1.0] * num_sectors
    width = 2.0 * math.pi / num_sectors
    for angle, rng in zip(angles, ranges):
        idx = int(math.floor(((angle % (2.0 * math.pi)) + width * 0.5) / width)) % num_sectors
        if rng is None:
            out[idx] = 0.0                      # 未知读数 → 保守按遮挡
            continue
        if math.isinf(rng):
            if rng > 0.0:
                continue                        # 合法无回波 → 确证无遮挡
            out[idx] = 0.0                      # -inf 非法 → 保守按遮挡
            continue
        if math.isnan(rng) or rng <= 0.0:
            out[idx] = 0.0                      # NaN / 非正 → 未知 / 无效 → 保守按遮挡
            continue
        norm = min(max(rng / max_range, 0.0), 1.0)
        if norm < out[idx]:
            out[idx] = norm
    return out


def scan_has_valid_reading(ranges):
    """扫描是否含至少一个**可用**读数（合法回波或合法无回波 `+inf`）。

    全为 `NaN` / `<= 0` / 非法值时返回 `False`——此扫描**不得**据以推断「无遮挡」，
    推理节点据此判定 scan 输入**无效**并失效停车（R-010）。
    """
    for rng in ranges:
        if rng is None or math.isnan(rng):
            continue
        if math.isinf(rng):
            if rng > 0.0:
                return True
            continue
        if rng > 0.0:
            return True
    return False


def build_observation(sectors, pose_x, pose_y, dist_to_goal, last_action,
                      num_sectors, pose_norm_m, track_norm_m):
    """组装冻结的 obs 向量（顺序见模块 docstring，不得变更）。"""
    obs = list(sectors[:num_sectors])
    obs += [1.0] * max(0, num_sectors - len(obs))
    obs.append(min(max(pose_x / pose_norm_m, -2.0), 2.0))
    obs.append(min(max(pose_y / pose_norm_m, -2.0), 2.0))
    obs.append(min(dist_to_goal / track_norm_m, 1.0))
    one_hot = [0.0] * N_ACTIONS
    if 0 <= last_action < N_ACTIONS:
        one_hot[last_action] = 1.0
    obs += one_hot
    return obs


def forward_clearance(sectors, num_sectors, half_width_sectors):
    """车前方向净空：正前方 ± half_width_sectors 扇区的最小归一量程。"""
    if not sectors:
        return 1.0
    values = []
    idx = (-half_width_sectors) % num_sectors
    for _ in range(2 * half_width_sectors + 1):
        values.append(sectors[idx % num_sectors])
        idx += 1
    return min(values) if values else 1.0


def side_clearance(sectors, num_sectors, side):
    """一侧（side>0 左 / side<0 右）净空：四分之一圈方向的扇区最小归一量程。"""
    if not sectors:
        return 1.0
    idx = int(round((num_sectors / 4.0) * (1 if side > 0 else -1))) % num_sectors
    out = sectors[idx]
    for d in (-1, 1):
        out = min(out, sectors[(idx + d) % num_sectors])
    return out


def action_to_twist(action, heading_err, forward_clearance_m, params):
    """把离散 action 映射为车体系 `(v, w)`。

    **量纲（一致为米）**：`forward_clearance_m` 为车前方向**净空距离（米）**，
      `params['clearance_ref_m']`（米）为「净空充足」参考值——**两者同为米**，
      故动作强度与 `lidar_max_range`（归一用量程，无量纲）**解耦**：
      同一真实净空下改 `lidar_max_range` 不改变 `(v, w)`。

    **符号（ROS 2 `geometry_msgs/Twist` 约定，REP-103）**：车体系 +左（逆时针，z 向上右手定则）
      对应 **正 `angular.z`**（左转为正）。故：
      * `left` / `left_fast` → **正 `angular.z`**（左转）；
      * `right` / `right_fast` → **负 `angular.z`**（右转）。
      航向项（`k_heading·heading_err`）与横向项（`k_lateral·lateral`）**同号**，统一左 / 右约定：
      正 lateral = 左 = 正 `angular.z`。

    action 语义（见模块 docstring）：0 沿原路径 · 1 左移 · 2 右移 · 3 左移并加速 · 4 右移并加速。

    `params` 键（由 config 单一注入，见 `rl.yaml`）：
      reference_speed / fast_speed / min_offset / max_offset / clearance_ref_m（米）
      k_heading / k_lateral / max_yaw_rate
    行为对齐旧树 `ground_env.py` 行为事实（WHAT）：动作偏置为横向偏移量 → diff-drive 速度。
    """
    # 净空越小于参考值 → 偏移越大（夹在 [min_offset, max_offset]）；净空与参考同为米。
    ref = max(params['clearance_ref_m'], 1e-6)
    ratio = min(max((ref - forward_clearance_m) / ref, 0.0), 1.0)
    magnitude = params['min_offset'] + ratio * (params['max_offset'] - params['min_offset'])

    speed = params['reference_speed']
    lateral = 0.0
    if action == 1:
        lateral = +magnitude      # left
    elif action == 2:
        lateral = -magnitude      # right
    elif action == 3:
        lateral = +magnitude      # left_fast
        speed = params['fast_speed']
    elif action == 4:
        lateral = -magnitude      # right_fast
        speed = params['fast_speed']

    v = speed * max(0.0, math.cos(heading_err))          # 航向偏差大则减速（旧树同口径）
    # 正 lateral = 左 = 正 angular.z；航向项与横向项同号（统一左 / 右约定）。
    w = params['k_heading'] * heading_err + params['k_lateral'] * lateral
    w = min(max(w, -params['max_yaw_rate']), params['max_yaw_rate'])
    return v, w
