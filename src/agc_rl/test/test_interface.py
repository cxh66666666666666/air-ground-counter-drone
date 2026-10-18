"""R-011 判据：`interface.action_to_twist` 符号（ROS 约定）与量纲一致 / 量程解耦。

判据（spec「车体系动作方向与量纲一致性（R-011）」）：
  * 符号：`left` / `left_fast` → 正 `angular.z`（左转）；`right` / `right_fast` → 负（右转）；
    航向项（`k_heading·heading_err`）与横向项（`k_lateral·lateral`）同号（统一左 / 右约定）。
  * 量纲：净空与偏移同为**米**；同一真实净空下 `lidar_max_range` 10 → 20，动作强度**不变**（解耦）。
"""

import pytest

from agc_rl import interface

# 与 `config/rl.yaml` 的 control 段一致（不读文件，避免依赖安装路径）
PARAMS = {
    'reference_speed': 1.0,
    'fast_speed': 1.3,
    'min_offset': 0.3,
    'max_offset': 1.2,
    'k_heading': 1.5,
    'k_lateral': 0.8,
    'max_yaw_rate': 1.0,
    'clearance_ref_m': 1.0,
}


@pytest.mark.parametrize('action', [1, 3])
def test_left_actions_positive_yaw(action):
    """left / left_fast：heading_err=0 → 正 angular.z（左转）。"""
    _v, w = interface.action_to_twist(action, 0.0, PARAMS['clearance_ref_m'], PARAMS)
    assert w > 0.0


@pytest.mark.parametrize('action', [2, 4])
def test_right_actions_negative_yaw(action):
    """right / right_fast：heading_err=0 → 负 angular.z（右转）。"""
    _v, w = interface.action_to_twist(action, 0.0, PARAMS['clearance_ref_m'], PARAMS)
    assert w < 0.0


def test_heading_and_lateral_sign_consistency():
    """航向项与横向项**同号**：正航向误差与 left 一致（更左），负航向误差与 right 一致。"""
    _v, w_left_h_pos = interface.action_to_twist(1, +0.2, PARAMS['clearance_ref_m'], PARAMS)
    _v, w_left_h_zero = interface.action_to_twist(1, 0.0, PARAMS['clearance_ref_m'], PARAMS)
    assert w_left_h_pos > w_left_h_zero > 0.0

    _v, w_right_h_neg = interface.action_to_twist(2, -0.2, PARAMS['clearance_ref_m'], PARAMS)
    _v, w_right_h_zero = interface.action_to_twist(2, 0.0, PARAMS['clearance_ref_m'], PARAMS)
    assert w_right_h_neg < w_right_h_zero < 0.0

    # 纯航向项（沿原路径 action=0，无横向）：正航向误差 → 正 angular.z
    _v, w_h_only = interface.action_to_twist(0, +0.2, PARAMS['clearance_ref_m'], PARAMS)
    assert w_h_only > 0.0


def test_magnitude_bounds_in_meters():
    """净空充足 → min_offset；净空为 0 → max_offset（同量纲、夹取）。"""
    _v, w_full = interface.action_to_twist(1, 0.0, PARAMS['clearance_ref_m'], PARAMS)
    assert abs(w_full - PARAMS['k_lateral'] * PARAMS['min_offset']) < 1e-9
    _v, w_zero = interface.action_to_twist(1, 0.0, 0.0, PARAMS)
    assert abs(w_zero - PARAMS['k_lateral'] * PARAMS['max_offset']) < 1e-9


def test_clearance_decoupled_from_lidar_max_range():
    """同一真实净空（米）下，lidar_max_range 10 → 20，动作输出逐一致（量纲解耦）。

    模拟推理节点链路：`sectors_from_angles(range, max_range)` 归一回 [0,1]，
    再乘回 `max_range` 还原为**米**，故不同量程下`forward_clearance_m` 相同 → `(v, w)` 相同。
    """
    d_m = 0.5  # 同一真实车前净空（米）
    outputs = []
    for rng in (10.0, 20.0):
        sectors = interface.sectors_from_angles([0.0], [d_m], 12, rng)
        forward_clearance_m = interface.forward_clearance(sectors, 12, 1) * rng
        assert abs(forward_clearance_m - d_m) < 1e-9  # 还原为一致的米值
        outputs.append([interface.action_to_twist(a, 0.0, forward_clearance_m, PARAMS)
                        for a in (1, 2, 3, 4)])
    assert outputs[0] == outputs[1]
