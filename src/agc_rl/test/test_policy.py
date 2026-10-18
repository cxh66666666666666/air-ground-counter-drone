"""W5-b R6 判据：RL 接口冻结 + 策略可加载（改策略路径 → 行为变化可复现）。"""

import os

from agc_rl import interface
from agc_rl.policy import load_policy

POLICIES = os.path.join(os.path.dirname(__file__), '..', 'policies')


def _obstacle_obs(forward_blocked, left_clear, right_clear, num_sectors=12):
    sectors = [1.0] * num_sectors
    # 正前方扇区（索引 11/0/1，见 interface.forward_clearance）
    for i in (-1, 0, 1):
        sectors[i % num_sectors] = forward_blocked
    sectors[3] = left_clear      # 左侧（interface.side_clearance(+1)）
    sectors[9] = right_clear     # 右侧（interface.side_clearance(-1)）
    return interface.build_observation(sectors, 0.0, 0.0, 5.0, 0, num_sectors, 10.0, 5.0)


def test_obs_dim_frozen():
    assert interface.obs_dim(12) == 12 + 3 + interface.N_ACTIONS == 20
    assert interface.N_ACTIONS == 5


def test_go_to_goal_policy_ignores_obstacle():
    policy = load_policy(os.path.join(POLICIES, 'go_to_goal.json'))
    assert policy.act(_obstacle_obs(0.1, 1.0, 0.0), 12) == 0  # 纯 go-to-goal：沿原路径


def test_avoid_policy_turns_to_clear_side():
    policy = load_policy(os.path.join(POLICIES, 'avoid_obstacles.json'))
    # 前方遮挡、左侧净空更大 → 左移（动作 1）
    assert policy.act(_obstacle_obs(0.1, 1.0, 0.0), 12) == 1
    # 前方遮挡、右侧净空更大 → 右移（动作 2）
    assert policy.act(_obstacle_obs(0.1, 0.0, 1.0), 12) == 2


def test_policy_path_switch_changes_behavior():
    forward_only = load_policy(os.path.join(POLICIES, 'go_to_goal.json'))
    avoid = load_policy(os.path.join(POLICIES, 'avoid_obstacles.json'))
    obs = _obstacle_obs(0.1, 1.0, 0.0)
    assert forward_only.act(obs, 12) != avoid.act(obs, 12)
