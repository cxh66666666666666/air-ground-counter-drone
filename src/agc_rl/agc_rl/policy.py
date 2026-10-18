"""RL 策略加载与推理（W5-b · spec R6）。

策略**可加载**：由 config 的 `policy_path` 指向一个 JSON 参数文件（最简确定性策略 / 预置权重）。
改 `policy_path` → 载入不同参数 → 行为变化可复现（spec R6 判据）。

本模块只做"obs → action"的决策；action → `cmd_vel` 的映射在 `interface.action_to_twist`。
策略以规则门控（对齐旧树 `ground_env.py` 的 DQN + shield 口径的行为事实）决定左右/加速：
净空充足→沿原路径；净空不足→向净空更大的一侧偏转。
"""

import json
import os

from agc_rl import interface


class Policy:
    """策略基类：只暴露 `load` 与 `act`（接口冻结，供推理节点调用）。"""

    name = "base"

    def load(self, path):
        raise NotImplementedError

    def act(self, obs, num_sectors):
        raise NotImplementedError


class DeterministicPolicy(Policy):
    """最简确定性策略（可加载 JSON 参数）。"""

    name = "deterministic_go_to_goal"

    # 默认参数（policy_path 缺省/不可读时使用；与 `policies/go_to_goal.json` 一致）
    DEFAULTS = {
        "name": "deterministic_go_to_goal",
        "avoid_obstacles": False,         # False → 纯 go-to-goal（安全层为唯一 shield，见 ADR-0004/C6）
        "safe_distance_norm": 0.35,       # 前方净空低于此 → 触发避让（旧树 safe_distance_norm）
        "forward_half_width_sectors": 1,  # 前方判定扇区半宽
        "prefer_side": "left",            # 左右净空相等时的偏好：left | right
    }

    def __init__(self):
        self.params = dict(self.DEFAULTS)

    def load(self, path):
        if not path:
            return
        with open(os.path.expanduser(path), 'r') as fh:
            loaded = json.load(fh)
        name = loaded.get("name")
        if name and name != self.name:
            raise ValueError(
                "policy '%s' 与加载器 '%s' 不匹配（W5-b 仅提供确定性策略）" % (name, self.name))
        self.params.update({k: v for k, v in loaded.items() if k != "name"})

    def act(self, obs, num_sectors):
        if not bool(self.params["avoid_obstacles"]):
            return 0  # 纯 go-to-goal：安全层（collision_monitor）为唯一 shield（ADR-0004 / C6）
        half = int(self.params["forward_half_width_sectors"])
        forward = interface.forward_clearance(obs, num_sectors, half)
        if forward >= float(self.params["safe_distance_norm"]):
            return 0  # 沿原路径
        left = interface.side_clearance(obs, num_sectors, +1)
        right = interface.side_clearance(obs, num_sectors, -1)
        if left > right:
            return 1  # 左移
        if right > left:
            return 2  # 右移
        return 1 if self.params["prefer_side"] == "left" else 2


def load_policy(policy_path):
    """按 config 路径加载策略；path 为空则返回默认确定性策略。"""
    policy = DeterministicPolicy()
    policy.load(policy_path)
    return policy
