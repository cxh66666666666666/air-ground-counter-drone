"""种子分解与派生（W7-a · spec R3）。

分离 `spawn / sensor / channel / target / policy / physics` 六个随机流，并保存**派生规则**；
`scene / episode / attempt` **三级区分**；配对比较用**共同外生随机过程**（CRN）。

成熟做法对齐：numpy 文档的 `SeedSequence.spawn` 思路——由**主种子**经层级派生出**统计独立**
的子流。本模块为保证**跨进程 / 跨平台确定**（避免 Python `hash()` 的进程随机化），改用
`hashlib.sha256` 做层级派生；RNG 取 stdlib `random.Random`（无第三方依赖，spec R2/R3 选择）。

三级语义：
  * **scene**     —— 场景级（同 `master_seed` + `scene_id`）→ `scene_seed`；
  * **episode**   —— 第 N 回合（同 scene）→ `episode_seed`；
  * **attempt**   —— 同回合的**重试**（如 infra 后重跑）→ **仅**改变 `policy` 流。

**共同外生随机过程**（CRN）：`spawn / sensor / channel / target / physics` **只由 (scene, episode)
决定**，**不**随 `attempt` 变化——故同一回合的不同 attempt 面对**相同的外生随机过程**，可用于
**配对比较**；`policy` 流随 attempt 变化（智能体侧的探索随机）。

不可复现边界（成文）：外生流（spawn / 噪声序列）**逐位可复现**；但**物理步进调度**（实时异步
墙钟调度，见 [ADR-0069](../../docs/adr/0069-w7-entry-and-exit-definition.md)(b)）**不承诺** bitwise
一致——轨迹在时间维度上的分叉属**已知边界**，由 W7 runner 记录（spec R3）。
"""

import hashlib
import random

# 六个随机流；前五个（含 physics）为**外生**（不随 attempt 变），policy 为**智能体侧**。
STREAMS = ('spawn', 'sensor', 'channel', 'target', 'policy', 'physics')
EXOGENOUS_STREAMS = ('spawn', 'sensor', 'channel', 'target', 'physics')
POLICY_STREAM = 'policy'
LEVELS = ('scene', 'episode', 'attempt')

DERIVATION_RULE = ('seed(master, level_parts..., stream) = int(sha256("|".join(parts))[:8], big-endian)；'
                   'scene_seed = f(master, scene_id)；episode_seed = f(scene_seed, episode_index)；'
                   'exogenous[流] = f(episode_seed, 流)；policy = f(episode_seed, attempt_index, 流)')


def _derive(*parts):
    """确定性派生：层级化 sha256 → 64 位正整数种子（跨进程 / 跨平台一致）。"""
    key = '|'.join(str(p) for p in parts).encode('utf-8')
    return int.from_bytes(hashlib.sha256(key).digest()[:8], 'big')


def scene_seed(master_seed, scene_id):
    return _derive('agc', 'scene', master_seed, scene_id)


def episode_seed(master_seed, scene_id, episode_index):
    return _derive('agc', 'episode', scene_seed(master_seed, scene_id), episode_index)


def derive_stream(master_seed, scene_id, episode_index, attempt_index, stream):
    """取某流的种子：外生流只看 (scene, episode)；policy 流另看 attempt（CRN 语义）。"""
    if stream not in STREAMS:
        raise ValueError('未知随机流 %r（可用：%s）' % (stream, list(STREAMS)))
    ep = episode_seed(master_seed, scene_id, episode_index)
    if stream == POLICY_STREAM:
        return _derive('agc', 'stream', ep, attempt_index, stream)
    return _derive('agc', 'stream', ep, stream)


class SeedPlan:
    """一个 episode（含 attempt）的完整种子计划：六流种子 + 派生规则 + 三级身份。"""

    def __init__(self, master_seed, scene_id, episode_index=0, attempt_index=0):
        self.master_seed = int(master_seed)
        self.scene_id = str(scene_id)
        self.episode_index = int(episode_index)
        self.attempt_index = int(attempt_index)
        self.scene_seed = scene_seed(self.master_seed, self.scene_id)
        self.episode_seed = episode_seed(self.master_seed, self.scene_id, self.episode_index)
        self.streams = {
            s: derive_stream(self.master_seed, self.scene_id, self.episode_index,
                             self.attempt_index, s)
            for s in STREAMS
        }

    @property
    def spawn_seed(self):
        """spawn 流种子——注入 `scenario_reset.spawn_seed`（+ `run_logger.seed`）。"""
        return self.streams['spawn']

    def rng(self, stream):
        """取某流的 stdlib RNG（`random.Random`，给定种子确定可复现）。"""
        return random.Random(self.streams[stream])

    def to_dict(self):
        return {
            'master_seed': self.master_seed,
            'level': {'scene_id': self.scene_id, 'episode_index': self.episode_index,
                      'attempt_index': self.attempt_index},
            'scene_seed': self.scene_seed,
            'episode_seed': self.episode_seed,
            'streams': self.streams,
            'exogenous_streams': list(EXOGENOUS_STREAMS),
            'policy_stream': POLICY_STREAM,
            'derivation_rule': DERIVATION_RULE,
            'reproducibility_note': (
                '外生流（spawn / sensor / channel / target / physics）逐位可复现且**不随 attempt 变化**'
                '（配对比较用共同外生随机过程）；policy 流随 attempt 变化。物理步进调度为实时异步，'
                '**不承诺** bitwise 轨迹一致（ADR-0069(b)）。'),
        }


def make_seed_plan(master_seed, scene_id, episode_index=0, attempt_index=0):
    return SeedPlan(master_seed, scene_id, episode_index, attempt_index)
