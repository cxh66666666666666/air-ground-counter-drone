"""obs / action 版本化机制（W7-a · spec R2）。

引入 `obs_version`、归一化统计与 checkpoint 元数据；**旧版本可回放**。

原则（spec R2）：
  * **不**静默改冻结接口（契约 [#messages](../../docs/ROS2迁移交接文档.md#messages)）；观测语义变更须
    **新版本号**（旧版本 `v1` 的定义**永久保留**，不原地改）；
  * obs **维度扩展**（目标方位 / 相对速度 / 观测年龄）依 W7-b 的策略职责裁定 —— 本模块只落**机制**、
    **留位**（`register` 可追加新版本），**不**在本段定扩展维度；
  * 动作语义**沿用** [ADR-0026](../../docs/adr/0026-rl-action-conventions-and-ground-escape-calibration.md)
    已修约定（符号 / 量纲），**不**回退。

`v1` = W5-b 冻结的观测（`agc_rl.interface.build_observation`）——本模块**引用**该真源构造，**不**复制其定义。
"""

import datetime
import hashlib
import math
import json
import os

from agc_rl import interface

# 当前默认观测版本（checkpoint 元数据据此区分）。
# W7-b（T4/E4，F-04 / ADR-0070(d)）：升级到 v2（新增目标 bearing / 相对速度 / 观测年龄）；
# v1 定义**永久保留**、可回放（旧权重不静默失效）。
OBS_VERSION = 2
ACTION_KIND = 'discrete'


class ActionSpaceDescriptor:
    """离散动作空间描述（沿用 [ADR-0026](../../docs/adr/0026-rl-action-conventions-and-ground-escape-calibration.md)）。"""

    def __init__(self, n_actions=interface.N_ACTIONS, names=None):
        self.kind = ACTION_KIND
        self.n = int(n_actions)
        self.names = dict(names or interface.ACTION_NAMES)

    def to_dict(self):
        return {'kind': self.kind, 'n': self.n, 'names': {str(k): v for k, v in self.names.items()}}


class ObsSpec:
    """单版本观测空间描述（维度 / 字段 / 归一化 / 数值域）。"""

    def __init__(self, version, num_sectors, fields, low, high, norm, dtype='float32'):
        self.version = int(version)
        self.num_sectors = int(num_sectors)
        self.fields = list(fields)
        self.dim = len(self.fields)
        self.low = float(low)
        self.high = float(high)
        self.norm = dict(norm)
        self.dtype = dtype

    def to_dict(self):
        return {'version': self.version, 'num_sectors': self.num_sectors,
                'dim': self.dim, 'fields': self.fields, 'low': self.low, 'high': self.high,
                'dtype': self.dtype, 'norm': self.norm}


def _fields_v1(num_sectors):
    """v1 字段名（顺序 = `interface.build_observation` 冻结顺序）。"""
    names = ['sector_%02d' % i for i in range(num_sectors)]
    names += ['pose_x_over_norm', 'pose_y_over_norm', 'dist_to_goal_over_norm']
    names += ['prev_action_%d' % i for i in range(interface.N_ACTIONS)]
    return names


def _build_v1(num_sectors, **kw):
    return ObsSpec(
        version=1, num_sectors=num_sectors, fields=_fields_v1(num_sectors),
        low=-2.0, high=1.0,
        norm={'pose_norm_m': kw.get('pose_norm_m', 10.0),
              'track_norm_m': kw.get('track_norm_m', 5.0),
              'lidar_max_range': kw.get('lidar_max_range', 10.0)})


# v2 追加字段（W7-b T4/E4，F-04）：目标方位（sin/cos 避免 ±π 跳变）+ 相对速度 + 观测年龄。
EXTRA_FIELDS_V2 = ['target_bearing_sin', 'target_bearing_cos',
                   'target_rel_speed_over_norm', 'target_obs_age_over_norm']


def _fields_v2(num_sectors):
    return _fields_v1(num_sectors) + list(EXTRA_FIELDS_V2)


def _build_v2(num_sectors, **kw):
    return ObsSpec(
        version=2, num_sectors=num_sectors, fields=_fields_v2(num_sectors),
        low=-2.0, high=1.0,
        norm={'pose_norm_m': kw.get('pose_norm_m', 10.0),
              'track_norm_m': kw.get('track_norm_m', 5.0),
              'lidar_max_range': kw.get('lidar_max_range', 10.0),
              'rel_speed_norm_mps': kw.get('rel_speed_norm_mps', 2.0),
              'obs_age_norm_s': kw.get('obs_age_norm_s', 0.5)})


# 版本注册表：**永久**保留各版本构造器（旧版本回放可用，不静默失效）。
_BUILDERS = {1: _build_v1, 2: _build_v2}


def register(version, builder):
    """登记新版本构造器（W7-b 扩展维度时调用；本段不新增版本）。"""
    _BUILDERS[int(version)] = builder


def get_obs_spec(version=OBS_VERSION, num_sectors=12, **kw):
    """取某版本观测空间描述；未知版本 → 明确报错（不静默回落到当前版本）。"""
    if int(version) not in _BUILDERS:
        raise ValueError('未知 obs_version=%r（已登记：%s）' % (version, sorted(_BUILDERS)))
    return _BUILDERS[int(version)](num_sectors, **kw)


def build_obs_vector(obs_version, sectors, pose_x, pose_y, dist_to_goal, last_action,
                     num_sectors, pose_norm_m, track_norm_m, **kw):
    """按 `obs_version` 构造观测向量。

    v1 直接**引用** `agc_rl.interface.build_observation`（冻结真源）；v2 在 v1 基础上追加
    **目标 bearing（sin/cos）/ 相对速度 / 观测年龄**（W7-b T4/E4，F-04）。旧版本分支始终保留，
    故"换版本不使旧权重静默失效"。

    v2 额外输入（`kw`，缺省 0 = 未知，保守）：
      `target_bearing_rad`（rad，车体系，0=正前、左为正）、`rel_speed_mps`、`obs_age_s`、
      `rel_speed_norm_mps`（默认 2.0）、`obs_age_norm_s`（默认 0.5）。
    """
    base = interface.build_observation(
        sectors, pose_x, pose_y, dist_to_goal, last_action,
        num_sectors, pose_norm_m, track_norm_m)
    if int(obs_version) == 1:
        return base
    if int(obs_version) == 2:
        b = float(kw.get('target_bearing_rad', 0.0))
        rel = float(kw.get('rel_speed_mps', 0.0))
        age = float(kw.get('obs_age_s', 0.0))
        rel_norm = max(float(kw.get('rel_speed_norm_mps', 2.0)), 1e-6)
        age_norm = max(float(kw.get('obs_age_norm_s', 0.5)), 1e-6)
        base = list(base)
        base.append(math.sin(b))
        base.append(math.cos(b))
        base.append(min(abs(rel) / rel_norm, 1.0))
        base.append(min(max(age, 0.0) / age_norm, 1.0))
        return base
    raise NotImplementedError(
        'obs_version=%d 的构造未实现（已登记：%s）' % (int(obs_version), sorted(_BUILDERS)))


class NormStats:
    """在线归一化统计（Welford 递推；可序列化进 checkpoint 元数据）。

    `v1` 的 obs 已在 `interface` 内做**固定尺度**归一（`pose_norm_m` 等）；本类提供**可选**的
    运行时统计（如对新增维度做 z-score），训练侧可据此复现同一归一化。
    """

    def __init__(self, dim, count=0, mean=None, m2=None):
        self.dim = int(dim)
        self.count = int(count)
        self.mean = list(mean) if mean is not None else [0.0] * self.dim
        self.m2 = list(m2) if m2 is not None else [0.0] * self.dim

    def update(self, x):
        self.count += 1
        for i in range(self.dim):
            d = x[i] - self.mean[i]
            self.mean[i] += d / self.count
            self.m2[i] += d * (x[i] - self.mean[i])

    @property
    def std(self):
        if self.count < 2:
            return [1.0] * self.dim
        return [max((self.m2[i] / (self.count - 1)) ** 0.5, 1e-8) for i in range(self.dim)]

    def to_dict(self):
        return {'dim': self.dim, 'count': self.count, 'mean': self.mean, 'm2': self.m2}

    @classmethod
    def from_dict(cls, d):
        return cls(d['dim'], d.get('count', 0), d.get('mean'), d.get('m2'))


class CheckpointMeta:
    """checkpoint 元数据：观测版本 / 动作空间 / 归一化统计 / 参数来源（含哈希）/ 代码指纹。

    训练侧保存权重时随附本元数据；加载时经 `check_compatible` 判定语义是否一致——
    版本不同 → **不**静默失效，而是给出**可区分**的加载决定（spec R2 判据）。
    """

    def __init__(self, obs_version, action_space, norm_stats=None, num_sectors=12,
                 params=None, git_commit='', created_at=None, note=''):
        self.obs_version = int(obs_version)
        self.action_space = action_space
        self.norm_stats = norm_stats
        self.num_sectors = int(num_sectors)
        self.params = dict(params or {})
        self.git_commit = git_commit
        self.created_at = created_at or datetime.datetime.now(
            datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
        self.note = note

    def to_dict(self):
        return {
            'meta_version': 1,
            'obs_version': self.obs_version,
            'action_space': self.action_space.to_dict(),
            'num_sectors': self.num_sectors,
            'norm_stats': self.norm_stats.to_dict() if self.norm_stats else None,
            'params': self.params,
            'git_commit': self.git_commit,
            'created_at': self.created_at,
            'note': self.note,
        }

    @classmethod
    def from_dict(cls, d):
        return cls(
            obs_version=d['obs_version'],
            action_space=ActionSpaceDescriptor(
                (d.get('action_space') or {}).get('n', interface.N_ACTIONS)),
            norm_stats=(NormStats.from_dict(d['norm_stats']) if d.get('norm_stats') else None),
            num_sectors=d.get('num_sectors', 12), params=d.get('params'),
            git_commit=d.get('git_commit', ''), created_at=d.get('created_at'), note=d.get('note', ''))

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, indent=2, ensure_ascii=False)
            f.write('\n')

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as f:
            return cls.from_dict(json.load(f))


def build_checkpoint_meta(num_sectors=12, params=None, git_commit='', obs_version=OBS_VERSION,
                          norm_stats=None, note=''):
    """构造当前（或指定版本）checkpoint 元数据。"""
    return CheckpointMeta(
        obs_version=obs_version, action_space=ActionSpaceDescriptor(), norm_stats=norm_stats,
        num_sectors=num_sectors, params=params, git_commit=git_commit, note=note)


def check_compatible(meta, current_version=OBS_VERSION, num_sectors=None):
    """加载期语义判定：`meta` 是否与当前环境**语义一致**（返回 `(ok, reasons)`）。

    **不**静默失效：版本不同即为**可区分**的 `False` + 明确原因；调用方据此拒绝或走旧版本回放。
    """
    reasons = []
    if int(meta.obs_version) != int(current_version):
        reasons.append('obs_version 不同：checkpoint=%d，当前=%d' % (meta.obs_version, current_version))
    if num_sectors is not None and int(meta.num_sectors) != int(num_sectors):
        reasons.append('num_sectors 不同：checkpoint=%d，当前=%d' % (meta.num_sectors, num_sectors))
    if (meta.action_space.kind != ACTION_KIND) or (meta.action_space.n != interface.N_ACTIONS):
        reasons.append('action 空间不同：checkpoint=%s，当前=%s/%d' % (
            meta.action_space.kind, ACTION_KIND, interface.N_ACTIONS))
    return (not reasons), reasons


def checkpoint_fingerprint(meta):
    """checkpoint 元数据的稳定指纹（sha256），供产物 / 报告引用。"""
    blob = json.dumps(meta.to_dict(), sort_keys=True, ensure_ascii=False).encode('utf-8')
    return hashlib.sha256(blob).hexdigest()
