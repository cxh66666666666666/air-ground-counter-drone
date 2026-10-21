"""证据标注与判定口径的回归测试（不启仿真）。

守护今天修掉的三类**静默错误**：

1. **backend 标注**：`run_episode` 曾把 `result['backend']` / 验收命题硬写成常量
   `lightweight` → `backend:=full` 的 episode 被误标为"轻量栈"。
2. **轴账本**：`axis_coverage` 曾按模块常量判 backend → full 链的空中依赖轴被误标 `not_carried`。
3. **判定口径**：primary DV 曾对"缺值"整对剔除 → infra / 失败回合（其 DV 常缺）被排除在分母外，
   与预注册 `keep_all_failures_and_infra` 不一致；现须记 0 并**计入分母**。

另有 infra 重试协议的行为约束：`mission` 结果**一律不重试**（不得以重试掩盖任务结果）。

运行（需先 source ROS 与工程）：
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 -m pytest scripts/tests/test_episode_runner_labels.py -q
"""

import importlib.util
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))

import episode_runner as er  # noqa: E402


def _load_adjudicate():
    path = os.path.join(ROOT, 'scripts', 'w7b_p1_adjudicate.py')
    spec = importlib.util.spec_from_file_location('w7b_p1_adjudicate', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------- 1. backend / 命题标注 ----------------

def test_proposition_labels_full_chain():
    p = er.build_proposition('full', 'headless', 's08_static_none_nominal_capture',
                             'INTERCEPT_CONFIRMED', 'CAPTURE_NET')
    assert '整链' in p and '轻量栈' not in p


def test_proposition_labels_lightweight():
    p = er.build_proposition('lightweight', 'headless', 's01_static_none_nominal_track',
                             'TRACKING_SUCCESS', 'tracking_milestone')
    assert '轻量栈' in p and '非整链' in p


# ---------------- 2. 轴账本按实际 backend ----------------

def test_axis_coverage_air_axes_covered_on_full():
    reg, car = er._registry(), er._carriers()
    scene = 's08_static_none_nominal_capture'
    _, full = er.axis_coverage(scene, reg, car, 'full')
    _, lite = er.axis_coverage(scene, reg, car, 'lightweight')
    # 整链：无 not_carried（空中链路在位）
    assert all(v['status'] != 'not_carried' for v in full.values())
    # 轻量栈：至少有一个空中依赖轴被如实标为 not_carried
    assert any(v['status'] == 'not_carried' for v in lite.values())


# ---------------- 3. 重试协议 ----------------

def _fake_sequence(monkeypatch, seq, seen):
    def fake(*_args, **kwargs):
        seen.append(kwargs.get('log_tag'))
        return dict(seq[len(seen) - 1])
    monkeypatch.setattr(er, 'run_episode', fake)


def test_infra_retry_stops_on_mission_result(monkeypatch):
    seen = []
    _fake_sequence(monkeypatch, [
        {'classification': 'infra', 'terminal_state': 'INFRA_FAILURE', 'reason_code': 'air_hover_timeout'},
        {'classification': 'mission', 'terminal_state': 'TRACKING_SUCCESS', 'reason_code': 'tracking_milestone'},
    ], seen)
    out = er.run_episode_with_infra_retry('s08', 0, {}, {}, 10, '/tmp', {}, 1,
                                          log_tag='p1_coop', infra_retries=3)
    assert seen == ['p1_coop', 'p1_coop.r1']          # 首次 + 1 次重试后停止
    assert out['infra_retries_used'] == 1
    assert out['terminal_state'] == 'TRACKING_SUCCESS'


def test_mission_failure_is_never_retried(monkeypatch):
    """契约 C4：超时属**任务终态**，不得重试掩盖。"""
    seen = []
    _fake_sequence(monkeypatch, [
        {'classification': 'mission', 'terminal_state': 'MISSION_FAILED', 'reason_code': 'TIMEOUT'},
    ], seen)
    out = er.run_episode_with_infra_retry('s08', 0, {}, {}, 10, '/tmp', {}, 1,
                                          log_tag='p1_coop', infra_retries=3)
    assert seen == ['p1_coop']
    assert out['infra_retries_used'] == 0


def test_infra_retry_is_bounded(monkeypatch):
    seen = []

    def always_infra(*_args, **kwargs):
        seen.append(kwargs.get('log_tag'))
        return {'classification': 'infra', 'terminal_state': 'INFRA_FAILURE', 'reason_code': 'x'}

    monkeypatch.setattr(er, 'run_episode', always_infra)
    out = er.run_episode_with_infra_retry('s08', 0, {}, {}, 10, '/tmp', {}, 1,
                                          log_tag='p1_coop', infra_retries=2)
    assert len(seen) == 3                              # K+1 次后停止
    assert out['infra_retries_used'] == 2


# ---------------- 4. 判定口径（预注册 keep_all_failures_and_infra） ----------------

@pytest.mark.parametrize('terminal, value, expect_zero', [
    ('INFRA_FAILURE', None, True),       # infra 且缺值 → 记 0（旧实现会整对剔除）
    ('MISSION_FAILED', None, True),      # 失败且缺值 → 记 0
    ('MISSION_FAILED', 0.3, True),       # 失败但有值 → 也记 0（失败即无有效火控机会）
    ('TRACKING_SUCCESS', 0.4, False),    # 成功 → 用真实值
])
def test_primary_dv_zero_on_failure(terminal, value, expect_zero):
    adj = _load_adjudicate()
    v, degraded = adj.dv_value({'terminal_state': terminal,
                                'fire_control_opportunity_rate': value}, 
                               'fire_control_opportunity_rate', True)
    if expect_zero:
        assert v == 0.0 and degraded
    else:
        assert v == value and not degraded


def test_secondary_dv_drops_missing_instead_of_zero():
    """最小净空记 0 会误表"碰撞" → 缺值必须剔除（返回 None），不得零填。"""
    adj = _load_adjudicate()
    v, _ = adj.dv_value({'terminal_state': 'MISSION_FAILED', 'min_clearance_m': None},
                        'min_clearance_m', False)
    assert v is None
