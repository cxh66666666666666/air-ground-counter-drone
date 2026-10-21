"""场景配置不变量（ADR-0076）——纯配置层测试，不启仿真。

守护的 bug 类别：**配置自相矛盾导致结局永不可达**。
实例：`capture_adjudicator.interception_authorized: true`（授权开火）但
`task_manager.tracking_milestone_is_terminal` 仍为 `true` → 跟踪里程碑先进终态，
交火窗口（GROUND_APPROACH）被关闭 → `INTERCEPT_CONFIRMED/CAPTURE_NET` **永不发生**
（实测 s08 协同臂 20/20 落 `TRACKING_SUCCESS`、`pass=false`）。

测试对象是真源文件（`config/scenarios.yaml` + `config/w6_base.yaml`），
与 `w6.launch.py` 的 `assertions()` 共用同一判据（此处做**全场景**静态扫描，
启动时只校验当前场景）。
"""

import os

import pytest
import yaml

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'config')


def _load():
    with open(os.path.join(CONFIG_DIR, 'scenarios.yaml')) as f:
        scenarios = yaml.safe_load(f)['scenes']   # 真源根键为 `scenes`
    with open(os.path.join(CONFIG_DIR, 'w6_base.yaml')) as f:
        base = yaml.safe_load(f)
    return scenarios, base


def _param(scenarios, base, scene, node, key, default):
    """按「基参 → 场景覆盖」解析实际生效值（与 launch 的合并口径一致）。"""
    value = (base.get(node, {}).get('ros__parameters', {}) or {}).get(key, default)
    override = ((scenarios.get(scene, {}).get('params') or {}).get(node) or {}).get(key, value)
    return override


def test_capture_authorized_scenes_disable_milestone_terminal():
    """所有授权射击的场景，必须显式关闭「里程碑即终态」。"""
    scenarios, base = _load()
    checked = []
    for scene in scenarios:
        authorized = bool(_param(scenarios, base, scene, 'capture_adjudicator',
                                 'interception_authorized', False))
        if not authorized:
            continue
        terminal = bool(_param(scenarios, base, scene, 'task_manager',
                               'tracking_milestone_is_terminal', True))
        assert not terminal, (
            '场景 %s 已授权射击（interception_authorized=true）但 tracking_milestone_is_terminal=true '
            '→ 捕获结局不可达（ADR-0076）' % scene)
        checked.append(scene)
    # 真源里应至少有一个已授权场景（s08）；退化保护：若该轴被误删，测试应失败而非静默通过。
    assert checked, '未找到任何 interception_authorized=true 的场景——场景真源可能被误改（ADR-0076）'


def test_base_default_keeps_existing_behavior():
    """基参必须保持 `true`（既有行为不变）；capture 场景在**场景层**覆盖。"""
    _, base = _load()
    assert base['task_manager']['ros__parameters']['tracking_milestone_is_terminal'] is True


def test_non_capture_scenes_unaffected():
    """非授权场景不得被本次修正波及（否则会破坏既有 W6 回归）。"""
    scenarios, base = _load()
    affected = [
        scene for scene in scenarios
        if not bool(_param(scenarios, base, scene, 'capture_adjudicator',
                           'interception_authorized', False))
        and not bool(_param(scenarios, base, scene, 'task_manager',
                            'tracking_milestone_is_terminal', True))
    ]
    assert affected == [], (
        '以下非授权场景被关闭了里程碑终态（非预期，会改变既有行为）：%s' % affected)


if __name__ == '__main__':
    raise SystemExit(pytest.main([os.path.realpath(__file__)]))
