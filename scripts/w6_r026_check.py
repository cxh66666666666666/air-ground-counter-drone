#!/usr/bin/env python3
"""w6_r026_check —— R-026（场景轴承载者校验与覆盖记账）自包含核查。

**起过 launch 后**读产物 + 静态核验源文件，逐项判定：
  (a) 轴承载者登记与启动校验：统一入口 `w6.launch.py` 经共享模块 `w6_composition.py`
      读 scenario_carriers.yaml 且无承载者 → 拒绝运行；各场景 `support_status` 齐备
      （supported / unsupported:<axis>）。
  (b) obstacle none/static 正负对照（轻量实跑）：none 世界**不含** pole_*、`obstacles_file` 为空文件；
      static 世界含 pole_*、`obstacles_file` 与之一致；manifest 记实际障碍集。
  (c) dynamic 无承载者 → launch 拒绝运行（日志含显式原因，退出非零）。
  (d) 轻量栈运动轴 / 感知轴 → 覆盖账本标 not_carried（manifest + runner 报告）。
  (e) scene_manifest 实际生效参数 == 声明轴（断言全真）。

输出 `experiments/w6-task-eval/<run_id>/metrics/r026.json`；`overall_pass` = (a)∧(b)∧(c)∧(d)∧(e)。
"""

import argparse
import glob
import json
import os
import sys
from datetime import datetime
from pathlib import Path

ROOT = '/home/chen/AGC'

# 轻量实跑样例场景（run_w6_r026.sh 产出 manifest）
SAMPLES = {
    'none': 's01_static_none_nominal_track',
    'static': 's04_static_static_obs_track',
    'linear': 's02_linear_none_nominal_track',
    'maneuver': 's03_maneuver_none_nominal_track',
    'dropout': 's06_static_none_dropout_track',
    'fov': 's07_linear_none_fov_track',
}


def _read(path):
    return Path(path).read_text(encoding='utf-8', errors='replace')


def _load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def case_a_carrier_registration():
    res = {}
    w6 = _read(os.path.join(ROOT, 'src/agc_task/launch/w6.launch.py'))
    comp = _read(os.path.join(ROOT, 'src/agc_task/launch/w6_composition.py'))
    reg = _read(os.path.join(ROOT, 'src/agc_task/config/scenario_carriers.yaml'))
    scen = _read(os.path.join(ROOT, 'src/agc_task/config/scenarios.yaml'))
    # S-05：统一入口 w6.launch.py 经共享模块 w6_composition.py 单一加载 / 校验 carrier 表；
    # 两 backend 复用同一模块 → 同一判据同时覆盖 light / full。
    res['shared_module_used'] = ('w6_composition' in w6) and ('load_scene' in w6)
    res['light_loads_carriers'] = ('scenario_carriers.yaml' in comp and 'carrier_entry' in comp)
    res['full_loads_carriers'] = res['light_loads_carriers']
    res['light_refuses'] = '拒绝运行' in comp
    res['full_refuses'] = res['light_refuses']
    res['dynamic_unsupported'] = 'dynamic:' in reg and 'supported: false' in reg
    res['scenes_have_support_status'] = scen.count('support_status:') >= 12
    res['dynamic_scenes_marked'] = 'unsupported:obstacle' in scen
    res['pass'] = bool(all(res[k] for k in (
        'shared_module_used', 'light_loads_carriers', 'full_loads_carriers', 'light_refuses',
        'full_refuses', 'dynamic_unsupported', 'scenes_have_support_status',
        'dynamic_scenes_marked')))
    return res


def case_b_obstacle_none_static(out_dir):
    res = {}
    canonical = _read(os.path.join(ROOT, 'src/agc_sim/worlds/agc_world.sdf'))
    geometry = _read(os.path.join(ROOT, 'src/agc_sim/config/world_geometry.yaml'))
    # S-06 / ADR-0062：base world 不含 pole_*；几何真源（world_geometry.yaml）含 pole_*；
    #   none 世界 = base world 副本（无 pole）；static 世界 = 真源注入生成（含 pole）。
    res['base_world_has_no_poles'] = ('<model name="pole_1">' not in canonical and
                                      '<model name="pole_2">' not in canonical)
    res['geometry_source_has_poles'] = ('id: pole_1' in geometry and 'id: pole_2' in geometry)
    # 兼容旧产物名：none 世界副本（= base world）
    stripped_path = os.path.join(out_dir, 'worlds', 'agc_world_no_obstacles.sdf')
    if os.path.exists(stripped_path):
        stripped = _read(stripped_path)
        res['stripped_exists'] = True
        res['stripped_has_no_poles'] = ('<model name="pole_1">' not in stripped and
                                        '<model name="pole_2">' not in stripped)
        res['stripped_keeps_physics'] = ('<physics' in stripped and 'ground_plane' in stripped)
    else:
        res['stripped_exists'] = False
        res['stripped_has_no_poles'] = False
        res['stripped_keeps_physics'] = False
    static_world_path = os.path.join(out_dir, 'worlds', 'agc_world_static_obstacles.sdf')
    if os.path.exists(static_world_path):
        static_world = _read(static_world_path)
        res['static_world_has_poles'] = ('<model name="pole_1">' in static_world and
                                         '<model name="pole_2">' in static_world)
    else:
        res['static_world_has_poles'] = False

    def _manifest(scene):
        p = os.path.join(out_dir, 'manifests', '%s.json' % scene)
        return _load(p) if os.path.exists(p) else None

    none_m = _manifest(SAMPLES['none'])
    static_m = _manifest(SAMPLES['static'])
    res['none_manifest'] = bool(none_m)
    res['static_manifest'] = bool(static_m)
    if none_m:
        eff = none_m['carriers']['obstacle'].get('effective', {})
        res['none_world_obstacles'] = eff.get('world_obstacles')
        res['none_obstacles_file'] = eff.get('obstacles_file')
        res['none_obstacle_ids'] = eff.get('obstacle_ids')
        res['none_manifest_records_set'] = bool(eff)
    if static_m:
        eff = static_m['carriers']['obstacle'].get('effective', {})
        res['static_world_obstacles'] = eff.get('world_obstacles')
        res['static_obstacles_file'] = eff.get('obstacles_file')
        res['static_obstacle_ids'] = eff.get('obstacle_ids')
    empty = _read(os.path.join(ROOT, 'src/agc_perception/config/obstacles_none.yaml'))
    res['empty_obstacles_file'] = 'obstacles: []' in empty
    res['pass'] = bool(
        res.get('base_world_has_no_poles') and res.get('geometry_source_has_poles') and
        res.get('stripped_exists') and
        res.get('stripped_has_no_poles') and res.get('stripped_keeps_physics') and
        res.get('static_world_has_poles') and
        res.get('none_manifest') and res.get('static_manifest') and
        res.get('none_world_obstacles') == 'none' and
        res.get('none_obstacles_file') == 'obstacles_none.yaml' and
        res.get('none_obstacle_ids') == [] and
        res.get('static_world_obstacles') == 'static' and
        res.get('static_obstacles_file') == 'obstacles.yaml' and
        res.get('static_obstacle_ids') == ['pole_1', 'pole_2'] and
        res.get('empty_obstacles_file'))
    return res


def case_c_dynamic_refusal(out_dir):
    res = {}
    log_path = os.path.join(out_dir, 'log', 'dynamic_refusal.log')
    rc_path = os.path.join(out_dir, 'log', 'dynamic_refusal.rc')
    if os.path.exists(log_path):
        log = _read(log_path)
        res['log_exists'] = True
        res['has_refusal_reason'] = ('拒绝运行' in log and 'dynamic' in log)
    else:
        res['log_exists'] = False
        res['has_refusal_reason'] = False
    if os.path.exists(rc_path):
        res['rc'] = _read(rc_path).strip()
        res['nonzero_rc'] = res['rc'] not in ('0', '')
    else:
        res['rc'] = None
        res['nonzero_rc'] = False
    res['pass'] = bool(res.get('log_exists') and res.get('has_refusal_reason') and
                       res.get('nonzero_rc'))
    return res


def case_d_not_carried(out_dir):
    """轻量栈：运动轴（s02/s03）与感知轴（s06/s07）manifest 记 not_carried。"""
    res = {}

    def _status(scene, axis):
        p = os.path.join(out_dir, 'manifests', '%s.json' % scene)
        if not os.path.exists(p):
            return None
        m = _load(p)
        return (m.get('carriers', {}).get(axis) or {}).get('status')

    res['s02_target_motion'] = _status(SAMPLES['linear'], 'target_motion')
    res['s03_target_motion'] = _status(SAMPLES['maneuver'], 'target_motion')
    res['s06_perception'] = _status(SAMPLES['dropout'], 'perception')
    res['s07_perception'] = _status(SAMPLES['fov'], 'perception')
    res['motion_not_carried'] = (res['s02_target_motion'] == 'not_carried' and
                                 res['s03_target_motion'] == 'not_carried')
    res['perception_not_carried'] = (res['s06_perception'] == 'not_carried' and
                                     res['s07_perception'] == 'not_carried')

    # runner 报告：unsupported（s05）与 not_carried 如实标注
    rp = os.path.join(out_dir, 'lightweight', 'metrics', 'report.json')
    if os.path.exists(rp):
        rep = _load(rp)
        res['runner_unsupported'] = rep.get('unsupported_episodes')
        res['runner_not_carried_axes'] = rep.get('not_carried_axes')
        res['runner_marks_unsupported'] = bool(
            any('s05' in s for s in (rep.get('unsupported_episodes') or [])))
        res['runner_marks_not_carried'] = bool(rep.get('not_carried_axes'))
    else:
        res['runner_marks_unsupported'] = False
        res['runner_marks_not_carried'] = False
    res['pass'] = bool(res.get('motion_not_carried') and res.get('perception_not_carried') and
                       res.get('runner_marks_unsupported') and res.get('runner_marks_not_carried'))
    return res


def case_e_effective_equals_declared(out_dir):
    res = {}
    problems = []
    n = 0
    for scene in SAMPLES.values():
        p = os.path.join(out_dir, 'manifests', '%s.json' % scene)
        if not os.path.exists(p):
            problems.append('%s manifest 缺失' % scene)
            continue
        m = _load(p)
        n += 1
        asserts = m.get('assertions') or {}
        if not all(asserts.values()):
            problems.append('%s 断言失败: %s' % (scene, [k for k, v in asserts.items() if not v]))
        declared = (m.get('declared_axes') or {}).get('target_motion')
        eff = (m.get('effective_params') or {}).get('intruder_driver', {}).get('motion')
        if declared != eff:
            problems.append('%s motion 声明 %s != 实际 %s' % (scene, declared, eff))
        # 覆盖账本应含 4 轴
        for ax in ('target_motion', 'obstacle', 'perception', 'outcome'):
            if ax not in (m.get('carriers') or {}):
                problems.append('%s manifest 缺轴 %s' % (scene, ax))
    res['n_manifests'] = n
    res['problems'] = problems
    res['pass'] = bool(n == len(SAMPLES) and not problems)
    return res


def case_f_adr():
    p = os.path.join(ROOT, 'docs/adr/0041-scenario-axis-carriers-and-coverage.md')
    res = {'exists': os.path.exists(p)}
    if res['exists']:
        t = _read(p)
        res['status_accepted'] = 'accepted' in t
        res['no_section_ref'] = '§' not in t
    else:
        res['status_accepted'] = False
        res['no_section_ref'] = False
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and res.get('no_section_ref'))
    return res


def dump(res, out_dir):
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    path = os.path.join(out_dir, 'metrics', 'r026.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-56s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
        for k, v in t.items():
            if k != 'pass':
                print('      %s = %s' % (k, v))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-r026-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)

    res = {'run_id': run_id, 'tests': {}}
    try:
        res['tests']['a_axis_carrier_registration'] = case_a_carrier_registration()
        res['tests']['b_obstacle_none_static'] = case_b_obstacle_none_static(out_dir)
        res['tests']['c_dynamic_refusal'] = case_c_dynamic_refusal(out_dir)
        res['tests']['d_lightweight_not_carried'] = case_d_not_carried(out_dir)
        res['tests']['e_effective_equals_declared'] = case_e_effective_equals_declared(out_dir)
        res['tests']['f_adr_recorded'] = case_f_adr()
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, out_dir)


if __name__ == '__main__':
    sys.exit(main())
