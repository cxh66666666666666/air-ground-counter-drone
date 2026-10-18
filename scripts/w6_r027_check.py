#!/usr/bin/env python3
"""w6_r027_check —— R-027（安全不可行率诚实口径与真实接触证据）自包含核查。

**起过 Phase A/B 后**读两份 summary + 静态核验源文件，逐项判定：
  (a) summary 含 `definition_version`（三键齐全）+ `metrics_comparability`；
  (b) 旧字段仍在（`shield_infeasible_rate` / `ticks_total` / `ticks_triggered` / `ticks_infeasible` 键存在）；
  (c) 负对照：Phase A `collision_count==0` 且 `true_shield_infeasible==false`（阈值外缓停不计碰撞）；
  (d) 正例：Phase B `collision_count>=1` 且 `collision_events` 有 `pole` 证据（真实接触计事件）；
  (e) 缺失掩码：Phase B `not_applicable.trigger_derived==true` 且 `true_shield_infeasible_rate is None`；
      Phase A `not_applicable` 键存在；
  (f) 口径记录于 `docs/adr/0042-*.md`（`Status: accepted`、无 `§` 章节号引用）。

输出 `experiments/w6-task-eval/<run_id>/metrics/r027.json`；`overall_pass` = (a)∧(b)∧(c)∧(d)∧(e)∧(f)。
"""

import argparse
import glob
import json
import os
import sys
from datetime import datetime

ROOT = '/home/chen/AGC'

DV_KEYS = ('shield_infeasible_rate', 'collision_truth', 'true_shield_infeasible')
OLD_KEYS = ('shield_infeasible_rate', 'ticks_total', 'ticks_triggered', 'ticks_infeasible')


def _load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _read(path):
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


def case_a_definition_version(a_sum, b_sum):
    res = {}
    ok = True
    for name, sm in (('phase_a', a_sum), ('phase_b', b_sum)):
        dv = (sm or {}).get('definition_version') if sm else None
        res['%s_definition_version' % name] = dv
        if not isinstance(dv, dict) or not all(k in dv for k in DV_KEYS):
            ok = False
        if sm is None or 'metrics_comparability' not in sm:
            ok = False
    res['metrics_comparability_present'] = bool(
        a_sum and 'metrics_comparability' in a_sum and b_sum and 'metrics_comparability' in b_sum)
    res['pass'] = bool(ok and res['metrics_comparability_present'])
    return res


def case_b_history_fields(a_sum, b_sum):
    """旧字段仍在（键存在）且**数值与旧口径不冲突**（0 ≤ infeasible ≤ triggered ≤ total；
    shield_infeasible_rate == infeasible/triggered（triggered>0）否则 0）。"""
    res = {}
    ok = True
    for name, sm in (('phase_a', a_sum), ('phase_b', b_sum)):
        present = bool(sm) and all(k in sm for k in OLD_KEYS)
        res['%s_history_keys' % name] = present
        if not present:
            ok = False
            continue
        tt, tr, inf = sm['ticks_total'], sm['ticks_triggered'], sm['ticks_infeasible']
        consistent = (0 <= inf <= tr <= tt)
        expect_rate = (inf / tr) if tr > 0 else 0.0
        rate_ok = abs(float(sm['shield_infeasible_rate']) - expect_rate) < 1e-6
        res['%s_counts' % name] = {'ticks_total': tt, 'ticks_triggered': tr,
                                   'ticks_infeasible': inf, 'rate_ok': rate_ok}
        if not (consistent and rate_ok):
            ok = False
    res['pass'] = ok
    return res


def case_c_negative_control(a_sum):
    res = {'collision_count': (a_sum or {}).get('collision_count') if a_sum else None,
           'true_shield_infeasible': (a_sum or {}).get('true_shield_infeasible') if a_sum else None}
    res['pass'] = bool(a_sum and a_sum.get('collision_count') == 0 and
                       a_sum.get('true_shield_infeasible') is False)
    return res


def case_d_positive_contact(b_sum):
    events = (b_sum or {}).get('collision_events') if b_sum else None
    res = {'collision_count': (b_sum or {}).get('collision_count') if b_sum else None,
           'collision_events': events}
    res['pole_evidence'] = bool(isinstance(events, list) and any(
        'pole' in (e.get('other') or '') for e in events if isinstance(e, dict)))
    # 接触事件含**几何**（首个接触点 world 坐标或最小穿透深度）
    res['geometry_evidence'] = bool(isinstance(events, list) and any(
        isinstance(e, dict) and ('position' in e or e.get('depth') is not None) for e in events))
    res['contact_msgs_received'] = (b_sum or {}).get('contact_msgs_received') if b_sum else None
    res['pass'] = bool(b_sum and isinstance(b_sum.get('collision_count'), int) and
                       b_sum.get('collision_count') >= 1 and res['pole_evidence'] and
                       res['geometry_evidence'])
    return res


def case_e_not_applicable(a_sum, b_sum):
    res = {}
    na_b = (b_sum or {}).get('not_applicable') if b_sum else None
    res['phase_b_not_applicable'] = na_b
    res['phase_b_trigger_derived'] = (na_b or {}).get('trigger_derived') if na_b else None
    res['phase_b_true_rate_is_none'] = bool(b_sum and b_sum.get('true_shield_infeasible_rate') is None)
    na_a = (a_sum or {}).get('not_applicable') if a_sum else None
    res['phase_a_not_applicable_present'] = bool(isinstance(na_a, dict))
    res['pass'] = bool(isinstance(na_b, dict) and (na_b or {}).get('trigger_derived') is True and
                       res['phase_b_true_rate_is_none'] and res['phase_a_not_applicable_present'])
    return res


def case_f_adr():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0042-*.md')))
    res = {'path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0])
        res['status_accepted'] = 'accepted' in t
        res['no_section_ref'] = '§' not in t
        res['has_r027_ref'] = 'r-027' in t
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('no_section_ref'))
    return res


def dump(res, out_dir):
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    path = os.path.join(out_dir, 'metrics', 'r027.json')
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
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-r027-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)

    a_path = os.path.join(out_dir, 'summaries', 'phase_a_safety_stop.json')
    b_path = os.path.join(out_dir, 'summaries', 'phase_b_real_contact.json')
    a_sum = _load(a_path) if os.path.exists(a_path) else None
    b_sum = _load(b_path) if os.path.exists(b_path) else None

    res = {'run_id': run_id, 'phase_a_summary': a_path, 'phase_b_summary': b_path, 'tests': {}}
    try:
        res['tests']['a_definition_version'] = case_a_definition_version(a_sum, b_sum)
        res['tests']['b_history_fields'] = case_b_history_fields(a_sum, b_sum)
        res['tests']['c_negative_control_no_collision'] = case_c_negative_control(a_sum)
        res['tests']['d_positive_real_contact'] = case_d_positive_contact(b_sum)
        res['tests']['e_not_applicable_mask'] = case_e_not_applicable(a_sum, b_sum)
        res['tests']['f_adr_recorded'] = case_f_adr()
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, out_dir)


if __name__ == '__main__':
    sys.exit(main())
