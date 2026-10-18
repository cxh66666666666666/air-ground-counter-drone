#!/usr/bin/env python3
"""w6_r035_check —— R-035（空中控制唯一权威与控制许可）自包含核查。

**起过 run_w6_r035.sh 后**读 `experiments/w6-task-eval/<run_id>/r035/bench.json`（Phase A **实跑台架**证据）
+ 源码 / 配置 / ADR 静态核查，逐项判定 (a)…(h)；
输出 `.../<run_id>/r035/check.json`（含 `overall_pass` 布尔）与 `.../<run_id>/metrics/r035.json`。
overall_pass = (a)∧…∧(h)。
"""

import argparse
import glob
import json
import os
import re
import subprocess
import sys

ROOT = '/home/chen/AGC'


def _load(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _read(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


def _code_only(txt):
    """剥离整行注释，仅保留可执行行（避免注释中提及 `rm -rf` 被误判）。"""
    return '\n'.join(s for s in (ln.strip() for ln in txt.splitlines()) if not s.startswith('#'))


def _scenario(bench, name):
    s = (bench or {}).get('scenarios', {}).get(name, {})
    res = dict(s)
    res['present'] = bool(s)
    return res


def case_a_late_release(bench):
    s = _scenario(bench, 'late_release')
    return {
        'present': s['present'],
        'setpoints_before_release': s.get('setpoints_before_release'),
        'setpoints_after_release': s.get('setpoints_after_release'),
        'pass': bool(s['present'] and s.get('setpoints_before_release', 1e9) <= 2 and
                     s.get('setpoints_after_release', 0) >= 20),
    }


def case_b_duplicate_node(bench):
    s = _scenario(bench, 'duplicate_node')
    return {
        'present': s['present'],
        'conflict_true_seen': s.get('conflict_true_seen'),
        'competitor_claim_seen': s.get('competitor_claim_seen'),
        'setpoints_last_window': s.get('setpoints_last_window'),
        'pass': bool(s['present'] and s.get('conflict_true_seen') and
                     s.get('setpoints_last_window', 1e9) <= 2),
    }


def case_c_manager_restart(bench):
    s = _scenario(bench, 'manager_restart')
    return {
        'present': s['present'],
        'manager1_active': s.get('manager1_active'),
        'setpoints_after_designation': s.get('setpoints_after_designation'),
        'node_detected_bond_break': s.get('node_detected_bond_break'),
        'state_after_manager_death': s.get('state_after_manager_death'),
        'manager2_active': s.get('manager2_active'),
        'setpoints_after_restart': s.get('setpoints_after_restart'),
        'permit_after_restart': s.get('permit_after_restart'),
        'pass': bool(s['present'] and s.get('manager1_active') and
                     s.get('setpoints_after_designation', 0) >= 20 and
                     s.get('node_detected_bond_break') and
                     s.get('manager2_active') and
                     s.get('setpoints_after_restart', 1e9) <= 2 and
                     s.get('permit_after_restart') == 'false'),
    }


def case_d_silent_publisher(bench):
    s = _scenario(bench, 'silent_publisher')
    return {
        'present': s['present'],
        'setpoints_window': s.get('setpoints_window'),
        'writers_max': s.get('writers_max'),
        'conflict_true_seen': s.get('conflict_true_seen'),
        'pass': bool(s['present'] and s.get('setpoints_window', 0) >= 20 and
                     s.get('writers_max', 0) >= 2 and
                     (not s.get('conflict_true_seen'))),
    }


def case_e_static_source():
    """静态：控制许可话题 / self_uid / is_authorized_；冲突判定**先于** publishSetpoint；
    写入记账接口；countSetpointPublishers **仅**告警（无「仅凭计数决定权威」）。"""
    tt = _read(os.path.join(ROOT, 'src/agc_air/src/trajectory_tracker.cpp')) or ''
    hpp = _read(os.path.join(ROOT, 'src/agc_air/include/agc_air/offboard_interface.hpp')) or ''
    cpp = _read(os.path.join(ROOT, 'src/agc_air/src/offboard_interface.cpp')) or ''
    i_permit = tt.find('const bool permit')
    i_update = tt.find('updateConflict()')
    i_conflict_pub = tt.find('publishConflict(conflict_)')
    i_setpoint = tt.find('offboard_->publishSetpoint')
    res = {
        'has_permit_topic': '/agc/air/control_permit' in tt,
        'has_self_uid': 'self_uid' in tt,
        'has_is_authorized': 'is_authorized_' in tt,
        'permit_defined_before_setpoint': 0 <= i_permit < i_setpoint,
        'update_conflict_before_setpoint': 0 <= i_update < i_setpoint,
        'publish_conflict_before_setpoint': 0 <= i_conflict_pub < i_setpoint,
        'offboard_write_accounting': ('setpointWriteCount' in hpp and 'setpointRateHz' in hpp and
                                      'setpoint_write_count_' in cpp),
        'count_only_warn': ('countSetpointPublishers() > 1' not in tt and
                            'countSetpointPublishers()' in tt),
    }
    res['pass'] = all(res.values())
    return res


def case_f_no_abi_no_second_commander():
    """冻结 9 消息 ABI 未改（src/agc_msgs 无改动）；setpoint 发布者仍单一（offboard_interface.cpp）。"""
    p = subprocess.run(['git', 'status', '--porcelain', 'src/agc_msgs'],
                       cwd=ROOT, capture_output=True, text=True)
    abi_clean = (p.stdout.strip() == '')
    hits = []
    for root, _dirs, files in os.walk(os.path.join(ROOT, 'src')):
        for fn in files:
            if fn.endswith('.cpp'):
                t = _read(os.path.join(root, fn)) or ''
                if 'create_publisher<px4_msgs::msg::TrajectorySetpoint>' in t:
                    hits.append(fn)
    res = {'agc_msgs_porcelain': p.stdout.strip(), 'abi_clean': abi_clean,
           'setpoint_publisher_files': hits,
           'single_setpoint_publisher': hits == ['offboard_interface.cpp']}
    res['pass'] = bool(abi_clean and res['single_setpoint_publisher'])
    return res


def case_g_adr():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0050-*.md')))
    res = {'path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = ('## Status' in t and 'accepted' in t)
        res['no_section_ref'] = '\u00a7' not in t
        res['has_r035_ref'] = 'r-035' in t.lower()
        res['has_evidence_section'] = '取证' in t
        res['has_permit'] = '许可' in t
        res['has_single_authority'] = '唯一' in t and ('权威' in t or '授权' in t)
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('no_section_ref') and res.get('has_r035_ref') and
                       res.get('has_evidence_section') and res.get('has_permit') and
                       res.get('has_single_authority'))
    return res


def case_h_scripts_hygiene():
    """无 `rm -rf`、无裸名 `pkill`；bench 用 killpg + wait。"""
    bench_txt = _read(os.path.join(ROOT, 'scripts/w6_r035_bench.py')) or ''
    sh_txt = _read(os.path.join(ROOT, 'scripts/run_w6_r035.sh')) or ''
    bench_code = _code_only(bench_txt)
    sh_code = _code_only(sh_txt)
    res = {
        'bench_no_rm_rf': 'rm -rf' not in bench_code,
        'sh_no_rm_rf': 'rm -rf' not in sh_code,
        'bench_scoped_reap': ('killpg' in bench_code) and ('proc.wait(' in bench_code),
        'sh_uses_pgrep_pathkills': ('pgrep -f' in sh_code) and ('kill -9' in sh_code),
    }
    bare = re.findall(r'pkill\s+-\w+\s+-f\s+"(?!\[)[^"]+"', bench_code + '\n' + sh_code)
    res['bare_pkill'] = bare
    res['pass'] = bool(res['bench_no_rm_rf'] and res['sh_no_rm_rf'] and
                       res['bench_scoped_reap'] and res['sh_uses_pgrep_pathkills'] and not bare)
    return res


def case_i_scope_note(bench):
    """范围如实标注：非整链；unsupported 列表存在（本批无 unsupported 则显式登记为空）。"""
    note = (bench or {}).get('scope_note', '')
    uns = (bench or {}).get('unsupported', None)
    res = {
        'mentions_not_full_chain': ('非整链' in note),
        'mentions_logic_bench': ('台架' in note),
        'unsupported_is_list': isinstance(uns, list),
    }
    res['pass'] = bool(res['mentions_not_full_chain'] and res['mentions_logic_bench'] and
                       res['unsupported_is_list'])
    return res


def dump(res, run_dir):
    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(run_dir, 'r035'), exist_ok=True)
    with open(os.path.join(run_dir, 'r035', 'check.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'metrics', 'r035.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-30s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL',
                          os.path.join(run_dir, 'r035', 'check.json')))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261017-w6-r035-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    bench = _load(os.path.join(run_dir, 'r035', 'bench.json'))
    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        res['tests']['a_late_release'] = case_a_late_release(bench)
        res['tests']['b_duplicate_node'] = case_b_duplicate_node(bench)
        res['tests']['c_manager_restart'] = case_c_manager_restart(bench)
        res['tests']['d_silent_publisher'] = case_d_silent_publisher(bench)
        res['tests']['e_static_source'] = case_e_static_source()
        res['tests']['f_no_abi_no_second_commander'] = case_f_no_abi_no_second_commander()
        res['tests']['g_adr_0050'] = case_g_adr()
        res['tests']['h_scripts_hygiene'] = case_h_scripts_hygiene()
        res['tests']['i_scope_note'] = case_i_scope_note(bench)
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
