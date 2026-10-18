#!/usr/bin/env python3
"""w6_r032_check —— R-032（C10 冻结下游封闭）自包含核查。

**起过 run_w6_r032.sh 后**读 `experiments/w6-task-eval/<run_id>/r032/bench.json`（Phase A
**实跑逻辑台架**证据）+ 源码 / 配置 / ADR 静态核查，逐项判定 (a)…(h)；
输出 `.../<run_id>/metrics/r032.json`；overall_pass = (a)∧…∧(h)。
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


def case_a_follower_death(bench):
    s = (bench or {}).get('scenarios', {}).get('a_follower_death', {})
    res = dict(s)
    res['present'] = bool(s)
    return res


def case_b_old_ref_replay(bench):
    s = (bench or {}).get('scenarios', {}).get('b_old_ref_replay', {})
    res = dict(s)
    res['present'] = bool(s)
    return res


def case_c_low_confidence(bench):
    s = (bench or {}).get('scenarios', {}).get('c_low_confidence', {})
    res = dict(s)
    res['present'] = bool(s)
    return res


def case_d_offset_lifetime(bench):
    s = (bench or {}).get('scenarios', {}).get('d_offset_lifetime', {})
    res = dict(s)
    res['present'] = bool(s)
    return res


def case_e_scripts_hygiene():
    """进程管理：无 `rm -rf`、无裸名 `pkill`；bench 用 terminate/killpg + wait（仅判可执行行）。"""
    res = {}
    bench_txt = _read(os.path.join(ROOT, 'scripts', 'w6_r032_bench.py')) or ''
    sh_txt = _read(os.path.join(ROOT, 'scripts', 'run_w6_r032.sh')) or ''
    bench_code = _code_only(bench_txt)
    sh_code = _code_only(sh_txt)
    res['bench_no_rm_rf'] = 'rm -rf' not in bench_code
    res['sh_no_rm_rf'] = 'rm -rf' not in sh_code
    res['bench_scoped_reap'] = ('killpg' in bench_code) and ('proc.wait(' in bench_code)
    bare = re.findall(r'pkill\s+-\w+\s+-f\s+"(?!\[)[^"]+"', bench_code + '\n' + sh_code)
    res['bare_pkill'] = bare
    res['pass'] = bool(res['bench_no_rm_rf'] and res['sh_no_rm_rf'] and
                       res['bench_scoped_reap'] and not bare)
    return res


def case_f_adr():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0047-*.md')))
    res = {'path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = bool('Status' in t and 'accepted' in t)
        res['no_section_ref'] = '\u00a7' not in t
        res['has_r032_ref'] = bool('r-032' in t.lower())
        res['has_source_time'] = bool('header.stamp' in t or '源时间' in t)
        res['has_offset_lifetime'] = bool('寿命' in t or '生效窗口' in t)
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('no_section_ref') and res.get('has_r032_ref') and
                       res.get('has_source_time') and res.get('has_offset_lifetime'))
    return res


def case_g_no_abi_change():
    p = subprocess.run(['git', 'status', '--porcelain', 'src/agc_msgs'],
                       cwd=ROOT, capture_output=True, text=True)
    return {'porcelain': p.stdout.strip(), 'pass': (p.stdout.strip() == '')}


def case_h_config_carriers():
    """air.yaml 承载 R-032 新参数（confidence_min / target_max_age_s / offset 寿命与幅值）。"""
    y = _read(os.path.join(ROOT, 'src', 'agc_air', 'config', 'air.yaml')) or ''
    res = {
        'has_confidence_min': bool(re.search(r'^\s*confidence_min:', y, re.M)),
        'has_target_max_age': bool(re.search(r'^\s*target_max_age_s:', y, re.M)),
        'has_offset_timeout': bool(re.search(r'^\s*rl_offset_timeout_s:', y, re.M)),
        'has_offset_max': bool(re.search(r'^\s*rl_offset_max_m:', y, re.M)),
        'has_status_topic': 'tracking_status_topic' in y,
    }
    res['pass'] = all(res[k] for k in (
        'has_confidence_min', 'has_target_max_age', 'has_offset_timeout',
        'has_offset_max', 'has_status_topic'))
    return res


def case_i_scope_separation(bench):
    """**未**把常开冻结（正常 drop-out）与全链故障保证混为一项（如实标注限定范围）。"""
    note = (bench or {}).get('scope_note', '')
    res = {'scope_note': note,
           'mentions_logic_bench': bool('逻辑台架' in note),
           'not_full_chain': bool('非整链' in note)}
    res['pass'] = bool(res['mentions_logic_bench'] and res['not_full_chain'])
    return res


def dump(res, run_dir):
    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)
    path = os.path.join(run_dir, 'metrics', 'r032.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-28s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
        for k, v in t.items():
            if k != 'pass':
                print('      %s = %s' % (k, v))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='r032-bench')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    bench = _load(os.path.join(run_dir, 'r032', 'bench.json'))
    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        res['tests']['a_follower_death'] = case_a_follower_death(bench)
        res['tests']['b_old_ref_replay'] = case_b_old_ref_replay(bench)
        res['tests']['c_low_confidence'] = case_c_low_confidence(bench)
        res['tests']['d_offset_lifetime'] = case_d_offset_lifetime(bench)
        res['tests']['e_scripts_hygiene'] = case_e_scripts_hygiene()
        res['tests']['f_adr_0047'] = case_f_adr()
        res['tests']['g_no_abi_change'] = case_g_no_abi_change()
        res['tests']['h_config_carriers'] = case_h_config_carriers()
        res['tests']['i_scope_separation'] = case_i_scope_separation(bench)
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
