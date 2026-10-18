#!/usr/bin/env python3
"""w6_r033_check —— R-033（UAV 联合安全屏障与显式不可行路径）自包含核查。

**起过 run_w6_r033.sh 后**读 `experiments/w6-task-eval/<run_id>/r033/bench.json`（Phase A
**实跑逻辑台架**证据）+ 源码 / 配置 / ADR 静态核查，逐项判定 (a)…(i)；
输出 `.../<run_id>/metrics/r033.json`；overall_pass = (a)∧…∧(i)。
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


def case_a_joint_overlap(bench):
    return _scenario(bench, 'a_joint_overlap')


def case_b_swept_through(bench):
    return _scenario(bench, 'b_swept_through')


def case_c_target_closure(bench):
    return _scenario(bench, 'c_target_closure')


def case_d_truth_clearance(bench):
    return _scenario(bench, 'd_truth_clearance')


def case_e_scripts_hygiene():
    """进程管理：无 `rm -rf`、无裸名 `pkill`；bench 用 killpg + wait（仅判可执行行）。"""
    res = {}
    bench_txt = _read(os.path.join(ROOT, 'scripts', 'w6_r033_bench.py')) or ''
    sh_txt = _read(os.path.join(ROOT, 'scripts', 'run_w6_r033.sh')) or ''
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
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0048-*.md')))
    res = {'path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = bool('Status' in t and 'accepted' in t)
        res['no_section_ref'] = '\u00a7' not in t
        res['has_r033_ref'] = bool('r-033' in t.lower())
        res['has_joint'] = bool('联合' in t and ('复检' in t or '约束' in t))
        res['has_infeasible'] = bool('不可行' in t)
        res['has_swept'] = bool('扫掠' in t)
        res['has_2d_limit'] = bool('二维' in t and ('限定' in t or 'unsupported' in t.lower()))
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('no_section_ref') and res.get('has_r033_ref') and
                       res.get('has_joint') and res.get('has_infeasible') and
                       res.get('has_swept') and res.get('has_2d_limit'))
    return res


def case_g_no_abi_change():
    p = subprocess.run(['git', 'status', '--porcelain', 'src/agc_msgs'],
                       cwd=ROOT, capture_output=True, text=True)
    return {'porcelain': p.stdout.strip(), 'pass': (p.stdout.strip() == '')}


def case_h_config_carriers():
    """air.yaml 承载 R-033 新参数（反应时间 / 最大速度 / 可达性余量）+ 原障碍表保留。"""
    y = _read(os.path.join(ROOT, 'src', 'agc_air', 'config', 'air.yaml')) or ''
    res = {
        'has_reaction_time': bool(re.search(r'^\s*reaction_time_s:', y, re.M)),
        'has_max_speed': bool(re.search(r'^\s*max_speed_mps:', y, re.M)),
        'has_reachability': bool(re.search(r'^\s*reachability_margin:', y, re.M)),
        'keeps_obstacle_margin': bool(re.search(r'^\s*obstacle_margin:', y, re.M)),
        'keeps_obstacle_arrays': all(re.search(r'^\s*obstacle_%s:' % k, y, re.M)
                                     for k in ('names', 'x', 'y', 'safety_radius')),
    }
    res['pass'] = all(res[k] for k in (
        'has_reaction_time', 'has_max_speed', 'has_reachability',
        'keeps_obstacle_margin', 'keeps_obstacle_arrays'))
    return res


def case_i_scope_2d_static_note(bench):
    """二维静态限定模型 + 动态障碍状态 `unsupported` **如实标注**。"""
    note = (bench or {}).get('scope_note', '')
    lim = (bench or {}).get('model_limitations', {}) or {}
    res = {
        'mentions_2d': bool('二维' in note),
        'mentions_limited_model': bool('限定实验模型' in note or '限定' in note),
        'dynamic_obstacle_unsupported': lim.get('dynamic_obstacle_state') == 'unsupported',
        'not_full_chain': bool('非整链' in note),
    }
    res['pass'] = bool(res['mentions_2d'] and res['mentions_limited_model'] and
                       res['dynamic_obstacle_unsupported'] and res['not_full_chain'])
    return res


def dump(res, run_dir):
    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)
    path = os.path.join(run_dir, 'metrics', 'r033.json')
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
    ap.add_argument('--run-id', default='r033-bench')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    bench = _load(os.path.join(run_dir, 'r033', 'bench.json'))
    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        res['tests']['a_joint_overlap'] = case_a_joint_overlap(bench)
        res['tests']['b_swept_through'] = case_b_swept_through(bench)
        res['tests']['c_target_closure'] = case_c_target_closure(bench)
        res['tests']['d_truth_clearance'] = case_d_truth_clearance(bench)
        res['tests']['e_scripts_hygiene'] = case_e_scripts_hygiene()
        res['tests']['f_adr_0048'] = case_f_adr()
        res['tests']['g_no_abi_change'] = case_g_no_abi_change()
        res['tests']['h_config_carriers'] = case_h_config_carriers()
        res['tests']['i_scope_2d_static_note'] = case_i_scope_2d_static_note(bench)
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
