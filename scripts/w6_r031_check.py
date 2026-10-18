#!/usr/bin/env python3
"""w6_r031_check —— R-031（runner 三层验收判定与完整性硬门）自包含核查。

**起过 run_w6_r031.sh 后**读 experiments/w6-task-eval/<run_id>/ 下的**实跑 runner 报告** +
**直接 import episode_runner 的判定 / 汇总纯函数**做逻辑核查，逐项判定 (a)…(g)；
输出 experiments/w6-task-eval/<run_id>/metrics/r031.json；overall_pass = (a)∧…∧(g)。

Case d 用**实跑** runner 报告（Phase A）；Case a/b/c 用**合成 episode** 直接验证
`classify_acceptance_layers` / `summarize_metrics`（**不**跑仿真，非口径假绿）。
"""

import argparse
import glob
import importlib.util
import json
import os
import re
import sys

ROOT = '/home/chen/AGC'
RUNNER_PY = os.path.join(ROOT, 'scripts', 'episode_runner.py')


def _load_runner():
    """从 scripts/ 直接 import episode_runner（调用其判定 / 汇总纯函数）。"""
    sys.path.insert(0, os.path.join(ROOT, 'scripts'))
    spec = importlib.util.spec_from_file_location('episode_runner', RUNNER_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


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


def case_a_three_layer_logic(er):
    """三层判定：正常全过 / 缺 summary 完整性失败 / 无效指标 → **全必过**方 pass。"""
    res = {}
    normal = {'scene': 'n', 'started': True, 'proc_exit_confirmed': True, 'summary_present': True,
              'same_run_verified': True, 'state_run_verified': True, 'expected_match': True,
              'shield_infeasible_rate': 0.1, 'reset_failure_rate': 0.0}
    miss = dict(normal, scene='m', summary_present=False, same_run_verified=False,
                shield_infeasible_rate=None, reset_failure_rate=None)
    bad = dict(normal, scene='b', shield_infeasible_rate=None)
    res['normal'] = er.classify_acceptance_layers(normal)
    res['missing_summary'] = er.classify_acceptance_layers(miss)
    res['bad_metrics'] = er.classify_acceptance_layers(bad)
    res['pass'] = bool(
        res['normal']['pass'] is True and all(res['normal'][k] for k in
                                             ('layer_integrity', 'layer_proposition', 'layer_metrics'))
        and res['missing_summary']['pass'] is False and res['missing_summary']['layer_integrity'] is False
        and res['missing_summary']['layer_proposition'] is True
        and res['bad_metrics']['pass'] is False and res['bad_metrics']['layer_metrics'] is False)
    return res


def case_b_missing_not_excluded(er):
    """缺 summary 回合**不**剔分母：valid + missing 各 1 → missing_count=1、ratio=0.5、值=唯一有效值。"""
    eps = [
        {'scene': 'ok', 'shield_infeasible_rate': 0.25, 'reset_failure_rate': 1.0},
        {'scene': 'miss', 'shield_infeasible_rate': None, 'reset_failure_rate': None},
    ]
    m = er.summarize_metrics(eps)
    sh = m['shield_infeasible_rate']
    res = {'shield': sh, 'reset': m['reset_failure_rate']}
    res['pass'] = bool(sh['valid_count'] == 1 and sh['missing_count'] == 1 and
                       abs(sh['missing_ratio'] - 0.5) < 1e-9 and abs(sh['value'] - 0.25) < 1e-9)
    return res


def case_c_empty_not_applicable(er):
    """空有效样本集合 → not_applicable（**不**返回 0 冒充有效均值）。"""
    eps = [{'scene': 'm1', 'shield_infeasible_rate': None, 'reset_failure_rate': None}]
    m = er.summarize_metrics(eps)
    res = {'shield_value': m['shield_infeasible_rate']['value'],
           'reset_value': m['reset_failure_rate']['value']}
    res['pass'] = bool(m['shield_infeasible_rate']['value'] == 'not_applicable' and
                       m['reset_failure_rate']['value'] == 'not_applicable')
    return res


def case_d_real_runner_report(run_dir):
    """**实跑** runner 报告：三层全过 + pass==n + metric_completeness 存在 + 无缺 summary 回合。"""
    res = {}
    rp = os.path.join(run_dir, 'runner', 'metrics', 'report.json')
    res['report_path'] = rp
    report = _load(rp)
    if not isinstance(report, dict):
        res['pass'] = False
        return res
    n = report.get('n_episodes', 0)
    res['n_episodes'] = n
    res['layer_integrity_count'] = report.get('layer_integrity_count')
    res['layer_proposition_count'] = report.get('layer_proposition_count')
    res['layer_metrics_count'] = report.get('layer_metrics_count')
    res['pass_count'] = report.get('pass_count')
    res['integrity_failures'] = report.get('integrity_failures')
    res['summary_missing_episodes'] = report.get('summary_missing_episodes')
    res['has_metric_completeness'] = bool(
        isinstance(report.get('metric_completeness'), dict) and
        'shield_infeasible_rate' in report['metric_completeness'])
    res['all_three_ok'] = bool(
        isinstance(report.get('metric_completeness'), dict) and
        'shield_infeasible_rate' in report['metric_completeness'])
    res['pass'] = bool(
        n > 0 and report.get('layer_integrity_count') == n and
        report.get('layer_proposition_count') == n and report.get('layer_metrics_count') == n and
        report.get('pass_count') == n and not res['summary_missing_episodes'] and
        res['has_metric_completeness'])
    return res


def _code_only(txt):
    """剥离整行注释，仅保留可执行行（避免注释中提及 `rm -rf` 被误判为实际调用）。"""
    return '\n'.join(s for s in (ln.strip() for ln in txt.splitlines()) if not s.startswith('#'))


def case_e_scripts_hygiene():
    """进程管理无 rm -rf / 裸名 pkill；runner 用 killpg + wait 确认退出（仅判**可执行行**）。"""
    res = {}
    runner_txt = _read(RUNNER_PY) or ''
    sh_txt = _read(os.path.join(ROOT, 'scripts', 'run_w6_r031.sh')) or ''
    runner_code = _code_only(runner_txt)
    sh_code = _code_only(sh_txt)
    res['runner_has_wait'] = 'proc.wait(' in runner_code
    res['runner_has_killpg'] = 'killpg' in runner_code
    res['runner_no_rm_rf'] = 'rm -rf' not in runner_code
    res['sh_no_rm_rf'] = 'rm -rf' not in sh_code
    # 裸名 pkill：`pkill -f "name"`（无 [x] 括号规避）——仅判可执行行
    bare = re.findall(r'pkill\s+-\w+\s+-f\s+"(?!\[)[^"]+"', runner_code + '\n' + sh_code)
    res['bare_pkill'] = bare
    res['pass'] = bool(res['runner_has_wait'] and res['runner_has_killpg'] and
                       res['runner_no_rm_rf'] and res['sh_no_rm_rf'] and not bare)
    return res


def case_f_adr():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0046-*.md')))
    res = {'path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = bool('Status' in t and 'accepted' in t)
        res['no_section_ref'] = '\u00a7' not in t
        res['has_r031_ref'] = bool('r-031' in t.lower())
        res['has_three_layer'] = ('三层' in t or '完整性' in t)
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('no_section_ref') and res.get('has_r031_ref') and
                       res.get('has_three_layer'))
    return res


def case_g_no_abi_change():
    """契约冻结 9 消息 ABI 未改（git status src/agc_msgs 空）。"""
    import subprocess
    res = {}
    p = subprocess.run(['git', 'status', '--porcelain', 'src/agc_msgs'],
                       cwd=ROOT, capture_output=True, text=True)
    res['porcelain'] = p.stdout.strip()
    res['pass'] = (p.stdout.strip() == '')
    return res


def dump(res, run_dir):
    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)
    path = os.path.join(run_dir, 'metrics', 'r031.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-40s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
        for k, v in t.items():
            if k != 'pass':
                print('      %s = %s' % (k, v))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261016-r031-0')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        er = _load_runner()
        res['tests']['a_three_layer_logic'] = case_a_three_layer_logic(er)
        res['tests']['b_missing_not_excluded'] = case_b_missing_not_excluded(er)
        res['tests']['c_empty_not_applicable'] = case_c_empty_not_applicable(er)
        res['tests']['d_real_runner_report'] = case_d_real_runner_report(run_dir)
        res['tests']['e_scripts_hygiene'] = case_e_scripts_hygiene()
        res['tests']['f_adr_0046'] = case_f_adr()
        res['tests']['g_no_abi_change'] = case_g_no_abi_change()
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
