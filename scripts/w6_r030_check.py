#!/usr/bin/env python3
"""w6_r030_check —— R-030（resetfail 证据矛盾登记与唯一新 run 重做）自包含核查。

**起过 run_w6_r030.sh 后**读 experiments/w6-task-eval/<run_id>/ 下各证据，逐项判定 (a)…(h)；
输出 experiments/w6-task-eval/<run_id>/metrics/r030.json；overall_pass = (a)∧…∧(h)。

本 check **不**重建 / 伪造旧原件：Case g 断言旧丢失路径不存在，Case h 断言 Runbook `#status` 未被凭推断改写。
"""

import argparse
import glob
import json
import os
import re
import sys

ROOT = '/home/chen/AGC'

RUN_DIR_RE = re.compile(r'^\d{8}-c9-resetfail-0-\d{6}-\d{4}$')
OLD_LOST_DIR = '20261014-c9-resetfail-0'

IDENTITY_FILES = ('params_applied.yaml', 'git_head.txt', 'git_status_porcelain.txt',
                  'git_diff.patch', 'git_untracked.txt', 'workspace_fingerprint.json',
                  'toolchain_world.json')

HEX64 = re.compile(r'[0-9a-fA-F]{64}')


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


def case_a_start_rejected(run_dir):
    res = {}
    txt = _read(os.path.join(run_dir, 'log', 'start_call.log')) or ''
    res['start_call_log'] = os.path.join(run_dir, 'log', 'start_call.log')
    res['has_success_false'] = bool(re.search(r'success\s*[:=]\s*[Ff]alse', txt))
    res['has_reset_failed'] = 'reset_failed' in txt
    res['pass'] = bool(res['has_success_false'] and res['has_reset_failed'])
    return res


def case_b_summary_stats(run_dir):
    res = {}
    sm_path = os.path.join(run_dir, 'resetfail', 'summary.json')
    sm = _load(sm_path)
    res['summary_path'] = sm_path
    res['reset_retries'] = sm.get('reset_retries') if isinstance(sm, dict) else None
    res['reset_failed'] = sm.get('reset_failed') if isinstance(sm, dict) else None
    res['reset_failure_rate'] = sm.get('reset_failure_rate') if isinstance(sm, dict) else None
    res['pass'] = bool(isinstance(sm, dict) and sm.get('reset_retries') == 3 and
                       sm.get('reset_failed') is True and sm.get('reset_failure_rate') == 1.0)
    return res


def case_c_raw_log(run_dir):
    res = {}
    log_path = os.path.join(run_dir, 'log', 'launch.log')
    txt = _read(log_path) or ''
    res['launch_log'] = log_path
    res['has_clearance_rejection'] = 'car_target_clearance' in txt
    res['has_reset_retry'] = bool('scenario/reset' in txt and '重试' in txt)
    res['pass'] = bool(res['has_clearance_rejection'] and res['has_reset_retry'])
    return res


def case_d_identity(run_dir):
    res = {}
    cfg = os.path.join(run_dir, 'resetfail', 'config')
    res['missing'] = [f for f in IDENTITY_FILES if not os.path.exists(os.path.join(cfg, f))]
    wf = _load(os.path.join(cfg, 'workspace_fingerprint.json'))
    fp = wf.get('fingerprint_sha256') if isinstance(wf, dict) else None
    res['fingerprint_sha256'] = fp
    res['fingerprint_ok'] = bool(isinstance(fp, str) and re.fullmatch(r'[0-9a-fA-F]{64}', fp))
    tw = _load(os.path.join(cfg, 'toolchain_world.json'))
    res['ros_distro'] = tw.get('ros_distro') if isinstance(tw, dict) else None
    res['ros_distro_ok'] = res['ros_distro'] == 'jazzy'
    wh = tw.get('world_sha256') if isinstance(tw, dict) else None
    res['world_sha256'] = wh
    res['world_sha256_ok'] = bool(isinstance(wh, str) and re.fullmatch(r'[0-9a-fA-F]{64}', wh))
    res['pass'] = bool(not res['missing'] and res['fingerprint_ok'] and
                       res['ros_distro_ok'] and res['world_sha256_ok'])
    return res


def case_e_unique_identity(run_dir):
    res = {}
    d = (_read(os.path.join(run_dir, 'resetfail', 'run_dir.txt')) or '').strip() or None
    res['run_dir'] = d
    res['matches_pattern'] = bool(d and RUN_DIR_RE.match(d))
    res['not_old_lost'] = bool(d and d != OLD_LOST_DIR)
    sm = _load(os.path.join(run_dir, 'resetfail', 'summary.json'))
    res['summary_run_id'] = sm.get('run_id') if isinstance(sm, dict) else None
    res['run_id_matches_dir'] = bool(d and res['summary_run_id'] == d)
    res['pass'] = bool(res['matches_pattern'] and res['not_old_lost'] and res['run_id_matches_dir'])
    return res


def case_f_adr():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0045-*.md')))
    res = {'path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = bool('Status' in t and 'accepted' in t)
        res['no_section_ref'] = '\u00a7' not in t
        res['has_r030_ref'] = bool('r-030' in t.lower())
        res['has_loss_registration'] = any(k in t for k in ('rm -rf', '永久丢失', '不可恢复'))
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('no_section_ref') and res.get('has_r030_ref') and
                       res.get('has_loss_registration'))
    return res


def case_g_not_forged(experiments_root):
    res = {}
    old = os.path.join(experiments_root, 'w6-task-eval', OLD_LOST_DIR, 'metrics', 'summary.json')
    res['old_summary_path'] = old
    res['old_summary_exists'] = os.path.exists(old)
    res['pass'] = not res['old_summary_exists']
    return res


def case_h_runbook_untouched():
    res = {}
    p = os.path.join(ROOT, 'docs', 'ROS2迁移步骤.md')
    t = _read(p) or ''
    res['runbook'] = p
    res['has_old_ref'] = '20261014-c9-resetfail' in t
    res['has_new_ref'] = '20261016-r030' in t
    res['pass'] = bool(not res['has_old_ref'] and not res['has_new_ref'])
    return res


def dump(res, out_dir):
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    path = os.path.join(out_dir, 'metrics', 'r030.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-48s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
        for k, v in t.items():
            if k != 'pass':
                print('      %s = %s' % (k, v))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261016-r030-0')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        res['tests']['a_start_reset_rejected'] = case_a_start_rejected(run_dir)
        res['tests']['b_summary_reset_stats_nonzero'] = case_b_summary_stats(run_dir)
        res['tests']['c_raw_log_clearance_rejection'] = case_c_raw_log(run_dir)
        res['tests']['d_run_config_and_identity'] = case_d_identity(run_dir)
        res['tests']['e_unique_run_identity'] = case_e_unique_identity(run_dir)
        res['tests']['f_adr_0045'] = case_f_adr()
        res['tests']['g_not_forged_old_summary'] = case_g_not_forged(args.experiments_root)
        res['tests']['h_runbook_status_untouched'] = case_h_runbook_untouched()
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
