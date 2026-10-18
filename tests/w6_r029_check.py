#!/usr/bin/env python3
"""w6_r029_check —— R-029（唯一运行身份与可复现证据）自包含核查。

**起过 run_w6_r029.sh 的 Phase A…E 后**读 experiments/w6-task-eval/<run_id>/ 下各证据，
逐项判定 (a)…(f)；输出 experiments/w6-task-eval/<run_id>/metrics/r029.json；
overall_pass = (a)∧…∧(f)。
"""

import argparse
import glob
import json
import os
import re
import sys

ROOT = '/home/chen/AGC'

SCENARIO = 'r029-probe'
SEED = 0
PROBE_RE = re.compile(r'^\d{8}-%s-%d-\d{6}-\d{4}$' % (re.escape(SCENARIO), SEED))

IDENTITY_FILES = ('params_applied.yaml', 'git_head.txt', 'git_status_porcelain.txt',
                  'git_diff.patch', 'git_untracked.txt', 'workspace_fingerprint.json',
                  'toolchain_world.json')


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


def _read_bytes(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, 'rb') as f:
        return f.read()


def case_a_unique(run_dir):
    res = {}
    a = (_read(os.path.join(run_dir, 'dup', 'dup_a', 'run_dir.txt')) or '').strip() or None
    b = (_read(os.path.join(run_dir, 'dup', 'dup_b', 'run_dir.txt')) or '').strip() or None
    res['dup_a'] = a
    res['dup_b'] = b
    res['different'] = bool(a and b and a != b)
    res['a_matches'] = bool(a and PROBE_RE.match(a))
    res['b_matches'] = bool(b and PROBE_RE.match(b))
    res['summary_a_exists'] = os.path.exists(os.path.join(run_dir, 'dup', 'dup_a', 'summary.json'))
    res['summary_b_exists'] = os.path.exists(os.path.join(run_dir, 'dup', 'dup_b', 'summary.json'))
    res['pass'] = bool(res['different'] and res['a_matches'] and res['b_matches'] and
                       res['summary_a_exists'] and res['summary_b_exists'])
    return res


def case_b_reject(run_dir, experiments_root):
    res = {}
    rc = (_read(os.path.join(run_dir, 'reject', 'rc.txt')) or '').strip() or None
    res['rc'] = rc
    existing = (_read(os.path.join(run_dir, 'reject', 'existing_dir.txt')) or '').strip() or None
    res['existing_dir'] = existing
    sentinel = _read_bytes(os.path.join(run_dir, 'reject', 'sentinel.json'))
    summary_path = os.path.join(experiments_root, 'w6-task-eval', existing or '',
                                'metrics', 'summary.json')
    res['summary_path'] = summary_path
    actual = _read_bytes(summary_path)
    res['sentinel_len'] = len(sentinel) if sentinel is not None else None
    res['actual_len'] = len(actual) if actual is not None else None
    res['unchanged'] = bool(sentinel is not None and actual is not None and sentinel == actual)
    before = _read_bytes(os.path.join(run_dir, 'reject', 'sentinel_before.json'))
    res['sentinel_before_matches'] = bool(before is None or before == sentinel)
    res['pass'] = bool(rc == '2' and res['unchanged'] and res['sentinel_before_matches'])
    return res


def case_c_episode(run_dir):
    res = {}
    sm_path = os.path.join(run_dir, 'episode', 'summaries', 'summary.json')
    sm = _load(sm_path)
    res['summary_path'] = sm_path
    res['episode_id'] = sm.get('episode_id') if isinstance(sm, dict) else None
    ep_txt = os.path.join(run_dir, 'episode', 'config', 'episode_id.txt')
    res['config_episode_id'] = (_read(ep_txt) or '').strip() or None
    pa = os.path.join(run_dir, 'episode', 'config', 'params_applied.yaml')
    res['params_applied_exists'] = os.path.exists(pa)
    res['pass'] = bool(res['episode_id'] == 7 and res['config_episode_id'] == '7' and
                       res['params_applied_exists'])
    return res


def case_d_identity(run_dir):
    res = {}
    cfg = os.path.join(run_dir, 'identity', 'config')
    res['missing'] = [f for f in IDENTITY_FILES if not os.path.exists(os.path.join(cfg, f))]
    wf = _load(os.path.join(cfg, 'workspace_fingerprint.json'))
    fp = wf.get('fingerprint_sha256') if isinstance(wf, dict) else None
    res['fingerprint_sha256'] = fp
    res['fingerprint_ok'] = bool(isinstance(fp, str) and len(fp) == 64)
    res['dirty'] = wf.get('dirty') if isinstance(wf, dict) else None
    res['dirty_ok'] = bool(isinstance(wf, dict) and wf.get('dirty') is True)
    tw = _load(os.path.join(cfg, 'toolchain_world.json'))
    res['ros_distro'] = tw.get('ros_distro') if isinstance(tw, dict) else None
    res['ros_distro_ok'] = res['ros_distro'] == 'jazzy'
    wh = tw.get('world_sha256') if isinstance(tw, dict) else None
    res['world_sha256'] = wh
    res['world_sha256_ok'] = bool(isinstance(wh, str) and re.fullmatch(r'[0-9a-fA-F]{64}', wh))
    res['pass'] = bool(not res['missing'] and res['fingerprint_ok'] and res['dirty_ok'] and
                       res['ros_distro_ok'] and res['world_sha256_ok'])
    return res


def case_e_runner(run_dir):
    res = {}
    er = _read(os.path.join(ROOT, 'scripts', 'episode_runner.py')) or ''
    res['no_fixed_sleep'] = 'time.sleep(18)' not in er
    res['has_wait_ready'] = 'def wait_ready' in er
    rep_path = os.path.join(run_dir, 'runner', 'metrics', 'report.json')
    rep = _load(rep_path)
    res['report_path'] = rep_path
    res['report_exists'] = isinstance(rep, dict)
    eps = rep.get('episodes') if isinstance(rep, dict) else None
    verified = []
    for e in (eps or []):
        rid = e.get('run_id')
        sc = e.get('scenario')
        sd = e.get('seed')
        if (rid and sc is not None and sd is not None and
                ('-%s-%s-' % (sc, sd)) in rid and e.get('same_run_verified') is True):
            verified.append(rid)
    res['same_run_verified_found'] = verified
    res['pass'] = bool(res['no_fixed_sleep'] and res['has_wait_ready'] and
                       res['report_exists'] and verified)
    return res


def case_f_adr():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0044-*.md')))
    res = {'path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = bool('accepted' in t and 'Status' in t)
        res['no_section_ref'] = '\u00a7' not in t
        res['has_r029_ref'] = 'r-029' in t
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('no_section_ref') and res.get('has_r029_ref'))
    return res


def dump(res, out_dir):
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    path = os.path.join(out_dir, 'metrics', 'r029.json')
    with open(path, 'w', encoding='utf-8') as f:
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
    ap.add_argument('--run-id', default='20261016-r029-0')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        res['tests']['a_unique_run_id'] = case_a_unique(run_dir)
        res['tests']['b_reject_existing_dir'] = case_b_reject(run_dir, args.experiments_root)
        res['tests']['c_episode_id_propagated'] = case_c_episode(run_dir)
        res['tests']['d_expanded_params_and_identity'] = case_d_identity(run_dir)
        res['tests']['e_runner_same_run_association'] = case_e_runner(run_dir)
        res['tests']['f_adr_0044'] = case_f_adr()
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
