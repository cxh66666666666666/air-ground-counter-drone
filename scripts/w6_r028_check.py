#!/usr/bin/env python3
"""w6_r028_check —— R-028（episode 窗口统计、终态锁存与有效样本掩码）自包含核查。

**起过 Phase A/B 后**读 `summaries/harness-*.json`（6 个合成 case）与 `summaries/real_s11.json`，
逐项判定：
  (a) 新键齐备（`episode_windows` / `episode_window_metrics` / `evaluation_applicable` /
      `evaluation_na_reasons` / `terminal_state_source` / `terminal_state_latched` /
      `process_metrics_note`；`not_applicable` 含 `episode_window`/`window_scan`/`window_action`；
      `definition_version` 含 `shield_infeasible_rate_window`），且旧字段仍在；
  (b) 陈旧 scan 不重复计数（stale-scan：invalid_scan>0；valid+invalid_scan+invalid_action==total；
      triggered<=valid；rate==infeasible/triggered）；
  (c) 预热不改变窗口统计（warmup-short vs warmup-long：窗口 ticks/triggered/infeasible 差 ≤2，
      进程 ticks_total 差 ≥20）；
  (d) 终态锁存（terminal-latch：terminal_state==TRACKING_SUCCESS 且 latched 且 source 首个终态）；
  (e) 无任务 / 无 scan 拒绝（no-task → no_episode_window；noscan → no_scan_in_window、window_scan、
      shield_infeasible_rate is None）；
  (f) 分子分母 / 缺失比例 / not_applicable 输出；
  (g) 真实回合（real_s11：窗口非空、有 ticks、applicable、窗口 ticks_total < 进程 ticks_total）；
  (h) ADR 记录于 `docs/adr/0043-*.md`（`Status: accepted`、`r-028`、无 `§`）。

输出 `experiments/w6-task-eval/<run_id>/metrics/r028.json`；`overall_pass` = (a)∧…∧(h)。
"""

import argparse
import glob
import json
import os
import sys
from datetime import datetime

ROOT = '/home/chen/AGC'

NEW_KEYS = ('episode_windows', 'episode_window_metrics', 'evaluation_applicable',
            'evaluation_na_reasons', 'terminal_state_source', 'terminal_state_latched',
            'process_metrics_note')
OLD_KEYS = ('shield_infeasible_rate', 'ticks_total', 'ticks_triggered', 'ticks_infeasible',
            'duration_s', 'odom_valid_ratio', 'cmd_vel_rate_hz')
NA_KEYS = ('episode_window', 'window_scan', 'window_action')
DV_KEY = 'shield_infeasible_rate_window'
EWM_RAW = ('ticks_total', 'ticks_valid', 'ticks_invalid_scan', 'ticks_invalid_action',
           'triggered_ticks', 'infeasible_ticks', 'scan_samples',
           'ticks_missing_ratio_scan', 'ticks_missing_ratio_action')

HARNESS_CASES = {
    'no_task': 'harness-r028-no-task.json',
    'noscan': 'harness-r028-noscan.json',
    'stale': 'harness-r028-stale-scan.json',
    'warmup_short': 'harness-r028-warmup-short.json',
    'warmup_long': 'harness-r028-warmup-long.json',
    'latch': 'harness-r028-terminal-latch.json',
}


def _load(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _read(path):
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


def case_a_keys(summaries):
    """summaries: {name: summary} —— 逐份检查新键 / 旧键 / 掩码键 / 口径版本键。"""
    res = {}
    ok = True
    for name, sm in summaries.items():
        if not isinstance(sm, dict):
            res[name] = 'MISSING'
            ok = False
            continue
        missing_new = [k for k in NEW_KEYS if k not in sm]
        missing_old = [k for k in OLD_KEYS if k not in sm]
        na = sm.get('not_applicable')
        missing_na = [k for k in NA_KEYS if not isinstance(na, dict) or k not in na]
        dv = sm.get('definition_version')
        dv_ok = isinstance(dv, dict) and DV_KEY in dv
        res[name] = {'missing_new': missing_new, 'missing_old': missing_old,
                     'missing_na': missing_na, 'dv_key': dv_ok}
        if missing_new or missing_old or missing_na or not dv_ok:
            ok = False
    res['pass'] = ok
    return res


def case_b_stale(stale):
    res = {}
    ewm = (stale or {}).get('episode_window_metrics') if stale else None
    if not isinstance(ewm, dict):
        res['pass'] = False
        return res
    tt = ewm.get('ticks_total')
    tv = ewm.get('ticks_valid')
    tis = ewm.get('ticks_invalid_scan')
    tia = ewm.get('ticks_invalid_action')
    tr = ewm.get('triggered_ticks')
    inf = ewm.get('infeasible_ticks')
    rate = ewm.get('shield_infeasible_rate')
    res.update({'ticks_total': tt, 'ticks_valid': tv, 'ticks_invalid_scan': tis,
                'ticks_invalid_action': tia, 'triggered_ticks': tr,
                'infeasible_ticks': inf, 'rate': rate})
    sum_ok = isinstance(tt, int) and tt > 0 and tv + tis + tia == tt
    res['invalid_scan_positive'] = bool(isinstance(tis, int) and tis > 0)
    res['sum_ok'] = bool(sum_ok)
    res['triggered_le_valid'] = bool(tr is not None and tv is not None and tr <= tv)
    rate_ok = False
    if isinstance(tr, int) and tr > 0 and isinstance(inf, int) and rate is not None:
        rate_ok = abs(float(rate) - inf / tr) < 1e-6
    res['rate_ok'] = rate_ok
    res['pass'] = bool(res['invalid_scan_positive'] and res['sum_ok'] and
                       res['triggered_le_valid'] and rate_ok)
    return res


def case_c_warmup(short, long):
    res = {}
    se = (short or {}).get('episode_window_metrics') if short else None
    le = (long or {}).get('episode_window_metrics') if long else None
    if not isinstance(se, dict) or not isinstance(le, dict):
        res['pass'] = False
        return res
    d_ticks = abs(se.get('ticks_total', 0) - le.get('ticks_total', 0))
    d_trig = abs(se.get('triggered_ticks', 0) - le.get('triggered_ticks', 0))
    d_inf = abs(se.get('infeasible_ticks', 0) - le.get('infeasible_ticks', 0))
    proc_d = abs((short or {}).get('ticks_total', 0) - (long or {}).get('ticks_total', 0))
    res.update({'window_ticks': [se.get('ticks_total'), le.get('ticks_total')],
                'window_triggered': [se.get('triggered_ticks'), le.get('triggered_ticks')],
                'window_infeasible': [se.get('infeasible_ticks'), le.get('infeasible_ticks')],
                'diff_ticks': d_ticks, 'diff_triggered': d_trig, 'diff_infeasible': d_inf,
                'proc_ticks': [(short or {}).get('ticks_total'),
                               (long or {}).get('ticks_total')],
                'proc_ticks_diff': proc_d})
    res['pass'] = bool(d_ticks <= 2 and d_trig <= 2 and d_inf <= 2 and proc_d >= 20)
    return res


def case_d_latch(latch):
    res = {'terminal_state': (latch or {}).get('terminal_state') if latch else None,
           'terminal_state_latched': (latch or {}).get('terminal_state_latched') if latch else None,
           'terminal_state_source': (latch or {}).get('terminal_state_source') if latch else None}
    res['pass'] = bool(latch and latch.get('terminal_state') == 'TRACKING_SUCCESS' and
                       latch.get('terminal_state_latched') is True and
                       latch.get('terminal_state_source') == 'first_terminal_latched')
    return res


def case_e_reject(no_task, noscan):
    res = {}
    nt_ewm = (no_task or {}).get('episode_window_metrics') if no_task else None
    res['no_task_applicable'] = (no_task or {}).get('evaluation_applicable') if no_task else None
    res['no_task_na_reasons'] = (no_task or {}).get('evaluation_na_reasons') if no_task else None
    res['no_task_rate'] = (nt_ewm or {}).get('shield_infeasible_rate') if nt_ewm else None
    nt_ok = bool(no_task and no_task.get('evaluation_applicable') is False and
                 isinstance(no_task.get('evaluation_na_reasons'), list) and
                 'no_episode_window' in no_task['evaluation_na_reasons'] and
                 nt_ewm is not None and nt_ewm.get('shield_infeasible_rate') is None)

    ns_ewm = (noscan or {}).get('episode_window_metrics') if noscan else None
    ns_na = (noscan or {}).get('not_applicable') if noscan else None
    res['noscan_applicable'] = (noscan or {}).get('evaluation_applicable') if noscan else None
    res['noscan_na_reasons'] = (noscan or {}).get('evaluation_na_reasons') if noscan else None
    res['noscan_rate'] = (ns_ewm or {}).get('shield_infeasible_rate') if ns_ewm else None
    res['noscan_window_scan_na'] = (ns_na or {}).get('window_scan') if ns_na else None
    ns_ok = bool(noscan and noscan.get('evaluation_applicable') is False and
                 isinstance(noscan.get('evaluation_na_reasons'), list) and
                 'no_scan_in_window' in noscan['evaluation_na_reasons'] and
                 ns_ewm is not None and ns_ewm.get('shield_infeasible_rate') is None and
                 isinstance(ns_na, dict) and ns_na.get('window_scan') is True)
    res['pass'] = bool(nt_ok and ns_ok)
    return res


def case_f_raw_and_na(summaries):
    res = {}
    ok = True
    for name, sm in summaries.items():
        ewm = (sm or {}).get('episode_window_metrics') if sm else None
        if not isinstance(ewm, dict) or any(k not in ewm for k in EWM_RAW):
            res[name + '_ewm_raw'] = False
            ok = False
        else:
            res[name + '_ewm_raw'] = True
    ep_na_true = [name for name, sm in summaries.items()
                  if isinstance((sm or {}).get('not_applicable'), dict) and
                  sm['not_applicable'].get('episode_window') is True]
    res['episode_window_na_true_in'] = ep_na_true
    res['pass'] = bool(ok and ep_na_true)
    return res


def case_g_real(real):
    res = {}
    windows = (real or {}).get('episode_windows') if real else None
    ewm = (real or {}).get('episode_window_metrics') if real else None
    res['episode_windows_len'] = len(windows) if isinstance(windows, list) else None
    res['any_window_ticks'] = bool(isinstance(windows, list) and any(
        (w or {}).get('ticks_total', 0) > 0 for w in windows if isinstance(w, dict)))
    res['evaluation_applicable'] = (real or {}).get('evaluation_applicable') if real else None
    wtt = (ewm or {}).get('ticks_total') if ewm else None
    ptt = (real or {}).get('ticks_total') if real else None
    res['window_ticks_total'] = wtt
    res['process_ticks_total'] = ptt
    res['window_lt_process'] = bool(isinstance(wtt, int) and isinstance(ptt, int) and wtt < ptt)
    res['pass'] = bool(isinstance(windows, list) and windows and res['any_window_ticks'] and
                       real.get('evaluation_applicable') is True and res['window_lt_process'])
    return res


def case_h_adr():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0043-*.md')))
    res = {'path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0])
        res['status_accepted'] = bool('accepted' in t and 'Status' in t)
        res['no_section_ref'] = '§' not in t
        res['has_r028_ref'] = 'r-028' in t
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('no_section_ref') and res.get('has_r028_ref'))
    return res


def dump(res, out_dir):
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    path = os.path.join(out_dir, 'metrics', 'r028.json')
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
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-r028-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    summ_dir = os.path.join(out_dir, 'summaries')

    harness = {name: _load(os.path.join(summ_dir, fn)) for name, fn in HARNESS_CASES.items()}
    real = _load(os.path.join(summ_dir, 'real_s11.json'))

    summaries = dict(harness)
    summaries['real_s11'] = real

    res = {'run_id': run_id, 'summaries_dir': summ_dir, 'tests': {}}
    try:
        res['tests']['a_keys'] = case_a_keys(summaries)
        res['tests']['b_stale_no_double_count'] = case_b_stale(harness['stale'])
        res['tests']['c_warmup_window_invariant'] = case_c_warmup(
            harness['warmup_short'], harness['warmup_long'])
        res['tests']['d_terminal_latch'] = case_d_latch(harness['latch'])
        res['tests']['e_reject_no_task_noscan'] = case_e_reject(harness['no_task'], harness['noscan'])
        res['tests']['f_raw_and_na'] = case_f_raw_and_na(summaries)
        res['tests']['g_real_episode'] = case_g_real(real)
        res['tests']['h_adr'] = case_h_adr()
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, out_dir)


if __name__ == '__main__':
    sys.exit(main())
