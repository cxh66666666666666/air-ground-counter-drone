#!/usr/bin/env python3
"""w6_r036_check —— R-036（C1 高度语义统一与同时间证据曲线）自包含核查。

**起过 run_w6_r036.sh 后**读 `experiments/w6-task-eval/<run_id>/r036/bench.json`（Phase A **实跑台架**证据）
+ `r036/curves.csv`（同时间轴 5 曲线）+ 源码 / 配置 / ADR 静态核查，逐项判定 (a)…(j)；
C1 容限**读取** `src/agc_air/config/air.yaml`（`trajectory_tracker.ros__parameters.c1`，不复制参数）。
输出 `.../<run_id>/r036/check.json`（含 `overall_pass` 布尔）与 `.../<run_id>/metrics/r036.json`。
overall_pass = (a)∧…∧(j)。
"""

import argparse
import csv
import glob
import json
import os
import re
import subprocess
import sys

import yaml

ROOT = '/home/chen/AGC'
AIR_YAML = os.path.join(ROOT, 'src/agc_air/config/air.yaml')


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


def _c1_tols():
    with open(AIR_YAML, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    return doc['trajectory_tracker']['ros__parameters']['c1']


def _curves(bench, name):
    return ((bench or {}).get('evidence', {}).get('curves', {}) or {}).get(name, {}) or {}


def case_a_three_metrics(bench):
    """三指标（base / est / gt）分别报告（各自 P95 / max / 持续越限）且可区分。"""
    curves = (bench or {}).get('evidence', {}).get('curves', {}) or {}
    names = ['base', 'est', 'gt']
    present = {n: bool(curves.get(n)) for n in names}
    keys = ['speed_err_full_p95', 'speed_err_full_max', 'speed_err_sustained_s',
            'height_err_full_p95', 'height_err_full_max', 'height_err_sustained_s']
    complete = {n: all(curves.get(n, {}).get(k) is not None for k in keys) for n in names}
    distinguishable = bool((bench or {}).get('checks', {}).get('three_metrics_distinguishable'))
    res = {'present': present, 'stats_complete': complete, 'distinguishable': distinguishable,
           'gt_est_z_gap': None}
    gt_z = curves.get('gt', {}).get('height_z_median')
    est_z = curves.get('est', {}).get('height_z_median')
    if gt_z is not None and est_z is not None:
        res['gt_est_z_gap'] = abs(gt_z - est_z)
    res['pass'] = bool(all(present.values()) and all(complete.values()) and distinguishable)
    return res


def case_b_same_time_axis(bench, run_dir):
    """同时间轴：CSV 时间列单调且对齐（统一 ROS 时钟）。"""
    ta = (bench or {}).get('evidence', {}).get('time_axis', {}) or {}
    csv_path = os.path.join(run_dir, 'r036', 'curves.csv')
    csv_ok = False
    t_mono = base_mono = sp_mono = False
    n_rows = 0
    if os.path.exists(csv_path):
        with open(csv_path, newline='', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
        n_rows = len(rows)
        if n_rows > 10:
            ts = [float(r['t_rel']) for r in rows]
            bs = [float(r['base_stamp']) for r in rows if r['base_stamp'] != '']
            ss = [float(r['sp_recv']) for r in rows if r['sp_recv'] != '']
            mono = lambda v: all(v[i] <= v[i + 1] + 1e-9 for i in range(len(v) - 1)) if len(v) > 1 else False
            t_mono, base_mono, sp_mono = mono(ts), mono(bs), mono(ss)
            csv_ok = t_mono and base_mono and sp_mono
    res = {
        'csv_exists': os.path.exists(csv_path), 'n_rows': n_rows,
        'csv_t_monotonic': t_mono, 'csv_base_stamp_monotonic': base_mono,
        'csv_sp_recv_monotonic': sp_mono,
        't_monotonic': bool(ta.get('t_monotonic')),
        'base_stamp_monotonic': bool(ta.get('base_stamp_monotonic')),
        'sp_recv_monotonic': bool(ta.get('sp_recv_monotonic')),
        'align_err_p95_s': ta.get('align_err_p95_s'), 'align_err_max_s': ta.get('align_err_max_s'),
        'align_tol_s': ta.get('align_tol_s', 0.25),
    }
    aligned = (ta.get('align_err_p95_s') is not None and
               ta['align_err_p95_s'] <= ta.get('align_tol_s', 0.25))
    res['aligned'] = aligned
    res['pass'] = bool(csv_ok and res['t_monotonic'] and res['base_stamp_monotonic'] and
                       res['sp_recv_monotonic'] and aligned)
    return res


def case_c_height_datum(bench):
    """base 与 effective 的**高度基准一致**（z 差在容限内）；z_offset 可辨。"""
    tol = float(_c1_tols()['height_datum_tol_m'])
    hd = (bench or {}).get('evidence', {}).get('height_datum', {}) or {}
    p95 = hd.get('base_minus_eff_p95')
    mx = hd.get('base_minus_eff_max')
    z_ok = (p95 is not None and p95 <= tol)
    zoff = (bench or {}).get('bench', {}).get('z_offset_seen')
    res = {'tol_m': tol, 'base_minus_eff_p95': p95, 'base_minus_eff_max': mx,
           'z_offset_seen': zoff,
           'z_offset_detectable': bool((bench or {}).get('checks', {}).get('z_offset_detectable'))}
    res['pass'] = bool(z_ok and res['z_offset_detectable'])
    return res


def case_d_p95_max_sustained(bench):
    """P95 / max / 持续越限 三值齐备；且**稳态 P95** 与**全程 max** 分开（不以稳态 P95 自证全程 max）。"""
    curves = (bench or {}).get('evidence', {}).get('curves', {}) or {}
    out = {}
    ok = True
    for n in ('base', 'est', 'gt'):
        c = curves.get(n, {}) or {}
        full_p95, full_max, steady_p95 = (c.get('speed_err_full_p95'),
                                          c.get('speed_err_full_max'),
                                          c.get('speed_err_steady_p95'))
        sust = c.get('speed_err_sustained_s')
        h_full_p95, h_full_max = c.get('height_err_full_p95'), c.get('height_err_full_max')
        h_sust = c.get('height_err_sustained_s')
        have_all = all(x is not None for x in (full_p95, full_max, steady_p95, sust,
                                                h_full_p95, h_full_max, h_sust))
        max_ge_p95 = (full_max is not None and full_p95 is not None and full_max + 1e-12 >= full_p95)
        # 全程 max 必须 ≥ 稳态 P95（证明 max 取自全程、非稳态 P95 冒充）。
        fullmax_ge_steadyp95 = (full_max is not None and steady_p95 is not None and
                                full_max + 1e-12 >= steady_p95)
        out[n] = {'have_all': have_all, 'full_max_ge_full_p95': max_ge_p95,
                  'full_max_ge_steady_p95': fullmax_ge_steadyp95}
        ok = ok and have_all and max_ge_p95 and fullmax_ge_steadyp95
    out['separate_fields'] = bool((bench or {}).get('checks', {}).get(
        'steady_p95_reported_separately'))
    out['pass'] = bool(ok and out['separate_fields'])
    return out


def case_e_ref_and_body_speed(bench):
    """参考 |v|≡目标（max 偏差 ≤ 数值容限）；机体（world GT）**稳态**速度误差 P95 ≤ 裁定容限。"""
    c1 = _c1_tols()
    tol_ref = float(c1['ref_speed_tol'])
    tol_body = float(c1['body_speed_p95_tol_mps'])
    target = float(c1['ref_speed_target'])
    base = _curves(bench, 'base')
    gt = _curves(bench, 'gt')
    ref_max = base.get('speed_err_full_max')
    body_p95 = gt.get('speed_err_steady_p95')
    res = {'target_speed': target, 'ref_speed_tol': tol_ref, 'body_speed_p95_tol': tol_body,
           'base_speed_err_full_max': ref_max, 'gt_speed_err_steady_p95': body_p95,
           'base_speed_abs_p95': base.get('speed_abs_p95')}
    res['pass'] = bool(ref_max is not None and ref_max <= tol_ref and
                       body_p95 is not None and body_p95 <= tol_body)
    return res


def case_f_static_source():
    """静态：发布 base_ref 前**加 z_offset_**（源文件含该 offset 应用）；publishTrackStatus 含 z_offset。"""
    tt = _read(os.path.join(ROOT, 'src/agc_air/src/trajectory_tracker.cpp')) or ''
    # 发布副本加 z_offset：base_ref 副本 pos.z() += z_offset_ 在 ref_pub_->publish 之前。
    i_add = tt.find('base_ref_out.pos.z() += z_offset_')
    i_pub = tt.find('ref_pub_->publish(agc_air::toMsg(base_ref_out')
    res = {
        'applies_z_offset_to_base_ref': i_add >= 0,
        'offset_before_ref_publish': (0 <= i_add and 0 <= i_pub and i_add < i_pub),
        'track_status_has_z_offset': ('z_offset' in tt),
        'track_status_has_height_datum': ('height_datum' in tt),
    }
    res['pass'] = all(res.values())
    return res


def case_g_stock_retained():
    """缺失 stock 材料（/tmp/stock_rec*.csv）在 ADR-0051 **作为未证实归因保留**（不静默删除）。"""
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0051-*.md')))
    res = {'adr_path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['mentions_stock'] = 'stock' in t.lower()
        res['mentions_tmp_csv'] = '/tmp/stock_rec' in t
        res['marks_unverified'] = ('未证实' in t)
        res['retains_record'] = ('保留' in t)
    res['pass'] = bool(res['exists'] and res.get('mentions_stock') and
                       res.get('mentions_tmp_csv') and res.get('marks_unverified') and
                       res.get('retains_record'))
    return res


def case_h_no_abi_no_second_commander():
    """冻结 9 消息 ABI 未改（src/agc_msgs 无改动）；setpoint 发布者仍单一（offboard_interface.cpp）；无 § 引用。"""
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
    sec = subprocess.run(['grep', '-rnE', '§[0-9]', 'docs/', '.trae/specs/'],
                         cwd=ROOT, capture_output=True, text=True)
    res = {'agc_msgs_porcelain': p.stdout.strip(), 'abi_clean': abi_clean,
           'setpoint_publisher_files': hits,
           'single_setpoint_publisher': hits == ['offboard_interface.cpp'],
           'section_refs': sec.stdout.strip()}
    res['pass'] = bool(abi_clean and res['single_setpoint_publisher'] and
                       sec.stdout.strip() == '')
    return res


def case_i_adr_0051():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0051-*.md')))
    res = {'path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = ('## Status' in t and 'accepted' in t)
        res['no_section_ref'] = '\u00a7' not in t
        res['has_r036_ref'] = 'r-036' in t.lower()
        res['has_evidence_section'] = '取证' in t
        res['has_world_height'] = ('世界 ENU' in t) or ('world_enu' in t.lower())
        res['has_decision'] = '## 决策' in t
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and res.get('no_section_ref') and
                       res.get('has_r036_ref') and res.get('has_evidence_section') and
                       res.get('has_world_height') and res.get('has_decision'))
    return res


def case_j_scripts_hygiene():
    """无 `rm -rf`、无裸名 `pkill`；bench 用 killpg + wait。"""
    bench_code = _code_only(_read(os.path.join(ROOT, 'scripts/w6_r036_bench.py')) or '')
    sh_code = _code_only(_read(os.path.join(ROOT, 'scripts/run_w6_r036.sh')) or '')
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


def dump(res, run_dir):
    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(run_dir, 'r036'), exist_ok=True)
    with open(os.path.join(run_dir, 'r036', 'check.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'metrics', 'r036.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-28s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL',
                          os.path.join(run_dir, 'r036', 'check.json')))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261017-w6-r036-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    bench = _load(os.path.join(run_dir, 'r036', 'bench.json'))
    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        res['tests']['a_three_metrics'] = case_a_three_metrics(bench)
        res['tests']['b_same_time_axis'] = case_b_same_time_axis(bench, run_dir)
        res['tests']['c_height_datum'] = case_c_height_datum(bench)
        res['tests']['d_p95_max_sustained'] = case_d_p95_max_sustained(bench)
        res['tests']['e_ref_and_body_speed'] = case_e_ref_and_body_speed(bench)
        res['tests']['f_static_source'] = case_f_static_source()
        res['tests']['g_stock_retained'] = case_g_stock_retained()
        res['tests']['h_no_abi_no_second_commander'] = case_h_no_abi_no_second_commander()
        res['tests']['i_adr_0051'] = case_i_adr_0051()
        res['tests']['j_scripts_hygiene'] = case_j_scripts_hygiene()
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
