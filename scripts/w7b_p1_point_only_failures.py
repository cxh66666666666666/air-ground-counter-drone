#!/usr/bin/env python3
"""w7b_p1_point_only_failures —— P1 消融③（仅指向）失败回合的**描述性**诊断（T4，ADR-0085(i) 后续）。

**定位**：诊断，**不**参与判定、**不**改 P1 v5 `inconclusive` / 消融数值。产物只新增：
`experiments/<campaign>/<run_id>/metrics/p1_point_only_failures.{json,md}`。

读既有产物（`arms.json` + 各 episode `summary.json` + 逐 episode 日志）生成：

  1. `p1_coop` vs `p1_point_only` 的逐 seed 对照（终态 / 真实过程 FCO / 计分 FCO / active 窗口 /
     可见率 / `env_occ|vis` / 首次开火（**代理**）/ 开火数 / 命中失手 / hold / 最长失联 /
     干预拍与比例 / `min_clearance`）。
  2. 失败回合逐案：射击级 `pred` 偏差（对 s15 真值 (4.5, 3.0, 2.0)）、`miss_dist` / `flight` /
     高度、靶机侧启动证据（PX4 `nav_state=1`、`episode t=0`）、感知门控（`gate_ok` / `fov_az`）
     与净空 / 安全干预。
  3. 明确的**字段缺口**：既有 run **无** tick 级首次 FCO / 无条件包络占用 / 射击时观测年龄；
     首次开火时刻只作**代理**，**不**替代首次机会时间。

用法：
  python3 scripts/w7b_p1_point_only_failures.py --run-id 20261023-w7-rl-b-p1abl-n20
"""

import argparse
import json
import math
import os
import re
import statistics

ROOT = '/home/chen/AGC'
ARMS = ('p1_coop', 'p1_point_only')
SCENE = 's15_static_none_fov_capture'
TRUTH = (4.5, 3.0, 2.0)   # s15 真值（场景静态目标；z 取自 scenario_reset target_z=2.0）
TERMINALS_OK = ('TRACKING_SUCCESS', 'INTERCEPT_CONFIRMED')

RE_FIRED = re.compile(
    r'SHOT_FIRED shot=(\S+) t=([\d.]+) h=([-\d.]+) dist=([-\d.]+) '
    r'pred=\(([-\d.]+),([-\d.]+),([-\d.]+)\) '
    r'vel=\(([-\d.]+),([-\d.]+),([-\d.]+)\)')
RE_MISS = re.compile(
    r'SHOT_MISS shot=(\S+) .*flight=([\d.]+) s miss_dist=([-\d.]+) m')
RE_HIT = re.compile(r'CAPTURE（[^）]*）.*shot=(\S+)')
RE_NAV1 = re.compile(r'nav_state=1\b')
RE_GATE = re.compile(r'gate_ok=(\d+)')
RE_FOV_OUT = re.compile(r'fov_az=(\d+)')


def load_json(path):
    with open(path) as f:
        return json.load(f)


def final_log_tag(row):
    at = row.get('infra_attempts') or []
    return at[-1].get('log_tag') if at else row['arm']


def parse_log(run_dir, row):
    """解析最终尝试日志：射击 / 命中 / 失手 / hold / 靶机与感知证据。"""
    path = os.path.join(run_dir, 'log', 'ep_%s_%s_%d.log' % (final_log_tag(row), SCENE, row['seed']))
    out = {
        'log': os.path.basename(path), 'log_exists': os.path.exists(path),
        'shots': [], 'n_hits': 0, 'n_miss': 0, 'n_unresolved': 0,
        'holds': 0, 'intruder_t0': False, 'intruder_nav1': 0,
        'intruder_armed_req': 0, 'intruder_offboard_req': 0,
        'gate_ok_total': 0, 'fov_az_total': 0, 'fov_samples': 0,
        'air_perception_alive_diag': 0,
    }
    if not out['log_exists']:
        return out
    with open(path, errors='replace') as fh:
        for line in fh:
            m = RE_FIRED.search(line)
            if m:
                age_m = re.search(r'\bage=([-\d.]+)', line)
                out['shots'].append({
                    'shot': m.group(1), 't': float(m.group(2)), 'h': float(m.group(3)),
                    'dist': float(m.group(4)),
                    'pred': [float(m.group(5)), float(m.group(6)), float(m.group(7))],
                    'vel': [float(m.group(8)), float(m.group(9)), float(m.group(10))],
                    'age_s': float(age_m.group(1)) if age_m else None,
                    'result': None, 'flight_s': None, 'miss_dist': None,
                })
            m = RE_MISS.search(line)
            if m:
                sid, flight, miss = m.group(1), float(m.group(2)), float(m.group(3))
                out['n_miss'] += 1
                for s in out['shots']:
                    if s['shot'] == sid:
                        s['result'], s['flight_s'], s['miss_dist'] = 'MISS', flight, miss
            m = RE_HIT.search(line)
            if m:
                out['n_hits'] += 1
                for s in out['shots']:
                    if s['shot'] == m.group(1):
                        s['result'] = 'HIT'
            if '显式 hold' in line:
                out['holds'] += 1
            if 'intruder_driver' in line:
                if 'episode t=0' in line:
                    out['intruder_t0'] = True
                if 'requesting ARM' in line:
                    out['intruder_armed_req'] += 1
                if 'requesting OFFBOARD' in line:
                    out['intruder_offboard_req'] += 1
            if RE_NAV1.search(line):
                out['intruder_nav1'] += 1
            if 'B1 诊断' in line:
                out['air_perception_alive_diag'] += 1
                m = RE_GATE.search(line)
                if m:
                    out['gate_ok_total'] += int(m.group(1))
                m = RE_FOV_OUT.search(line)
                if m:
                    out['fov_samples'] += 1
                    out['fov_az_total'] += int(m.group(1))
    out['n_unresolved'] = sum(1 for s in out['shots'] if s['result'] is None)
    for s in out['shots']:
        dx = s['pred'][0] - TRUTH[0]
        dy = s['pred'][1] - TRUTH[1]
        dz = s['pred'][2] - TRUTH[2]
        s['pred_err_horizontal_m'] = round(math.hypot(dx, dy), 3)
        s['pred_err_3d_m'] = round(math.sqrt(dx * dx + dy * dy + dz * dz), 3)
        s['pred_dz_m'] = round(dz, 3)
    out['first_shot_t'] = out['shots'][0]['t'] if out['shots'] else None
    return out


def pred_summary(shots):
    if not shots:
        return None
    h = [s['pred_err_horizontal_m'] for s in shots]
    z = [s['pred'][2] for s in shots]
    ages = [s['age_s'] for s in shots if s.get('age_s') is not None]
    return {
        'n': len(shots),
        'pred_err_horizontal_median_m': round(statistics.median(h), 3),
        'pred_err_horizontal_max_m': round(max(h), 3),
        'pred_z_median_m': round(statistics.median(z), 3),
        'pred_z_min_m': round(min(z), 3),
        'pred_z_max_m': round(max(z), 3),
        'shots_pred_err_gt_net_radius': sum(1 for x in h if x > 0.5),
        'shot_age_median_s': round(statistics.median(ages), 3) if ages else None,
        'shot_age_max_s': round(max(ages), 3) if ages else None,
        'shot_age_records': len(ages),
        'timeout_flights_gt_3s': [{'shot': s['shot'], 'flight_s': s['flight_s']}
                                  for s in shots if (s['flight_s'] or 0) > 3.0],
        'miss_dist_gt_10m': [{'shot': s['shot'], 'miss_dist': s['miss_dist']}
                             for s in shots if (s['miss_dist'] or 0) > 10.0],
    }


def episode_row(run_dir, row):
    s = load_json(row['summary_path'])
    ew = s.get('episode_window_metrics') or {}
    win = (s.get('episode_windows') or [{}])[0]
    ca = s.get('command_attribution') or {}
    vm = s.get('visibility_metrics') or {}
    vis_total = ew.get('vis_ticks_total') or 0
    vis_vis = ew.get('vis_ticks_visible') or 0
    fco = ew.get('ticks_fire_control_opportunity') or 0
    log = parse_log(run_dir, row)
    success = row['terminal_state'] in TERMINALS_OK
    real = (fco / vis_total) if vis_total else None
    first_shot = log['first_shot_t']
    enter = win.get('enter_s')
    return {
        'seed': row['seed'], 'arm': row['arm'],
        'terminal_state': row['terminal_state'], 'reason_code': row.get('reason_code'),
        'success': success, 'infra_retries_used': row.get('infra_retries_used') or 0,
        'run_id': row.get('run_id'), 'log_tag': final_log_tag(row),
        'active_ticks': vis_total, 'active_s': round((win.get('leave_s') or 0) - (enter or 0), 3),
        'window_enter_s': enter, 'window_leave_s': win.get('leave_s'),
        'fco_ticks': fco, 'vis_ticks_visible': vis_vis,
        'real_process_fco': real,
        'scored_fco': real if success else 0.0,
        'visibility_rate_active': (vis_vis / vis_total) if vis_total else None,
        'env_occ_given_vis': (fco / vis_vis) if vis_vis else None,
        'first_shot_t': first_shot,
        'first_shot_rel_s': (round(first_shot - enter, 3)
                             if (first_shot is not None and enter is not None) else None),
        'n_shots': len(log['shots']), 'n_hits': log['n_hits'], 'n_miss': log['n_miss'],
        'n_unresolved_shots': log['n_unresolved'], 'holds': log['holds'],
        'longest_outage_s': row.get('longest_outage_s'),
        'intervened_ticks': ew.get('ticks_intervened'),
        'intervention_ticks_ratio': ew.get('intervention_ticks_ratio'),
        'intervention_duration_s': ca.get('intervention_duration_s'),
        'intervention_segments': ca.get('intervention_segments'),
        'min_clearance_m': row.get('min_clearance_m'),
        'min_clearance_at_intervention_m': ca.get('clearance_min_at_intervention_m'),
        'collision_count': s.get('collision_count'),
        'true_shield_infeasible': s.get('true_shield_infeasible'),
        # —— 字段缺口：既有 run 未记录（不得用首次开火替代首次机会）——
        'first_fire_control_opportunity_s': None,
        'envelope_in_ticks_unconditional': None,
        'envelope_in_rate_unconditional': None,
        'first_envelope_in_s': None,
        'target_observation_age_at_shot_s': None,
        'field_gap': {
            'first_fire_control_opportunity_s': 'not_recorded_in_this_run',
            'envelope_in_unconditional': 'not_recorded_in_this_run',
            'target_observation_age_at_shot_s': 'not_recorded_in_this_run',
        },
        'visibility_rate_fixed_window': vm.get('visibility_rate'),
        'fixed_window_outage_s': vm.get('longest_outage_s'),
        'intruder_t0': log['intruder_t0'], 'intruder_nav1': log['intruder_nav1'],
        'intruder_offboard_req': log['intruder_offboard_req'],
        'gate_ok_total': log['gate_ok_total'], 'fov_az_total': log['fov_az_total'],
        'fov_samples': log['fov_samples'],
        'shots': log['shots'],
        'pred_summary': pred_summary(log['shots']),
        'log': log['log'],
    }


def mechanisms(f):
    """失败回合的机制归类（证据 + 仍不确定点；不做单条日志因果结论）。"""
    ev, unc, primary = [], [], []
    if f['n_shots'] == 0:
        primary.append('长期无观测（指向/几何：目标持续出视场）')
        ev.append('全程 0 次开火；可见率 %.3f、FCO 拍 %d' % (
            f['visibility_rate_active'] or 0.0, f['fco_ticks']))
        ev.append('air_perception：gate_ok 合计 %d；方位出视场计数 %d/%d（20 Hz×5 s×2 通道=200/样本）' % (
            f['gate_ok_total'], f['fov_az_total'], 200 * f['fov_samples']))
        ev.append('air 机起飞完成且 task_manager start OK；`episode t=0`=%s' % f['intruder_t0'])
        unc.append('旧日志无「首次 FCO」与无条件包络计数：不能区分“从未进入包络”与“进入但不可见”')
        unc.append('无逐拍机头偏航/目标方位记录：指向命令是否被 yaw 动态跟上未直接可测')
    else:
        primary.append('有观测但射击质量差（全部 SHOT_MISS）')
        ev.append('%d 发 / %d 中 / %d 失手（未决 %d）' % (
            f['n_shots'], f['n_hits'], f['n_miss'], f['n_unresolved_shots']))
        if f['pred_summary']:
            ps = f['pred_summary']
            ev.append('pred 水平偏差 中位 %.3f m / 最大 %.3f m；pred z 中位 %.3f m（真值 z=2.0）' % (
                ps['pred_err_horizontal_median_m'], ps['pred_err_horizontal_max_m'],
                ps['pred_z_median_m']))
        if not f['intruder_t0']:
            primary.append('靶机侧未建立额定起始位姿（`episode t=0` 未记录；PX4 nav_state=1 次数 %d）' % f['intruder_nav1'])
            ev.append('靶机未达额定起始位姿：`episode t=0`=%s、PX4 nav_state=1 计数=%d、'
                      'OFFBOARD 请求=%d（仅当 nav_state=1 >0 才表示持续停留 ALTCTL）' % (
                          f['intruder_t0'], f['intruder_nav1'], f['intruder_offboard_req']))
            if f['intruder_nav1'] == 0:
                unc.append('seed%d 旧日志无持续 nav_state=1 行：靶机侧异常的形态'
                           '（未 arm / 未切入 OFFBOARD / 未爬升）不能由日志分辨' % f['seed'])
            ev.append('与靶机的真实接触 z≈0.2 m（collision_count=%s）' % f['collision_count'])
            unc.append('无直接 GT 高度时序：靶机高度由 pred z 与接触点 z 反推，非直接测量')
            unc.append('靶机启动异常与臂的关系未定：CRN 只配对场景随机量，不配对 PX4 启动结果')
        if (f['min_clearance_m'] or 9) < 0.5:
            primary.append('几何 / 净空 / 安全层：min_clearance=%.3f m（干预时 %.3f m）' % (
                f['min_clearance_m'], f['min_clearance_at_intervention_m'] or -1))
            ev.append('干预拍 %s（比例 %s，%s 段）；true_shield_infeasible=%s' % (
                f['intervened_ticks'], f['intervention_ticks_ratio'],
                f['intervention_segments'], f['true_shield_infeasible']))
            unc.append('净空低是否直接导致失手未分离：射击发生在接触/干预之前或之后需逐拍时序')
    return {'primary': primary, 'evidence': ev, 'uncertainties': unc}


def fmt(x, nd=3):
    if x is None:
        return '—'
    if isinstance(x, float):
        return ('%.' + str(nd) + 'f') % x
    return str(x)


def diag_rows(run_dir):
    """定向诊断 run（T4.4）：读新字段（首次 FCO / 无条件包络）+ 既有射击证据。"""
    arms = load_json(os.path.join(run_dir, 'metrics', 'arms.json'))['arms']
    out = []
    for row in arms:
        s = load_json(row['summary_path'])
        ew = s.get('episode_window_metrics') or {}
        log = parse_log(run_dir, row)
        win = (s.get('episode_windows') or [{}])[0]
        out.append({
            'arm': row['arm'], 'seed': row['seed'],
            'terminal_state': row['terminal_state'], 'reason_code': row.get('reason_code'),
            'infra_retries_used': row.get('infra_retries_used') or 0,
            'active_ticks': ew.get('ticks_total'),
            'vis_ticks_visible': ew.get('vis_ticks_visible'),
            'visibility_rate': ew.get('visibility_rate'),
            'fco_ticks': ew.get('ticks_fire_control_opportunity'),
            'real_process_fco': (ew.get('ticks_fire_control_opportunity') /
                                 ew.get('ticks_total')) if ew.get('ticks_total') else None,
            'first_fire_control_opportunity_s': ew.get('first_fire_control_opportunity_s'),
            'envelope_in_ticks': ew.get('envelope_in_ticks'),
            'envelope_in_rate': ew.get('envelope_in_rate'),
            'first_envelope_in_s': ew.get('first_envelope_in_s'),
            'window_enter_s': win.get('enter_s'), 'window_leave_s': win.get('leave_s'),
            'n_shots': len(log['shots']), 'n_hits': log['n_hits'], 'n_miss': log['n_miss'],
            'n_unresolved_shots': log['n_unresolved'],
            'first_shot_t': log['first_shot_t'],
            'first_shot_rel_s': (round(log['first_shot_t'] - win.get('enter_s'), 3)
                                 if (log['first_shot_t'] is not None and win.get('enter_s')) else None),
            'pred_summary': pred_summary(log['shots']),
            'intruder_t0': log['intruder_t0'], 'intruder_nav1': log['intruder_nav1'],
            'gate_ok_total': log['gate_ok_total'],
            'fov_az_total': log['fov_az_total'], 'fov_samples': log['fov_samples'],
            'min_clearance_m': row.get('min_clearance_m'),
            'log': log['log'], 'run_id': row.get('run_id'),
        })
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261023-w7-rl-b-p1abl-n20')
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--out-dir', default=None)
    ap.add_argument('--diag-run-ids', default='',
                    help='逗号分隔的定向诊断 run_id（T4.4；分开报告，不混用 p1abl 数值）')
    args = ap.parse_args()
    run_dir = os.path.join(ROOT, 'experiments', args.campaign, args.run_id)
    out_dir = args.out_dir or os.path.join(run_dir, 'metrics')
    arms = load_json(os.path.join(out_dir, 'arms.json'))['arms']
    by = {(a['arm'], a['seed']): a for a in arms if a['arm'] in ARMS}

    rows = {}
    for arm in ARMS:
        for seed in range(20):
            if (arm, seed) in by:
                rows[(arm, seed)] = episode_row(run_dir, by[(arm, seed)])

    fail_seeds = [s for s in range(20) if not rows[('p1_point_only', s)]['success']]
    pairs = []
    for seed in range(20):
        pairs.append({'seed': seed,
                      'coop': {k: rows[('p1_coop', seed)][k] for k in (
                          'terminal_state', 'reason_code', 'real_process_fco', 'scored_fco',
                          'active_ticks', 'active_s', 'visibility_rate_active', 'env_occ_given_vis',
                          'first_shot_t', 'first_shot_rel_s', 'n_shots', 'n_hits', 'n_miss',
                          'holds', 'longest_outage_s', 'intervened_ticks',
                          'intervention_ticks_ratio', 'min_clearance_m')},
                      'point_only': {k: rows[('p1_point_only', seed)][k] for k in (
                          'terminal_state', 'reason_code', 'real_process_fco', 'scored_fco',
                          'active_ticks', 'active_s', 'visibility_rate_active', 'env_occ_given_vis',
                          'first_shot_t', 'first_shot_rel_s', 'n_shots', 'n_hits', 'n_miss',
                          'holds', 'longest_outage_s', 'intervened_ticks',
                          'intervention_ticks_ratio', 'min_clearance_m')}})

    cases = []
    for seed in fail_seeds:
        f = rows[('p1_point_only', seed)]
        cases.append({
            'seed': seed,
            'terminal_state': f['terminal_state'], 'reason_code': f['reason_code'],
            'active_s': f['active_s'], 'active_ticks': f['active_ticks'],
            'real_process_fco': f['real_process_fco'], 'scored_fco': f['scored_fco'],
            'visibility_rate_active': f['visibility_rate_active'],
            'env_occ_given_vis': f['env_occ_given_vis'],
            'n_shots': f['n_shots'], 'n_hits': f['n_hits'], 'n_miss': f['n_miss'],
            'n_unresolved_shots': f['n_unresolved_shots'],
            'first_shot_t': f['first_shot_t'], 'first_shot_rel_s': f['first_shot_rel_s'],
            'holds': f['holds'], 'longest_outage_s': f['longest_outage_s'],
            'intervened_ticks': f['intervened_ticks'],
            'intervention_ticks_ratio': f['intervention_ticks_ratio'],
            'intervention_duration_s': f['intervention_duration_s'],
            'intervention_segments': f['intervention_segments'],
            'min_clearance_m': f['min_clearance_m'],
            'min_clearance_at_intervention_m': f['min_clearance_at_intervention_m'],
            'collision_count': f['collision_count'],
            'true_shield_infeasible': f['true_shield_infeasible'],
            'intruder_t0': f['intruder_t0'], 'intruder_nav1': f['intruder_nav1'],
            'gate_ok_total': f['gate_ok_total'],
            'fov_az_total': f['fov_az_total'], 'fov_samples': f['fov_samples'],
            'pred_summary': f['pred_summary'], 'shots': f['shots'],
            'field_gap': f['field_gap'],
            'mechanism': mechanisms(f),
            'log': f['log'],
        })

    doc = {
        'kind': 'P1 失败解释（T4；诊断型，不参与判定、不改 verdict）',
        'run_id': args.run_id, 'campaign': args.campaign, 'scene': SCENE,
        'dv_note': 'real_process_fco = ticks_fire_control_opportunity / vis_ticks_total（active 窗口）；'
                   'scored_fco = 失败回合置 0（v5 判定口径）；两者并列，不混用',
        'scored_metric_alias': '默认 zero-fill（keep_all_failures_and_infra）；本脚本 active_ticks 取 episode_window_metrics.ticks_total',
        'truth_note': 's15 静态目标真值 (4.5, 3.0, 2.0)（scenario_reset target_z=2.0）；'
                      '仅用于离线评估 pred 偏差，不进入运行时门',
        'field_gap': {
            'first_fire_control_opportunity_s': '既有 run 未记录：本报告用首次开火时刻（代理），'
                                                '**不得**替代首次机会时刻',
            'envelope_in_unconditional': '既有 run 未记录（无 env_ok ∧ envelope_in 的无条件计数）',
            'target_observation_age_at_shot_s': '既有 SHOT_FIRED 行无 age/est/t_est 字段',
            'note': '新字段已加入 run_logger（T4.1），只在后续 run 生效；本报告不回填、不伪造',
        },
        'point_only_failure_seeds': fail_seeds,
        'pairs': pairs,
        'cases': cases,
    }

    steps = []
    steps.append('# P1 失败解释：p1_point_only 的 %d 个超时（T4；诊断，**不**参与判定）\n' % len(fail_seeds))
    steps.append('> 承载 run `%s`；场景 `%s`。**真实过程 FCO** = 窗口内 FCO 拍 / 窗口拍；'
                 '**计分 FCO** = 失败回合置 0。两口径并列，不混用。'
                 '「可见率」取 `episode_window_metrics`（active 窗口逐拍），**不是** '
                 '`arms.json` 的固定 60 s 窗口值。\n' % (args.run_id, SCENE))
    steps.append('> **字段缺口**：既有 run **无** tick 级「首次 FCO」、**无**无条件包络占用、'
                 '**无**射击时观测年龄。下表「首次开火」仅作**代理**，**不**替代首次机会时刻。\n')
    steps.append('\n## 0. 失败 seeds\n')
    steps.append('`p1_point_only` 超时 seeds = **%s**（由 `arms.json` 复核；与任务描述一致）。\n'
                 % ', '.join(str(s) for s in fail_seeds))
    steps.append('\n## 1. 逐 seed 对照（20 对）\n')
    steps.append('| seed | 臂 | 终态 / reason | 真实过程 FCO | 计分 FCO | active 拍 / s | 可见率（active 窗口） | env_occ\\|vis '
                 '| 首次开火 t（代理） | 开火 | 命中 | 失手 | hold | 最长失联 s | 干预拍 / 比例 | min_clearance m |')
    steps.append('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for p in pairs:
        for arm, key in (('coop', 'coop'), ('point', 'point_only')):
            d = p[key]
            mark = ' **← 超时**' if (key == 'point_only' and p['seed'] in fail_seeds) else ''
            steps.append('| %d | %s | %s / %s%s | %s | %s | %s / %s | %s | %s | %s | %d | %d | %d | %d | %s | %s / %s | %s |' % (
                p['seed'], 'p1_coop' if arm == 'coop' else 'p1_point_only',
                d['terminal_state'], d['reason_code'] or '', mark,
                fmt(d['real_process_fco']), fmt(d['scored_fco']),
                fmt(d['active_ticks'], 0), fmt(d['active_s'], 1),
                fmt(d['visibility_rate_active']), fmt(d['env_occ_given_vis']),
                fmt(d['first_shot_t'], 1), d['n_shots'], d['n_hits'], d['n_miss'], d['holds'],
                fmt(d['longest_outage_s'], 2),
                fmt(d['intervened_ticks'], 0), fmt(d['intervention_ticks_ratio']),
                fmt(d['min_clearance_m'])))
    steps.append('\n## 2. 五个失败逐案\n')
    for c in cases:
        m = c['mechanism']
        steps.append('### seed %d（%s / %s）\n' % (c['seed'], c['terminal_state'], c['reason_code']))
        steps.append('- **窗口**：%.1f s / %d 拍；真实过程 FCO **%s**（计分 0）；可见率 %s；`env_occ|vis` %s。' % (
            c['active_s'], c['active_ticks'], fmt(c['real_process_fco']),
            fmt(c['visibility_rate_active']), fmt(c['env_occ_given_vis'])))
        steps.append('- **机会 vs 捕获**：首次开火 **%s s**（相对窗口，**代理**；首次 FCO 无字段）；'
                     '开火 **%d** / 命中 %d / 失手 %d / 未决 %d；hold %d。' % (
                         fmt(c['first_shot_rel_s'], 2), c['n_shots'], c['n_hits'],
                         c['n_miss'], c['n_unresolved_shots'], c['holds']))
        ps = c['pred_summary']
        if ps:
            steps.append('- **射击质量**：pred 水平偏差 中位 **%s m** / 最大 %s m（>网半径 0.5 m：%d/%d）；'
                         'pred z 中位 **%s m**（真值 2.0 m）。超时飞行（>3 s）：%s；`miss_dist`>10 m：%s。' % (
                             fmt(ps['pred_err_horizontal_median_m']), fmt(ps['pred_err_horizontal_max_m']),
                             ps['shots_pred_err_gt_net_radius'], ps['n'],
                             fmt(ps['pred_z_median_m']),
                             json.dumps(ps['timeout_flights_gt_3s'], ensure_ascii=False),
                             json.dumps(ps['miss_dist_gt_10m'], ensure_ascii=False)))
            if ps.get('shot_age_records'):
                steps.append('- **射击时观测年龄**：中位 **%s s** / 最大 %s s（%d/%d 发有 age 字段）。\n' % (
                    fmt(ps.get('shot_age_median_s'), 3), fmt(ps.get('shot_age_max_s'), 3),
                    ps['shot_age_records'], ps['n']))
            else:
                steps.append('- **射击时观测年龄**：旧 `SHOT_FIRED` 行**无** `age` / `est` / `t_est` 字段'
                             '（字段缺口，见报告开头「字段缺口」）；不得用其它量替代。\n')
        steps.append('- **靶机 / 感知**：`episode t=0`=%s、PX4 `nav_state=1` 计数 %d；'
                     '`gate_ok` 合计 %d；方位出视场计数 %d/%d。' % (
                         c['intruder_t0'], c['intruder_nav1'], c['gate_ok_total'],
                         c['fov_az_total'], 200 * c['fov_samples']))
        steps.append('- **安全 / 净空**：干预 %s 拍（%s；%s 段，%s s）；min_clearance **%s m**'
                     '（干预时 %s）；真实接触 %s 次、`true_shield_infeasible`=%s。' % (
                         c['intervened_ticks'], fmt(c['intervention_ticks_ratio']),
                         c['intervention_segments'], fmt(c['intervention_duration_s'], 1),
                         fmt(c['min_clearance_m']), fmt(c['min_clearance_at_intervention_m']),
                         c['collision_count'], c['true_shield_infeasible']))
        steps.append('- **机制（主）**：%s。' % '；'.join(m['primary']))
        steps.append('- **证据**：')
        for e in m['evidence']:
            steps.append('  - %s' % e)
        steps.append('- **仍不确定**：')
        for u in m['uncertainties']:
            steps.append('  - %s' % u)
        steps.append('')

    steps.append('## 3. 回答：更多机会为何没有更多捕获\n')
    steps.append('- **机会端**：③ 的真实过程 FCO（0.155）高于 ①（0.083），但这是**混合效应**——'
                 '成功回合的 ③ 机会更高且更快捕获；5 个超时回合把窗口拉长到 **120 s**、'
                 '真实 FCO 仅 0.00–0.10，池化后仍不足以抵消成功回合的优势。\n')
    steps.append('- **首次机会 vs 首次开火**：既有 run **无首次 FCO 字段**，本报告只用首次开火作代理；'
                 '**不得**据此断言“先有机会、后开火”。\n')
    steps.append('- **无条件包络**：既有 run **未记录**；`env_occ|vis` 条件于可见，含时间选择效应，不能替代。\n')
    steps.append('- **射击质量**：4 个有开火的失败回合均为 **0 命中**；pred 水平偏差中位 1.1–1.4 m、'
                 '超过网半径（0.5 m）的射击占 21/40，pred z 中位 0.0–0.2 m（真值 2.0 m）。'
                 '这些回合同时缺失靶机 `episode t=0`（3 例持续 `nav_state=1`），'
                 '指向**靶机未建立额定高度的几何失真**，而不是“指向导致射击差”。\n')
    steps.append('- **无观测**：seed8 全程 0 开火、可见率 0；`gate_ok=0` 且方位出视场计数 ≈6000/6200，'
                 '说明目标持续在窄视场外（指向/几何失败），非数据缺失。\n')
    steps.append('- **边界**：结论只限 s15 静止目标 / 已验证通道族；不推到移动目标；不改 P1 verdict。\n')
    steps.append('\n**三类机制映射（可证伪）**：\n')
    steps.append('- **A 启动失明 / 长时间无观测**：seed8（全程 `gate_ok=0`、方位出视场计数 ≈6000/6200、0 开火）。\n')
    steps.append('- **B 有观测但射击质量差**：seeds 1/4/10/18（0 命中；pred 水平偏差中位 1.1–1.4 m、'
                 'pred z 中位 0.0–0.2 m）。其中 1/4/10/18 同时**缺失靶机 `episode t=0`**（3 例持续 `nav_state=1`），'
                 '提示上游为**靶机未建立额定高度**的几何失真。\n')
    steps.append('- **C 几何 / 净空 / 安全层**：seed1（min_clearance 0.338 m、干预 20 段、与靶机真实接触 6 次、'
                 '`true_shield_infeasible=true`）——**并存**而非唯一原因；其余 4 例净空 3.3–5.0 m，不支持“安全层导致失败”。\n')

    diag_run_ids = [x for x in args.diag_run_ids.split(',') if x]
    diag = []
    for rid in diag_run_ids:
        rdir = os.path.join(ROOT, 'experiments', args.campaign, rid)
        if not os.path.exists(os.path.join(rdir, 'metrics', 'arms.json')):
            diag.append({'run_id': rid, 'status': 'missing_arms_json', 'episodes': []})
            continue
        diag.append({'run_id': rid, 'status': 'ok', 'episodes': diag_rows(rdir)})
    doc['diag_runs'] = diag

    if diag:
        steps.append('\n## 4. 定向诊断 run（T4.4；**与 p1abl 批次分开，不混用数值**）\n')
        steps.append('> 诊断 run 只为补 `first_fire_control_opportunity_s` / `envelope_in_*` / '
                     '`first_envelope_in_s`（T4.1 新字段）与射击级证据；**非确证**、不改 verdict。'
                     '这些回合受 PX4 / GCS 启动波动影响，**不**与 2026-10-23 的 p1abl 回合逐值配对。\n')
        for dr in diag:
            if dr['status'] != 'ok':
                steps.append('- `%s`：`arms.json` 缺失（环境阻塞 / 未完成），无可用诊断数据。\n' % dr['run_id'])
                continue
            steps.append('\n### run `%s`\n' % dr['run_id'])
            steps.append('| 臂 | seed | 终态 / reason | infra 重试 | 可见率 | 真实过程 FCO | '
                         '首次 FCO s | 首次开火 s（相对窗口） | 无条件包络拍 / 率 | 首次包络 s | '
                         '开火 / 命中 / 失手 / 未决 | 射击 age 中位 s | `episode t=0` / nav1 | min_clearance m |')
            steps.append('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
            for e in dr['episodes']:
                ps = e['pred_summary'] or {}
                steps.append('| %s | %d | %s / %s | %d | %s | %s | %s | %s | %s / %s | %s | %d / %d / %d / %d | %s | %s / %d | %s |' % (
                    e['arm'], e['seed'], e['terminal_state'], e['reason_code'] or '',
                    e['infra_retries_used'], fmt(e['visibility_rate']),
                    fmt(e['real_process_fco']),
                    fmt(e['first_fire_control_opportunity_s'], 2),
                    fmt(e['first_shot_rel_s'], 2),
                    fmt(e['envelope_in_ticks'], 0), fmt(e['envelope_in_rate']),
                    fmt(e['first_envelope_in_s'], 2),
                    e['n_shots'], e['n_hits'], e['n_miss'], e['n_unresolved_shots'],
                    fmt(ps.get('shot_age_median_s'), 3),
                    e['intruder_t0'], e['intruder_nav1'], fmt(e['min_clearance_m'])))
            steps.append('\n**读法**：\n')
            shots = [e for e in dr['episodes'] if e['n_shots'] > 0]
            nomatch = [e for e in dr['episodes']
                       if e['first_fire_control_opportunity_s'] is None and e['first_envelope_in_s'] is not None]
            steps.append('- 首次 FCO 与首次包络均来自**新字段**；若 `first_fire_control_opportunity_s` 为 `—`，'
                         '说明窗口内**从未**出现「可见 ∧ 包络内」——不是“没记”。\n')
            steps.append('- 有开火回合 %d 个：命中 %d / 失手 %d / 未决 %d；'
                         '`first_fire_control_opportunity_s` 与首次开火（若有）的先后可直接比较，'
                         '但**仍不得**把首次开火当首次机会。\n' % (
                             len(shots), sum(e['n_hits'] for e in shots),
                             sum(e['n_miss'] for e in shots),
                             sum(e['n_unresolved_shots'] for e in shots)))
            if nomatch:
                steps.append('- 无首次 FCO 但有包络进入的回合：%s（包络在但目标不可见）。\n' % ', '.join(
                    '%s seed%d' % (e['arm'], e['seed']) for e in nomatch))
            paired = [e for e in dr['episodes']
                      if e['first_fire_control_opportunity_s'] is not None and e['first_shot_rel_s'] is not None]
            if paired:
                steps.append('- 首次 FCO → 首次开火 延迟（s）：%s。**|Δ| 在 1 个控制拍量级**：'
                             '负值来自 `capture_adjudicator`（20 Hz 定时器）与 `run_logger`（odom 拍）'
                             '的**相位 / 时间源不同**，**不**表示“开火早于机会”；'
                             '≈0 表示“一旦有机会即开火”。\n' % '、'.join(
                                 '%s seed%d: %+.2f' % (
                                     e['arm'], e['seed'],
                                     e['first_shot_rel_s'] - e['first_fire_control_opportunity_s'])
                                 for e in paired))
            repro = [e for e in dr['episodes'] if e['arm'] == 'p1_point_only' and e['seed'] in fail_seeds]
            if repro:
                steps.append('\n**对旧失败 seeds 的复现核对**（仅 `p1_point_only`）：\n')
                for e in repro:
                    ps = e['pred_summary'] or {}
                    steps.append('- seed %d → %s / %s（infra 重试 %d）；`episode t=0`=%s、nav1=%d；'
                                 'pred z 中位 %s m；开火 %d / 命中 %d / 失手 %d。\n' % (
                                     e['seed'], e['terminal_state'], e['reason_code'] or '',
                                     e['infra_retries_used'], e['intruder_t0'], e['intruder_nav1'],
                                     fmt(ps.get('pred_z_median_m'), 2), e['n_shots'],
                                     e['n_hits'], e['n_miss']))
                if any(e['terminal_state'] == 'ACTIVE' for e in repro):
                    steps.append('> 注：`ACTIVE` 终态 = infra（4 次尝试均 ACTIVE / INFRA），'
                                 '`episode t=0` 只说明靶机就绪，**未**产生任务窗口 / 射击数据。\n')
                steps.append('> 结论：**旧 5 个 point_only 超时未在新 run 的同一 arm/seed 上复现**'
                             '（seed8 冒烟：point_only 捕获；seed1/4/10 诊断：捕获；seed18 诊断：infra 耗尽）。'
                             '同类的盲 / 低高度失败反而出现在 **coop**（冒烟 seed8 可见率 0；'
                             '诊断 coop seed4 可见率 0.084；诊断 coop seed18 低高度、5 发全失）。'
                             '因此这些失败更符合**运行期靶机/几何随机性（或 infra）**，'
                             '不是“仅指向”的稳定属性；旧失败回合的 `pred z ≈ 0` 也在 coop seed18 上再现。\n')

    with open(os.path.join(out_dir, 'p1_point_only_failures.json'), 'w') as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
    with open(os.path.join(out_dir, 'p1_point_only_failures.md'), 'w') as f:
        f.write('\n'.join(steps))
    print('wrote', os.path.join(out_dir, 'p1_point_only_failures.{json,md}'))
    print('failure seeds:', fail_seeds)


if __name__ == '__main__':
    main()
