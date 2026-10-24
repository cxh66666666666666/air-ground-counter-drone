#!/usr/bin/env python3
"""w7b_p1_decompose_fco —— **描述性**拆解 P1 的 primary DV（FCO）组成（ADR-0086/0087）。

**定位**：本脚本**不**参与判定、**不**改写任何 verdict；只把既有 run 产物（`metrics/arms.json` +
各 episode `summary.json` + 逐 episode 日志）重排为可解释的分解表，回答：

  1. **零填压缩**：判定口径把失败 / infra 回合的 primary 记 0（`keep_all_failures_and_infra`）。
     但"超时=失败"**不等于**"此前没有火控机会"——本脚本报出**失败回合的真实 active FCO**，
     并对比三种口径的均值 / 配对差：(a) 计分（零填，= 判定用）(b) 全回合真实 (c) 仅成功回合（有选择偏差）。
  2. **乘法分解**：`FCO_rate = vis_rate × env_occ|vis`，其中
     `vis_rate = vis_ticks_visible / vis_ticks_total`（active 窗口），
     `env_occ|vis = ticks_fire_control_opportunity / vis_ticks_visible`
     （**可见拍中落在交战包络内的比例**；包络 = `/agc/task/envelope_in`）。
     用恒等式 `Δfco = env_n·Δvis + vis_n·Δenv + Δvis·Δenv` 给出两个因子的贡献。
  3. 每回合：终态 / 失败原因 / 时长 / 开火时刻（`SHOT_FIRED t=`）/ 命中飞行时长（`CAPTURE flight=`）。

**已知缺口（如实标注）**：`ticks` 级"**首次机会拍时刻**"与"**无条件包络占用率**"
（不与可见性取交）在既有产物中**未记录**，本脚本只能给"首次**开火**时刻"作为代理；
两者需给 `run_logger` 加诊断计数（后续 run 生效，不改判定口径）。

用法：
  python3 scripts/w7b_p1_decompose_fco.py --run-id 20261023-w7-rl-b-p1v5-n20
  # 产物：experiments/<campaign>/<run_id>/metrics/p1_fco_decomposition.{json,md}
"""

import argparse
import json
import os
import re
import sys

ROOT = '/home/chen/AGC'
TERMINALS_OK = ('TRACKING_SUCCESS', 'INTERCEPT_CONFIRMED')


def load_episode(row):
    """从 arms.json 行 + summary.json 提取描述性字段（失败回合**保留真实** FCO，不零填）。"""
    with open(row['summary_path']) as f:
        s = json.load(f)
    ew = s.get('episode_window_metrics') or {}
    windows = s.get('episode_windows') or []
    vis_total = ew.get('vis_ticks_total') or 0
    vis_vis = ew.get('vis_ticks_visible') or 0
    fco = ew.get('ticks_fire_control_opportunity') or 0
    success = row['terminal_state'] in TERMINALS_OK
    real_fco = (fco / vis_total) if vis_total else None
    return {
        'arm': row['arm'], 'seed': row['seed'],
        'terminal_state': row['terminal_state'], 'reason_code': row.get('reason_code'),
        'success': success, 'infra_retries_used': row.get('infra_retries_used') or 0,
        'active_ticks': vis_total, 'ticks_total_all': ew.get('ticks_total'),
        'vis_ticks_visible': vis_vis, 'fco_ticks': fco,
        'vis_rate_active': (vis_vis / vis_total) if vis_total else None,
        'fco_rate_real': real_fco,
        'env_occ_given_visible': (fco / vis_vis) if vis_vis else None,   # 可见拍中在包络内的比例
        'fco_rate_scored': real_fco if success else 0.0,                 # 计分（零填）口径
        'duration_s': round((s.get('sim_end_s') or 0) - (s.get('sim_start_s') or 0), 3),
        'window_enter_s': windows[0]['enter_s'] if windows else None,
        'window_leave_s': windows[0]['leave_s'] if windows else None,
        'run_id': row.get('run_id'),
        'final_log_tag': ((row.get('infra_attempts') or [{}])[-1].get('log_tag') or row['arm']),
    }


def parse_log(run_dir, ep):
    """从**最终尝试**的日志提取开火 / 命中证据（描述性；首次机会拍不可得，见文件头缺口说明）。"""
    f = os.path.join(run_dir, 'log', 'ep_%s_s15_static_none_fov_capture_%d.log'
                     % (ep['final_log_tag'], ep['seed']))
    out = {'log': os.path.basename(f), 'n_shots': 0, 'first_shot_t': None,
           'shots_t': [], 'capture_flight_s': None, 'capture_at_s': None}
    if not os.path.exists(f):
        return out
    with open(f, errors='replace') as fh:
        for line in fh:
            m = re.search(r'SHOT_FIRED shot=\S+ t=([\d.]+)', line)
            if m:
                t = float(m.group(1))
                out['n_shots'] += 1
                out['shots_t'].append(t)
                if out['first_shot_t'] is None:
                    out['first_shot_t'] = t
            m = re.search(r'CAPTURE（发射式网动态命中）.*?flight=([\d.]+) s', line)
            if m:
                out['capture_flight_s'] = float(m.group(1))
    if out['first_shot_t'] is not None and out['capture_flight_s'] is not None:
        # 命中时刻 ≈ 首射时刻 + 网飞行时长（同一回合内；描述性近似）
        out['capture_at_s'] = round(out['first_shot_t'] + out['capture_flight_s'], 3)
    return out


def sums(eps):
    """按臂聚合（比率用**总量比**，避免小样本比率平均偏置）。"""
    by = {}
    for e in eps:
        a = by.setdefault(e['arm'], {'n': 0, 'vis_total': 0, 'vis_vis': 0, 'fco': 0,
                                     'scored_vals': [], 'real_vals': []})
        a['n'] += 1
        a['vis_total'] += e['active_ticks']
        a['vis_vis'] += e['vis_ticks_visible']
        a['fco'] += e['fco_ticks']
        a['scored_vals'].append(e['fco_rate_scored'])
        if e['fco_rate_real'] is not None:
            a['real_vals'].append(e['fco_rate_real'])
    for a in by.values():
        a['vis_rate'] = (a['vis_vis'] / a['vis_total']) if a['vis_total'] else None
        a['env_occ_given_visible'] = (a['fco'] / a['vis_vis']) if a['vis_vis'] else None
        a['fco_rate_pooled'] = (a['fco'] / a['vis_total']) if a['vis_total'] else None
        a['scored_mean'] = (sum(a['scored_vals']) / a['n']) if a['n'] else None
        a['real_mean'] = (sum(a['real_vals']) / len(a['real_vals'])) if a['real_vals'] else None
    return by


def paired(eps, key_coop, key_non, mode):
    """逐 seed 配对差（协同 − 非协同）三种口径：scored（判定用）/ real（全回合）/ success_only。"""
    by = {(e['arm'], e['seed']): e for e in eps}
    seeds = sorted({e['seed'] for e in eps})
    vals = []
    for s in seeds:
        c, n = by.get((key_coop, s)), by.get((key_non, s))
        if not c or not n:
            continue
        if mode == 'scored':
            vals.append(c['fco_rate_scored'] - n['fco_rate_scored'])
        elif mode == 'real':
            if c['fco_rate_real'] is None or n['fco_rate_real'] is None:
                continue
            vals.append(c['fco_rate_real'] - n['fco_rate_real'])
        else:   # success_only：**有选择偏差**（条件于成功），仅描述
            if not (c['success'] and n['success']):
                continue
            if c['fco_rate_real'] is None or n['fco_rate_real'] is None:
                continue
            vals.append(c['fco_rate_real'] - n['fco_rate_real'])
    m = (sum(vals) / len(vals)) if vals else None
    return {'n_pairs': len(vals), 'mean_delta': m, 'values': [round(v, 6) for v in vals]}


def factor_decomposition(by, key_c='p1_coop', key_n='p1_noncoop'):
    """恒等式：Δ(FCO) = env_n·Δvis + vis_n·Δenv + Δvis·Δenv（比率口径，聚合量）。"""
    c, n = by.get(key_c), by.get(key_n)
    if not c or not n or None in (c['vis_rate'], n['vis_rate'],
                                  c['env_occ_given_visible'], n['env_occ_given_visible']):
        return None
    dv = c['vis_rate'] - n['vis_rate']
    de = c['env_occ_given_visible'] - n['env_occ_given_visible']
    return {
        'vis_rate': {'coop': c['vis_rate'], 'noncoop': n['vis_rate'], 'delta': dv},
        'env_occ_given_visible': {'coop': c['env_occ_given_visible'],
                                  'noncoop': n['env_occ_given_visible'], 'delta': de},
        'fco_rate_pooled': {'coop': c['fco_rate_pooled'], 'noncoop': n['fco_rate_pooled'],
                            'delta': c['fco_rate_pooled'] - n['fco_rate_pooled']},
        'contribution': {
            'visibility_term_env_n_times_dvis': n['env_occ_given_visible'] * dv,
            'envelope_term_vis_n_times_denv': n['vis_rate'] * de,
            'interaction_dvis_times_denv': dv * de,
        },
        'identity_check': n['env_occ_given_visible'] * dv + n['vis_rate'] * de + dv * de,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--coop-arm', default='p1_coop')
    ap.add_argument('--noncoop-arm', default='p1_noncoop')
    args = ap.parse_args()

    run_dir = os.path.join(ROOT, 'experiments', args.campaign, args.run_id)
    with open(os.path.join(run_dir, 'metrics', 'arms.json')) as f:
        arms = json.load(f)['arms']
    p1 = [r for r in arms if r['arm'] in (args.coop_arm, args.noncoop_arm)]
    eps = []
    for r in p1:
        e = load_episode(r)
        e.update(parse_log(run_dir, e))
        eps.append(e)

    by = sums(eps)
    out = {
        'kind': 'descriptive_posthoc（不参与判定；verdict 字段不改写）',
        'run_id': args.run_id, 'campaign': args.campaign,
        'generated_by': 'scripts/w7b_p1_decompose_fco.py',
        'dv_definition': 'active 窗口（GROUND_APPROACH→终态）：FCO_rate = vis_rate × env_occ|vis',
        'note_scored': '计分口径 = 失败 / infra 回合记 0（keep_all_failures_and_infra，判定用）',
        'note_real': '真实口径 = 失败回合**保留其真实** active FCO（描述用；超时≠此前无机会）',
        'episodes': eps,
        'per_arm': by,
        'paired': {m: paired(eps, args.coop_arm, args.noncoop_arm, m)
                   for m in ('scored', 'real', 'success_only')},
        'factor_decomposition': factor_decomposition(by, args.coop_arm, args.noncoop_arm),
        'zero_fill': {
            'n_zero_episodes': sum(1 for e in eps if not e['success']),
            'by_arm': {a: sum(1 for e in eps if e['arm'] == a and not e['success'])
                       for a in by},
            'real_fco_of_failed': [{'arm': e['arm'], 'seed': e['seed'],
                                    'terminal': e['terminal_state'],
                                    'reason': e['reason_code'],
                                    'vis_rate_active': e['vis_rate_active'],
                                    'fco_rate_real': e['fco_rate_real'],
                                    'fco_ticks': e['fco_ticks'],
                                    'n_shots': e['n_shots'],
                                    'first_shot_t': e['first_shot_t']}
                                   for e in eps if not e['success']],
            'note': '失败回合真实 FCO 在**计分**中被记 0；本表把它们单列，用于解释零填压缩量。',
        },
        'gaps_not_available': [
            '首次火控机会**拍**时刻（tick 级）——产物未记录，需 run_logger 增诊断计数（后续 run）',
            '**无条件**包络占用率（不与可见取交）——同上；本表给"可见拍中包络占比"',
        ],
    }

    # —— 描述性 markdown（读给人看）——
    md = ['# P1 FCO 描述性拆解 · %s' % args.run_id, '',
          '> **描述性**后验分析（`scripts/w7b_p1_decompose_fco.py`）：**不**参与判定、**不**改写 verdict；',
          '> 目的：拆开"任务成功"与"过程中的火控机会"，解释可见率差 → 计分差的压缩。', '',
          '## 逐回合（失败 / infra 回合保留**真实** FCO）', '',
          '| arm | seed | terminal | reason | retries | active ticks | vis_rate | fco(real) | env_occ\\|vis | 时长(s) | 开火次数 | 首射 t(s) |',
          '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for e in sorted(eps, key=lambda x: (x['arm'], x['seed'])):
        fmt = lambda v, p='%.3f': ('-' if v is None else (p % v))
        md.append('| %s | %d | %s | %s | %d | %d | %s | %s | %s | %.1f | %d | %s |' % (
            e['arm'], e['seed'], e['terminal_state'], e['reason_code'] or '-',
            e['infra_retries_used'], e['active_ticks'], fmt(e['vis_rate_active']),
            fmt(e['fco_rate_real']), fmt(e['env_occ_given_visible']),
            e['duration_s'], e['n_shots'], fmt(e['first_shot_t'], '%.1f')))
    md += ['', '## 按臂聚合（比率用总量比）', '',
           '| arm | episodes | vis_rate | env_occ\\|vis | FCO_rate(pooled) | FCO 计分均值(零填) | FCO 真实均值(全回合) |',
           '|---|---|---|---|---|---|---|']
    for a, v in by.items():
        md.append('| %s | %d | %.3f | %.4f | %.4f | %.4f | %.4f |' % (
            a, v['n'], v['vis_rate'] or 0, v['env_occ_given_visible'] or 0,
            v['fco_rate_pooled'] or 0, v['scored_mean'] or 0, v['real_mean'] or 0))
    pa = out['paired']
    md += ['', '## 配对差（协同 − 非协同）三种口径', '',
           '| 口径 | n 配对 | 均值配对差 | 说明 |', '|---|---|---|---|',
           '| scored（判定用，失败记 0） | %d | %s | v5 判定口径 |' % (
               pa['scored']['n_pairs'], '%.4f' % (pa['scored']['mean_delta'] or 0)),
           '| real（全回合真实 FCO） | %d | %s | 描述；超时回合保留其真实机会 |' % (
               pa['real']['n_pairs'], '%.4f' % (pa['real']['mean_delta'] or 0)),
           '| success_only（仅两臂皆成功） | %d | %s | **有选择偏差**，仅描述 |' % (
               pa['success_only']['n_pairs'],
               '%.4f' % (pa['success_only']['mean_delta'] or 0))]
    fd = out['factor_decomposition']
    if fd:
        c = fd['contribution']
        md += ['', '## 乘法分解（Δfco = env_n·Δvis + vis_n·Δenv + Δvis·Δenv）', '',
               '- 可见率：协同 %.3f vs 非协同 %.3f（Δ = %+.3f）' % (
                   fd['vis_rate']['coop'], fd['vis_rate']['noncoop'], fd['vis_rate']['delta']),
               '- 可见拍中包络占比：协同 %.4f vs 非协同 %.4f（Δ = %+.4f）' % (
                   fd['env_occ_given_visible']['coop'],
                   fd['env_occ_given_visible']['noncoop'],
                   fd['env_occ_given_visible']['delta']),
               '- FCO（pooled）：协同 %.4f vs 非协同 %.4f（Δ = %+.4f）' % (
                   fd['fco_rate_pooled']['coop'], fd['fco_rate_pooled']['noncoop'],
                   fd['fco_rate_pooled']['delta']),
               '- 贡献：可见项 %+.4f ｜ 包络项 %+.4f ｜ 交互项 %+.4f' % (
                   c['visibility_term_env_n_times_dvis'], c['envelope_term_vis_n_times_denv'],
                   c['interaction_dvis_times_denv'])]
    zf = out['zero_fill']
    md += ['', '## 零填回合（%d 个）' % zf['n_zero_episodes'], '',
           '| arm | seed | terminal | reason | vis_rate | fco(real) | fco ticks | 开火 | 首射 t |',
           '|---|---|---|---|---|---|---|---|---|']
    for z in zf['real_fco_of_failed']:
        fmt = lambda v, p='%.3f': ('-' if v is None else (p % v))
        md.append('| %s | %d | %s | %s | %s | %s | %d | %d | %s |' % (
            z['arm'], z['seed'], z['terminal'], z['reason'] or '-',
            fmt(z['vis_rate_active']), fmt(z['fco_rate_real']), z['fco_ticks'],
            z['n_shots'], fmt(z['first_shot_t'], '%.1f')))
    md += ['', '## 既有产物缺口（如实标注）', '']
    md += ['- %s' % g for g in out['gaps_not_available']]

    json_path = os.path.join(run_dir, 'metrics', 'p1_fco_decomposition.json')
    md_path = os.path.join(run_dir, 'metrics', 'p1_fco_decomposition.md')
    with open(json_path, 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    with open(md_path, 'w') as f:
        f.write('\n'.join(md) + '\n')
    print('-> %s\n-> %s' % (json_path, md_path))
    print('\n'.join(md[md.index('## 按臂聚合（比率用总量比）'):]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
