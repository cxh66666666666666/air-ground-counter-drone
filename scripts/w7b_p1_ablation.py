#!/usr/bin/env python3
"""w7b_p1_ablation —— P1 机制消融（指向 × 跟踪参考 2×2）的**估计算**分析（ADR-0087）。

**定位**：**估计型**（析因对比 + 配对 CI），**不设门槛判定**、**不改**任何 verdict
（门槛语义属 [ADR-0086](../../docs/adr/0086-decision-rule-three-state-ci-versus-mme.md)，本脚本不参与）。

2×2 cell（同批单 run，CRN 逐 seed 配对）：

| cell | observe_yaw_mode | air_tracking_ref | arm |
|---|---|---|---|
| ① 指向+跟踪 | target   | on  | p1_coop |
| ② 无指向+无跟踪 | velocity | off | p1_noncoop |
| ③ **仅指向** | target   | off | p1_point_only |
| ④ **仅跟踪** | velocity | on  | p1_track_only |

对比量（逐 seed 配对差，均值 + 95% CI + 配对 t）：
  * 指向主效应：tracking=on → ①−④；tracking=off → ③−②
  * 跟踪主效应：pointing=target → ①−③；pointing=velocity → ④−②
  * 交互：(①−④)−(③−②)
DV = 真实过程 FCO（`fire_control_opportunity_rate_active`；失败回合保留真实值、缺测配对剔除；**不等于** v5 失败置零判定口径）；
辅报 `visibility_rate`（fixed 窗，描述）/ active 可见率 / 可见拍中包络占比。

**预登记风险（ADR-0087(a)4）**：cell ④ 可能因 C10 hold 自锁**退化**——本脚本不掩饰：
cell 画像表直接给每 cell 的终态分布与真实 FCO，退化即照实呈现。

用法：
  python3 scripts/w7b_p1_ablation.py --run-id 20261023-w7-rl-b-p1abl-pilot
  # 产物：experiments/<campaign>/<run_id>/metrics/p1_ablation.{json,md}
"""

import argparse
import json
import os
import sys
from collections import Counter

ROOT = '/home/chen/AGC'
CELLS = [('p1_coop', 'target', 'on'), ('p1_noncoop', 'velocity', 'off'),
         ('p1_point_only', 'target', 'off'), ('p1_track_only', 'velocity', 'on')]
TERMINALS_OK = ('TRACKING_SUCCESS', 'INTERCEPT_CONFIRMED')


def load(run_dir):
    with open(os.path.join(run_dir, 'metrics', 'arms.json')) as f:
        rows = json.load(f)['arms']
    eps = {}
    for r in rows:
        if r['arm'] not in {c[0] for c in CELLS}:
            continue
        e = {'arm': r['arm'], 'seed': r['seed'], 'terminal': r['terminal_state'],
             'reason': r.get('reason_code'), 'success': r['terminal_state'] in TERMINALS_OK,
             'fco_active': r.get('fire_control_opportunity_rate_active'),
             'fco_scored': (r.get('fire_control_opportunity_rate_active') if r['terminal_state'] in TERMINALS_OK else 0.0),
             'vis_fixed': r.get('visibility_rate'), 'intervened': r.get('intervened_ticks'),
             'summary_path': r.get('summary_path')}
        if e['summary_path'] and os.path.exists(e['summary_path']):
            s = json.load(open(e['summary_path']))
            ew = s.get('episode_window_metrics') or {}
            vt = ew.get('vis_ticks_total') or 0
            vv = ew.get('vis_ticks_visible') or 0
            fco = ew.get('ticks_fire_control_opportunity') or 0
            e.update({'active_ticks': vt, 'vis_active': (vv / vt) if vt else None,
                      'env_occ_given_vis': (fco / vv) if vv else None})
        eps[(r['arm'], r['seed'])] = e
    return eps


def paired_contrast(eps, arm_x, arm_y, key='fco_active', seeds=None, success_only=False):
    """逐 seed 配对差（x − y），返回 n / mean / CI / 配对 t p。"""
    import numpy as np
    from scipy import stats
    seeds = seeds if seeds is not None else sorted({s for (_, s) in eps})
    vals = []
    for s in seeds:
        a, b = eps.get((arm_x, s)), eps.get((arm_y, s))
        if not a or not b:
            continue
        if success_only and not (a['success'] and b['success']):
            continue
        if a.get(key) is None or b.get(key) is None:
            continue
        vals.append(a[key] - b[key])
    if len(vals) < 2:
        return {'n': len(vals), 'mean': None, 'ci95': None, 'p_two_sided': None}
    d = np.array(vals)
    m, sd = float(d.mean()), float(d.std(ddof=1))
    sem = sd / np.sqrt(len(d))
    tcrit = stats.t.ppf(0.975, len(d) - 1)
    return {'n': len(vals), 'mean': round(m, 6),
            'ci95': [round(m - tcrit * sem, 6), round(m + tcrit * sem, 6)],
            'p_two_sided': float(stats.ttest_rel([v for v in d], [0.0] * len(d)).pvalue),
            'values': [round(v, 6) for v in vals]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    run_dir = os.path.join(ROOT, 'experiments', args.campaign, args.run_id)
    eps = load(run_dir)
    seeds = sorted({s for (_, s) in eps})

    cell_stats = {}
    for arm, oym, atr in CELLS:
        rows = [eps[(arm, s)] for s in seeds if (arm, s) in eps]
        act = [r['fco_active'] for r in rows if r.get('fco_active') is not None]
        cell_stats[arm] = {
            'observe_yaw_mode': oym, 'air_tracking_ref': atr, 'n': len(rows),
            'terminals': dict(Counter(r['terminal'] for r in rows)),
            'success_rate': round(sum(1 for r in rows if r['success']) / len(rows), 4) if rows else None,
            'fco_active_mean': round(sum(act) / len(act), 6) if act else None,
            'fco_scored_mean': round(sum(r['fco_scored'] for r in rows) / len(rows), 6) if rows else None,
            'vis_active_mean': round(sum(r['vis_active'] for r in rows if r.get('vis_active') is not None)
                                     / max(1, len([r for r in rows if r.get('vis_active') is not None])), 6),
            'env_occ_given_vis_mean': round(
                sum(r['env_occ_given_vis'] for r in rows if r.get('env_occ_given_vis') is not None)
                / max(1, len([r for r in rows if r.get('env_occ_given_vis') is not None])), 6),
            'intervened_mean': round(sum(r['intervened'] or 0 for r in rows) / len(rows), 2) if rows else None,
        }

    c = {arm: arm for arm, _, _ in CELLS}
    contrasts = {
        'pointing_effect_tracking_on':  paired_contrast(eps, c['p1_coop'], c['p1_track_only']),
        'pointing_effect_tracking_off': paired_contrast(eps, c['p1_point_only'], c['p1_noncoop']),
        'tracking_effect_pointing_target': paired_contrast(eps, c['p1_coop'], c['p1_point_only']),
        'tracking_effect_pointing_velocity': paired_contrast(eps, c['p1_track_only'], c['p1_noncoop']),
        'corner_total_coop_minus_noncoop': paired_contrast(eps, c['p1_coop'], c['p1_noncoop']),
    }
    # 交互项：逐 seed (①−④)−(③−②)
    inter = []
    for s in seeds:
        a, d, p, n = (eps.get((c['p1_coop'], s)), eps.get((c['p1_track_only'], s)),
                      eps.get((c['p1_point_only'], s)), eps.get((c['p1_noncoop'], s)))
        if not all([a, d, p, n]) or None in (a.get('fco_active'), d.get('fco_active'),
                                             p.get('fco_active'), n.get('fco_active')):
            continue
        inter.append((a['fco_active'] - d['fco_active']) - (p['fco_active'] - n['fco_active']))
    if len(inter) >= 2:
        import numpy as np
        from scipy import stats
        arr = np.array(inter)
        m, sd = float(arr.mean()), float(arr.std(ddof=1))
        sem = sd / np.sqrt(len(arr))
        tcrit = stats.t.ppf(0.975, len(arr) - 1)
        contrasts['interaction'] = {'n': len(arr), 'mean': round(m, 6),
                                    'ci95': [round(m - tcrit * sem, 6), round(m + tcrit * sem, 6)],
                                    'values': [round(v, 6) for v in inter]}
    else:
        contrasts['interaction'] = {'n': len(inter), 'mean': None}

    out = {'kind': 'estimational（ADR-0087；无门槛判定；不改 verdict）',
           'run_id': args.run_id, 'campaign': args.campaign, 'seeds': seeds,
           'cells': cell_stats, 'contrasts': contrasts,
           'dv': 'fire_control_opportunity_rate_active（真实过程 FCO；失败回合保留真实值、缺测配对剔除；不等于 v5 失败置零判定口径）',
           'known_risk': 'cell ④ p1_track_only 可能因 C10 hold 自锁退化（ADR-0087(a)4 预先登记）'}

    md = ['# P1 机制消融（2×2）· %s' % args.run_id, '',
          '> **估计型**分析（ADR-0087）：只做归因/区间估计，**不设门槛判定**、**不改** verdict。', '',
          '> DV = **真实过程 FCO**（失败回合保留真实值、缺测配对剔除；**不等于** v5 primary 的失败置零计分口径）。', '',
          '## Cell 画像', '',
          '| arm | 指向 | 跟踪 | n | 终态分布 | 真实过程 FCO | 计分均值 | active 可见率 | 可见拍中包络占比 | 干预拍均值 |',
          '|---|---|---|---|---|---|---|---|---|---|']
    for arm, oym, atr in CELLS:
        v = cell_stats[arm]
        fmt = lambda x, p='%.4f': ('-' if x is None else (p % x))
        md.append('| %s | %s | %s | %d | %s | %s | %s | %s | %s | %s |' % (
            arm, oym, atr, v['n'], v['terminals'], fmt(v['fco_active_mean']),
            fmt(v['fco_scored_mean']), fmt(v['vis_active_mean']),
            fmt(v['env_occ_given_vis_mean']), fmt(v['intervened_mean'], '%.1f')))
    md += ['', '## 对比（逐 seed 配对差；均值 + 95% CI）', '',
           '| 对比 | n | 均值 | 95% CI | p(双侧) |', '|---|---|---|---|---|']
    for k, v in contrasts.items():
        md.append('| %s | %s | %s | %s | %s |' % (
            k, v.get('n'), ('%.4f' % v['mean']) if v.get('mean') is not None else '-',
            v.get('ci95') if v.get('ci95') else '-',
            ('%.4g' % v['p_two_sided']) if v.get('p_two_sided') is not None else '-'))
    md += ['', '## 读法约束（先定）', '',
           '- 消融只回答"贡献来自哪一项"，**不**对 P1 命题另立判定（须新预注册版本，ADR-0087(a)）。',
           '- cell ④ 若退化（如终态集中在失败 / FCO≈0），**跟踪主效应**只按 pointing=target 一侧（①−③）解读，',
           '  并如实登记该 cell 为"已知机制失效组合"，不得当作公平基线。']

    json_path = os.path.join(run_dir, 'metrics', 'p1_ablation.json')
    md_path = os.path.join(run_dir, 'metrics', 'p1_ablation.md')
    with open(json_path, 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    with open(md_path, 'w') as f:
        f.write('\n'.join(md) + '\n')
    print('-> %s\n-> %s' % (json_path, md_path))
    print('\n'.join(md[md.index('## Cell 画像'):]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
