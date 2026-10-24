#!/usr/bin/env python3
"""w7b_p2_arm_b_diagnose —— P2 臂 B（`p2_rl_high`）**负向结果**的诊断（ADR-0087(b)）。

**定位**：**描述性 / 归因**（用既有产物），**不改** verdict、**不**以"调到 B 赢"为目标。

拆解四类组件（A = `p2_c2_low_level` 对照）：
  1. **重规划**：逐回合统计 `ground_baseline` 的 `重规划触发：…`（按原因分类）——B 的低层消费高层拦截点，
     目标跳动会触发重规划（ADR-0083(g) 已修一版，本脚本量**残留**）；
  2. **抵近过程**：active 窗口时长 / 开火时刻（`SHOT_FIRED t=`，回合时基）/ 首次开火相对窗口开始的延迟；
  3. **预测**：`SHOT_FIRED` 的 `pred=(x,y,z)` 与**目标真值**的水平偏差（s08 目标静止于 (4.5, 3.0)，
     理想提前量 = 0）与发射时的 `h`（相对高度）分布；
  4. **安全层干预**：`command_attribution` 的 `intervened_ticks` / 干预时长 / `max_abs_delta_v`。

产物：`experiments/<campaign>/<run_id>/metrics/p2_arm_b_diagnosis.{json,md}`（描述性）。
用法：
  python3 scripts/w7b_p2_arm_b_diagnose.py --run-id 20261022-w7-rl-b-v3-n20r
"""

import argparse
import json
import os
import re
import sys
from collections import Counter

ROOT = '/home/chen/AGC'
SCENE = 's08_static_none_nominal_capture'
TARGET_XY = (4.5, 3.0)          # s08 scenario_reset 目标真值（静止）
ARMS = ('p2_c2_low_level', 'p2_rl_high')


def load_arms(run_dir):
    with open(os.path.join(run_dir, 'metrics', 'arms.json')) as f:
        rows = json.load(f)['arms']
    return [r for r in rows if r['arm'] in ARMS]


def log_name(row):
    tag = (row.get('infra_attempts') or [{}])[-1].get('log_tag') or row['arm']
    return 'ep_%s_%s_%d.log' % (tag, SCENE, row['seed'])


def parse_log(run_dir, row):
    """从**最终尝试**日志提取：重规划分类计数 / 开火证据 / 首射相对窗口延迟。"""
    path = os.path.join(run_dir, 'log', log_name(row))
    out = {'log': os.path.basename(path), 'replans': {}, 'n_replans': 0,
           'shots_t': [], 'first_shot_t': None, 'first_shot_h': None,
           'first_shot_pred_offset_m': None, 'shot_h': [], 'pred_offsets': []}
    if not os.path.exists(path):
        return out
    for line in open(path, errors='replace'):
        for m in re.finditer(r'重规划触发：([^\s,]+)', line):
            k = m.group(1)
            out['replans'][k] = out['replans'].get(k, 0) + 1
        m = re.search(r'SHOT_FIRED shot=\S+ t=([\d.]+) h=([\d.]+) dist=([\d.]+) pred=\(([-\d.]+),([-\d.]+),([-\d.]+)\)', line)
        if m:
            t, h = float(m.group(1)), float(m.group(2))
            px, py = float(m.group(4)), float(m.group(5))
            off = ((px - TARGET_XY[0]) ** 2 + (py - TARGET_XY[1]) ** 2) ** 0.5
            out['shots_t'].append(t)
            out['shot_h'].append(h)
            out['pred_offsets'].append(round(off, 3))
            if out['first_shot_t'] is None:
                out['first_shot_t'], out['first_shot_h'] = t, h
                out['first_shot_pred_offset_m'] = round(off, 3)
    out['n_replans'] = sum(out['replans'].values())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    run_dir = os.path.join(ROOT, 'experiments', args.campaign, args.run_id)
    rows = load_arms(run_dir)

    eps = []
    for r in rows:
        with open(r['summary_path']) as f:
            s = json.load(f)
        ew = s.get('episode_window_metrics') or {}
        attr = s.get('command_attribution') or {}
        win = (s.get('episode_windows') or [{}])[0]
        e = {
            'arm': r['arm'], 'seed': r['seed'], 'terminal': r['terminal_state'],
            'reason': r.get('reason_code'), 'fco_active': r.get('fire_control_opportunity_rate_active'),
            'vis_fixed': r.get('visibility_rate'), 'min_clearance_m': r.get('min_clearance_m'),
            'active_ticks': ew.get('ticks_total'), 'vis_ticks_visible': ew.get('vis_ticks_visible'),
            'fco_ticks': ew.get('ticks_fire_control_opportunity'),
            'env_occ_given_vis': ((ew.get('ticks_fire_control_opportunity') or 0)
                                  / ew['vis_ticks_visible']) if ew.get('vis_ticks_visible') else None,
            'window_enter_s': win.get('enter_s'), 'window_leave_s': win.get('leave_s'),
            'window_len_s': (round((win.get('leave_s') or 0) - (win.get('enter_s') or 0), 2)
                             if win.get('enter_s') is not None else None),
            'intervened_ticks': attr.get('intervened_ticks'),
            'intervention_duration_s': attr.get('intervention_duration_s'),
            'max_abs_delta_v': attr.get('max_abs_delta_v'),
            'infra_retries_used': r.get('infra_retries_used') or 0,
        }
        e.update(parse_log(run_dir, r))
        if e['first_shot_t'] is not None and e['window_enter_s'] is not None:
            e['first_shot_after_window_enter_s'] = round(e['first_shot_t'] - e['window_enter_s'], 2)
        eps.append(e)

    def agg(arm):
        rs = [e for e in eps if e['arm'] == arm]
        n = len(rs)
        def mean(key):
            v = [r[key] for r in rs if r.get(key) is not None]
            return round(sum(v) / len(v), 4) if v else None
        return {
            'n': n, 'terminals': dict(Counter(r['terminal'] for r in rs)),
            'replan_total': sum(r['n_replans'] for r in rs),
            'replan_per_episode': round(sum(r['n_replans'] for r in rs) / n, 2) if n else None,
            'replan_by_reason': dict(Counter(k for r in rs for k, c in r['replans'].items() for _ in range(c))),
            'shots_total': sum(len(r['shots_t']) for r in rs),
            'episodes_with_shot': sum(1 for r in rs if r['first_shot_t'] is not None),
            'first_shot_t_mean': mean('first_shot_t'),
            'first_shot_after_window_enter_mean': mean('first_shot_after_window_enter_s'),
            'first_shot_pred_offset_mean_m': mean('first_shot_pred_offset_m'),
            'pred_offset_mean_m': round(
                sum(o for r in rs for o in r['pred_offsets']) / max(1, sum(len(r['pred_offsets']) for r in rs)), 4),
            'shot_h_mean': round(sum(h for r in rs for h in r['shot_h'])
                                 / max(1, sum(len(r['shot_h']) for r in rs)), 4),
            'fco_active_mean': mean('fco_active'), 'vis_fixed_mean': mean('vis_fixed'),
            'active_ticks_mean': mean('active_ticks'), 'window_len_s_mean': mean('window_len_s'),
            'env_occ_given_vis_mean': mean('env_occ_given_vis'),
            'intervened_ticks_mean': mean('intervened_ticks'),
            'intervention_duration_s_mean': mean('intervention_duration_s'),
            'max_abs_delta_v_mean': mean('max_abs_delta_v'),
            'min_clearance_mean_m': mean('min_clearance_m'),
        }

    out = {'kind': 'descriptive_diagnosis（ADR-0087(b)；不改 verdict）',
           'run_id': args.run_id, 'campaign': args.campaign,
           'scene': SCENE, 'target_truth_xy': list(TARGET_XY),
           'per_arm': {a: agg(a) for a in ARMS},
           'episodes': eps}

    md = ['# P2 臂 B 负向诊断 · %s' % args.run_id, '',
          '> 描述性归因（ADR-0087(b)）：**不**改 verdict、**不**以“调到 B 赢”为目标。', '',
          '## 按臂汇总（A = c2 / B = 规则高层）', '',
          '| 量 | A `p2_c2_low_level` | B `p2_rl_high` |', '|---|---|---|']
    A, B = out['per_arm'][ARMS[0]], out['per_arm'][ARMS[1]]
    keys = [('n', 'episodes'), ('terminals', '终态分布'), ('fco_active_mean', 'active FCO 均值'),
            ('vis_fixed_mean', '可见率(fixed)'), ('env_occ_given_vis_mean', '可见拍中包络占比'),
            ('active_ticks_mean', 'active 拍均值'), ('window_len_s_mean', 'active 窗口时长均值(s)'),
            ('replan_per_episode', '重规划次数/回合'), ('replan_by_reason', '重规划原因分布'),
            ('shots_total', '开火总次数'), ('episodes_with_shot', '有开火回合数'),
            ('first_shot_t_mean', '首射时刻均值(s,回合时基)'),
            ('first_shot_after_window_enter_mean', '首射相对窗口开始延迟(s)'),
            ('pred_offset_mean_m', '预测点与真值水平偏差均值(m)'),
            ('shot_h_mean', '发射时相对高度均值(m)'),
            ('intervened_ticks_mean', '安全层干预拍均值'),
            ('intervention_duration_s_mean', '干预时长均值(s)'),
            ('max_abs_delta_v_mean', '干预最大速度修正均值(m/s)'),
            ('min_clearance_mean_m', '最小净空均值(m)')]
    for k, label in keys:
        md.append('| %s | %s | %s |' % (label, A.get(k), B.get(k)))
    md += ['', '## 读法', '',
           '- 只做归因：若 B 的 `重规划次数/回合`、`首射延迟`、`预测偏差`、`干预量` 中某一项显著劣于 A，',
           '  该组件即 B 负向（active FCO 低于 A）的候选解释；多组件同时劣化时按其量级排序登记。',
           '- 若诊断指向**实现缺陷**（如重规划振荡 / 预测偏移超网半径），先修缺陷并留 ADR；',
           '  再评估是否需要**新预注册**重判 P2（不得事后改 v3 判定）。']

    with open(os.path.join(run_dir, 'metrics', 'p2_arm_b_diagnosis.json'), 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    with open(os.path.join(run_dir, 'metrics', 'p2_arm_b_diagnosis.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')
    print('-> metrics/p2_arm_b_diagnosis.{json,md}')
    print('\n'.join(md[md.index('## 按臂汇总（A = c2 / B = 规则高层）'):]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
