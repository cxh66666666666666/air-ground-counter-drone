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
RATE_HZ = 20.0                  # capture / 安全层拍频；用于把窗口干预拍换算成每秒干预拍


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
           'first_shot_pred_offset_m': None, 'shot_h': [], 'pred_offsets': [],
           'map_trigger_details': []}
    if not os.path.exists(path):
        return out
    for line in open(path, errors='replace'):
        for m in re.finditer(r'重规划触发：([^\s,]+)', line):
            k = m.group(1)
            out['replans'][k] = out['replans'].get(k, 0) + 1
        mt = re.search(
            r'重规划触发：地图变化.*?hard_cells=(\d+) layers=\[([^\]]*)\]', line)
        if mt:
            layers = {}
            unavailable = []
            for item in mt.group(2).split(','):
                if not item:
                    continue
                name, _, val = item.partition(':')
                if val.startswith('n/a'):
                    unavailable.append(name)
                else:
                    try:
                        layers[name] = int(val)
                    except ValueError:
                        pass
            out['map_trigger_details'].append({
                'hard_cells': int(mt.group(1)), 'layers': layers,
                'layer_unavailable': unavailable,
                'line': line.strip()[:400]})
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
            # W7-b T3.6：安全干预按 **active 窗口** 归一化（只增字段，不改既有口径）。
            'window_intervened_ticks': ew.get('ticks_intervened'),
            'window_ticks_total': ew.get('ticks_total'),
            'window_intervention_ratio': ((ew.get('ticks_intervened') or 0) / ew['ticks_total'])
                                         if ew.get('ticks_total') else None,
            'window_intervention_ticks_per_s': (((ew.get('ticks_intervened') or 0) / ew['ticks_total'])
                                                * RATE_HZ) if ew.get('ticks_total') else None,
            'window_intervention_duration_s': ((ew.get('ticks_intervened') or 0) / RATE_HZ)
                                              if ew.get('ticks_intervened') is not None else None,
        }
        e['attr_intervened_per_active_s'] = (
            (attr.get('intervened_ticks') or 0) / (e['active_ticks'] / RATE_HZ)
            if e.get('active_ticks') else None)
        e.update(parse_log(run_dir, r))
        # W7-b T2.6：地图触发的实际量（AX-0089 后 hard_cells 仅计 LETHAL_OBSTACLE=254）。
        mt = e.get('map_trigger_details') or []
        e['map_trigger_count'] = len(mt)
        e['map_trigger_count_from_replans'] = e['replans'].get('地图变化（costmap', 0)
        e['map_evidence_available'] = bool(mt)
        e['map_hard_cells_values'] = [m['hard_cells'] for m in mt]
        e['map_hard_cells_mean'] = (round(sum(e['map_hard_cells_values']) / len(mt), 3)
                                    if mt else None)
        e['map_zero_hard_cells_count'] = sum(1 for m in mt if m['hard_cells'] == 0)
        e['map_layer_counts'] = {}
        for m in mt:
            for name, cnt in m['layers'].items():
                e['map_layer_counts'][name] = e['map_layer_counts'].get(name, 0) + cnt
        e['map_layer_unavailable'] = sorted({n for m in mt for n in m['layer_unavailable']})
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
                sum(o for r in rs for o in r['pred_offsets']) /
                sum(len(r['pred_offsets']) for r in rs), 4)
                if sum(len(r['pred_offsets']) for r in rs) else None,
            'shot_h_mean': round(sum(h for r in rs for h in r['shot_h']) /
                                 sum(len(r['shot_h']) for r in rs), 4)
                if sum(len(r['shot_h']) for r in rs) else None,
            'fco_active_mean': mean('fco_active'), 'vis_fixed_mean': mean('vis_fixed'),
            'active_ticks_mean': mean('active_ticks'), 'window_len_s_mean': mean('window_len_s'),
            'env_occ_given_vis_mean': mean('env_occ_given_vis'),
            'intervened_ticks_mean': mean('intervened_ticks'),
            'intervention_duration_s_mean': mean('intervention_duration_s'),
            'max_abs_delta_v_mean': mean('max_abs_delta_v'),
            'min_clearance_mean_m': mean('min_clearance_m'),
            # W7-b T3.6：active 窗口内干预（新增；旧字段保持原值）。
            'window_intervened_ticks_mean': mean('window_intervened_ticks'),
            'window_intervention_ratio_mean': mean('window_intervention_ratio'),
            'window_intervention_ticks_per_s_mean': mean('window_intervention_ticks_per_s'),
            'window_intervention_duration_s_mean': mean('window_intervention_duration_s'),
            'attr_intervened_per_active_s_mean': mean('attr_intervened_per_active_s'),
            # W7-b T2.6：地图触发实际量（AX-0089 后）
            'map_trigger_total': sum(r['map_trigger_count_from_replans'] for r in rs),
            'map_trigger_total_with_details': sum(r['map_trigger_count'] for r in rs),
            'map_evidence_available': any(r['map_evidence_available'] for r in rs),
            'map_hard_cells_mean': mean('map_hard_cells_mean'),
            'map_zero_hard_cells_count': sum(r['map_zero_hard_cells_count'] for r in rs),
            'map_layer_counts_total': dict(Counter(
                k for r in rs for k, c in r['map_layer_counts'].items() for _ in range(c))),
            'map_layer_unavailable': sorted({n for r in rs for n in r['map_layer_unavailable']}),
        }

    # W7-b T3.6：整段累计 vs active 窗口归一的比值（区分“窗口更长”与“单位时间更差”）。
    A_win = [e['window_intervention_ratio'] for e in eps
             if e['arm'] == ARMS[0] and e.get('window_intervention_ratio') is not None]
    B_win = [e['window_intervention_ratio'] for e in eps
             if e['arm'] == ARMS[1] and e.get('window_intervention_ratio') is not None]

    def _ratio(b, a):
        return round(b / a, 4) if (a and b is not None) else None

    norm = {
        'definition': ('窗口内干预占比 = episode_window_metrics.ticks_intervened / ticks_total；'
                       '每秒干预拍 = 占比 × rate_hz(%.0f)；整段累计 = command_attribution.intervened_ticks。'
                       % RATE_HZ),
        'window_intervention_ratio_mean': {
            ARMS[0]: round(sum(A_win) / len(A_win), 4) if A_win else None,
            ARMS[1]: round(sum(B_win) / len(B_win), 4) if B_win else None,
        },
        'B_over_A_window_ratio': _ratio(
            (sum(B_win) / len(B_win)) if B_win else None,
            (sum(A_win) / len(A_win)) if A_win else None),
        'B_over_A_window_ticks_per_s': _ratio(
            (sum(B_win) / len(B_win) * RATE_HZ) if B_win else None,
            (sum(A_win) / len(A_win) * RATE_HZ) if A_win else None),
        'B_over_A_active_window_len': _ratio(
            (sum(e['window_len_s'] for e in eps if e['arm'] == ARMS[1] and e.get('window_len_s')) /
             max(1, len([e for e in eps if e['arm'] == ARMS[1] and e.get('window_len_s')]))),
            (sum(e['window_len_s'] for e in eps if e['arm'] == ARMS[0] and e.get('window_len_s')) /
             max(1, len([e for e in eps if e['arm'] == ARMS[0] and e.get('window_len_s')])))),
        'B_over_A_window_intervened_ticks': _ratio(
            (sum(e['window_intervened_ticks'] or 0 for e in eps if e['arm'] == ARMS[1]) /
             max(1, len([e for e in eps if e['arm'] == ARMS[1] and e.get('window_intervened_ticks') is not None]))),
            (sum(e['window_intervened_ticks'] or 0 for e in eps if e['arm'] == ARMS[0]) /
             max(1, len([e for e in eps if e['arm'] == ARMS[0] and e.get('window_intervened_ticks') is not None])))),
        'B_over_A_attr_intervened_total': _ratio(
            sum(e['intervened_ticks'] or 0 for e in eps if e['arm'] == ARMS[1]),
            sum(e['intervened_ticks'] or 0 for e in eps if e['arm'] == ARMS[0])),
    }
    norm['reading'] = (
        '若“窗口内每秒干预拍”的 B/A 比与“整段累计”的 B/A 比相近，则累计更多主要来自窗口更长；'
        '若前者明显更高，才支持“单位时间控制更差”。窗口占比/每秒干预拍为归一化主量，'
        '整段累计只作补充。')

    out = {'kind': 'descriptive_diagnosis（ADR-0087(b)；不改 verdict）',
           'run_id': args.run_id, 'campaign': args.campaign,
           'scene': SCENE, 'target_truth_xy': list(TARGET_XY),
           'rate_hz': RATE_HZ,
           'per_arm': {a: agg(a) for a in ARMS},
           'intervention_normalization': norm,
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
            ('intervened_ticks_mean', '安全层干预拍均值（整段累计）'),
            ('intervention_duration_s_mean', '干预时长均值(s，整段累计)'),
            ('window_intervened_ticks_mean', 'active 窗口内干预拍均值'),
            ('window_intervention_ratio_mean', 'active 窗口内干预占比'),
            ('window_intervention_ticks_per_s_mean', 'active 窗口内每秒干预拍'),
            ('window_intervention_duration_s_mean', 'active 窗口内干预时长均值(s)'),
            ('attr_intervened_per_active_s_mean', '整段累计干预拍 / active 窗口秒'),
            ('max_abs_delta_v_mean', '干预最大速度修正均值(m/s)'),
            ('min_clearance_mean_m', '最小净空均值(m)')]
    for k, label in keys:
        md.append('| %s | %s | %s |' % (label, A.get(k), B.get(k)))
    md += ['', '## 安全干预按 active 窗口归一化（W7-b T3.6）', '',
           '> 口径：%s' % norm['definition'], '',
           '| 量 | A `p2_c2_low_level` | B `p2_rl_high` | B/A |', '|---|---|---|---|',
           '| active 窗口内干预拍均值 | %s | %s | %s |' % (
               A.get('window_intervened_ticks_mean'), B.get('window_intervened_ticks_mean'),
               norm['B_over_A_window_intervened_ticks']),
           '| active 窗口内干预占比 | %s | %s | %s |' % (
               A.get('window_intervention_ratio_mean'), B.get('window_intervention_ratio_mean'),
               norm['B_over_A_window_ratio']),
           '| active 窗口内每秒干预拍 | %s | %s | %s |' % (
               A.get('window_intervention_ticks_per_s_mean'), B.get('window_intervention_ticks_per_s_mean'),
               norm['B_over_A_window_ticks_per_s']),
           '| active 窗口时长均值 (s) | %s | %s | %s |' % (
               A.get('window_len_s_mean'), B.get('window_len_s_mean'),
               norm['B_over_A_active_window_len']),
           '| 整段累计干预拍均值 | %s | %s | %s |' % (
               A.get('intervened_ticks_mean'), B.get('intervened_ticks_mean'),
               norm['B_over_A_attr_intervened_total']),
           '', '- **读法**：%s' % norm['reading'], '',
           '- 说明：B/A 的“窗口内干预拍”与“整段累计”略有差异（窗口边界），'
           '两者并列用于分离“窗口更长 → 累计更多”与“单位时间控制更差”。', '',
           '## 地图触发证据（AX-0089 之后；T2.6）', '',
           '> 修复后 `mapSignature()` 只计 `LETHAL_OBSTACLE=254`；`hard_cells` 是该口径下的致命占据格数，'
           '分层计数来自各 `CostmapLayer` 自身栅格（inflation 层不可拆）。', '',
           '| arm | 地图触发计数 | 含 hard_cells 明细 | hard_cells 均值 | hard_cells=0 次数 | 分层计数累计 | 不可拆层 |',
           '|---|---|---|---|---|---|---|']
    for arm in ARMS:
        a = out['per_arm'][arm]
        md.append('| %s | %s | %s | %s | %s | %s | %s |' % (
            arm, a.get('map_trigger_total'), a.get('map_trigger_total_with_details'),
            a.get('map_hard_cells_mean'), a.get('map_zero_hard_cells_count'),
            a.get('map_layer_counts_total'), a.get('map_layer_unavailable')))
    md += ['', '- 若“含 hard_cells 明细”= 0 而计数 > 0：该日志为 **AX-0089 之前的旧格式**，'
           '无法核验未知 / inflation 伪触发；不得把计数 0 误读为“无地图触发”。',
           '- 判读：`hard_cells=0` 而仍触发 → 尺寸 / 原点或哈希口径变化（非未知 / inflation）；'
           '`hard_cells>0` → 确有致命占据格变化。若 `obstacle_layer / voxel_layer` 计数为主，'
           '可能是**移动目标被 marking 为静态障碍**（AX-0089 明确不消除该部分）；若 `static_layer` 为主，'
           '则是真实静态增删障。分层计数为触发时的**绝对值**，不含 last-plan 分层快照，'
           '故只能作组成证据、不能单独证明 delta。', '',
           '## 读法', '',
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
