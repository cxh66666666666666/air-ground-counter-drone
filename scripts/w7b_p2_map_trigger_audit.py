#!/usr/bin/env python3
"""w7b_p2_map_trigger_audit —— P2 臂 B「地图变化」触发来源 / 路径影响审计（ADR-0092）。

输入：既有 `w7b_arms_runner` run 的 `log/ep_*.log`。
输出：`experiments/<campaign>/<run_id>/metrics/p2_map_trigger_audit.{json,md}`。

判据（只读诊断，不改行为 / 不改 verdict）：
  * 每次「地图变化」触发记录的分层 lethal 计数（static / obstacle / voxel）；
  * 触发前 / 后最近一次「规划成功：N 点」的点数变化（路径受影响的代理量）；
  * 触发时刻最近的 `HL_DIAG est=(x,y)`（目标估计）→ 触发 goal 与目标估计的距离；
  * 聚合：哪一层在连续触发间发生变化、hard_cells 范围、点数变化比例。

不直接断言「目标点云 = 触发来源」；只报告分层计数、目标估计距离与路径代理量，
供 ADR-0092 决策。真值仅用于离线评估，不进入运行时门。
"""

import argparse
import json
import os
import re
import statistics
from pathlib import Path

ROOT = Path('/home/chen/AGC')
NUM = r'[-+]?\d*\.?\d+(?:e[-+]?\d+)?'
TS = r'\[(\d+\.\d+)\]'

TRIG_RE = re.compile(
    r'\[INFO\] ' + TS + r' \[ground_baseline\]: 重规划触发：地图变化.*?'
    r'hard_cells=(\d+) layers=\[([^\]]*)\] goal=\(' + '(' + NUM + ')' + r',' + '(' + NUM + ')' + r'\) t=(' + NUM + r')')
PLAN_RE = re.compile(r'\[INFO\] ' + TS + r' \[ground_baseline\]: 规划成功：(\d+) 点')
HL_RE = re.compile(
    r'\[INFO\] ' + TS + r' \[agc_rl_high_level\]: HL_DIAG .*?est=\(' + '(' + NUM + ')' + r',' + '(' + NUM + ')' + r'\)')


def parse_layers(text):
    out = {}
    for part in text.split(','):
        if ':' not in part:
            continue
        k, v = part.split(':', 1)
        try:
            out[k.strip()] = int(v.strip())
        except ValueError:
            out[k.strip()] = None
    return out


def parse_log(path):
    trig, plans, hl = [], [], []
    for line in path.read_text(errors='replace').splitlines():
        m = TRIG_RE.search(line)
        if m:
            trig.append({
                'ts': float(m.group(1)),
                'hard_cells': int(m.group(2)),
                'layers': parse_layers(m.group(3)),
                'goal': (float(m.group(4)), float(m.group(5))),
                't': float(m.group(6)),
            })
            continue
        m = PLAN_RE.search(line)
        if m:
            plans.append({'ts': float(m.group(1)), 'points': int(m.group(2))})
            continue
        m = HL_RE.search(line)
        if m:
            hl.append({'ts': float(m.group(1)), 'est': (float(m.group(2)), float(m.group(3)))})
    return trig, plans, hl


def nearest(items, t, key='ts'):
    if not items:
        return None
    return min(items, key=lambda x: abs(x[key] - t))


def analyze(run_dir, source_run_id=None):
    logs = sorted((run_dir / 'log').glob('ep_*.log'))
    rows = []
    for log in logs:
        trig, plans, hl = parse_log(log)
        if not trig:
            continue
        prev_layers = None
        for tr in trig:
            before = None
            after = None
            for p in plans:
                if p['ts'] <= tr['ts']:
                    before = p
                elif after is None:
                    after = p
            h = nearest(hl, tr['ts'])
            row = {
                'log': log.name,
                'trigger_ts': tr['ts'],
                'trigger_t': tr['t'],
                'hard_cells': tr['hard_cells'],
                'layers': tr['layers'],
                'plan_points_before': None if before is None else before['points'],
                'plan_points_after': None if after is None else after['points'],
                'plan_points_delta': (
                    None if before is None or after is None else after['points'] - before['points']),
                'hl_target_est': None if h is None else list(h['est']),
                'hl_dt_s': None if h is None else round(h['ts'] - tr['ts'], 3),
            }
            if h is not None:
                row['goal_to_hl_est_m'] = round(
                    ((tr['goal'][0] - h['est'][0]) ** 2 + (tr['goal'][1] - h['est'][1]) ** 2) ** 0.5, 3)
            rows.append(row)
    # second pass: layer changes vs previous trigger in the same log
    by_log = {}
    for r in rows:
        by_log.setdefault(r['log'], []).append(r)
    layer_changes = {'static_layer': 0, 'obstacle_layer': 0, 'voxel_layer': 0}
    for _, rs in by_log.items():
        for i in range(1, len(rs)):
            a, b = rs[i - 1]['layers'], rs[i]['layers']
            for k in layer_changes:
                if a.get(k) != b.get(k):
                    layer_changes[k] += 1
    deltas = [abs(r['plan_points_delta']) for r in rows if r['plan_points_delta'] is not None]
    changed = [d for d in deltas if d != 0]
    hard = [r['hard_cells'] for r in rows]
    return {
        'run_id': run_dir.name,
        'source_run_id': source_run_id or run_dir.name,
        'n_logs_with_triggers': len(by_log),
        'n_triggers': len(rows),
        'n_plans': sum(len(parse_log(p)[1]) for p in logs),
        'layer_changes_between_consecutive_triggers': layer_changes,
        'hard_cells_min': min(hard) if hard else None,
        'hard_cells_max': max(hard) if hard else None,
        'plan_points_delta_abs_median': round(statistics.median(deltas), 3) if deltas else None,
        'plan_points_changed_fraction': (round(len(changed) / len(deltas), 4) if deltas else None),
        'triggers': rows,
        'scope_note': ('只读诊断：分层 lethal 计数 + 规划点数变化代理；'
                       '未记录 lethal 格坐标与路径几何，路径影响为代理量；不改 needReplan / verdict。'),
    }


def markdown(a):
    lines = [
        '# P2 地图触发来源 / 路径影响审计（ADR-0092）', '',
        '> **诊断 / 非确证**；不改 `needReplan`、不改 P2 verdict。',
        '', '## 汇总', '',
        f"- run：`{a['run_id']}`（源 `{a['source_run_id']}`）",
        f"- 含触发的 log：**{a['n_logs_with_triggers']}**；地图触发：**{a['n_triggers']}** 次",
        f"- `hard_cells` 范围：**{a['hard_cells_min']}–{a['hard_cells_max']}**",
        f"- 连续触发间分层计数变化：`{a['layer_changes_between_consecutive_triggers']}`",
        f"- 触发前后规划点数绝对变化中位：**{a['plan_points_delta_abs_median']}** 点；"
        f"点数发生变化比例：**{a['plan_points_changed_fraction']}**",
        '', '## 逐触发', '',
        '| log | t | hard_cells | static | obstacle | voxel | plan before→after | Δ | HL est | HL Δt |',
        '|---|---|---|---|---|---|---|---|---|---|',
    ]
    for r in a['triggers']:
        L = r['layers']
        lines.append(
            f"| {r['log']} | {r['trigger_t']} | {r['hard_cells']} | {L.get('static_layer')} | "
            f"{L.get('obstacle_layer')} | {L.get('voxel_layer')} | "
            f"{r['plan_points_before']}→{r['plan_points_after']} | {r['plan_points_delta']} | "
            f"{r['hl_target_est']} | {r['hl_dt_s']} |")
    lines += ['', '## 边界', '', a['scope_note'], '']
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--run-id', required=True, help='输出诊断 run id')
    ap.add_argument('--source-run-id', default=None, help='读取 log 的源 run id（缺省同 run-id）')
    args = ap.parse_args()
    source = ROOT / 'experiments' / args.campaign / (args.source_run_id or args.run_id)
    if not (source / 'log').is_dir():
        print('未找到 log 目录：%s' % (source / 'log'))
        return 2
    run_dir = ROOT / 'experiments' / args.campaign / args.run_id
    a = analyze(source, source_run_id=args.source_run_id or args.run_id)
    a['run_id'] = args.run_id
    out = run_dir / 'metrics'
    (run_dir / 'config').mkdir(parents=True, exist_ok=True)
    (run_dir / 'config' / 'source_run.json').write_text(json.dumps({
        'source_run_id': args.source_run_id or args.run_id,
        'purpose': 'ADR-0092 map trigger audit (diagnostic only)',
    }, indent=2, ensure_ascii=False) + '\n')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'p2_map_trigger_audit.json').write_text(json.dumps(a, indent=2, ensure_ascii=False))
    (out / 'p2_map_trigger_audit.md').write_text(markdown(a))
    print('triggers=%d layer_changes=%s changed_fraction=%s' % (
        a['n_triggers'], a['layer_changes_between_consecutive_triggers'],
        a['plan_points_changed_fraction']))
    print('->', out / 'p2_map_trigger_audit.md')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
