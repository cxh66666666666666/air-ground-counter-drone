#!/usr/bin/env python3
"""w7b_fire_control_lifecycle —— 逐 shot 火控生命周期分析（ADR-0091 / FL-5）。

输入：`experiments/<campaign>/<run_id>/log/ep_*.log`。
输出：同 run `metrics/fire_control_lifecycle.{json,md}`。

判据（诊断，不改 verdict）：
  * 每个 SHOT_FIRED 恰有一个终局（SHOT_HIT / SHOT_MISS / SHOT_ABORTED）；
  * 无未闭合 shot；无重复终局；
  * 0 <= flight_s <= net_flight_timeout_s + 1/rate_hz + 容差（默认 3.15 s）；
  * miss_dist_m 无 5007 m 级异常；全程无有效观测时允许 inf（miss_dist_valid=false 语义）。
"""

import argparse
import json
import math
import re
from pathlib import Path

ROOT = Path('/home/chen/AGC')
NUM = r'[-+]?\d*\.?\d+(?:e[-+]?\d+)?'
FIRE_RE = re.compile(r'SHOT_FIRED shot=(\S+) t=(' + NUM + r')')
MISS_RE = re.compile(r'SHOT_MISS shot=(\S+) .*?flight=(' + NUM + r') s miss_dist=(' + NUM + r'|inf) m')
HIT_RE = re.compile(r'CAPTURE（发射式网动态命中）：shot=(\S+) .*?flight=(' + NUM + r') s miss_dist=(' + NUM + r'|inf) m')
ABORT_RE = re.compile(r'SHOT_ABORTED shot=(\S+) reason=(\S+) t=(' + NUM + r') flight=(' + NUM + r') s')


def parse(path):
    fired, terminals = {}, []
    for line in path.read_text(errors='replace').splitlines():
        m = FIRE_RE.search(line)
        if m:
            fired[m.group(1)] = {'t_fire': float(m.group(2))}
            continue
        m = MISS_RE.search(line)
        if m:
            terminals.append({'shot': m.group(1), 'kind': 'SHOT_MISS',
                              'flight_s': float(m.group(2)),
                              'miss_dist_m': (math.inf if m.group(3) == 'inf' else float(m.group(3)))})
            continue
        m = HIT_RE.search(line)
        if m:
            terminals.append({'shot': m.group(1), 'kind': 'SHOT_HIT',
                              'flight_s': float(m.group(2)),
                              'miss_dist_m': (math.inf if m.group(3) == 'inf' else float(m.group(3)))})
            continue
        m = ABORT_RE.search(line)
        if m:
            terminals.append({'shot': m.group(1), 'kind': 'SHOT_ABORTED',
                              'reason': m.group(2), 't_event': float(m.group(3)),
                              'flight_s': float(m.group(4)),
                              'miss_dist_m': math.inf})
    return fired, terminals


def analyze(run_dir):
    logs = sorted((run_dir / 'log').glob('*.log'))
    rows = []
    for log in logs:
        fired, terminals = parse(log)
        by_shot = {}
        for t in terminals:
            by_shot.setdefault(t['shot'], []).append(t)
        unresolved = sorted(set(fired) - set(by_shot))
        duplicated = sorted(k for k, v in by_shot.items() if len(v) > 1)
        flights = [t['flight_s'] for t in terminals if 'flight_s' in t]
        miss = [t['miss_dist_m'] for t in terminals
                if t['kind'] == 'SHOT_MISS' and math.isfinite(t['miss_dist_m'])]
        rows.append({
            'log': log.name,
            'n_fired': len(fired),
            'n_hit': sum(1 for t in terminals if t['kind'] == 'SHOT_HIT'),
            'n_miss': sum(1 for t in terminals if t['kind'] == 'SHOT_MISS'),
            'n_aborted': sum(1 for t in terminals if t['kind'] == 'SHOT_ABORTED'),
            'unresolved': unresolved,
            'duplicated': duplicated,
            'flight_min': min(flights) if flights else None,
            'flight_max': max(flights) if flights else None,
            'flight_negative': any(f < 0 for f in flights),
            'flight_over_bound': [f for f in flights if f > 3.15 + 1e-6],
            'miss_dist_max_finite': max(miss) if miss else None,
            'miss_dist_absurd': [m for m in miss if m > 100.0],
            'shots': sorted(fired),
        })
    return {
        'run_id': run_dir.name,
        'n_logs': len(logs),
        'n_fired': sum(r['n_fired'] for r in rows),
        'n_hit': sum(r['n_hit'] for r in rows),
        'n_miss': sum(r['n_miss'] for r in rows),
        'n_aborted': sum(r['n_aborted'] for r in rows),
        'n_unresolved': sum(len(r['unresolved']) for r in rows),
        'n_duplicated': sum(len(r['duplicated']) for r in rows),
        'n_negative_flight': sum(1 for r in rows if r['flight_negative']),
        'n_over_bound': sum(len(r['flight_over_bound']) for r in rows),
        'n_absurd_miss_dist': sum(len(r['miss_dist_absurd']) for r in rows),
        'logs': rows,
        'scope_note': '诊断 / 非确证；不改 P1/P2 verdict；flight bound=3.15 s（3.0 + 1/20 + 容差）。',
    }


def markdown(a):
    lines = [
        '# 火控生命周期逐 shot 分析（ADR-0091）', '',
        '> **诊断 / 非确证**；不改 P1 / P2 verdict。', '',
        '## 汇总', '',
        f"- run：`{a['run_id']}`",
        f"- fired **{a['n_fired']}** / hit **{a['n_hit']}** / miss **{a['n_miss']}** / aborted **{a['n_aborted']}**",
        f"- 未闭合 shot：**{a['n_unresolved']}**；重复终局：**{a['n_duplicated']}**",
        f"- 负飞行时长：**{a['n_negative_flight']}**；超上界（>3.15 s）：**{a['n_over_bound']}**",
        f"- 100 m 级以上 `miss_dist`：**{a['n_absurd_miss_dist']}**",
        '', '## 逐 log', '',
        '| log | fired | hit | miss | aborted | unresolved | duplicated | flight min–max | over bound | miss max |',
        '|---|---|---|---|---|---|---|---|---|---|',
    ]
    for r in a['logs']:
        lines.append(
            f"| {r['log']} | {r['n_fired']} | {r['n_hit']} | {r['n_miss']} | {r['n_aborted']} | "
            f"{len(r['unresolved'])} | {len(r['duplicated'])} | "
            f"{r['flight_min']}–{r['flight_max']} | {len(r['flight_over_bound'])} | "
            f"{r['miss_dist_max_finite']} |")
    lines += ['', '## 边界', '', a['scope_note'], '']
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    run_dir = ROOT / 'experiments' / args.campaign / args.run_id
    if not (run_dir / 'log').is_dir():
        print('未找到 log 目录：%s' % (run_dir / 'log'))
        return 2
    a = analyze(run_dir)
    out = run_dir / 'metrics'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'fire_control_lifecycle.json').write_text(json.dumps(a, indent=2, ensure_ascii=False))
    (out / 'fire_control_lifecycle.md').write_text(markdown(a))
    print('fired=%d unresolved=%d duplicated=%d over_bound=%d absurd_miss=%d' % (
        a['n_fired'], a['n_unresolved'], a['n_duplicated'], a['n_over_bound'], a['n_absurd_miss_dist']))
    print('->', out / 'fire_control_lifecycle.md')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
