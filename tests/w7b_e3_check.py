#!/usr/bin/env python3
"""w7b_e3_check —— W7-b T3（E3 / F-10）验收：可见性度量**按 episode active 窗口**切分。

读 campaign 报告里各 episode 的 `summary.json`，核查 `visibility_metrics`：
  * 存在且 `definition_version == "window_active_v1"`；
  * **分母 = 窗口内拍数**：`visibility_metrics.ticks_total == episode_window_metrics.ticks_total`
    （修正 F-10 所指「未按任务窗口切分」的分母失真）；
  * `ticks_visible ≤ ticks_total`，`visibility_rate ∈ [0,1]`；
  * `ticks_fire_control_opportunity ≤ ticks_visible`；
  * `not_applicable.target_truth == false`（目标真值可得）；
  * `longest_outage_s` / `recovery_time_last_s` 为有限非负。

判据依据 [F-10](docs/reviews/agc-review-2026-10-14/30-未来.md#f-10) 与 [ADR-0070](docs/adr/0070-w7b-research-propositions-and-rl-role-adjudication.md)(d)（E3）。

用法：
  python3 tests/w7b_e3_check.py --report experiments/w7-rl-b/<run_id>/metrics/report.json
"""

import argparse
import json
import os
import sys


def check_one(ep):
    sp = ep.get('summary_path')
    checks = {}
    if not sp or not os.path.exists(sp):
        return {'checks': {'summary_present': False}, 'pass': False, 'detail': None}
    with open(sp) as f:
        sm = json.load(f)
    vis = sm.get('visibility_metrics')
    if not vis:
        return {'checks': {'visibility_metrics_present': False}, 'pass': False, 'detail': None}
    wt = sm.get('episode_window_metrics', {}).get('ticks_total')
    tt = vis.get('ticks_total', 0)
    tv = vis.get('ticks_visible', 0)
    to = vis.get('ticks_fire_control_opportunity', 0)
    rate = vis.get('visibility_rate')
    checks = {
        'visibility_metrics_present': True,
        'definition_version_window_active': (vis.get('definition_version') == 'window_active_v1'),
        'denominator_is_window_ticks': (wt == tt and isinstance(wt, int) and wt > 0),
        'visible_le_total': (tv <= tt),
        'rate_in_unit_interval': (isinstance(rate, (int, float)) and 0.0 <= rate <= 1.0),
        'opportunity_le_visible': (to <= tv),
        'target_truth_available': (vis.get('not_applicable', {}).get('target_truth') is False),
        'outage_finite_nonneg': (isinstance(vis.get('longest_outage_s'), (int, float))
                                 and vis['longest_outage_s'] >= 0),
        'recovery_finite_nonneg': (isinstance(vis.get('recovery_time_last_s'), (int, float))
                                   and vis['recovery_time_last_s'] >= 0),
    }
    return {'checks': checks, 'pass': all(checks.values()),
            'detail': {'window_ticks': wt, 'visibility_ticks_total': tt, 'ticks_visible': tv,
                       'ticks_fire_control_opportunity': to, 'visibility_rate': rate,
                       'longest_outage_s': vis.get('longest_outage_s'),
                       'recovery_time_last_s': vis.get('recovery_time_last_s'),
                       'visibility_max_age_s': vis.get('visibility_max_age_s')}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--report', required=True)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()
    with open(args.report) as f:
        report = json.load(f)
    eps = [e for e in report.get('episodes', []) if e.get('summary_present')]
    results = []
    for e in eps:
        r = check_one(e)
        r.update({'scene': e.get('scene'), 'seed': e.get('seed'), 'run_id': e.get('run_id')})
        results.append(r)
    overall = bool(results) and all(r['pass'] for r in results)
    out = {'run_id': report.get('run_id'), 'report': args.report,
           'n_episodes_checked': len(results), 'episodes': results, 'overall_pass': overall,
           'proposition': 'E3 / F-10：可见率分母 = episode active 窗口内拍数（非全日志）；'
                          '含最长失联 / 恢复时长 / 有效火控机会。'}
    out_path = args.out or os.path.join(os.path.dirname(args.report), 'e3.json')
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    for r in results:
        print('=== %s seed=%s ===' % (r['scene'], r['seed']))
        for k, v in r['checks'].items():
            print('  %-38s %s' % (k, 'PASS' if v else 'FAIL'))
        print('  detail: %s' % json.dumps(r['detail'], ensure_ascii=False))
    print('\n%s (n=%d) -> %s' % ('PASS' if overall else 'FAIL', len(results), out_path))
    return 0 if overall else 1


if __name__ == '__main__':
    sys.exit(main())
