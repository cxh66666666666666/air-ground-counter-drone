#!/usr/bin/env python3
"""w7b_p2_adjudicate —— 按**预注册规则**裁定 P2（职责分层：臂 B 规则高层 vs 臂 A c2）。

输入：`experiments/<campaign>/<run_id>/metrics/arms.json`（`w7b_arms_runner.py` 产物）。
规则真源：`src/agc_rl/config/w7b_prereg.yaml` 的 `statistics.p2` / `decision.p2`
（**单一来源**——阈值 / 样本量 / 显著性**不**在本脚本硬编码；本脚本只读取并执行，
[F-13](../../docs/reviews/agc-review-2026-10-14/30-未来.md#f-13)）。

判定（预注册 **v3** 口径，[ADR-0082](../docs/adr/0082-w7b-p2-arm-b-rule-high-level-first.md) /[ADR-0083](../docs/adr/0083-w7b-p2-arm-information-equivalence-and-active-window-fco.md)）：
  * 自变量 β = 命令源：臂 A `p2_c2_low_level`（直接出 `cmd_vel`） vs
    臂 B `p2_rl_high`（规则高层拦截点 + 成熟低层 Navfn+LQR）；固定协同（full 链）。
  * DV primary（v3，ADR-0083）= `fire_control_opportunity_rate_active`（**active 窗口**：
    GROUND_APPROACH→终态；不含终态后停泊）；v2 的 `fire_control_opportunity_rate`
    （`window_fixed_v1`，60 s 固定窗）仅保留为历史/描述量。DV secondary = `min_clearance_m`（只报告）。
  * **同 seed 配对（CRN）**；每臂 `statistics.p2.n_paired_episodes_per_arm` 个 seed。
  * **超时一律失败**（契约 C4）→ 失败 / infra / 未达终态 的回合，其 **primary DV 按 0**
    计入分母（`keep_all_failures_and_infra`）；成功回合缺值 → 剔除该配对并计数。
  * **secondary**（最小净空）失败回合仍有真实净空，记 0 会误表「碰撞」→ 缺值配对剔除并计数。
  * 判定（**v6 三态**，ADR-0086）= 按 95% CI 与 **MME**（`decision.p2.minimum_meaningful_effect`）的关系：
    CI 下限 > MME → `supported`；CI 上限 < MME → `falsified`；**CI 覆盖 MME → `inconclusive`**。
    `mean(Δ) ≥ MDE` 属 v3 legacy 口径（输出 `holds_legacy_criterion` 仅供历史对照）；
    「样本量可检测效应」属**规划量**（`statistics.p2.planning`），与门槛分列。
  * 另报 95% 配对差 CI + Wilcoxon 符号秩 p（稳健性，**不**参与判定）。

用法：
  python3 scripts/w7b_p2_adjudicate.py --run-id 20261022-w7-rl-b-xxxxxx-xxxx \
      --out experiments/w7-rl-b/<run_id>/metrics/p2_adjudication.json
"""

import argparse
import json
import os
import sys

import yaml

ROOT = '/home/chen/AGC'
PREREG = os.path.join(ROOT, 'src/agc_rl/config/w7b_prereg.yaml')
# 复用 P1 的 DV 规约函数（同一 primary/secondary 失败口径，避免两命题漂移）。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from w7b_p1_adjudicate import dv_value, is_infra  # noqa: E402

# 与 P1 一致的标签 / 失败规约映射；字段名来自预注册 decision.p2。
DV_META = {
    # v3 primary（ADR-0083）：active 窗口 FCO（episode_window_metrics 派生，不含终态后停泊）。
    'fire_control_opportunity_rate_active': ('fco_rate_active', 'primary', True),
    # v2 历史口径（window_fixed_v1）：保留可复算，不再作为 v3 primary。
    'fire_control_opportunity_rate': ('fco_rate', 'legacy', True),
    'min_clearance_m': ('min_clearance', 'secondary', False),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--arm-a', default='')
    ap.add_argument('--arm-b', default='')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    arms_path = os.path.join(ROOT, 'experiments', args.campaign, args.run_id, 'metrics', 'arms.json')
    with open(arms_path) as f:
        arms = json.load(f)
    with open(PREREG) as f:
        pre = yaml.safe_load(f)['w7b_prereg']

    p2d = pre['decision']['p2']
    arm_a = args.arm_a or p2d['arms']['a']
    arm_b = args.arm_b or p2d['arms']['b']
    stat = pre['statistics']['p2']
    alpha = float(stat['alpha'])
    power = float(stat.get('power', 0.80))
    # legacy 门槛字段：v6 起只作历史（判定读 `decision.p2.minimum_meaningful_effect`）。
    mde = float(stat.get('min_detectable_effect',
                         pre['statistics'].get('min_detectable_effect', 0.10)))
    n_req = int(stat['n_paired_episodes_per_arm'])
    # ADR-0086：v6 三态判据（CI vs MME）与 v3 legacy 口径（版本守卫，互不冒充）。
    criterion = p2d.get('criterion', 'p < alpha AND mean(delta) >= MDE')
    mme = p2d.get('minimum_meaningful_effect')
    mme = float(mme) if (criterion == 'three_state_ci_vs_mme' and mme is not None) else None
    primary_key = p2d['primary_dv']
    secondary_key = p2d['secondary_dv']

    by = {(r['arm'], r['seed']): r for r in arms['arms']}
    seeds = sorted({r['seed'] for r in arms['arms'] if r['arm'] in (arm_a, arm_b)})

    import numpy as np
    from scipy import stats

    out = {
        'run_id': args.run_id, 'campaign': args.campaign,
        # P2 的注册版本 = 分命题版本（v2 新增 P2 细则；文件版本单列）。
        'prereg_version': pre.get('proposition_versions', {}).get('p2', pre['version']),
        'prereg_file_version': pre['version'],
        'prereg_status_before': pre['status'],
        'alpha': alpha, 'min_detectable_effect': mde,
        'criterion': criterion,
        'minimum_meaningful_effect': mme,
        'threshold_legacy_mde': mde,
        'n_paired_episodes_per_arm_prereg': n_req,
        'power': power,
        'planning': stat.get('planning', {}),
        'arm_a': arm_a, 'arm_b': arm_b, 'direction': 'b_minus_a',
        'n_seeds_total': len(seeds),
        'claim_scope': p2d.get('claim_scope', ''),
    }

    for dv_key in (primary_key, secondary_key):
        dv_label, role, zof = DV_META[dv_key]
        a_vals, b_vals, rows, n_infra, n_dropped, n_zero = [], [], [], 0, 0, 0
        for s in seeds:
            a, b = by.get((arm_a, s)), by.get((arm_b, s))
            if not a or not b:
                continue
            if is_infra(a) or is_infra(b):
                n_infra += 1
            av, a_deg = dv_value(a, dv_key, zof)
            bv, b_deg = dv_value(b, dv_key, zof)
            if av is None or bv is None:
                n_dropped += 1
                rows.append({
                    'seed': s, 'excluded': True,
                    'reason': 'dv_missing_on_non_success_terminal' if not zof else 'dv_missing',
                    'arm_a_terminal': a['terminal_state'], 'arm_b_terminal': b['terminal_state'],
                })
                continue
            n_zero += int(a_deg) + int(b_deg)
            a_vals.append(av)
            b_vals.append(bv)
            rows.append({
                'seed': s, 'a': av, 'b': bv, 'delta': round(bv - av, 6),
                'arm_a_terminal': a['terminal_state'], 'arm_b_terminal': b['terminal_state'],
                'degraded': bool(a_deg or b_deg),
            })
        n = len(a_vals)
        if n < 2:
            out[dv_label] = {
                'role': role, 'n_paired': n, 'n_infra_pairs': n_infra,
                'n_excluded_no_obs': n_dropped, 'n_zero_filled_values': n_zero,
                'verdict': 'insufficient_pairs', 'rows': rows,
            }
            continue
        d = np.array(b_vals) - np.array(a_vals)
        mean_d = float(np.mean(d))
        sd = float(np.std(d, ddof=1))
        sem = sd / np.sqrt(n)
        p_two = float(stats.ttest_rel(b_vals, a_vals).pvalue)
        tcrit = stats.t.ppf(1 - alpha / 2, n - 1)
        ci = [round(mean_d - tcrit * sem, 6), round(mean_d + tcrit * sem, 6)]
        w_p = None
        if np.any(d != 0):
            try:
                w_p = float(stats.wilcoxon(d).pvalue)
            except ValueError:
                w_p = None
        holds = bool(p_two < alpha and mean_d >= mde)
        out[dv_label] = {
            'role': role, 'n_paired': n, 'n_infra_pairs': n_infra,
            'n_excluded_no_obs': n_dropped, 'n_zero_filled_values': n_zero,
            'mean_delta': round(mean_d, 6), 'sd_delta': round(sd, 6),
            'ci95': ci, 'paired_t_p_two_sided': p_two,
            'wilcoxon_p_two_sided': w_p,
            'threshold_legacy_mde': mde,
            'holds_legacy_criterion': holds, 'rows': rows,
        }
        # v6（ADR-0086）：三态（CI vs MME）——只对参与判定的 DV（primary）计算。
        if role == 'primary' and mme is not None:
            t_vs_mme = (mean_d - mme) / sem
            p_one = float(stats.t.sf(t_vs_mme, n - 1))     # H0: Δ <= MME vs H1: Δ > MME
            if ci[0] > mme:
                state = 'supported'
            elif ci[1] < mme:
                state = 'falsified'
            else:
                state = 'inconclusive'
            z_need = stats.norm.ppf(1 - alpha / 2) + stats.norm.ppf(power)
            n_req_est = (int(round((z_need * sd / (mean_d - mme)) ** 2))
                         if mean_d > mme else None)
            out[dv_label].update({
                'minimum_meaningful_effect': mme,
                'state': state,
                'p_one_sided_vs_mme': p_one,
                'planned_detectable_effect_two_sided': round(z_need * sd / np.sqrt(n), 6),
                'decision_n_required_est': n_req_est,
                'decision_n_required_note': (
                    '若真实效应等于本观测点估计，需该配对数才能在 α / power 下断言 Δ>MME'
                    if n_req_est else '点估计 ≤ MME：按本估计任何 N 都不支持 Δ>MME（须更大效应或更低 MME）'),
            })

    # primary 标签由预注册 primary_dv 决定（v3 = fco_rate_active；v2 legacy = fco_rate）。
    primary_label = DV_META[primary_key][0]
    prim = out.get(primary_label, {})
    if criterion == 'three_state_ci_vs_mme':
        state = prim.get('state')
        verdict = {
            'supported': '成立（CI 下限 > MME：臂 B 的优势达到最小有意义量）',
            'falsified': '证伪（CI 上限 < MME：可排除达到门槛的效应 → 保留 c2）',
            'inconclusive': '未通过门槛（CI 覆盖 MME：既未支持也未排除 → 保留 c2）',
        }.get(state, '未判定（有效配对数不足）')
    elif 'holds_legacy_criterion' not in prim:
        verdict = '未判定（有效配对数不足）'
    elif prim['holds_legacy_criterion']:
        verdict = '成立（臂 B > 臂 A：职责分层成立；v3 legacy 口径）'
    elif (prim.get('paired_t_p_two_sided', 1.0) < alpha and prim.get('mean_delta', 0.0) < 0.0):
        verdict = '证伪（未达优越性判据；primary 方向为负且显著 → 保留 c2；legacy 口径）'
    else:
        verdict = '证伪（未达预注册显著性 / MDE → 保留 c2；legacy 口径）'
    out['p2_verdict'] = verdict
    out['decision_rule'] = p2d.get('rule', '')
    out['falsification_rule'] = p2d.get('falsification', '')

    # ADR-0075：infra 率与重试分布须随报告一并报出（机器特性，不得读作臂效应）。
    p2_rows = [r for r in arms['arms'] if r['arm'] in (arm_a, arm_b)]
    out['infra_report'] = {
        'n_runs_infra_terminal': sum(1 for r in p2_rows if r['terminal_state'] == 'INFRA_FAILURE'),
        'n_runs_final_infra': sum(1 for r in p2_rows if is_infra(r)),
        'n_runs_used_infra_retry': sum(1 for r in p2_rows if (r.get('infra_retries_used') or 0) > 0),
        'max_infra_retries_used': max([(r.get('infra_retries_used') or 0) for r in p2_rows] or [0]),
        'policy': 'ADR-0075（分类判据=agc_rl.termination；infra 重试上限见 arms.json 的 infra_retry_protocol）',
    }
    out['integrity'] = {
        'n_paired_matches_prereg': prim.get('n_paired') == n_req,
        'keep_all_failures_and_infra': {
            'policy': bool(pre['policies']['keep_all_failures_and_infra']),
            'primary_pairs_excluded': prim.get('n_excluded_no_obs'),
            'primary_pairs_zero_filled_values': prim.get('n_zero_filled_values'),
            'secondary_pairs_excluded': out.get('min_clearance', {}).get('n_excluded_no_obs'),
            'primary_honored': prim.get('n_excluded_no_obs') == 0,
            'note': 'primary(有效火控机会率)：失败 / infra / 缺值一律按 0 计入分母；'
                    'secondary(最小净空)：缺值配对剔除并在此计数（不参与判定）',
        },
        'same_seed_paired': bool(pre['policies']['same_seed_paired']),
    }

    text = json.dumps(out, ensure_ascii=False, indent=2)
    if args.out:
        with open(args.out, 'w') as f:
            f.write(text + '\n')
        print('-> %s' % args.out)
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
