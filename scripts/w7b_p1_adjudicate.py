#!/usr/bin/env python3
"""w7b_p1_adjudicate —— 按**预注册规则**裁定 P1（协同 vs 非协同）。

输入：`experiments/<campaign>/<run_id>/metrics/arms.json`（`w7b_arms_runner.py` 产物）。
规则真源：`src/agc_rl/config/w7b_prereg.yaml`（**单一来源**——阈值 / 样本量 / 显著性**不**在本脚本硬编码；
本脚本只**读取**并执行，防事后改阈值，[F-13](../../docs/reviews/agc-review-2026-10-14/30-未来.md#f-13)）。

判定（预注册口径）：
  * DV primary = `fire_control_opportunity_rate`；DV secondary = `min_clearance`。
  * 自变量 α = 观测臂 `observe_yaw_mode` ∈ {`target`=协同, `velocity`=非协同}。
  * **同 seed 配对（CRN）**；每臂 `n_paired_episodes_per_arm` 个 seed。
  * **超时一律失败**（契约 C4）→ 失败 / infra / 未达终态 的回合，其 **primary DV 一律按 0**
    （失败即无有效火控机会），**计入分母**（`keep_all_failures_and_infra`）；**缺值**同样按 0
    （旧版对缺值整对剔除 → infra 常缺值 ⇒ 被排除在分母外，与预注册不一致，2026-10-20 已修正）。
    infra 数**单列**如实报告（`infra_listed_separately` + `infra_report`，含 ADR-0075 的重试分布）。
  * **secondary DV**（最小净空）失败回合仍有真实净空，记 0 会误表「碰撞」→ 缺值配对**剔除**并计数。
  * 判定 = 双侧**配对 t 检验** `p < alpha` **且** 均值配对差 ≥ `min_detectable_effect` ⇒ P1 成立；否则**证伪**。
  * 另报 95% 配对差 CI + Wilcoxon 符号秩 p（稳健性，**不**参与判定）。

用法：
  python3 scripts/w7b_p1_adjudicate.py --run-id 20261020-w7-rl-b-xxxxxx-xxxx
"""

import argparse
import json
import os
import sys

import yaml

ROOT = '/home/chen/AGC'
PREREG = os.path.join(ROOT, 'src/agc_rl/config/w7b_prereg.yaml')
TERMINALS_OK = ('TRACKING_SUCCESS', 'INTERCEPT_CONFIRMED')


def dv_value(row, key, zero_on_fail):
    """返回 (值, 是否规约而来)；值为 None = 该 DV 本回合无有效观测 → 剔除该配对并如实报告。

    `zero_on_fail`：**primary**(有效火控机会率) 用真——契约 C4「超时一律失败」+ 预注册
      `keep_all_failures_and_infra` ⇒ **失败 / infra / 未达终态**的回合**无有效火控机会 → 0**，
      **计入分母**；**缺值**时同样记 0（失败即无有效机会，**不**因缺值把配对剔除）。
    **secondary**(最小净空) 用假——失败回合仍有真实净空，记 0 会误表「碰撞」；缺值即剔除该配对。
    """
    v = row.get(key)
    failed = row.get('terminal_state') not in TERMINALS_OK
    if v is None:
        # 修正（2026-10-20）：旧行为对**缺值一律剔除配对** → infra / 失败回合（其 DV 常缺）被排除在
        # 分母外，与预注册 `keep_all_failures_and_infra`（保留全部失败 / infra 计数）**不一致**。
        # 现：primary 且**非成功终态** → 记 0 计入分母。
        if zero_on_fail and failed:
            return 0.0, True
        return None, True
    if zero_on_fail and failed:
        return 0.0, True
    return float(v), False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--coop-arm', default='p1_coop')
    ap.add_argument('--noncoop-arm', default='p1_noncoop')
    args = ap.parse_args()

    arms_path = os.path.join(ROOT, 'experiments', args.campaign, args.run_id, 'metrics', 'arms.json')
    with open(arms_path) as f:
        arms = json.load(f)
    with open(PREREG) as f:
        pre = yaml.safe_load(f)['w7b_prereg']

    stat = pre['statistics']
    alpha = float(stat['alpha'])
    mde = float(stat['min_detectable_effect'])
    n_req = int(stat['n_paired_episodes_per_arm'])

    by = {(r['arm'], r['seed']): r for r in arms['arms']}
    seeds = sorted({r['seed'] for r in arms['arms']
                    if r['arm'] in (args.coop_arm, args.noncoop_arm)})

    import numpy as np
    from scipy import stats

    out = {'run_id': args.run_id, 'campaign': args.campaign,
           'prereg_version': pre['version'], 'prereg_status_before': pre['status'],
           'alpha': alpha, 'min_detectable_effect': mde,
           'n_paired_episodes_per_arm_prereg': n_req,
           'coop_arm': args.coop_arm, 'noncoop_arm': args.noncoop_arm,
           'n_seeds_total': len(seeds)}

    for dv_key, dv_label, role, zof in (
            ('fire_control_opportunity_rate', 'fco_rate', 'primary', True),
            ('min_clearance_m', 'min_clearance', 'secondary', False)):
        coop, noncoop, rows, n_infra, n_dropped, n_zero = [], [], [], 0, 0, 0
        for s in seeds:
            c, n = by.get((args.coop_arm, s)), by.get((args.noncoop_arm, s))
            if not c or not n:
                continue
            if c['terminal_state'] == 'INFRA_FAILURE' or n['terminal_state'] == 'INFRA_FAILURE':
                n_infra += 1
            cv, c_deg = dv_value(c, dv_key, zof)
            nv, n_deg = dv_value(n, dv_key, zof)
            if cv is None or nv is None:
                n_dropped += 1
                rows.append({'seed': s, 'excluded': True, 'reason': 'dv_missing_on_non_success_terminal'
                             if not zof else 'dv_missing',
                             'coop_terminal': c['terminal_state'],
                             'noncoop_terminal': n['terminal_state']})
                continue
            n_zero += int(c_deg) + int(n_deg)
            coop.append(cv)
            noncoop.append(nv)
            rows.append({'seed': s, 'coop': cv, 'noncoop': nv, 'delta': round(cv - nv, 6),
                         'coop_terminal': c['terminal_state'], 'noncoop_terminal': n['terminal_state'],
                         'degraded': bool(c_deg or n_deg)})
        n = len(coop)
        if n < 2:
            out[dv_label] = {'role': role, 'n_paired': n, 'n_excluded_no_obs': n_dropped,
                             'verdict': 'insufficient_pairs'}
            continue
        d = np.array(coop) - np.array(noncoop)
        mean_d = float(np.mean(d))
        sd = float(np.std(d, ddof=1))
        sem = sd / np.sqrt(n)
        p_two = float(stats.ttest_rel(coop, noncoop).pvalue)
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
            'role': role, 'n_paired': n, 'n_infra_pairs': n_infra, 'n_excluded_no_obs': n_dropped,
            'n_zero_filled_values': n_zero,
            'mean_delta': round(mean_d, 6), 'sd_delta': round(sd, 6),
            'ci95': ci, 'paired_t_p_two_sided': p_two,
            'wilcoxon_p_two_sided': w_p,
            'threshold_min_detectable_effect': mde,
            'holds': holds, 'rows': rows,
        }

    prim = out.get('fco_rate', {})
    out['p1_verdict'] = ('成立（cooperative > non_cooperative）' if prim.get('holds')
                         else '证伪（未达预注册显著性 / MDE）')
    out['decision_rule'] = pre['decision']['p1']
    out['falsification_rule'] = pre['decision']['falsification']['p1']
    # ADR-0075：infra 率与重试分布**须随报告一并报出**（机器特性，不得读作臂效应）
    p1_rows = [r for r in arms['arms'] if r['arm'] in (args.coop_arm, args.noncoop_arm)]
    out['infra_report'] = {
        'n_runs_infra_terminal': sum(1 for r in p1_rows if r['terminal_state'] == 'INFRA_FAILURE'),
        'n_runs_used_infra_retry': sum(1 for r in p1_rows if (r.get('infra_retries_used') or 0) > 0),
        'max_infra_retries_used': max([(r.get('infra_retries_used') or 0) for r in p1_rows] or [0]),
        'policy': 'ADR-0075（分类判据=agc_rl.termination；infra 重试上限见 arms.json 的 infra_retry_protocol）',
    }
    out['integrity'] = {
        'n_paired_matches_prereg': prim.get('n_paired') == n_req,
        # 预注册 `keep_all_failures_and_infra`：失败 / infra 回合**必须**计入分母（primary DV 记 0）。
        # 判据 = primary 未被剔除任何配对（`n_excluded_no_obs == 0`）。
        'keep_all_failures_and_infra': {
            'policy': bool(pre['policies']['keep_all_failures_and_infra']),
            'primary_pairs_excluded': prim.get('n_excluded_no_obs'),
            'primary_pairs_zero_filled_values': prim.get('n_zero_filled_values'),
            'secondary_pairs_excluded': out.get('min_clearance', {}).get('n_excluded_no_obs'),
            'primary_honored': prim.get('n_excluded_no_obs') == 0,
            'note': 'primary(有效火控机会率)：失败 / infra / 缺值一律按 0 计入分母；'
                    'secondary(最小净空)：记 0 会误表「碰撞」，故缺值配对剔除并在此计数（不参与二次检验）',
        },
        'same_seed_paired': bool(pre['policies']['same_seed_paired']),
    }

    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
