#!/usr/bin/env python3
"""w7b_p1_adjudicate —— 按**预注册规则**裁定 P1（协同 vs 非协同）。

输入：`experiments/<campaign>/<run_id>/metrics/arms.json`（`w7b_arms_runner.py` 产物）。
规则真源：`src/agc_rl/config/w7b_prereg.yaml`（**单一来源**——阈值 / 样本量 / 显著性**不**在本脚本硬编码；
本脚本只**读取**并执行，防事后改阈值，[F-13](../../docs/reviews/agc-review-2026-10-14/30-未来.md#f-13)）。

判定（预注册 **v4** 口径，ADR-0084）：
  * DV primary = `fire_control_opportunity_rate_active`（**active 窗口**：GROUND_APPROACH→终态；
    不含终态后停泊）；DV secondary = `min_clearance_m`。
    v1 的 `fire_control_opportunity_rate`（`window_fixed_v1`，60 s 固定窗）降为 legacy/描述量，
    **不**参与 v4 判定。
  * 自变量 α = 观测臂 `observe_yaw_mode` ∈ {`target`=协同, `velocity`=非协同}。
  * **同 seed 配对（CRN）**；每臂 `n_paired_episodes_per_arm` 个 seed。
  * **超时一律失败**（契约 C4）→ 失败 / infra / 未达终态 的回合，其 **primary DV 一律按 0**
    （失败即无有效火控机会），**计入分母**（`keep_all_failures_and_infra`）；**缺值**同样按 0
    （旧版对缺值整对剔除 → infra 常缺值 ⇒ 被排除在分母外，与预注册不一致，2026-10-20 已修正）。
    infra 数**单列**如实报告（`infra_listed_separately` + `infra_report`，含 ADR-0075 的重试分布）。
  * **secondary DV**（最小净空）失败回合仍有真实净空，记 0 会误表「碰撞」→ 缺值配对**剔除**并计数。
  * 判定（**v6 三态**，ADR-0086）= 按 95% CI 与 **MME**（`decision.p1.minimum_meaningful_effect`，
    决策门槛）的关系：CI 下限 > MME → `supported`；CI 上限 < MME → `falsified`（可排除达到门槛的效应）；
    **CI 覆盖 MME → `inconclusive`（未通过门槛；既未支持也未排除）**。
    **不**再把 `mean(Δ) ≥ MDE` 当判定门槛——那是 v1–v5 的 legacy 口径（一并输出 `holds_legacy_criterion`
    仅供历史对照）；「样本量可检测效应」属**规划量**（`statistics.p1.planning`），与门槛分列。
  * 另报：对 MME 的**单侧检验** p（`H₀: Δ ≤ MME`）、`planned_detectable_effect_two_sided`
    （本 run σ / n 下的可检测效应）、`decision_n_required_est`（点估计 > MME 时断言所需的配对数）。

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
# 字段名 → (输出标签, 失败 / 缺值是否按 0 计入分母)。角色由预注册 decision.p1 决定，
# 这里只给稳定的输出标签（与 P2 脚本同构）。
DV_META = {
    'fire_control_opportunity_rate_active': ('fco_rate_active', True),
    'fire_control_opportunity_rate': ('fco_rate', True),
    'min_clearance_m': ('min_clearance', False),
}


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


def is_infra(row):
    """最终尝试为 infra（含重试耗尽后的 ACTIVE）——infra 单列，不读作臂效应。"""
    if row.get('terminal_state') == 'INFRA_FAILURE':
        return True
    attempts = row.get('infra_attempts') or []
    return bool(attempts) and attempts[-1].get('classification') == 'infra'


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

    p1d = pre['decision']['p1']
    # P1 统计口径单列；顶层字段保留为 v1–v5 历史，不回溯改写。
    stat = pre['statistics'].get('p1', pre['statistics'])
    alpha = float(stat['alpha'])
    power = float(stat.get('power', 0.80))
    # legacy 门槛字段：v6 起只作历史（v6 判定读 `decision.p1.minimum_meaningful_effect`）。
    mde = float(stat.get('min_detectable_effect',
                         pre['statistics'].get('min_detectable_effect', 0.10)))
    n_req = int(stat['n_paired_episodes_per_arm'])
    # ADR-0086：v6 三态判据（CI vs MME）与 v1–v5 legacy 口径（版本守卫，互不冒充）。
    criterion = p1d.get('criterion', 'p < alpha AND mean(delta) >= MDE')
    mme = p1d.get('minimum_meaningful_effect')
    mme = float(mme) if (criterion == 'three_state_ci_vs_mme' and mme is not None) else None
    primary_key = p1d['primary_dv']
    secondary_key = p1d['secondary_dv']
    legacy_key = p1d.get('legacy_dv')

    by = {(r['arm'], r['seed']): r for r in arms['arms']}
    seeds = sorted({r['seed'] for r in arms['arms']
                    if r['arm'] in (args.coop_arm, args.noncoop_arm)})

    import numpy as np
    from scipy import stats

    out = {'run_id': args.run_id, 'campaign': args.campaign,
           # P1 的注册版本 = 分命题版本（v6 = 三态判据）；文件版本单列。
           'prereg_version': pre.get('proposition_versions', {}).get('p1', pre['version']),
           'prereg_file_version': pre['version'],
           'prereg_status_before': pre['status'],
           'criterion': criterion,
           'alpha': alpha, 'power': power,
           'minimum_meaningful_effect': mme,
           'threshold_legacy_mde': mde,
           'n_paired_episodes_per_arm_prereg': n_req,
           'coop_arm': args.coop_arm, 'noncoop_arm': args.noncoop_arm,
           'direction': p1d.get('direction', 'coop_minus_noncoop'),
           'claim_scope': p1d.get('claim_scope', ''),
           'primary_dv': primary_key, 'secondary_dv': secondary_key,
           'legacy_dv': legacy_key,
           'n_seeds_total': len(seeds)}

    dv_specs = [(primary_key, 'primary'), (secondary_key, 'secondary')]
    if legacy_key:
        dv_specs.append((legacy_key, 'legacy'))
    for dv_key, role in dv_specs:
        dv_label, zof = DV_META[dv_key]
        coop, noncoop, rows, n_infra, n_dropped, n_zero = [], [], [], 0, 0, 0
        for s in seeds:
            c, n = by.get((args.coop_arm, s)), by.get((args.noncoop_arm, s))
            if not c or not n:
                continue
            if is_infra(c) or is_infra(n):
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
            # 要断言 Δ>MME 所需的配对数（规划口径；σ 用本 run 实测 SD）。
            # 仅在**点估计高于门槛**时有意义（否则任何 N 都不能支持 Δ>MME）。
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

    prim = out.get(DV_META[primary_key][0], {})
    if criterion == 'three_state_ci_vs_mme':
        state = prim.get('state')
        verdict = {
            'supported': '成立（CI 下限 > MME：收益达到最小有意义量）',
            'falsified': '证伪（CI 上限 < MME：可排除达到门槛的效应）',
            'inconclusive': '未通过门槛（CI 覆盖 MME：既未支持也未排除）',
        }.get(state, '未判定（有效配对数不足）')
    elif 'holds_legacy_criterion' not in prim:
        verdict = '未判定（有效配对数不足）'
    elif prim['holds_legacy_criterion']:
        verdict = '成立（cooperative > non_cooperative；v1–v5 legacy 口径）'
    elif (prim.get('paired_t_p_two_sided', 1.0) < alpha and prim.get('mean_delta', 0.0) < 0.0):
        verdict = '证伪（primary 方向为负且显著：非协同 > 协同；legacy 口径）'
    else:
        verdict = '证伪（未达预注册显著性 / MDE；legacy 口径）'
    out['p1_verdict'] = verdict
    out['criterion'] = criterion
    out['decision_rule'] = p1d.get('rule', '')
    out['falsification_rule'] = p1d.get('falsification', pre['decision']['falsification']['p1'])
    # ADR-0075：infra 率与重试分布**须随报告一并报出**（机器特性，不得读作臂效应）
    p1_rows = [r for r in arms['arms'] if r['arm'] in (args.coop_arm, args.noncoop_arm)]
    out['infra_report'] = {
        'n_runs_infra_terminal': sum(1 for r in p1_rows if r['terminal_state'] == 'INFRA_FAILURE'),
        'n_runs_final_infra': sum(1 for r in p1_rows if is_infra(r)),
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
