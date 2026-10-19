#!/usr/bin/env python3
"""w7_core_check —— W7-a 核心库自包含核查（T1 判定 · T2 版本化 · T3 种子分解）。

**不依赖仿真**（纯库 / 纯 Python），验证三件机制：
  T1（R1）`agc_rl.termination` 为**单一判定来源**（与 eval adapter 同一对象）；terminated / truncated
           语义可分，且**超时一律失败**（契约 C4）、infra 单列不吞。
  T2（R2）`obs_version` 机制：v1 维度与 `interface` 冻结一致；不同版本元数据可区分；旧版本回放可用；
           checkpoint 元数据 save/load 往返 + 指纹稳定。
  T3（R3）种子分解：确定可复现；外生流**不随 attempt 变**（CRN）；policy 随 attempt 变；
           多 seed 下 **spawn 与噪声都变**。

证据落 `experiments/w7-rl/<run_id>/metrics/core.json`；退出码 0 = 全 PASS。
"""

import argparse
import json
import os
import subprocess
import sys

ROOT = '/home/chen/AGC'
CAMPAIGN = 'w7-rl'
sys.path.insert(0, os.path.join(ROOT, 'scripts'))                  # episode_runner（单一来源一致性核查）

from agc_rl import interface, obs_spec, seeding, termination  # noqa: E402


def t1_termination(res):
    c_success = termination.classify_episode_outcome('TRACKING_SUCCESS', '', 'TRACKING_SUCCESS', None, 10.0, 100.0)
    c_timeout = termination.classify_episode_outcome('MISSION_FAILED', 'TIMEOUT', 'TRACKING_SUCCESS', None, 40.0, 100.0)
    c_active = termination.classify_episode_outcome('ACTIVE', '', 'TRACKING_SUCCESS', None, 150.0, 150.0)
    c_infra = termination.classify_episode_outcome('INFRA_FAILURE', '', None, None, 5.0, 100.0)
    checks = {
        'success_is_terminated_counted': bool(c_success['is_task_terminal'] and not c_success['is_truncation']
                                              and c_success['counted_as_success']),
        'timeout_is_terminated_not_success': bool(c_timeout['is_task_terminal'] and not c_timeout['is_truncation']
                                                   and not c_timeout['counted_as_success']
                                                   and c_timeout['budget_kind'] == 'mission_sim_budget'),
        'active_is_truncated_watchdog': bool((not c_active['is_task_terminal']) and c_active['is_truncation']
                                             and c_active['budget_kind'] == 'episode_wall_watchdog'),
        'infra_is_terminated_but_infra_class': bool(c_infra['is_task_terminal'] and c_infra['classification'] == 'infra'
                                                     and not c_infra['counted_as_success']),
        'terminated_truncated_mutually_exclusive': all(
            not (c['is_task_terminal'] and c['is_truncation']) for c in (c_success, c_timeout, c_active, c_infra)),
    }
    # 单一判定来源：eval adapter 与训练侧引用**同一函数对象**。
    try:
        import episode_runner  # noqa: E402
        checks['single_source_identity_with_eval_adapter'] = bool(
            episode_runner.classify_episode_outcome is termination.classify_episode_outcome
            and episode_runner.TIME_BUDGETS is termination.TIME_BUDGETS)
    except Exception as exc:  # noqa: BLE001
        checks['single_source_identity_with_eval_adapter'] = False
        res['eval_import_error'] = repr(exc)
    res['t1_cases'] = {'success': c_success, 'timeout': c_timeout, 'active': c_active, 'infra': c_infra}
    return checks


def t2_obs_version(res):
    num = 12
    spec1 = obs_spec.get_obs_spec(1, num)
    m1 = obs_spec.build_checkpoint_meta(num, obs_version=1, params={'source': 'unit'})
    m2 = obs_spec.build_checkpoint_meta(num, obs_version=2, params={'source': 'unit'})
    ok_same, _ = obs_spec.check_compatible(m1, current_version=1, num_sectors=num)
    ok_diff, reasons = obs_spec.check_compatible(m2, current_version=1, num_sectors=num)
    unknown_raises = False
    try:
        obs_spec.get_obs_spec(99, num)
    except ValueError:
        unknown_raises = True

    # 旧版本回放：v1 向量与 `interface` 冻结构造**逐位一致**。
    sectors = [0.5] * num
    v1 = obs_spec.build_obs_vector(1, sectors, 1.0, -2.0, 3.0, 2, num, 10.0, 5.0)
    ref = interface.build_observation(sectors, 1.0, -2.0, 3.0, 2, num, 10.0, 5.0)

    tmp = os.path.join(res['run_dir'], 'tmp_ckpt_meta.json')
    m1.save(tmp)
    loaded = obs_spec.CheckpointMeta.load(tmp)
    fp_a = obs_spec.checkpoint_fingerprint(m1)
    fp_b = obs_spec.checkpoint_fingerprint(loaded)

    checks = {
        'v1_dim_matches_frozen_interface': spec1.dim == interface.obs_dim(num),
        'version_distinguishable_in_meta': m1.to_dict()['obs_version'] != m2.to_dict()['obs_version'],
        'v1_compatible': ok_same,
        'v2_incompatible_with_reason': (not ok_diff) and len(reasons) >= 1,
        'unknown_version_raises': unknown_raises,
        'old_version_replay_matches_frozen': v1 == ref,
        'checkpoint_roundtrip_fingerprint_stable': (fp_a == fp_b) and loaded.obs_version == 1,
    }
    res['t2'] = {'spec_v1': spec1.to_dict(), 'v2_reasons': reasons,
                 'fingerprint': fp_a, 'obs_dim': interface.obs_dim(num),
                 'action_space': obs_spec.ActionSpaceDescriptor().to_dict()}
    os.remove(tmp)
    return checks


def t3_seeding(res):
    p_a = seeding.make_seed_plan(7, 's01', 0, 0)
    p_a2 = seeding.make_seed_plan(7, 's01', 0, 0)
    p_attempt = seeding.make_seed_plan(7, 's01', 0, 1)
    p_episode = seeding.make_seed_plan(7, 's01', 1, 0)
    p_master = seeding.make_seed_plan(8, 's01', 0, 0)

    exo_same = all(p_a.streams[s] == p_attempt.streams[s] for s in seeding.EXOGENOUS_STREAMS)
    noise7 = [p_a.rng('sensor').gauss(0.0, 1.0) for _ in range(5)]
    noise8 = [p_master.rng('sensor').gauss(0.0, 1.0) for _ in range(5)]

    checks = {
        'deterministic_same_inputs': p_a.streams == p_a2.streams,
        'exogenous_invariant_across_attempt': exo_same,
        'policy_varies_across_attempt': p_a.streams['policy'] != p_attempt.streams['policy'],
        'spawn_varies_across_episode': p_a.streams['spawn'] != p_episode.streams['spawn'],
        'spawn_varies_across_master_seed': p_a.streams['spawn'] != p_master.streams['spawn'],
        'noise_varies_across_master_seed': noise7 != noise8,
        'all_six_streams_distinct': len(set(p_a.streams.values())) == len(seeding.STREAMS),
    }
    res['t3'] = {'plan_a': p_a.to_dict(), 'noise_master7': noise7, 'noise_master8': noise8}
    return checks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()

    run_dir = os.path.join(args.experiments_root, CAMPAIGN, args.run_id)
    for sub in ('config', 'metrics', 'log'):
        os.makedirs(os.path.join(run_dir, sub), exist_ok=True)

    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    res['tests']['t1_termination'] = t1_termination(res)
    res['tests']['t2_obs_version'] = t2_obs_version(res)
    res['tests']['t3_seeding'] = t3_seeding(res)

    flat = {('%s.%s' % (group, k)): v
            for group, checks in res['tests'].items() for k, v in checks.items()}
    res['flat'] = flat
    res['overall_pass'] = all(flat.values())

    res['git_commit'] = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                                       capture_output=True, text=True).stdout.strip()
    with open(os.path.join(run_dir, 'config', 'run_identity.json'), 'w', encoding='utf-8') as f:
        json.dump({'campaign': CAMPAIGN, 'run_id': args.run_id, 'git_commit': res['git_commit'],
                   'checker': 'tests/w7_core_check.py'}, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'metrics', 'core.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)

    for k in sorted(flat):
        print('%-52s %s' % (k, 'PASS' if flat[k] else 'FAIL'))
    print('\n%s -> %s' % ('PASS' if res['overall_pass'] else 'FAIL',
                          os.path.join(run_dir, 'metrics', 'core.json')))
    return 0 if res['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
