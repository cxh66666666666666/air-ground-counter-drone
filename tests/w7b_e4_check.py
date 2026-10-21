#!/usr/bin/env python3
"""w7b_e4_check —— W7-b T4（E4 / F-04）验收：obs 版本化（v2 可区分 + v1 可回放）。

判据（[ADR-0070](docs/adr/0070-w7b-research-propositions-and-rl-role-adjudication.md)(d) / [F-04]）：
  * **v1 可回放**：`obs_spec.build_obs_vector(1, ...)` 与冻结真源 `interface.build_observation(...)`
    **逐位一致**（旧权重不静默失效）；
  * **v2 维度** = v1 + 4（目标 bearing sin/cos / 相对速度 / 观测年龄）；
  * v2 追加值按归一公式正确（bearing sin/cos、rel/rel_norm 夹 1、age/age_norm 夹 1）；
  * **两版本 checkpoint 元数据可区分**：`check_compatible` 对版本不同返回 False + 原因；
    元数据指纹不同。

用法：
  python3 tests/w7b_e4_check.py
"""

import json
import os
import sys

ROOT = '/home/chen/AGC'
sys.path.insert(0, os.path.join(ROOT, 'src', 'agc_rl'))

from agc_rl import interface, obs_spec  # noqa: E402


def main():
    out_dir = os.path.join(ROOT, 'experiments', 'w7-rl-b',
                           os.environ.get('W7B_E4_RUN_ID', 'e4-obs-versioning'))
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)

    ns = 12
    args = dict(sectors=[1.0] * ns, pose_x=1.5, pose_y=-0.5, dist_to_goal=3.0,
                last_action=2, num_sectors=ns, pose_norm_m=10.0, track_norm_m=5.0)
    v1 = obs_spec.build_obs_vector(1, **args)
    v1_ref = interface.build_observation(**args)
    v2 = obs_spec.build_obs_vector(2, target_bearing_rad=1.5707963267948966,
                                   rel_speed_mps=1.0, obs_age_s=0.25,
                                   rel_speed_norm_mps=2.0, obs_age_norm_s=0.5, **args)

    spec1 = obs_spec.get_obs_spec(1, ns)
    spec2 = obs_spec.get_obs_spec(2, ns)
    meta1 = obs_spec.build_checkpoint_meta(num_sectors=ns, obs_version=1)
    meta2 = obs_spec.build_checkpoint_meta(num_sectors=ns, obs_version=2)
    ok_back, reasons_back = obs_spec.check_compatible(meta1, current_version=1)
    ok_fwd, reasons_fwd = obs_spec.check_compatible(meta1, current_version=2)

    tail = v2[len(v1):]
    checks = {
        'v1_bitwise_replay_matches_frozen_interface': (v1 == v1_ref),
        'v1_dim_is_interface_dim': (spec1.dim == interface.obs_dim(ns)),
        'v2_dim_is_v1_plus_4': (spec2.dim == spec1.dim + 4),
        'v2_fields_are_extra_v2': (spec2.fields[len(spec1.fields):] == obs_spec.EXTRA_FIELDS_V2),
        'v2_bearing_sin_cos_correct': (abs(tail[0] - 1.0) < 1e-9 and abs(tail[1]) < 1e-9),
        'v2_rel_speed_normalized': (abs(tail[2] - 0.5) < 1e-9),
        'v2_obs_age_normalized': (abs(tail[3] - 0.5) < 1e-9),
        'same_version_compatible': bool(ok_back),
        'cross_version_distinguishable': (not ok_fwd) and bool(reasons_fwd),
        'fingerprints_differ': (obs_spec.checkpoint_fingerprint(meta1)
                                != obs_spec.checkpoint_fingerprint(meta2)),
        'default_obs_version_is_2': (obs_spec.OBS_VERSION == 2),
    }
    result = {
        'campaign': 'w7-rl-b', 'evidence': 'E4',
        'checks': checks, 'overall_pass': all(checks.values()),
        'detail': {'v1_dim': spec1.dim, 'v2_dim': spec2.dim,
                   'cross_version_reasons': reasons_fwd,
                   'v2_tail': tail,
                   'meta_fingerprints': {
                       'v1': obs_spec.checkpoint_fingerprint(meta1),
                       'v2': obs_spec.checkpoint_fingerprint(meta2)}},
        'proposition': 'E4 / F-04：obs_version 版本化——v2 可区分（v1+4 维），v1 逐位可回放。',
    }
    with open(os.path.join(out_dir, 'metrics', 'e4.json'), 'w') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    for k, v in checks.items():
        print('  %-46s %s' % (k, 'PASS' if v else 'FAIL'))
    print('\n%s -> %s' % ('PASS' if result['overall_pass'] else 'FAIL',
                          os.path.join(out_dir, 'metrics', 'e4.json')))
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
