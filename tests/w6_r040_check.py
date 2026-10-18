#!/usr/bin/env python3
"""w6_r040_check —— R-040（地图内容身份与单调更新序号）自包含核查。

**起过 run_w6_r040.sh 后**读 `experiments/w6-task-eval/<run_id>/r040/bench.json`（真实节点台架）
+ 源码 / ADR 静态核查，逐项判定；输出 `.../<run_id>/r040/check.json`（含 `overall_pass`）与
`.../<run_id>/metrics/r040.json`。overall_pass = (a)∧…∧(j)。

核查口径：
  ① 内容身份**含几何元数据**——用**从 `/agc/map` 消息自身**（frame/res/width/height/origin）
     重算期望内容 hash，须与发布的 `version_hash` 一致（证明 hash 确实折入这些字段）；
  ② 同字节不同 origin → `version_hash` 不同、data-only hash 相同；
  ③ 跨边界 → raster 变化且 `update_seq` 递增（不欠包）；
  ④ 序号单调（同内容不变、实质变化严格 +1）；
  ⑤ R-006 语义保持（valid 行为 + 三态存在 + info↔data 长度一致）。
"""

import argparse
import glob
import json
import os
import re
import struct
import subprocess
import sys

ROOT = '/home/chen/AGC'


def _read(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


def _load(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _code_only(txt):
    return '\n'.join(
        s for s in (ln.strip() for ln in (txt or '').splitlines())
        if not (s.startswith('#') or s.startswith('//')))


def fnv(blob):
    h = 1469598103934665603
    for b in blob:
        h ^= b
        h = (h * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return '%016x' % h


def recompute_content_hash(s):
    """从**消息自身**字段 + 源 / 场景版本重算内容身份 hash（须与 C++ contentIdentityHash 逐字节一致）。

    组装顺序：frame\\0, source\\0, scenario_version\\0, resolution(<d), width(<Q), height(<Q),
    origin_x(<d), origin_y(<d), data(字节)。double/uint64 按 little-endian（x86_64）。
    """
    blob = bytearray()
    blob += s['frame'].encode() + b'\x00'
    blob += s['source'].encode() + b'\x00'
    blob += s['scenario_version'].encode() + b'\x00'
    blob += struct.pack('<d', float(s['res']))
    blob += struct.pack('<Q', int(s['w']))
    blob += struct.pack('<Q', int(s['h']))
    blob += struct.pack('<d', float(s['ox']))
    blob += struct.pack('<d', float(s['oy']))
    blob += bytes((v & 0xff) for v in s['data'])
    return fnv(blob)


def _all_contents(bench):
    """收集所有内容快照 (标签, snapshot)。"""
    out = []
    sc = (bench or {}).get('scenarios', {})
    if sc.get('same_bytes_diff_origin', {}).get('snapshot'):
        out.append(('sbdo_A', sc['same_bytes_diff_origin']['snapshot']))
    if sc.get('same_bytes_diff_origin_B', {}).get('snapshot'):
        out.append(('sbdo_B', sc['same_bytes_diff_origin_B']['snapshot']))
    for k in ('content_before', 'content_after'):
        if sc.get('cross_boundary', {}).get(k):
            out.append(('cross_' + k, sc['cross_boundary'][k]))
    if sc.get('monotonic_seq', {}).get('content_last'):
        out.append(('mono_last', sc['monotonic_seq']['content_last']))
    if sc.get('r006_config_valid', {}).get('content'):
        out.append(('cfg', sc['r006_config_valid']['content']))
    if sc.get('r006_air_silent', {}).get('content'):
        out.append(('air', sc['r006_air_silent']['content']))
    return out


# ------------------------------------------------------------------ 判定用例

def case_a_content_identity_includes_geometry(bench):
    """(a) 内容身份 hash 折入 frame/res/width/height/origin：从消息自身重算须与发布版一致。"""
    rows = []
    ok = True
    for label, s in _all_contents(bench):
        exp = recompute_content_hash(s)
        match = (exp == s['published_hash'])
        rows.append({'label': label, 'expected': exp, 'published': s['published_hash'],
                     'match': match, 'frame': s['frame'], 'res': s['res'],
                     'w': s['w'], 'h': s['h'], 'ox': s['ox'], 'oy': s['oy']})
        ok = ok and match
    res = {'rows': rows, 'all_match': ok, 'content_count': len(rows)}
    res['pass'] = bool(rows and ok)
    return res


def case_b_hash_vs_seq_separate(bench):
    """(b) 内容 hash 与单调更新序号**分离**：两话题各自独立承载。"""
    sc = (bench or {}).get('scenarios', {})
    cf = sc.get('cross_boundary', {})
    res = {
        'version_hash_topic_present': cf.get('hash_before') is not None,
        'update_seq_topic_present': cf.get('seq_after') is not None,
        'hash_and_seq_both_recorded': (cf.get('hash_before') is not None and
                                       cf.get('seq_after') is not None and
                                       cf.get('hash_before') != cf.get('seq_after')),
        'mono_seq_values_present': bool(sc.get('monotonic_seq', {}).get('seq_values')),
        'mono_hash_values_present': bool(sc.get('monotonic_seq', {}).get('hash_values')),
    }
    src = _read(os.path.join(ROOT, 'src/agc_perception/src/map_adapter.cpp')) or ''
    code = _code_only(src)
    res['source_seq_publisher'] = ('create_publisher<std_msgs::msg::UInt64>' in code and
                                   'seq_pub_' in code and 'u_int64.hpp' in src)
    res['source_seq_increments_on_rebuild'] = ('++update_seq_' in code)
    res['source_hash_topic_preserved'] = ('"/agc/map/version_hash"' in code)
    res['pass'] = all(v for v in res.values() if isinstance(v, bool))
    return res


def case_c_same_bytes_diff_origin(bench):
    """(c) 同 data 字节不同 origin → version_hash 不同、data-only hash 相同。"""
    sc = (bench or {}).get('scenarios', {})
    cmp_ = sc.get('same_bytes_diff_origin', {}).get('comparison', {})
    sa = sc.get('same_bytes_diff_origin', {}).get('snapshot')
    sb = sc.get('same_bytes_diff_origin_B', {}).get('snapshot')
    res = {
        'data_only_hash_A': cmp_.get('data_only_hash_A'),
        'data_only_hash_B': cmp_.get('data_only_hash_B'),
        'version_hash_A': cmp_.get('version_hash_A'),
        'version_hash_B': cmp_.get('version_hash_B'),
        'origin_x_A': cmp_.get('origin_x_A'), 'origin_x_B': cmp_.get('origin_x_B'),
        'data_only_equal': bool(cmp_.get('data_only_equal')),
        'version_hash_differs': bool(cmp_.get('version_hash_differs')),
        'origin_differs': bool(cmp_.get('origin_differs')),
    }
    # 直接自证：字节序列相等（同长度、同内容），但身份 hash 不同
    res['bytes_identical_checked'] = bool(
        sa and sb and sa['data'] == sb['data'] and sa['published_data_hash'] == sb['published_data_hash'])
    res['pass'] = bool(res['data_only_equal'] and res['version_hash_differs'] and
                       res['origin_differs'] and res['bytes_identical_checked'])
    return res


def case_d_cross_boundary_not_missed(bench):
    """(d) 同分箱跨边界：真实 raster 变化且 update_seq 递增（不欠包）。"""
    cf = (bench or {}).get('scenarios', {}).get('cross_boundary', {})
    res = {
        'raster_changed': bool(cf.get('raster_changed')),
        'seq_before': cf.get('seq_before'), 'seq_after': cf.get('seq_after'),
        'seq_incremented': bool(cf.get('seq_incremented')),
        'bin_before': cf.get('bin_before'), 'bin_after': cf.get('bin_after'),
        'same_bin_old_impl_would_miss': bool(cf.get('same_bin_old_impl_would_miss')),
        'hash_changed': cf.get('hash_before') != cf.get('hash_after'),
    }
    src = _read(os.path.join(ROOT, 'src/agc_perception/src/map_adapter.cpp')) or ''
    code = _code_only(src)
    res['source_raster_compare'] = ('data == grid_.data' in code)
    res['source_removed_binning'] = ('sameObstacles' not in code)
    res['pass'] = bool(res['raster_changed'] and res['seq_incremented'] and
                       res['same_bin_old_impl_would_miss'] and res['hash_changed'] and
                       res['source_raster_compare'] and res['source_removed_binning'])
    return res


def case_e_monotonic_seq(bench):
    """(e) 序号单调：同内容不变；移动 / 过期严格 +1 且 hash 变化。"""
    mo = (bench or {}).get('scenarios', {}).get('monotonic_seq', {})
    vals = mo.get('seq_values', [])
    non_decreasing = all(vals[i] <= vals[i + 1] for i in range(len(vals) - 1)) if vals else False
    res = {
        'seq_same_content_stable': bool(mo.get('seq_same_content_stable')),
        'seq_after_first': mo.get('seq_after_first'), 'seq_after_repeat': mo.get('seq_after_repeat'),
        'seq_after_move': mo.get('seq_after_move'), 'seq_after_expire': mo.get('seq_after_expire'),
        'move_strictly_increased': bool(mo.get('move_strictly_increased')),
        'expire_strictly_increased': bool(mo.get('expire_strictly_increased')),
        'unknown_seen_after_expire': bool(mo.get('unknown_seen_after_expire')),
        'seq_non_decreasing_overall': non_decreasing,
        'same_content_window_distinct': mo.get('same_content_window_distinct'),
    }
    res['pass'] = bool(res['seq_same_content_stable'] and res['move_strictly_increased'] and
                       res['expire_strictly_increased'] and res['unknown_seen_after_expire'] and
                       res['seq_non_decreasing_overall'])
    return res


def case_f_r006_preserved(bench):
    """(f) 保持 R-006：valid 行为 + 三态存在 + info↔data 长度一致。"""
    sc = (bench or {}).get('scenarios', {})
    cfg = sc.get('r006_config_valid', {})
    air = sc.get('r006_air_silent', {})
    mo = sc.get('monotonic_seq', {})
    contents = _all_contents(bench)
    len_ok = all(s['data_len'] == s['w'] * s['h'] for _l, s in contents) if contents else False
    res = {
        'config_valid_true': bool(cfg.get('valid_seen_true')),
        'config_has_occupied_100': bool(cfg.get('has100')),
        'air_silent_valid_false': bool(air.get('valid_seen_false')),
        'unknown_minus1_present': bool(mo.get('unknown_seen_after_expire')),
        'info_data_len_consistent': len_ok,
    }
    src = _read(os.path.join(ROOT, 'src/agc_perception/src/map_adapter.cpp')) or ''
    code = _code_only(src)
    res['source_valid_semantics_kept'] = ('geometry_ok_ && knowledge_current()' in code)
    res['source_static_prior_kept'] = ('static_prior_' in code and 'loadStaticObstacles' in code)
    res['pass'] = all(v for v in res.values() if isinstance(v, bool))
    return res


def case_g_adr_0055():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0055-*.md')))
    res = {'path': os.path.relpath(matches[0], ROOT) if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = ('## Status' in t and 'accepted' in t)
        res['has_evidence_section'] = '取证' in t
        res['has_r040_ref'] = 'r-040' in t.lower()
        res['no_section_ref'] = not bool(re.search(r'§\s*[0-9]', t))
        res['has_contract_ref'] = 'hard-constraints' in t
        res['has_boundary_note'] = ('冻结' in t) and ('唯一执行口语义' in t) and ('第二 ROS 命令者' in t)
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('has_evidence_section') and res.get('has_r040_ref') and
                       res.get('no_section_ref') and res.get('has_contract_ref') and
                       res.get('has_boundary_note'))
    return res


def case_h_abi_single_commander_section():
    p = subprocess.run(['git', 'status', '--porcelain', 'src/agc_msgs'],
                       cwd=ROOT, capture_output=True, text=True)
    abi_clean = (p.stdout.strip() == '')
    hits = []
    for root, _dirs, files in os.walk(os.path.join(ROOT, 'src')):
        for fn in files:
            if fn.endswith('.cpp'):
                t = _read(os.path.join(root, fn)) or ''
                if 'create_publisher<px4_msgs::msg::TrajectorySetpoint>' in t:
                    hits.append(fn)
    sec = []
    for base in (os.path.join(ROOT, 'docs'), os.path.join(ROOT, '.trae', 'specs')):
        for root, _dirs, files in os.walk(base):
            for fn in files:
                if not fn.endswith('.md'):
                    continue
                t = _read(os.path.join(root, fn)) or ''
                if re.search(r'§\s*[0-9]', t):
                    sec.append(os.path.relpath(os.path.join(root, fn), ROOT))
    res = {'agc_msgs_porcelain': p.stdout.strip(), 'abi_clean': abi_clean,
           'setpoint_publisher_files': hits,
           'single_setpoint_publisher': hits == ['offboard_interface.cpp'],
           'section_number_refs': sec, 'no_section_refs': (len(sec) == 0)}
    res['pass'] = bool(abi_clean and res['single_setpoint_publisher'] and res['no_section_refs'])
    return res


def case_i_script_hygiene():
    sh = _code_only(_read(os.path.join(ROOT, 'scripts/archive/run_w6_r040.sh')) or '')
    bch = _code_only(_read(os.path.join(ROOT, 'scripts/archive/w6_r040_bench.py')) or '')
    res = {
        'sh_no_rm_rf': 'rm -rf' not in sh,
        'bench_no_rm_rf': 'rm -rf' not in bch,
        'sh_uses_pgrep_pathkills': ('pgrep -f' in sh) and ('kill -9' in sh),
        'bench_scoped_reap': ('killpg' in bch) and ('proc.wait(' in bch),
        'sh_has_trap': 'trap' in sh,
        'sh_discovery_localhost': 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' in sh,
    }
    bare = re.findall(r'pkill\s+-\w+\s+-f\s+"(?!\[)[^"]+"', sh + '\n' + bch)
    res['bare_pkill'] = bare
    res['pass'] = bool(res['sh_no_rm_rf'] and res['bench_no_rm_rf'] and
                       res['sh_uses_pgrep_pathkills'] and res['bench_scoped_reap'] and
                       res['sh_has_trap'] and res['sh_discovery_localhost'] and not bare)
    return res


def case_j_scope_note(bench):
    note = (bench or {}).get('scope_note', '')
    uns = (bench or {}).get('unsupported', None)
    res = {
        'mentions_not_full_chain': ('非整链' in note),
        'mentions_bench': ('台架' in note),
        'unsupported_is_list': isinstance(uns, list),
    }
    res['pass'] = all(res.values())
    return res


def dump(res, run_dir):
    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(run_dir, 'r040'), exist_ok=True)
    with open(os.path.join(run_dir, 'r040', 'check.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'metrics', 'r040.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-34s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL',
                          os.path.join(run_dir, 'r040', 'check.json')))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261018-w6-r040-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    bench = _load(os.path.join(run_dir, 'r040', 'bench.json'))
    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        res['tests']['a_content_identity_includes_geometry'] = case_a_content_identity_includes_geometry(bench)
        res['tests']['b_hash_vs_seq_separate'] = case_b_hash_vs_seq_separate(bench)
        res['tests']['c_same_bytes_diff_origin'] = case_c_same_bytes_diff_origin(bench)
        res['tests']['d_cross_boundary_not_missed'] = case_d_cross_boundary_not_missed(bench)
        res['tests']['e_monotonic_seq'] = case_e_monotonic_seq(bench)
        res['tests']['f_r006_preserved'] = case_f_r006_preserved(bench)
        res['tests']['g_adr_0055'] = case_g_adr_0055()
        res['tests']['h_abi_single_commander_section'] = case_h_abi_single_commander_section()
        res['tests']['i_script_hygiene'] = case_i_script_hygiene()
        res['tests']['j_scope_note'] = case_j_scope_note(bench)
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
