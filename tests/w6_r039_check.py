#!/usr/bin/env python3
"""w6_r039_check —— R-039（地面 odom 源时刻分离 + 复位 epoch 速度历史）自包含核查。

**起过 run_w6_r039.sh 后**读 `experiments/w6-task-eval/<run_id>/r039/bench.json`（真实节点台架）
+ 源码 / ADR 静态核查，逐项判定 (a)…(k)；输出 `.../<run_id>/r039/check.json`（含 `overall_pass`）
与 `.../<run_id>/metrics/r039.json`。overall_pass = (a)∧…∧(k)。

核查口径（**不**只看发布率 / `valid=true` 比例）：以**同一时间轴**上「真值 vs odom」记录判定。
"""

import argparse
import glob
import json
import os
import re
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
    """剥离整行注释（`#` / `//`），仅保留可执行行（避免注释中提及 tokens 被误判）。"""
    return '\n'.join(
        s for s in (ln.strip() for ln in txt.splitlines())
        if not (s.startswith('#') or s.startswith('//')))


# ------------------------------------------------------------------ 判定用例

def case_a_stamp_is_source_time(bench):
    """(a) 发布 Header.stamp == 源采样时刻（**不** now() 冒充测量时刻）。"""
    res = {'per_scenario': {}}
    ok = True
    for name, sc in (bench or {}).get('scenarios', {}).items():
        rec = sc.get('records', {})
        gt_stamps = [g['stamp'] for g in rec.get('gt', [])]
        bad = 0
        for o in rec.get('odom', []):
            if not any(abs(o['stamp'] - s) < 1e-9 for s in gt_stamps):
                bad += 1
        res['per_scenario'][name] = {'odom_count': len(rec.get('odom', [])),
                                     'unmatched_stamps': bad}
        ok = ok and bad == 0
    res['all_odom_stamp_match_source'] = ok
    src = _read(os.path.join(ROOT, 'src/agc_perception/src/ground_odom_adapter.cpp')) or ''
    code = _code_only(src)
    res['source_uses_src_stamp'] = ('o.header.stamp = stamp;' in code) and \
                                   ('const rclcpp::Time stamp = src_stamp_;' in code)
    res['source_no_now_as_measurement'] = ('= now();' not in code) and ('Time stamp = now()' not in src)
    res['publish_only_on_new_sample'] = ('src_stamp_ != last_pub_stamp_' in code)
    res['frames_preserved'] = ('base_frame_ = declare_parameter<std::string>("base_frame", "car/base_link")'
                               in src)
    res['pass'] = bool(res['all_odom_stamp_match_source'] and res['source_uses_src_stamp'] and
                       res['source_no_now_as_measurement'] and res['publish_only_on_new_sample'] and
                       res['frames_preserved'])
    return res


def case_b_negative_age_invalid(bench, ):
    """(b) 负年龄 / 时钟回拨 → 显式无效（不钳 0）。"""
    tj = (bench or {}).get('scenarios', {}).get('time_jump', {})
    src = _read(os.path.join(ROOT, 'src/agc_perception/src/ground_odom_adapter.cpp')) or ''
    code = _code_only(src)
    res = {
        'time_jump_pass': tj.get('pass'),
        'diag_has_clock_anomaly': tj.get('diag_has_clock_anomaly'),
        'min_reported_age_negative': (tj.get('min_reported_age') is not None and
                                      tj.get('min_reported_age') < -1.0),
        'valid_off': tj.get('valid_off_wall') is not None,
        'no_odom_published_in_jump_window': tj.get('odom_count_in_jump_window') == 0,
        'source_checks_negative_age': ('age_s < 0.0' in code) and ('clock_anomaly' in src),
        'source_removed_clamp_zero': ('std::max(0.0, now_s - last_rx_sim_)' not in code) and
                                     ('last_rx_sim_' not in code),
        'has_signed_age': 'signedAge' in code,
    }
    res['pass'] = bool(res['time_jump_pass'] and res['diag_has_clock_anomaly'] and
                       res['min_reported_age_negative'] and res['valid_off'] and
                       res['no_odom_published_in_jump_window'] and
                       res['source_checks_negative_age'] and res['source_removed_clamp_zero'] and
                       res['has_signed_age'])
    return res


def case_c_no_teleport_spike(bench):
    """(c) teleport 后速度历史按 reset epoch 清空 → 无速度尖峰。"""
    tp = (bench or {}).get('scenarios', {}).get('teleport', {})
    src = _read(os.path.join(ROOT, 'src/agc_perception/src/ground_odom_adapter.cpp')) or ''
    code = _code_only(src)
    res = {
        'teleport_pass': tp.get('pass'),
        'spike_free': tp.get('spike_free'),
        'max_speed_after_teleport': tp.get('max_speed_after_teleport'),
        'first_after_speed_zero': tp.get('first_after_speed_zero'),
        'max_abs_err_odom_vs_gt': tp.get('max_abs_err_odom_vs_gt'),
        'paired_with_truth': (tp.get('paired_count') or 0) > 0,
        'source_subscribes_reset_epoch': ('reset_epoch_topic' in code and
                                          'create_subscription<std_msgs::msg::Int32>' in code),
        'source_clears_history_on_epoch': ('onResetEpoch' in code) and ('history_.clear();' in code),
        'source_has_jump_guard': ('reset_jump_clear_m_' in code and '位置跳变' in src),
        'source_velocity_zero_until_2_samples': ('history_.size() < 2' in code),
    }
    res['pass'] = bool(res['teleport_pass'] and res['spike_free'] and
                       res['first_after_speed_zero'] and (res['max_abs_err_odom_vs_gt'] is not None) and
                       res['paired_with_truth'] and res['source_subscribes_reset_epoch'] and
                       res['source_clears_history_on_epoch'] and res['source_has_jump_guard'] and
                       res['source_velocity_zero_until_2_samples'])
    return res


def case_d_disconnect_invalid(bench):
    """(d) 断源：valid=false、年龄如实增大、无新样本不再发布。"""
    dc = (bench or {}).get('scenarios', {}).get('disconnect', {})
    res = {
        'disconnect_pass': dc.get('pass'),
        'valid_off_wall': dc.get('valid_off_wall'),
        'diag_has_stale': dc.get('diag_has_stale'),
        'odom_count_after_stop': dc.get('odom_count_after_stop'),
    }
    res['pass'] = bool(res['disconnect_pass'] and res['valid_off_wall'] is not None and
                       res['diag_has_stale'] and res['odom_count_after_stop'] == 0)
    return res


def case_e_stale_sample_preserved(bench):
    """(e) 旧样本：stamp 保持源时刻（不被 now() 重盖）、age 增大、valid 翻转。"""
    st = (bench or {}).get('scenarios', {}).get('stale_sample', {})
    res = {
        'stale_pass': st.get('pass'),
        'stamps_kept_source_time': st.get('stamps_kept_source_time'),
        'odom_stamp_near_now_count': st.get('odom_stamp_near_now_count'),
        'valid_off_wall': st.get('valid_off_wall'),
        'diag_has_stale': st.get('diag_has_stale'),
    }
    res['pass'] = bool(res['stale_pass'] and res['stamps_kept_source_time'] and
                       res['odom_stamp_near_now_count'] == 0 and
                       res['valid_off_wall'] is not None and res['diag_has_stale'])
    return res


def case_f_same_timeline_records(bench):
    """(f) 四场景均**同时间轴**记录真值 / odom 速度（非仅发布率 / valid 比例）。"""
    out = {}
    ok = True
    for name in ('disconnect', 'stale_sample', 'teleport', 'time_jump'):
        rec = (bench or {}).get('scenarios', {}).get(name, {}).get('records', {})
        has_gt = len(rec.get('gt', [])) > 0
        has_odom_or_expected = len(rec.get('odom', [])) > 0 or name == 'time_jump'
        # 每条 odom 均带 wall（时间轴）以便与真值同轴比对
        odom_with_wall = all('wall' in o for o in rec.get('odom', []))
        out[name] = {'gt_records': len(rec.get('gt', [])), 'odom_records': len(rec.get('odom', [])),
                     'odom_with_wall': odom_with_wall, 'has_gt': has_gt}
        ok = ok and has_gt and has_odom_or_expected and odom_with_wall
    out['teleport_paired_truth'] = ((bench or {}).get('scenarios', {}).get('teleport', {})
                                    .get('paired_count', 0) > 0)
    out['pass'] = bool(ok and out['teleport_paired_truth'])
    return out


def case_g_diag_topic_source():
    """(g) 诊断话题（latched std_msgs/String）承载 state / age / epoch，显式可诊断。"""
    src = _read(os.path.join(ROOT, 'src/agc_perception/src/ground_odom_adapter.cpp')) or ''
    res = {
        'has_diag_publisher': 'diag_pub_' in src and 'std_msgs::msg::String' in src,
        'diag_latched': 'transient_local' in src,
        'diag_state_tokens': all(t in src for t in
                                 ('clock_anomaly', 'stale', 'no_sample', 'reset_rebuild')),
        'diag_reports_age_epoch': ('state=' in src and 'age=' in src and 'epoch=' in src),
        'param_registered_in_config': None,
    }
    cfg = _read(os.path.join(ROOT, 'src/agc_perception/config/perception.yaml')) or ''
    res['param_registered_in_config'] = all(
        k in cfg for k in ('diag_topic:', 'reset_epoch_topic:', 'reset_jump_clear_m:'))
    res['pass'] = all(v for k, v in res.items() if k != 'param_registered_in_config') and \
        res['param_registered_in_config']
    return res


def case_h_abi_single_commander_section():
    """(h) 冻结 9 消息 ABI 未改；唯一 setpoint 命令者；跨文档无 `§ 数字` 章节号。"""
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


def case_i_adr():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0054-*.md')))
    res = {'path': os.path.relpath(matches[0], ROOT) if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = ('## Status' in t and 'accepted' in t)
        res['has_evidence_section'] = '取证' in t
        res['has_r039_ref'] = 'r-039' in t.lower()
        res['no_section_ref'] = not bool(re.search(r'§\s*[0-9]', t))
        res['has_contract_ref'] = 'hard-constraints' in t
        res['has_boundary_note'] = ('冻结' in t) and ('唯一执行口语义' in t) and ('第二 ROS 命令者' in t)
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('has_evidence_section') and res.get('has_r039_ref') and
                       res.get('no_section_ref') and res.get('has_contract_ref') and
                       res.get('has_boundary_note'))
    return res


def case_j_script_hygiene():
    sh = _code_only(_read(os.path.join(ROOT, 'scripts/archive/run_w6_r039.sh')) or '')
    bch = _code_only(_read(os.path.join(ROOT, 'scripts/archive/w6_r039_bench.py')) or '')
    res = {
        'sh_no_rm_rf': 'rm -rf' not in sh,
        'bench_no_rm_rf': 'rm -rf' not in bch,
        'sh_uses_pgrep_pathkills': ('pgrep -f' in sh) and ('kill -9' in sh),
        'bench_scoped_reap': ('killpg' in bch) and ('proc.wait(' in bch),
        'sh_has_trap': 'trap' in sh,
    }
    bare = re.findall(r'pkill\s+-\w+\s+-f\s+"(?!\[)[^"]+"', sh + '\n' + bch)
    res['bare_pkill'] = bare
    res['pass'] = bool(res['sh_no_rm_rf'] and res['bench_no_rm_rf'] and
                       res['sh_uses_pgrep_pathkills'] and res['bench_scoped_reap'] and
                       res['sh_has_trap'] and not bare)
    return res


def case_k_scope_note(bench):
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
    os.makedirs(os.path.join(run_dir, 'r039'), exist_ok=True)
    with open(os.path.join(run_dir, 'r039', 'check.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'metrics', 'r039.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-30s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL',
                          os.path.join(run_dir, 'r039', 'check.json')))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261018-w6-r039-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    bench = _load(os.path.join(run_dir, 'r039', 'bench.json'))
    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        res['tests']['a_stamp_is_source_time'] = case_a_stamp_is_source_time(bench)
        res['tests']['b_negative_age_invalid'] = case_b_negative_age_invalid(bench)
        res['tests']['c_no_teleport_spike'] = case_c_no_teleport_spike(bench)
        res['tests']['d_disconnect_invalid'] = case_d_disconnect_invalid(bench)
        res['tests']['e_stale_sample_preserved'] = case_e_stale_sample_preserved(bench)
        res['tests']['f_same_timeline_records'] = case_f_same_timeline_records(bench)
        res['tests']['g_diag_topic_source'] = case_g_diag_topic_source()
        res['tests']['h_abi_single_commander_section'] = case_h_abi_single_commander_section()
        res['tests']['i_adr_0054'] = case_i_adr()
        res['tests']['j_script_hygiene'] = case_j_script_hygiene()
        res['tests']['k_scope_note'] = case_k_scope_note(bench)
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
