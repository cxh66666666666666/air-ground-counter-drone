#!/usr/bin/env python3
"""w6_r041_check —— R-041（目标失效策略与位移阈值化重规划）自包含核查。

**起过 run_w6_r041.sh 后**读 `experiments/w6-task-eval/<run_id>/r041/bench.json`（真实节点台架）
+ 源码 / ADR 静态核查，逐项判定；输出 `.../<run_id>/r041/check.json`（含 `overall_pass`）与
`.../<run_id>/metrics/r041.json`。overall_pass = (a)∧…∧(h)。

核查口径：
  ① 目标失效策略显式（有期限最后有效目标 → 超窗安全停车）且**传递有效性与身份**；
  ② goal 关联目标**身份与时刻**（stamp=源时刻，非 now），**不**以新时间戳固定点掩盖失联；
  ③ ground_baseline 重规划**位移阈值 / 地图 / 时限 / 周期**触发，**移除**任意浮点变化强制重规划；
  ④ 失效安全停车（valid=0 → cmd_vel_raw≈0）；
  ⑤ 口径 ADR / 契约边界 / 脚本卫生 / 范围标注。
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
    return '\n'.join(
        s for s in (ln.strip() for ln in (txt or '').splitlines())
        if not (s.startswith('#') or s.startswith('//')))


# ------------------------------------------------------------------ 判定用例

def case_a_target_expire_policy(bench):
    """(a) 目标失效策略显式：有期限最后有效目标 → 超窗 EXPIRED（不反射固定点、不刷新时间戳）。"""
    te = (bench or {}).get('scenarios', {}).get('target_expire', {})
    src = _code_only(_read(os.path.join(ROOT, 'src/agc_task/src/task_manager.cpp')) or '')
    res = {
        'bench_fresh_ok': bool(te.get('fresh_ok')),
        'bench_hold_ok': bool(te.get('hold_ok')),
        'bench_expired_seen': bool(te.get('expired_seen')),
        'bench_expired_count': te.get('expired_count'),
        'bench_no_goal_after_expiry': bool(te.get('no_new_stamped_goal_after_expiry')),
        'bench_goals_after_expiry': te.get('goals_after_expiry'),
        'bench_fixed_goal_hits': te.get('fixed_goal_hits'),
        'reasons_seen': te.get('reasons_seen'),
        'source_has_hold_valid_param': 'goal_hold_valid_s' in src,
        'source_has_hold_reason': '"HOLD_LAST_VALID"' in src,
        'source_has_expired_reason': '"EXPIRED"' in src,
        'source_has_target_lost_reason': '"TARGET_LOST"' in src,
    }
    res['pass'] = bool(res['bench_fresh_ok'] and res['bench_hold_ok'] and res['bench_expired_seen'] and
                       res['bench_no_goal_after_expiry'] and res['bench_fixed_goal_hits'] == 0 and
                       res['source_has_hold_valid_param'] and res['source_has_hold_reason'] and
                       res['source_has_expired_reason'] and res['source_has_target_lost_reason'])
    return res


def case_b_goal_identity_and_time(bench):
    """(b) goal 关联目标身份与时刻：stamp=源时刻（≠now）；身份/有效性经伴随诊断传递，不改 PoseStamped 类型。"""
    te = (bench or {}).get('scenarios', {}).get('target_expire', {})
    fd = te.get('fresh_detail') or {}
    src = _code_only(_read(os.path.join(ROOT, 'src/agc_task/src/task_manager.cpp')) or '')
    res = {
        'bench_fresh_src_stamp_ok': bool(fd.get('src_stamp_ok')),
        'bench_fresh_not_now': bool(fd.get('not_now')),
        'bench_fresh_pos_ok': bool(fd.get('pos_ok')),
        'source_goal_stamp_is_target_stamp': ('goal.header.stamp = target_stamp_' in src),
        'source_goal_info_pub': ('goal_info_pub_' in src and 'create_publisher<std_msgs::msg::String>' in src),
        'source_goal_info_format': ('valid=%d;reason=%s;target_id=%s;src_stamp=%.6f;age=%.6f' in src),
        'source_goal_topic_type_kept': ('create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic' in src),
    }
    res['pass'] = bool(res['bench_fresh_src_stamp_ok'] and res['bench_fresh_not_now'] and
                       res['bench_fresh_pos_ok'] and res['source_goal_stamp_is_target_stamp'] and
                       res['source_goal_info_pub'] and res['source_goal_info_format'] and
                       res['source_goal_topic_type_kept'])
    return res


def case_c_displacement_thresholded_replan(bench):
    """(c) 重规划位移阈值化：微噪声不触发；真实位移触发；移除任意浮点变化强制重规划。"""
    mn = (bench or {}).get('scenarios', {}).get('micro_noise_goal', {})
    src = _code_only(_read(os.path.join(ROOT, 'src/agc_ground/src/ground_baseline.cpp')) or '')
    res = {
        'bench_micro_replan_count': mn.get('micro_replan_count'),
        'bench_micro_expected_periodic_max': mn.get('micro_expected_periodic_max'),
        'bench_micro_goal_disp_logs': mn.get('micro_replan_goal_disp_logs'),
        'bench_micro_any_trigger_logs': mn.get('micro_any_trigger_logs'),
        'bench_disp_new_path_matches_goal': bool(mn.get('disp_new_path_matches_goal')),
        'bench_disp_goal_disp_logs': mn.get('disp_goal_disp_logs'),
        'bench_disp_replan_latency_s': mn.get('disp_replan_latency_s'),
        'bench_micro_rate_hz': mn.get('micro_replan_rate_hz'),
        'source_no_1e-6_force': ('hypot(gx - goal_x_, gy - goal_y_) > 1e-6' not in src and '> 1e-6' not in src),
        'source_first_goal_force': ('if (!have_goal_) {have_goal_ = true; force_replan_ = true;}' in src),
        'source_needreplan_disp_threshold': ('replan_disp_m_' in src and '重规划触发：目标身份变' in src),
        'source_needreplan_period': ('replan_period_s_' in src),
        'source_needreplan_maxage': ('max_path_age_s_' in src),
    }
    res['pass'] = bool(res['bench_micro_goal_disp_logs'] == 0 and
                       res['bench_micro_any_trigger_logs'] == 0 and
                       res['bench_micro_replan_count'] is not None and
                       res['bench_micro_replan_count'] <= (res['bench_micro_expected_periodic_max'] or 0) and
                       res['bench_disp_new_path_matches_goal'] and
                       res['source_no_1e-6_force'] and res['source_first_goal_force'] and
                       res['source_needreplan_disp_threshold'] and res['source_needreplan_period'] and
                       res['source_needreplan_maxage'])
    return res


def case_d_failsafe_stop(bench):
    """(d) 失效安全停车：valid=0 → cmd_vel_raw≈0；地面失效动作与 C10 空中 freeze 分别定义。"""
    fs = (bench or {}).get('scenarios', {}).get('failsafe_stop', {})
    src = _code_only(_read(os.path.join(ROOT, 'src/agc_ground/src/ground_baseline.cpp')) or '')
    src_raw = _read(os.path.join(ROOT, 'src/agc_ground/src/ground_baseline.cpp')) or ''
    res = {
        'bench_pre_valid_max_lin': fs.get('pre_valid_max_lin'),
        'bench_post_invalid_max_lin': fs.get('post_invalid_max_lin'),
        'bench_post_invalid_max_ang': fs.get('post_invalid_max_ang'),
        'bench_post_samples': fs.get('post_samples'),
        'source_goal_valid_gate': ('if (!goal_valid_)' in src),
        'source_parse_goal_info': ('parseGoalInfoValid' in src),
        'source_goal_info_sub': ('goal_info_topic_' in src and 'goal_info_sub_' in src),
        'source_c10_freeze_divergence_note': ('C10' in src_raw and 'freeze' in src_raw),
    }
    res['pass'] = bool(res['bench_pre_valid_max_lin'] is not None and res['bench_pre_valid_max_lin'] > 0.1 and
                       res['bench_post_invalid_max_lin'] is not None and res['bench_post_invalid_max_lin'] < 1e-6 and
                       res['bench_post_invalid_max_ang'] is not None and res['bench_post_invalid_max_ang'] < 1e-6 and
                       res['bench_post_samples'] and res['bench_post_samples'] > 0 and
                       res['source_goal_valid_gate'] and res['source_parse_goal_info'] and
                       res['source_goal_info_sub'] and res['source_c10_freeze_divergence_note'])
    return res


def case_e_adr_0056():
    """(e) 口径记录 ADR-0056：accepted + 取证节 + R-041 + 契约引用 + 边界 + C10 分野。"""
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0056-*.md')))
    res = {'path': os.path.relpath(matches[0], ROOT) if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = ('## Status' in t and 'accepted' in t)
        res['has_evidence_section'] = '取证' in t
        res['has_r041_ref'] = 'r-041' in t.lower()
        res['no_section_ref'] = not bool(re.search(r'§\s*[0-9]', t))
        res['has_contract_ref'] = 'hard-constraints' in t
        res['has_boundary_note'] = ('冻结' in t) and ('唯一执行口语义' in t) and ('第二 ROS 命令者' in t)
        res['has_c10_freeze_divergence'] = ('C10' in t) and ('freeze' in t)
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and res.get('has_evidence_section') and
                       res.get('has_r041_ref') and res.get('no_section_ref') and res.get('has_contract_ref') and
                       res.get('has_boundary_note') and res.get('has_c10_freeze_divergence'))
    return res


def case_f_abi_single_commander_section():
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


def case_g_script_hygiene():
    sh = _code_only(_read(os.path.join(ROOT, 'scripts/archive/run_w6_r041.sh')) or '')
    bch = _code_only(_read(os.path.join(ROOT, 'scripts/archive/w6_r041_bench.py')) or '')
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


def case_h_scope_note(bench):
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
    os.makedirs(os.path.join(run_dir, 'r041'), exist_ok=True)
    with open(os.path.join(run_dir, 'r041', 'check.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'metrics', 'r041.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-34s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL',
                          os.path.join(run_dir, 'r041', 'check.json')))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261018-w6-r041-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    bench = _load(os.path.join(run_dir, 'r041', 'bench.json'))
    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        res['tests']['a_target_expire_policy'] = case_a_target_expire_policy(bench)
        res['tests']['b_goal_identity_and_time'] = case_b_goal_identity_and_time(bench)
        res['tests']['c_displacement_thresholded_replan'] = case_c_displacement_thresholded_replan(bench)
        res['tests']['d_failsafe_stop'] = case_d_failsafe_stop(bench)
        res['tests']['e_adr_0056'] = case_e_adr_0056()
        res['tests']['f_abi_single_commander_section'] = case_f_abi_single_commander_section()
        res['tests']['g_script_hygiene'] = case_g_script_hygiene()
        res['tests']['h_scope_note'] = case_h_scope_note(bench)
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
