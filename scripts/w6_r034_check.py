#!/usr/bin/env python3
"""w6_r034_check —— R-034（生命周期可重复资源状态机与完整失效清理）自包含核查。

**起过 run_w6_r034.sh 后**读 `experiments/w6-task-eval/<run_id>/r034/bench.json`（Phase A **实跑台架**证据）
+ 源码 / 配置 / ADR 静态核查，逐项判定 (a)…(i)；
输出 `.../<run_id>/r034/check.json`（含 `overall_pass` 布尔）与 `.../<run_id>/metrics/r034.json`。
overall_pass = (a)∧…∧(i)。
"""

import argparse
import glob
import json
import os
import re
import subprocess
import sys

ROOT = '/home/chen/AGC'


def _load(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _read(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


def _code_only(txt):
    """剥离整行注释，仅保留可执行行（避免注释中提及 `rm -rf` 被误判）。"""
    return '\n'.join(s for s in (ln.strip() for ln in txt.splitlines()) if not s.startswith('#'))


def _scenario(bench, name):
    s = (bench or {}).get('scenarios', {}).get(name, {})
    res = dict(s)
    res['present'] = bool(s)
    return res


def case_a_cycle(bench):
    s = _scenario(bench, 'cycle_repeatable')
    return s


def case_b_shutdown(bench):
    return _scenario(bench, 'direct_shutdown')


def case_c_error(bench):
    return _scenario(bench, 'error_injection')


def case_d_manager_node_death(bench):
    return _scenario(bench, 'manager_node_death')


def case_e_manager_death(bench):
    return _scenario(bench, 'manager_death')


def case_f_base_state_machine():
    """基类：幂等 declare + shutdown/error 转发 + bond + 失效停机。"""
    t = _read(os.path.join(ROOT, 'src/agc_air/include/agc_air/agc_lifecycle_node.hpp')) or ''
    res = {
        'has_idempotent_declare': ('declare_param_idempotent' in t) and ('has_parameter' in t),
        'has_shutdown_forward': ('on_shutdown_impl' in t) and ('on_shutdown(const' in t),
        'has_error_forward': ('on_error_impl' in t) and ('on_error(const' in t),
        'has_bond': ('bondcpp/bond.hpp' in t) and ('createBond' in t and 'destroyBond' in t),
        'has_bond_watchdog': ('bond_watchdog_timer_' in t) and ('isBroken' in t),
        'has_failstop': ('失效停机' in t) and ('TRANSITION_DEACTIVATE' in t and 'TRANSITION_CLEANUP' in t),
    }
    res['pass'] = all(res.values())
    return res


def case_g_nodes_idempotent():
    """被管空中节点均使用幂等声明（无裸 `declare_parameter<`）。"""
    files = ['trajectory_tracker.cpp', 'takeoff_controller.cpp', 'standoff_follower.cpp',
             'px4_adapter.cpp']
    res = {}
    ok = True
    for f in files:
        t = _read(os.path.join(ROOT, 'src/agc_air/src', f)) or ''
        has_bare = 'declare_parameter<' in t
        has_idem = 'declare_param_idempotent<' in t
        res[f] = {'uses_idempotent': has_idem, 'bare_declare_present': has_bare}
        ok = ok and has_idem and (not has_bare)
    res['pass'] = ok
    return res


def case_h_launch_bond_timeout():
    """bond_timeout 不无条件置 0：w6.launch.py(backend:=full) + air_bringup 均为正（启用兼容失联检测）。"""
    res = {}
    for rel, key in (('src/agc_task/launch/w6.launch.py', 'w6'),
                     ('src/agc_air/launch/air_bringup.launch.py', 'air_bringup')):
        t = _read(os.path.join(ROOT, rel)) or ''
        code = _code_only(t)
        zero = bool(re.search(r"bond_timeout'?\s*:\s*0(\.0+)?\b", code))
        pos = bool(re.search(r"bond_timeout'?\s*:\s*[1-9]", code))
        res[key] = {'bond_timeout_zero': zero, 'bond_timeout_positive': pos}
    res['pass'] = all((not v['bond_timeout_zero']) and v['bond_timeout_positive']
                      for v in res.values() if isinstance(v, dict))
    return res


def case_i_scripts_hygiene():
    """无 `rm -rf`、无裸名 `pkill`；bench 用 killpg + wait。"""
    bench_txt = _read(os.path.join(ROOT, 'scripts/w6_r034_bench.py')) or ''
    sh_txt = _read(os.path.join(ROOT, 'scripts/run_w6_r034.sh')) or ''
    bench_code = _code_only(bench_txt)
    sh_code = _code_only(sh_txt)
    res = {
        'bench_no_rm_rf': 'rm -rf' not in bench_code,
        'sh_no_rm_rf': 'rm -rf' not in sh_code,
        'bench_scoped_reap': ('killpg' in bench_code) and ('proc.wait(' in bench_code),
        'sh_uses_pgrep_pathkills': ('pgrep -f' in sh_code) and ('kill -9' in sh_code),
    }
    bare = re.findall(r'pkill\s+-\w+\s+-f\s+"(?!\[)[^"]+"', bench_code + '\n' + sh_code)
    res['bare_pkill'] = bare
    res['pass'] = bool(res['bench_no_rm_rf'] and res['sh_no_rm_rf'] and
                       res['bench_scoped_reap'] and res['sh_uses_pgrep_pathkills'] and not bare)
    return res


def case_j_adr():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0049-*.md')))
    res = {'path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = ('## Status' in t and 'accepted' in t)
        res['no_section_ref'] = '\u00a7' not in t
        res['has_r034_ref'] = 'r-034' in t.lower()
        res['has_idempotent'] = '幂等' in t
        res['has_bond'] = 'bond' in t.lower()
        res['has_failstop'] = '失效停机' in t
        res['has_evidence_section'] = '取证' in t
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('no_section_ref') and res.get('has_r034_ref') and
                       res.get('has_idempotent') and res.get('has_bond') and
                       res.get('has_failstop') and res.get('has_evidence_section'))
    return res


def case_k_no_abi_no_second_commander():
    """冻结 9 消息 ABI 未改（src/agc_msgs 无改动）；setpoint 发布者仍单一（offboard_interface）。"""
    p = subprocess.run(['git', 'status', '--porcelain', 'src/agc_msgs'],
                       cwd=ROOT, capture_output=True, text=True)
    abi_clean = (p.stdout.strip() == '')
    # setpoint 发布器只应存在于 offboard_interface.cpp（唯一执行口语义不变）
    hits = []
    for root, _dirs, files in os.walk(os.path.join(ROOT, 'src')):
        for fn in files:
            if fn.endswith('.cpp'):
                t = _read(os.path.join(root, fn)) or ''
                if 'create_publisher<px4_msgs::msg::TrajectorySetpoint>' in t:
                    hits.append(fn)
    res = {'agc_msgs_porcelain': p.stdout.strip(), 'abi_clean': abi_clean,
           'setpoint_publisher_files': hits,
           'single_setpoint_publisher': hits == ['offboard_interface.cpp']}
    res['pass'] = bool(abi_clean and res['single_setpoint_publisher'])
    return res


def case_l_scope_note(bench):
    """范围如实标注：非整链；unsupported 列表存在（本批无 unsupported 则显式登记为空）。"""
    note = (bench or {}).get('scope_note', '')
    uns = (bench or {}).get('unsupported', None)
    res = {
        'mentions_not_full_chain': ('非整链' in note),
        'mentions_logic_bench': ('逻辑台架' in note or '台架' in note),
        'unsupported_is_list': isinstance(uns, list),
    }
    res['pass'] = bool(res['mentions_not_full_chain'] and res['mentions_logic_bench'] and
                       res['unsupported_is_list'])
    return res


def dump(res, run_dir):
    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(run_dir, 'r034'), exist_ok=True)
    with open(os.path.join(run_dir, 'r034', 'check.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'metrics', 'r034.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-28s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL',
                          os.path.join(run_dir, 'r034', 'check.json')))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261017-w6-r034-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    bench = _load(os.path.join(run_dir, 'r034', 'bench.json'))
    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        res['tests']['a_cycle_repeatable'] = case_a_cycle(bench)
        res['tests']['b_direct_shutdown'] = case_b_shutdown(bench)
        res['tests']['c_error_injection'] = case_c_error(bench)
        res['tests']['d_manager_node_death'] = case_d_manager_node_death(bench)
        res['tests']['e_manager_death'] = case_e_manager_death(bench)
        res['tests']['f_base_state_machine'] = case_f_base_state_machine()
        res['tests']['g_nodes_idempotent'] = case_g_nodes_idempotent()
        res['tests']['h_launch_bond_timeout'] = case_h_launch_bond_timeout()
        res['tests']['i_scripts_hygiene'] = case_i_scripts_hygiene()
        res['tests']['j_adr_0049'] = case_j_adr()
        res['tests']['k_no_abi_no_second_commander'] = case_k_no_abi_no_second_commander()
        res['tests']['l_scope_note'] = case_l_scope_note(bench)
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
