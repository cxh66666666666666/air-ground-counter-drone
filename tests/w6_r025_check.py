#!/usr/bin/env python3
"""w6_r025_check —— R-025（整链覆盖矩阵 + 同一 run 因果记录 + pass 命题化）自包含验收核查。

**不起 launch**：静态分析源文件（后端能力列）+ 读产物（轻量 report / 整链 same_run），构造覆盖矩阵，
逐轴标 covered / uncovered / not_applicable + evidence 出处 + 未覆盖的如实原因。**绝不**把无证据的轴标 covered。

校验：
  (a) 轻量 `expected_match` 已标 `lightweight`（且每命中 episode 有 `proposition`）；
  (b)/(b') 整链同一 run 含全部 5 问（`start_ok` / `own_takeover` / `target_motion` / `capture or jam` /
      `terminal`）且都取自真实观测（`full_chain_executed==true`；值可为 true/false）；
  (c) 每个 `pass` 都有命题（pass ↔ 命题强绑定）；
  (d) 未覆盖轴如实标注（无「无证据却称已覆盖」）。
  (e) 冻结 ABI 回归：`capture_hit` 字面量仍恰为 `CAPTURE_NET`；`git status --porcelain src/agc_msgs` 未改。

输出 `experiments/w6-task-eval/<run_id>/metrics/r025.json` 与 `<run_id>/coverage_matrix.md`。
`overall_pass` = (a) ∧ (c) ∧ (d) ∧ (b') ∧ (e)（(e) 为红线回归，并入以保 ABI 不变）。
"""

import argparse
import ast
import json
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path

ROOT = '/home/chen/AGC'
SELF = Path(__file__).resolve()

AXES = ['task_start', 'own_takeover', 'target_motion', 'real_terminal']


def _read(path):
    return Path(path).read_text(encoding='utf-8', errors='replace')


def _cell(status, value, evidence, reason=''):
    return {'status': status, 'value': value, 'evidence': evidence, 'reason': reason}


def _backend_table():
    """S-05：从共享模块 `w6_composition.py` 源文本解析 backend 能力表（BACKENDS）。

    两 backend 的能力事实（air_link / px4 / required_nodes）唯一来源即该能力表，供覆盖矩阵
    「后端能力」列派生（**不**再逐文件读旧入口）。
    """
    comp = _read(os.path.join(ROOT, 'src/agc_task/launch/w6_composition.py'))
    tree = ast.parse(comp)
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'BACKENDS' for t in node.targets):
            return ast.literal_eval(node.value)
    return {}


def topology_facts():
    """静态拓扑核验：读统一入口 `w6.launch.py` + 共享模块源文本，派生两 backend 能力事实。"""
    w6 = _read(os.path.join(ROOT, 'src/agc_task/launch/w6.launch.py'))
    air_cfg = _read(os.path.join(ROOT, 'src/agc_air/config/air.yaml'))
    takeoff = _read(os.path.join(ROOT, 'src/agc_air/src/takeoff_controller.cpp'))
    table = _backend_table()

    own_px4 = ("px4_bin, '-d', '-s'" in w6) or ("px4_bin, '-d'" in w6)
    intruder_px4 = ("'-i', '1'" in w6)
    agents = w6.count("agent_bin, 'udp4'")
    air_nodes = [n for n in ('px4_adapter', 'takeoff_controller', 'trajectory_tracker',
                             'standoff_follower') if n in w6]
    takeover_server = (("release_service: \"/agc/air/release_control\"" in air_cfg) and
                       ('create_service<std_srvs::srv::Trigger>' in takeoff) and
                       ('release_srv_' in takeoff))
    full_nodes = (table.get('full') or {}).get('required_nodes') or []
    light_nodes = (table.get('lightweight') or {}).get('required_nodes') or []
    full_facts = {
        'launch': 'w6.launch.py',
        'backend': 'full',
        'own_px4_instance0': bool(own_px4),
        'intruder_px4_instance1': bool(intruder_px4),
        'px4_instances': int(bool(own_px4)) + int(bool(intruder_px4)),
        'agents': int(agents),
        'air_nodes': sorted(air_nodes),
        'takeover_server': bool(takeover_server),
        'task_nodes': sorted([n for n in ('task_manager', 'scenario_reset', 'run_logger',
                                          'capture_adjudicator', 'jammer', 'intruder_driver')
                              if n in full_nodes]),
        'has_m0_skeleton': ('m0_skeleton' in w6) or ('world_bringup' in w6),
        'required_nodes': full_nodes,
    }
    light_facts = {
        'launch': 'w6.launch.py',
        'backend': 'lightweight',
        'includes_m0_skeleton': 'm0_skeleton.launch.py' in w6,
        'capture_adjudicator': 'capture_adjudicator' in w6,
        'jammer': 'jammer' in w6,
        'px4_instances': 0,
        'air_nodes': [],
        'takeover_server': False,
        'required_nodes': light_nodes,
    }
    checks = {
        'full_dual_px4': bool(full_facts['px4_instances'] == 2),
        'full_two_agents': bool(full_facts['agents'] == 2),
        'full_air_nodes_ok': bool(full_facts['air_nodes'] ==
                                   ['px4_adapter', 'standoff_follower', 'takeoff_controller',
                                    'trajectory_tracker']),
        'full_takeover_server': bool(full_facts['takeover_server']),
        'light_m0_capture_jammer': bool(light_facts['includes_m0_skeleton'] and
                                        light_facts['capture_adjudicator'] and
                                        light_facts['jammer']),
        'light_no_px4': bool(not (table.get('lightweight') or {}).get('px4', False)),
        'light_no_air_link': bool(not (table.get('lightweight') or {}).get('air_link', False)),
    }
    return full_facts, light_facts, checks


def read_lightweight(exp_root, run_id):
    path = os.path.join(exp_root, 'w6-task-eval', run_id, 'lightweight', 'metrics', 'report.json')
    if not os.path.exists(path):
        return None, path
    with open(path) as f:
        return json.load(f), path


def read_same_run(exp_root, run_id):
    path = os.path.join(exp_root, 'w6-task-eval', run_id, 'same_run.json')
    if not os.path.exists(path):
        return None, path
    with open(path) as f:
        return json.load(f), path


def case_a_lightweight(report, path):
    """(a) 轻量 expected_match 已标 lightweight；每命中 episode 有 proposition。"""
    res = {'pass': False}
    if report is None:
        res['error'] = 'lightweight report.json 缺失: %s' % path
        return res
    res['report_path'] = path
    res['report_backend'] = report.get('backend')
    res['report_launch'] = report.get('launch')
    episodes = report.get('episodes', [])
    matched = [e for e in episodes if e.get('expected_match')]
    res['n_episodes'] = len(episodes)
    res['n_expected_match'] = len(matched)
    res['matched_all_tagged'] = bool(all(e.get('backend') == 'lightweight' for e in matched))
    res['matched_all_have_proposition'] = bool(all(bool(e.get('proposition')) for e in matched))
    res['report_tagged'] = bool(report.get('backend') == 'lightweight' and
                                report.get('launch') == 'w6.launch.py')
    res['pass'] = bool(res['report_tagged'] and res['matched_all_tagged'] and
                       res['matched_all_have_proposition'] and len(matched) > 0)
    return res


def _five_questions_present(causal, full_chain):
    """(b') 5 问字段齐全 + 取自真实观测（full_chain_executed==true）。"""
    have = {}
    if not isinstance(causal, dict):
        return False, {}
    have['start_ok'] = isinstance(causal.get('start_ok'), bool)
    own = causal.get('own_takeover') or {}
    have['own_takeover'] = all(k in own for k in
                               ('release_requested', 'takeover_ok',
                                'active_controller_before', 'active_controller_after'))
    tm = causal.get('target_motion') or {}
    have['target_motion'] = isinstance(tm.get('classified'), str) and ('stats' in tm)
    cap = causal.get('capture') or {}
    jam = causal.get('jam') or {}
    have['capture_or_jam'] = isinstance(cap.get('observed'), bool) and ('event' in cap) and \
        isinstance(jam.get('observed'), bool) and ('event' in jam)
    term = causal.get('terminal') or {}
    have['terminal'] = isinstance(term.get('reached'), bool) and \
        ('state_name' in term) and ('reason_code' in term)
    return bool(full_chain and all(have.values())), have


def case_b_full_same_run(same, path):
    """(b)/(b') 整链同一 run 5 问齐全且真实（full_chain_executed==true）。"""
    res = {'pass': False}
    if same is None:
        res['error'] = 'same_run.json 缺失: %s' % path
        return res
    res['same_run_path'] = path
    causal = same.get('causal_record') or {}
    full_chain = bool(same.get('full_chain_executed'))
    ok, have = _five_questions_present(causal, full_chain)
    res['full_chain_executed'] = full_chain
    res['backend'] = same.get('backend')
    res['five_questions_present'] = have
    res['causal_record'] = causal
    res['pass'] = bool(ok and same.get('backend') == 'full')
    return res


def case_c_pass_has_proposition(report, same):
    """(c) 每个 pass 都有命题（pass ↔ 命题强绑定）。"""
    res = {'pass': False}
    lt = []
    if report is not None:
        for e in report.get('episodes', []):
            if e.get('pass'):
                lt.append({'scene': e.get('scene'), 'has_prop': bool(e.get('proposition')),
                           'prop': e.get('proposition')})
    res['lightweight_pass_episodes'] = lt
    res['lightweight_ok'] = bool(all(x['has_prop'] for x in lt))
    sr_ok = True
    if same is not None and same.get('pass'):
        sr_ok = bool(same.get('proposition'))
        res['same_run_proposition'] = same.get('proposition')
    res['same_run_ok'] = bool(sr_ok)
    res['pass'] = bool(res['lightweight_ok'] and res['same_run_ok'])
    return res


def build_matrix(report, same, full_facts, light_facts, lw_path, sr_path):
    """构造覆盖矩阵：lightweight/full × GUI(headless/gui) × 各轴；covered/uncovered/N/A + evidence + reason。"""
    rows = []
    # lightweight × headless（Phase A 实跑）
    lw_cells = {}
    if report is not None:
        cov = report.get('coverage') or {}
        ts = (cov.get('task_start') or {}).get('value')
        started = bool(ts and not str(ts).startswith('0/'))
        lw_cells['task_start'] = _cell('covered' if started else 'uncovered', ts,
                                       '%s（coverage.task_start）' % lw_path,
                                       '' if started else '轻量 campaign 未观测到 episode start')
        lw_cells['own_takeover'] = _cell(
            'not_applicable', 'not_applicable',
            '源事实：w6.launch.py backend:=lightweight 无 PX4/空中链路（无 takeoff_controller / release_control）',
            '轻量栈不含空中链路 → 本轴 N/A')
        tm = (cov.get('target_motion') or {}).get('value')
        lw_cells['target_motion'] = _cell(
            'covered', tm, '%s（coverage.target_motion，场景登记轴）' % lw_path)
        rt = (cov.get('real_terminal') or {}).get('value')
        lw_cells['real_terminal'] = _cell(
            'covered', rt, '%s（coverage.real_terminal，/agc/task/state 观测）' % lw_path)
    else:
        for ax in AXES:
            lw_cells[ax] = _cell('uncovered', None, '', '轻量 report.json 缺失')
    rows.append({'backend': 'lightweight', 'gui': 'headless',
                 'executed': report is not None, 'cells': lw_cells})

    # lightweight × gui（本轮未执行）
    rows.append({'backend': 'lightweight', 'gui': 'gui', 'executed': False,
                 'cells': {ax: _cell('uncovered', None, '',
                                     '本轮未执行 GUI 模式轻量 campaign（--mode gui）') for ax in AXES}})

    # full × headless（Phase B 实跑）
    fc = {}
    if same is not None:
        causal = same.get('causal_record') or {}
        own = causal.get('own_takeover') or {}
        tm = causal.get('target_motion') or {}
        cap = causal.get('capture') or {}
        jam = causal.get('jam') or {}
        term = causal.get('terminal') or {}
        fc['task_start'] = _cell(
            'covered' if causal.get('start_ok') else 'uncovered',
            causal.get('start_ok'), '%s（causal_record.start_ok）' % sr_path,
            '' if causal.get('start_ok') else 'start 未 OK（见 same_run start_message）')
        fc['own_takeover'] = _cell(
            'covered' if own.get('takeover_ok') else 'uncovered',
            {'active_controller_before': own.get('active_controller_before'),
             'active_controller_after': own.get('active_controller_after'),
             'takeover_ok': own.get('takeover_ok')},
            '%s（causal_record.own_takeover，/agc/air/release_control）' % sr_path,
            '' if own.get('takeover_ok') else '己方接管未成功')
        tm_ok = tm.get('classified') not in (None, 'unclassified')
        fc['target_motion'] = _cell(
            'covered' if tm_ok else 'uncovered', {'classified': tm.get('classified'),
                                                  'stats': tm.get('stats')},
            '%s（causal_record.target_motion，/agc/gt/intruder_uav）' % sr_path,
            '' if tm_ok else '目标运动未能分类')
        ev = bool(cap.get('observed') or jam.get('observed'))
        fc['real_terminal'] = _cell(
            'covered' if term.get('reached') else 'uncovered',
            {'state_name': term.get('state_name'), 'reason_code': term.get('reason_code'),
             'reached': term.get('reached')},
            '%s（causal_record.terminal，/agc/task/state）' % sr_path,
            '' if term.get('reached') else '同一 run 未达终态（墙钟看门狗超时）')
        fc['_evidence_channels'] = _cell(
            'covered' if ev else 'uncovered',
            {'capture': cap.get('event'), 'jam': jam.get('event')},
            '%s（causal_record.capture / jam）' % sr_path,
            '' if ev else '未观测到捕获/干扰事件')
    else:
        for ax in AXES:
            fc[ax] = _cell('uncovered', None, '', '整链 same_run.json 缺失（整链未跑/未产出）')
        fc['_evidence_channels'] = _cell('uncovered', None, '', '整链 same_run.json 缺失')
    rows.append({'backend': 'full', 'gui': 'headless',
                 'executed': same is not None and bool(same.get('full_chain_executed')),
                 'cells': fc})

    # full × gui（本轮未执行）
    rows.append({'backend': 'full', 'gui': 'gui', 'executed': False,
                 'cells': {ax: _cell('uncovered', None, '',
                                     '本轮未执行 GUI 模式整链（--mode gui / gz 渲染）') for ax in AXES}})

    return {
        'axes': AXES,
        'rows': rows,
        'backends': {'lightweight': light_facts, 'full': full_facts},
    }


def case_d_honest_axes(matrix):
    """(d) 未覆盖轴如实标注（无「无证据却称已覆盖」）。"""
    res = {'pass': True, 'violations': []}
    for row in matrix['rows']:
        rid = '%s/%s' % (row['backend'], row['gui'])
        for ax, c in row['cells'].items():
            if c['status'] == 'covered':
                has_ev = bool(c.get('evidence'))
                ok = bool(has_ev and c.get('value') not in (None, '', [], {}))
                if not ok:
                    res['violations'].append('%s.%s covered 无证据/无 value' % (rid, ax))
            elif c['status'] in ('uncovered', 'not_applicable'):
                if not c.get('reason'):
                    res['violations'].append('%s.%s %s 无如实原因' % (rid, ax, c['status']))
    res['pass'] = bool(not res['violations'])
    return res


def case_e_frozen_abi():
    """(e) 冻结值（capture_hit 恰为 CAPTURE_NET）与 ABI 回归（src/agc_msgs 未改）。"""
    res = {}
    src = Path(ROOT) / 'src' / 'agc_task' / 'src' / 'capture_adjudicator.cpp'
    text = src.read_text(encoding='utf-8', errors='replace')
    res['hit_literal_present'] = ('hit.data = "CAPTURE_NET";' in text)

    def _git(args):
        try:
            out = subprocess.run(['git', '-C', ROOT] + args, capture_output=True, text=True,
                                 timeout=20.0)
            return out.returncode, (out.stdout or '').strip()
        except Exception as exc:  # noqa: BLE001
            return -1, repr(exc)

    rc1, d1 = _git(['diff', '--name-only', '--', 'src/agc_msgs'])
    rc2, d2 = _git(['diff', '--cached', '--name-only', '--', 'src/agc_msgs'])
    rc3, d3 = _git(['status', '--porcelain', '--', 'src/agc_msgs'])
    res['agc_msgs_status'] = d3
    res['agc_msgs_unchanged'] = (rc1 == 0 and rc2 == 0 and rc3 == 0 and not d1 and not d2 and not d3)
    res['pass'] = bool(res['hit_literal_present'] and res['agc_msgs_unchanged'])
    return res


def write_matrix_md(matrix, path):
    md = ['# W6 覆盖矩阵 · R-025（lightweight/full × GUI × 各轴）', '',
          '> 逐轴标 **covered / uncovered / not_applicable** + evidence 出处 + 未覆盖如实原因。',
          '> 轻量 PASS ≠ 整链 / C3 GUI 验收；未执行轴如实标注。', '',
          '## 后端能力（源文件事实）', '',
          '| backend | launch | PX4 实例 | uXRCE Agent | 空中节点 | takeover 服务端 |',
          '|---|---|---|---|---|---|']
    for b, f in matrix['backends'].items():
        md.append('| %s | %s | %s | %s | %s | %s |' % (
            b, f.get('launch'),
            f.get('px4_instances', 0), f.get('agents', 0),
            ','.join(f.get('air_nodes', [])) or '-',
            f.get('takeover_server', False)))
    md += ['', '## 覆盖矩阵', '',
           '| backend | gui | 已执行 | task_start | own_takeover | target_motion | real_terminal |',
           '|---|---|---|---|---|---|---|']
    for row in matrix['rows']:
        cells = row['cells']

        def fmt(ax):
            c = cells.get(ax)
            if not c:
                return '-'
            return '%s: %s' % (c['status'], c.get('value'))
        md.append('| %s | %s | %s | %s | %s | %s | %s |' % (
            row['backend'], row['gui'], row['executed'],
            fmt('task_start'), fmt('own_takeover'), fmt('target_motion'), fmt('real_terminal')))
    md += ['', '## 逐轴证据 / 未覆盖原因', '']
    for row in matrix['rows']:
        md.append('### %s × %s（executed=%s）' % (row['backend'], row['gui'], row['executed']))
        for ax in AXES:
            c = row['cells'].get(ax)
            if not c:
                continue
            md.append('- **%s** — `%s` · value=%s · evidence=%s%s' % (
                ax, c['status'], c.get('value'), c.get('evidence') or '(无)',
                (' · reason=%s' % c['reason']) if c.get('reason') else ''))
        md.append('')
    with open(path, 'w') as f:
        f.write('\n'.join(md) + '\n')


def dump(res, out_dir):
    path = os.path.join(out_dir, 'metrics', 'r025.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-56s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
        for k, v in t.items():
            if k != 'pass':
                print('      %s = %s' % (k, v))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r025-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    res = {'run_id': run_id, 'tests': {}}
    try:
        full_facts, light_facts, topo_checks = topology_facts()
        res['topology'] = {'checks': topo_checks, 'full': full_facts, 'lightweight': light_facts}

        report, lw_path = read_lightweight(args.experiments_root, run_id)
        same, sr_path = read_same_run(args.experiments_root, run_id)

        matrix = build_matrix(report, same, full_facts, light_facts, lw_path, sr_path)
        res['coverage_matrix'] = matrix
        write_matrix_md(matrix, os.path.join(out_dir, 'coverage_matrix.md'))

        res['tests']['a_lightweight_expected_match_tagged'] = case_a_lightweight(report, lw_path)
        res['tests']['b_full_same_run_five_questions_real'] = case_b_full_same_run(same, sr_path)
        res['tests']['c_every_pass_has_proposition'] = case_c_pass_has_proposition(report, same)
        res['tests']['d_uncovered_axes_honestly_marked'] = case_d_honest_axes(matrix)
        res['tests']['e_frozen_value_and_abi'] = case_e_frozen_abi()

        a = res['tests']['a_lightweight_expected_match_tagged'].get('pass')
        b = res['tests']['b_full_same_run_five_questions_real'].get('pass')
        c = res['tests']['c_every_pass_has_proposition'].get('pass')
        d = res['tests']['d_uncovered_axes_honestly_marked'].get('pass')
        e = res['tests']['e_frozen_value_and_abi'].get('pass')
        # spec 判据 (a)∧(c)∧(d)∧(b')；并入红线回归 (e) 以保 ABI 不变。
        res['overall_pass'] = bool(a and b and c and d and e)
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, out_dir)


if __name__ == '__main__':
    sys.exit(main())
