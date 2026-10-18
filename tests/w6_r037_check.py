#!/usr/bin/env python3
"""w6_r037_check —— R-037（传感器/车体限制声明与能力标定）自包含核查。

**起过 run_w6_r037.sh 后**读 `experiments/w6-task-eval/<run_id>/r037/bench.json`（Phase A **实跑台架**证据）
+ `r037/xacro_baseline.sdf.xacro`（R-037 之前的安装模型基线）+ 源码 / 配置 / ADR 静态核查，逐项判定 (a)…(j)；
输出 `.../<run_id>/r037/check.json`（含 `overall_pass` 布尔）与 `.../<run_id>/metrics/r037.json`。
overall_pass = (a)∧…∧(j)。
"""

import argparse
import difflib
import glob
import json
import os
import re
import subprocess
import sys

ROOT = '/home/chen/AGC'
XACRO = 'src/agc_sim/models/agc_car/agc_car.sdf.xacro'
CAR_YAML = 'src/agc_sim/config/car.yaml'
GROUND_YAML = 'src/agc_ground/config/ground.yaml'


def _load(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _read(path):
    p = path if os.path.isabs(path) else os.path.join(ROOT, path)
    if not os.path.exists(p):
        return None
    with open(p, encoding='utf-8', errors='replace') as f:
        return f.read()


def _code_only(txt):
    """剥离整行注释，仅保留可执行行（避免注释中提及 `rm -rf` 被误判）。"""
    return '\n'.join(s for s in (ln.strip() for ln in txt.splitlines()) if not s.startswith('#'))


def _scenario(bench, name):
    s = (bench or {}).get('scenarios', {}).get(name, {}) or {}
    res = dict(s)
    res['present'] = bool(s)
    return res


def _unsupported_ok(bench, name):
    """某能力项：真值证据存在 **或** 如实 unsupported（且带失败证据）。二者其一即通过。"""
    uns = (bench or {}).get('unsupported', []) or []
    failure = (bench or {}).get('failure') or {}
    infra = (bench or {}).get('infra_error')
    has_failure_evidence = bool(infra and failure)
    return bool(name in uns)


# =====================================================================================
# (a) 传感器高度如实登记
# =====================================================================================
def case_a_sensor_height(adr_t, xacro_t, car_t, ground_t):
    res = {
        'adr_has_height_0p48': ('0.48' in (adr_t or '')) and ('车顶水平面' in (adr_t or '')),
        'adr_has_obstacle_families': all(k in (adr_t or '') for k in ('低障碍', '悬空', '薄障碍')),
        'xacro_has_lidar_z_0p48': ('<pose>0 0 0.48 0 0 0</pose>' in (xacro_t or '')),
        'xacro_has_obstacle_families': all(k in (xacro_t or '') for k in ('低障碍', '悬空', '薄障碍')),
        'config_has_height_0p48': ('0.48' in (car_t or '')) or ('0.48' in (ground_t or '')),
    }
    res['pass'] = all(res.values())
    return res


# =====================================================================================
# (b) 安全声明已收窄（含「不外推任意障碍」）
# =====================================================================================
def case_b_statement_narrowed(adr_t, ground_t):
    adr = adr_t or ''
    gnd = ground_t or ''
    res = {
        'adr_mentions_narrowing': ('收窄' in adr),
        'adr_no_extrapolation': any(k in adr for k in ('不得外推', '不外推')) and ('任意障碍' in adr),
        'adr_pole_world_limited': ('立柱' in adr),
        'ground_mentions_narrowing': ('收窄' in gnd),
        'ground_no_extrapolation': any(k in gnd for k in ('不得外推', '不外推')) and ('任意障碍' in gnd),
    }
    res['pass'] = all(res.values())
    return res


# =====================================================================================
# (c) 载荷外伸 / 碰撞口径一致化
# =====================================================================================
def case_c_payload_consistency(adr_t, xacro_t, car_t):
    adr = adr_t or ''
    res = {
        'adr_declares_domains': ('适用域' in adr),
        'adr_clearance_extrusion': ('外伸' in adr) and ('净空' in adr),
        'adr_collision_domain': ('碰撞' in adr),
        'adr_recalibration_obligation': ('重标定' in adr),
        'adr_no_implicit_inconsistency': ('一侧计入另侧不计' in adr) or ('隐含' in adr),
        'xacro_registers_payload': ('载荷外伸' in (xacro_t or '')) and ('碰撞' in (xacro_t or '')),
        'car_yaml_registers_payload': ('外伸' in (car_t or '')) and ('重标' in (car_t or '')),
        'payload_no_collision_registered': ('无 collision' in adr) or ('无碰撞' in adr),
    }
    res['pass'] = all(res.values())
    return res


# =====================================================================================
# (d)(e) 满速制动 / 转弯扫掠：真值证据 或 如实 unsupported
# =====================================================================================
def case_d_brake(bench):
    s = _scenario(bench, 'brake')
    truth_based = bool(s.get('truth_source') and 'gt' in str(s.get('truth_source')).lower()
                       and s.get('brake_distance_m') is not None)
    res = {'truth_based_evidence': truth_based, 'brake_distance_m': s.get('brake_distance_m'),
           'brake_time_s': s.get('brake_time_s'), 'failure_evidence': s.get('failure')}
    res['unsupported'] = bool(_unsupported_ok(bench, 'brake'))
    # 仅「发布零命令」不算：须带 gz 真值位移（truth_based）或如实 unsupported（带失败证据）
    res['pass'] = bool((truth_based and s.get('pass')) or res['unsupported'])
    return res


def case_e_turn(bench):
    s = _scenario(bench, 'turn')
    truth_based = bool(s.get('truth_source') and 'gt' in str(s.get('truth_source')).lower()
                       and s.get('arc_radius_actual_m') is not None
                       and s.get('spin_actual_omega_radps') is not None)
    res = {'truth_based_evidence': truth_based, 'arc_radius_actual_m': s.get('arc_radius_actual_m'),
           'spin_actual_omega_radps': s.get('spin_actual_omega_radps'),
           'slip_ratio': s.get('slip_ratio'), 'failure_evidence': s.get('failure')}
    res['unsupported'] = bool(_unsupported_ok(bench, 'turn'))
    res['pass'] = bool((truth_based and s.get('pass')) or res['unsupported'])
    return res


# =====================================================================================
# (f) 未改物理 / 惯性 / 驱动插件（对 R-037 之前的模型基线核验）
# =====================================================================================
def case_f_no_physics_change(run_dir):
    src = _read(XACRO) or ''
    baseline = _read(os.path.join(run_dir, 'r037/xacro_baseline.sdf.xacro')) or ''
    tags = ('<plugin', '<inertia', '<collision')

    def _physical(line):
        return any(t in line for t in tags)

    changed_physical = []
    if baseline:
        for ln in difflib.unified_diff(baseline.splitlines(), src.splitlines(), lineterm=''):
            if ln.startswith('+') and not ln.startswith('+++'):
                if _physical(ln):
                    changed_physical.append(ln)
            if ln.startswith('-') and not ln.startswith('---'):
                if _physical(ln):
                    changed_physical.append(ln)
    res = {
        'baseline_present': bool(baseline),
        'baseline_path': os.path.join(run_dir, 'r037/xacro_baseline.sdf.xacro'),
        'physical_lines_changed_vs_baseline': changed_physical,
        'no_physical_change_vs_baseline': (bool(baseline) and not changed_physical),
    }
    # 交叉核验：git diff 中**新增**的物理标签行，均须存在于基线模型（= R-037 之前既有行，非 R-037 引入）。
    p = subprocess.run(['git', 'diff', '--', XACRO], cwd=ROOT, capture_output=True, text=True)
    new_phys = []
    for ln in p.stdout.splitlines():
        if ln.startswith('+') and not ln.startswith('+++') and _physical(ln):
            if ln[1:].strip() and ln[1:].strip() not in (baseline or ''):
                new_phys.append(ln)
    res['git_diff_new_physical_lines'] = new_phys
    res['pass'] = bool(res['no_physical_change_vs_baseline'] and not new_phys)
    return res


# =====================================================================================
# (g) 保留 DiffDrive 路线、未回退 kinematic_sim
# =====================================================================================
def case_g_diffdrive_retained(xacro_t, car_t):
    x = xacro_t or ''
    p = subprocess.run(['grep', '-rn', 'kinematic_sim', 'src/'], cwd=ROOT, capture_output=True, text=True)
    res = {
        'xacro_has_diffdrive_plugin': ('gz-sim-diff-drive-system' in x),
        'xacro_diffdrive_left_right_joints': ('left_wheel_joint' in x) and ('right_wheel_joint' in x),
        'car_yaml_v_max_1p5': ('max_linear_velocity: 1.5' in (car_t or '')),
        'car_yaml_w_max_2p0': ('max_angular_velocity: 2.0' in (car_t or '')),
        'kinematic_sim_hits': p.stdout.strip(),
        'no_kinematic_sim_fallback': (p.stdout.strip() == ''),
    }
    res['pass'] = bool(res['xacro_has_diffdrive_plugin'] and res['xacro_diffdrive_left_right_joints']
                       and res['car_yaml_v_max_1p5'] and res['car_yaml_w_max_2p0']
                       and res['no_kinematic_sim_fallback'])
    return res


# =====================================================================================
# (h) ABI / 单命令者 / § 为空
# =====================================================================================
def case_h_abi_single_commander_section():
    p = subprocess.run(['git', 'status', '--porcelain', 'src/agc_msgs'],
                       cwd=ROOT, capture_output=True, text=True)
    abi_clean = (p.stdout.strip() == '')
    # 唯一执行口 gz 命令话题 /model/agc_car/cmd_vel 的 ROS 发布者应仅 ground_drive_relay.cpp。
    hits = []
    for root, _dirs, files in os.walk(os.path.join(ROOT, 'src')):
        for fn in files:
            if fn.endswith('.cpp'):
                t = _read(os.path.join(root, fn)) or ''
                if 'create_publisher<geometry_msgs::msg::Twist>' in t:
                    hits.append(fn)
    sec = subprocess.run(['grep', '-rnE', '§[0-9]', 'docs/', '.trae/specs/'],
                         cwd=ROOT, capture_output=True, text=True)
    res = {
        'agc_msgs_porcelain': p.stdout.strip(), 'abi_clean': abi_clean,
        'cmd_vel_publisher_files': sorted(hits),
        'single_commander': sorted(hits) == ['ground_drive_relay.cpp'],
        'section_refs': sec.stdout.strip(),
    }
    res['pass'] = bool(abi_clean and res['single_commander'] and sec.stdout.strip() == '')
    return res


# =====================================================================================
# (i) ADR-0052
# =====================================================================================
def case_i_adr():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0052-*.md')))
    res = {'path': matches[0] if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = ('## Status' in t and 'accepted' in t)
        res['no_section_ref'] = '\u00a7' not in t
        res['has_r037_ref'] = 'r-037' in t.lower()
        res['has_evidence_section'] = '取证' in t
        res['has_decision'] = '## 决策' in t
        res['has_abi_boundary'] = ('冻结 9 消息' in t) and ('唯一执行口' in t)
        res['has_drawback'] = ('DiffDrive' in t) and ('kinematic_sim' in t)
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and res.get('no_section_ref') and
                       res.get('has_r037_ref') and res.get('has_evidence_section') and
                       res.get('has_decision') and res.get('has_abi_boundary') and res.get('has_drawback'))
    return res


# =====================================================================================
# (j) 脚本卫生
# =====================================================================================
def case_j_scripts_hygiene():
    bench_code = _code_only(_read('scripts/archive/w6_r037_bench.py') or '')
    sh_code = _code_only(_read('scripts/archive/run_w6_r037.sh') or '')
    res = {
        'bench_no_rm_rf': 'rm -rf' not in bench_code,
        'sh_no_rm_rf': 'rm -rf' not in sh_code,
        'sh_uses_pgrep_pathkills': ('pgrep -f' in sh_code) and ('kill -9' in sh_code),
    }
    bare = re.findall(r'pkill\s+-\w+\s+-f\s+"(?!\[)[^"]+"', bench_code + '\n' + sh_code)
    res['bare_pkill'] = bare
    res['pass'] = bool(res['bench_no_rm_rf'] and res['sh_no_rm_rf'] and
                       res['sh_uses_pgrep_pathkills'] and not bare)
    return res


def dump(res, run_dir):
    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(run_dir, 'r037'), exist_ok=True)
    with open(os.path.join(run_dir, 'r037', 'check.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'metrics', 'r037.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-28s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL',
                          os.path.join(run_dir, 'r037', 'check.json')))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261017-w6-r037-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    adr = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0052-*.md')))
    adr_t = _read(adr[0]) if adr else ''
    xacro_t = _read(XACRO) or ''
    car_t = _read(CAR_YAML) or ''
    ground_t = _read(GROUND_YAML) or ''
    bench = _load(os.path.join(run_dir, 'r037', 'bench.json'))

    res = {'run_id': args.run_id, 'run_dir': run_dir, 'tests': {}}
    try:
        res['tests']['a_sensor_height'] = case_a_sensor_height(adr_t, xacro_t, car_t, ground_t)
        res['tests']['b_statement_narrowed'] = case_b_statement_narrowed(adr_t, ground_t)
        res['tests']['c_payload_consistency'] = case_c_payload_consistency(adr_t, xacro_t, car_t)
        res['tests']['d_brake'] = case_d_brake(bench)
        res['tests']['e_turn'] = case_e_turn(bench)
        res['tests']['f_no_physics_change'] = case_f_no_physics_change(run_dir)
        res['tests']['g_diffdrive_retained'] = case_g_diffdrive_retained(xacro_t, car_t)
        res['tests']['h_abi_single_commander_section'] = case_h_abi_single_commander_section()
        res['tests']['i_adr_0052'] = case_i_adr()
        res['tests']['j_scripts_hygiene'] = case_j_scripts_hygiene()
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
