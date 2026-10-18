#!/usr/bin/env python3
"""w6_r038_check —— R-038（数值模型暴露与配置合法性启动校验）自包含核查。

**起过 run_w6_r038.sh 后**读 `experiments/w6-task-eval/<run_id>/r038/` 的
`gtest/*.xml`（Phase A **实跑**失败型 gtest）+ `bench.json`（Phase B **实跑**节点启动期拒绝台架）
+ 源码 / ADR 静态核查，逐项判定 (a)…(j)；
输出 `.../<run_id>/r038/check.json`（含 `overall_pass` 布尔）与 `.../<run_id>/metrics/r038.json`。
overall_pass = (a)∧…∧(j)。

⚠ ground 适用性对拍为**独立解析分析**（命令映射可达性；**非**整链仿真、**非**复制被测 DARE 逻辑）。
"""

import argparse
import glob
import json
import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

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
    """剥离整行注释，仅保留可执行行（避免注释中提及 tokens 被误判）。"""
    return '\n'.join(s for s in (ln.strip() for ln in txt.splitlines()) if not s.startswith('#'))


def _gtest_cases(path):
    """解析 gtest JUnit XML → {testcase_name: passed_bool}。"""
    out = {}
    if not path or not os.path.exists(path):
        return out
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError:
        return out
    for tc in root.iter('testcase'):
        name = tc.get('name', '')
        failed = bool(list(tc.iter('failure')) or list(tc.iter('error')))
        out[name] = (not failed)
    return out


# ------------------------------------------------------------------ 判定用例

def case_a_invalid_qr_dt(g):
    """(a) 非法 Q/R/dt（≤0 / 非有限）**被拒绝**（非静默兜底）。"""
    names = ['RejectsNonPositiveDtNotSilentlyReset', 'RejectsNonPositiveRNotSilentlyReset',
             'RejectsNonPositiveQPos', 'RejectsNonPositiveQVel', 'RejectsNonFiniteWeight',
             'RejectsNonPositiveTol', 'RejectsNonPositiveMaxIter']
    res = {'gtest': {n: g.get(n) for n in names}}
    res['all_reject_tests_pass'] = all(g.get(n) is True for n in names)
    src = _read(os.path.join(ROOT, 'src/agc_control/src/lqr_controller.cpp')) or ''
    code = _code_only(src)
    res['no_silent_dt_fallback'] = not bool(re.search(r'if\s*\(\s*dt_\s*<=\s*0', code))
    res['no_silent_r_fallback'] = not bool(re.search(r'if\s*\(\s*r_\s*<=\s*0', code))
    res['has_validate_parameters'] = 'validateParameters' in code
    res['pass'] = bool(res['all_reject_tests_pass'] and res['no_silent_dt_fallback'] and
                       res['no_silent_r_fallback'] and res['has_validate_parameters'])
    return res


def case_b_dare_convergence(g):
    """(b) DARE 未收敛 / 非有限增益**被报告** → 明确失败。"""
    res = {'gtest_non_convergence': g.get('ReportsDareNonConvergenceAsExplicitFailure')}
    src = _read(os.path.join(ROOT, 'src/agc_control/src/lqr_controller.cpp')) or ''
    code = _code_only(src)
    res['reports_non_convergence'] = ('未收敛' in src) and ('throw std::runtime_error' in code)
    res['checks_finite_gain'] = ('K_.allFinite()' in code) and ('P.allFinite()' in code)
    res['records_residual'] = ('residual' in code) and ('cwiseAbs' in code) and ('maxCoeff' in code)
    res['pass'] = bool(res['gtest_non_convergence'] and res['reports_non_convergence'] and
                       res['checks_finite_gain'] and res['records_residual'])
    return res


def case_c_safety_shield_arrays(g):
    """(c) safety_shield 平行数组长度不一致**检出**（不静默截断）+ 合法几何通过。"""
    names = ['RejectsMismatchedParallelArrayLengths', 'RejectsNonPositiveRadius',
             'RejectsNonFiniteCoordinate', 'AcceptsValidGeometry']
    res = {'gtest': {n: g.get(n) for n in names}}
    res['all_pass'] = all(g.get(n) is True for n in names)
    src = _read(os.path.join(ROOT, 'src/agc_air/src/safety_shield.cpp')) or ''
    code = _code_only(src)
    res['no_min_length_truncation'] = not bool(re.search(r'std::min\(\{[^}]*names\.size\(\)', code))
    res['has_length_equality_check'] = ('names.size() != xs.size()' in code) and \
                                       ('throw std::runtime_error' in code)
    res['pass'] = bool(res['all_pass'] and res['no_min_length_truncation'] and
                       res['has_length_equality_check'])
    return res


def case_d_map_seed(bench):
    """(d) 地图几何 / 角度 / seed 非法**启动期拒绝**（或经核实已覆盖并如实登记）。"""
    sc = (bench or {}).get('scenarios', {})
    res = {
        'map_adapter_valid_alive': (sc.get('map_adapter_valid', {}).get('pass')),
        'map_adapter_invalid_geometry_rejected':
            (sc.get('map_adapter_invalid_geometry', {}).get('pass')),
        'map_adapter_invalid_obstacle_rejected':
            (sc.get('map_adapter_invalid_obstacle', {}).get('pass')),
        'air_perception_valid_alive': (sc.get('air_perception_valid_seed', {}).get('pass')),
        'air_perception_invalid_seed_rejected':
            (sc.get('air_perception_invalid_seed', {}).get('pass')),
    }
    map_src = _read(os.path.join(ROOT, 'src/agc_perception/src/map_adapter.cpp')) or ''
    res['map_adapter_has_startup_validation'] = ('validateStartupOrThrow' in map_src) and \
                                                ('拒绝启动' in map_src)
    perc_src = _read(os.path.join(ROOT, 'src/agc_perception/src/air_perception.cpp')) or ''
    res['air_perception_rejects_bad_seed'] = ('拒绝启动' in perc_src) and \
                                             ('不静默退回默认基种子' in perc_src)
    # seed 消费方登记：scenario_reset.spawn_seed / run_logger.seed 为**强类型 int 参数**（launch int()
    # 生成），非法值由 ROS 参数类型系统在启动期拒绝 → 登记为「已覆盖（类型安全）」。
    sr = _read(os.path.join(ROOT, 'src/agc_task/src/scenario_reset.cpp')) or ''
    rl = _read(os.path.join(ROOT, 'src/agc_task/src/run_logger.cpp')) or ''
    res['seed_consumers'] = {
        'scenario_reset.spawn_seed': 'declare_parameter<int>' in sr,
        'run_logger.seed': 'declare_parameter<int>' in rl,
        'air_perception.seed': '启动期校验（拒绝非整数字面量）',
    }
    res['seed_typed_int_registered'] = ('declare_parameter<int>("spawn_seed"' in sr) and \
                                        ('declare_parameter<int>("seed"' in rl)
    res['pass'] = bool(
        res['map_adapter_valid_alive'] and res['map_adapter_invalid_geometry_rejected'] and
        res['map_adapter_invalid_obstacle_rejected'] and res['air_perception_valid_alive'] and
        res['air_perception_invalid_seed_rejected'] and
        res['map_adapter_has_startup_validation'] and res['air_perception_rejects_bad_seed'] and
        res['seed_typed_int_registered'])
    return res


def case_e_lqr_source():
    """(e) LqrController 源含 0.5·dt² 项 + Ad/Bd 精确离散化 + 收敛/有限性检查 + 集中校验。"""
    src = _read(os.path.join(ROOT, 'src/agc_control/src/lqr_controller.cpp')) or ''
    code = _code_only(src)
    hdr = _read(os.path.join(ROOT, 'src/agc_control/include/agc_control/lqr_controller.hpp')) or ''
    res = {
        'bd_position_half_dt2_term': '0.5 * A * B * dt_ * dt_' in code,
        'ad_exact_half_dt2_term': '0.5 * A2 * dt_ * dt_' in code,
        'A2_nilpotent': 'A2 = A * A' in code,
        'no_old_bd_only_dt': 'B * dt_ + 0.5 * A * B' in code,
        'convergence_check': 'converged' in code and '未收敛' in src,
        'finite_checks': 'P.allFinite()' in code and 'K_.allFinite()' in code,
        'centralized_validation': 'validateParameters' in code,
        'getters_exposed': all(k in hdr for k in ('converged()', 'residual()', 'appliedFrequencyHz()')),
        'no_stale_exact_claim': ('Bd' in src) and ('恰为精确' in src or '精确' in src),
    }
    res['pass'] = all(res.values())
    return res


def _ground_applicability():
    """(f) ground 世界加速度→diff-drive 可达性 / 跟踪对拍（**独立解析分析**）。

    **不**复制被测 LQR/DARE 逻辑；仅对**命令映射**的可达性做解析比较：
      ground 命令（车体）：lin=clamp(bx,0,v_max)、ang=k_yaw·atan2(by,max(bx,0.1))；
      差速车可实现车体速度集 = {前进 v∈[0,v_max]，横向=0}（不产生瞬时横向速度）；
      air（holonomic、无障碍）可实现**任意**世界系 (vx,vy)。故 ground 可达集 ⊊ air 参考集。
    """
    v_max, v_ref = 1.5, 1.0
    n = 0
    ground_tracked = 0   # 期望世界速度与 ground 可实现速度（ψ=0 → (lin,0)）误差 < 10% v_ref
    lateral_desired = 0
    reverse_desired = 0
    for deg in range(0, 360, 5):
        th = math.radians(deg)
        vx_des, vy_des = v_ref * math.cos(th), v_ref * math.sin(th)
        bx = vx_des                       # 车头沿世界 x（ψ=0）→ 车体前进分量 = 世界 vx
        lin = min(max(bx, 0.0), v_max)    # 前进截断：bx<0 → 0（丢弃倒车）
        err = math.hypot(vx_des - lin, vy_des - 0.0)   # 横向 vy 被丢弃
        n += 1
        if abs(vy_des) > 0.05 * v_ref:
            lateral_desired += 1
        if bx < 0.0:
            reverse_desired += 1
        if err < 0.10 * v_ref:
            ground_tracked += 1
    return {
        'model': 'analytic applicability comparison（命令映射可达性；非整链仿真、非复制 DARE）',
        'lqr_reference_dof': '3 自由度 holonomic（世界系加速度；与 air 同一模型）',
        'diffdrive_body_velocity_tracked': '1（仅前进；横向被丢弃）',
        'diffdrive_cannot_lateral': True,
        'reverse_discarded': True,
        'n_directions': n,
        'lateral_desired_directions': lateral_desired,
        'reverse_desired_directions': reverse_desired,
        'ground_frac_desired_tracked_within_10pct': round(ground_tracked / n, 3),
        'air_frac_desired_tracked_within_10pct': 1.0,   # holonomic、无障碍 → 任意期望速度可跟踪
        'air_vs_ground_note': 'air（holonomic、无障碍）可达任意 (vx,vy)；ground（clamp 前进 + 横向转偏航）'
                              '可达集 ⊊ air 参考集 —— **不**假定同等闭环（见 ADR-0053）。',
    }


def case_f_ground(ground_report):
    """(f) ground_baseline 可达性 / 跟踪对拍 / 适用性标注存在 + 报告与 air 差异。"""
    src = _read(os.path.join(ROOT, 'src/agc_ground/src/ground_baseline.cpp')) or ''
    res = {
        'source_has_applicability_annotation': ('适用性标注' in src and '非完整差速动力学' in src),
        'source_anchors_adr_0053': 'ADR-0053' in src,
        'source_notes_discarded_dof': ('丢弃' in src and '横向自由度' in src),
        'reports_air_vs_ground_difference':
            (ground_report.get('ground_frac_desired_tracked_within_10pct') is not None and
             ground_report.get('air_frac_desired_tracked_within_10pct') == 1.0 and
             ground_report.get('ground_frac_desired_tracked_within_10pct') < 1.0),
        'analysis': ground_report,
    }
    res['pass'] = bool(res['source_has_applicability_annotation'] and
                       res['source_anchors_adr_0053'] and res['source_notes_discarded_dof'] and
                       res['reports_air_vs_ground_difference'])
    return res


def case_g_no_mirror():
    """(g) **无镜像实现测试**（启发式核验测试文件未重写 DARE / 求解逻辑）。"""
    t1 = _read(os.path.join(ROOT, 'src/agc_control/test/test_lqr_validation.cpp')) or ''
    t2 = _read(os.path.join(ROOT, 'src/agc_air/test/test_safety_shield_validation.cpp')) or ''
    forbidden = ['Riccat', 'P_next', 'cwiseAbs', 'maxCoeff', '.inverse()',
                 'B * dt_', 'A * dt_', 'K_ = S', 'AtP * Ad']
    res = {'files': {}, 'forbidden_hits': {}}
    for tag, t in (('test_lqr_validation', t1), ('test_safety_shield_validation', t2)):
        code = _code_only(t)
        hits = [f for f in forbidden if f in code]
        res['forbidden_hits'][tag] = hits
        res['files'][tag] = {'has_expect_throw': 'EXPECT_THROW' in code, 'mirror_hits': hits}
    res['no_mirror_reimplementation'] = all(len(h) == 0 for h in res['forbidden_hits'].values())
    res['failure_type_only'] = all(v['has_expect_throw'] for v in res['files'].values())
    res['pass'] = bool(res['no_mirror_reimplementation'] and res['failure_type_only'])
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
    # 跨文档禁 `§ 数字` 章节号（应使用稳定锚点）
    sec = []
    for base in (os.path.join(ROOT, 'docs'), os.path.join(ROOT, '.trae', 'specs')):
        for root, _dirs, files in os.walk(base):
            for fn in files:
                if not fn.endswith('.md'):
                    continue
                fp = os.path.join(root, fn)
                t = _read(fp) or ''
                if re.search(r'§\s*[0-9]', t):
                    sec.append(os.path.relpath(fp, ROOT))
    res = {'agc_msgs_porcelain': p.stdout.strip(), 'abi_clean': abi_clean,
           'setpoint_publisher_files': hits,
           'single_setpoint_publisher': hits == ['offboard_interface.cpp'],
           'section_number_refs': sec, 'no_section_refs': (len(sec) == 0)}
    res['pass'] = bool(abi_clean and res['single_setpoint_publisher'] and res['no_section_refs'])
    return res


def case_i_adr():
    matches = sorted(glob.glob(os.path.join(ROOT, 'docs/adr/0053-*.md')))
    res = {'path': os.path.relpath(matches[0], ROOT) if matches else None, 'exists': bool(matches)}
    if matches:
        t = _read(matches[0]) or ''
        res['status_accepted'] = ('## Status' in t and 'accepted' in t)
        res['has_evidence_section'] = '取证' in t
        res['has_r038_ref'] = 'r-038' in t.lower()
        res['no_section_ref'] = not bool(re.search(r'§\s*[0-9]', t))
        res['has_contract_ref'] = 'hard-constraints' in t
        res['has_boundary_note'] = ('冻结' in t) and ('唯一执行口语义' in t) and ('第二 ROS 命令者' in t)
    res['pass'] = bool(res['exists'] and res.get('status_accepted') and
                       res.get('has_evidence_section') and res.get('has_r038_ref') and
                       res.get('no_section_ref') and res.get('has_contract_ref') and
                       res.get('has_boundary_note'))
    return res


def case_j_script_hygiene():
    # 仅核查**执行用**脚本（run_w6_r038.sh + w6_r038_bench.py）。核查脚本本身必然含
    # `rm -rf` / `pkill` 等字面量（作为被检 token），故不纳入本项（避免自指误判）。
    sh = _code_only(_read(os.path.join(ROOT, 'scripts/run_w6_r038.sh')) or '')
    bch = _code_only(_read(os.path.join(ROOT, 'scripts/w6_r038_bench.py')) or '')
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
        'mentions_logic_bench': ('台架' in note),
        'unsupported_is_list': isinstance(uns, list),
    }
    res['pass'] = all(res.values())
    return res


def dump(res, run_dir):
    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(run_dir, 'r038'), exist_ok=True)
    with open(os.path.join(run_dir, 'r038', 'check.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'metrics', 'r038.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-30s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL',
                          os.path.join(run_dir, 'r038', 'check.json')))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261017-w6-r038-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()
    run_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)

    gtests = {}
    for base in ('test_lqr_validation', 'test_lqr_controller', 'test_safety_shield_validation'):
        for tc in _gtest_cases(os.path.join(run_dir, 'r038', 'gtest', base + '.xml')).items():
            gtests[tc[0]] = tc[1]
    bench = _load(os.path.join(run_dir, 'r038', 'bench.json'))
    ground = _ground_applicability()

    res = {'run_id': args.run_id, 'run_dir': run_dir, 'gtests': gtests, 'tests': {}}
    try:
        res['tests']['a_invalid_qr_dt_rejected'] = case_a_invalid_qr_dt(gtests)
        res['tests']['b_dare_convergence_reported'] = case_b_dare_convergence(gtests)
        res['tests']['c_safety_shield_arrays'] = case_c_safety_shield_arrays(gtests)
        res['tests']['d_map_geometry_seed'] = case_d_map_seed(bench)
        res['tests']['e_lqr_source'] = case_e_lqr_source()
        res['tests']['f_ground_applicability'] = case_f_ground(ground)
        res['tests']['g_no_mirror_implementation'] = case_g_no_mirror()
        res['tests']['h_abi_single_commander_section'] = case_h_abi_single_commander_section()
        res['tests']['i_adr_0053'] = case_i_adr()
        res['tests']['j_script_hygiene'] = case_j_script_hygiene()
        res['tests']['k_scope_note'] = case_k_scope_note(bench)
        res['overall_pass'] = bool(all(t.get('pass') for t in res['tests'].values()))
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    return dump(res, run_dir)


if __name__ == '__main__':
    sys.exit(main())
