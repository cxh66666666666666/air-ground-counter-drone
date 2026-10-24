#!/usr/bin/env python3
"""w7b_arms_runner —— W7-b T6：臂与消融批量跑（R1–R3；P1 需整链双 PX4）。

按 [ADR-0070](docs/adr/0070-w7b-research-propositions-and-rl-role-adjudication.md)(f) 跑臂：
  * **基线锚**（R3）：`command_source=baseline`（Navfn + 自研 LQR，准确命名）；
  * **P2 臂 A（c2）**：`command_source=rl`（直接出 cmd_vel；full 链固定协同）；
  * **P2 臂 B（规则高层 + 成熟低层）**：`command_source=rl_high`（高层出拦截点；低层 Navfn+LQR；full 链固定协同）；
  * **消融·安全干预**（F-06）：同场景 with-shield / without-shield（等预算）；
  * **P1 协同 / 非协同**（R1，`--backend full`）：观测自变量 α = `observe_yaw_mode` ∈ {`target`, `velocity`}。

复用 `episode_runner.run_episode`（同一就绪门 / 回收 / 三层验收）。产物落
`experiments/w7-rl-b/<run_id>/{metrics/arms.json,metrics/arms.md,log/*}`。

**CRN 配对**（[ADR-0070](docs/adr/0070-w7b-research-propositions-and-rl-role-adjudication.md)(e)）：`--seeds` 同 seed 逐臂跑，
按 (arm, seed) 记录；P1 两臂给**逐 seed 配对差**（协同 − 非协同）块，P2 两臂给（臂 B − 臂 A）块，
供 `w7b_p1_adjudicate.py` / `w7b_p2_adjudicate.py` 按预注册判定。

用法（先 source 环境，否则 `agc_msgs` 不在 Python 路径）：
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 scripts/w7b_arms_runner.py --mode headless --campaign w7-rl-b
  python3 scripts/w7b_arms_runner.py --arms p1_coop,p1_noncoop --seeds 0,1 --timeout 200 --ready-timeout-s 300
  python3 scripts/w7b_arms_runner.py --arms p2_c2_low_level,p2_rl_high --seeds 0,1 --timeout 200 --ready-timeout-s 300
"""

import argparse
import json
import os
import random
import re
import sys
import time
from datetime import datetime

ROOT = '/home/chen/AGC'
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import rclpy  # noqa: E402
from episode_runner import _registry, _carriers, run_episode_with_infra_retry  # noqa: E402

TRACK_SCENE = 's02_linear_none_nominal_track'
SAFETY_SCENE = 's04_static_static_obs_track'
# P1 需「有效火控机会 + 净空」→ 取捕获场景（全链，含空中观测通道）
P1_SCENE = 's08_static_none_nominal_capture'
# P2 固定「协同」→ 同 P1 全链捕获场景（轻量栈无空中域，不能承载 P2 确证）。
P2_SCENE = P1_SCENE

# 通用字段缺省（保持既有轻量臂不变）：backend=lightweight / target_source='' / observe_yaw_mode=''
ARMS = [
    {'arm': 'baseline_anchor', 'group': 'R3', 'scene': TRACK_SCENE,
     'command_source': 'baseline', 'shield': 'true',
     'note': '基线锚：Navfn + 自研 LQR（准确命名）'},
    {'arm': 'p2_c2_low_level', 'group': 'P2', 'scene': P2_SCENE,
     'command_source': 'rl', 'shield': 'true', 'backend': 'full',
     'target_source': 'air_fusion', 'observe_yaw_mode': 'target', 'air_tracking_ref': 'on',
     'goal_follow_target': 'true',
     'note': 'P2 臂 A：c2 直接出 cmd_vel；固定协同（full 链，ADR-0082/预注册 v3）'},
    {'arm': 'p2_rl_high', 'group': 'P2', 'scene': P2_SCENE,
     'command_source': 'rl_high', 'shield': 'true', 'backend': 'full',
     'target_source': 'air_fusion', 'observe_yaw_mode': 'target', 'air_tracking_ref': 'on',
     'goal_follow_target': 'true',
     'note': 'P2 臂 B：规则高层拦截点 + 成熟低层 Navfn+LQR；固定协同（full 链，ADR-0082/预注册 v3）'},
    {'arm': 'ablation_with_shield', 'group': 'ABL', 'scene': SAFETY_SCENE,
     'command_source': 'rl', 'shield': 'true', 'note': '消融·安全干预：with-shield'},
    {'arm': 'ablation_without_shield', 'group': 'ABL', 'scene': SAFETY_SCENE,
     'command_source': 'rl', 'shield': 'false', 'note': '消融·安全干预：无 shield（纯仿真）'},
    # P1（R1，整链）：自变量 α = observe_yaw_mode。协同臂用**真值方位**指向 → **显式 oracle 指向臂**
    # （ADR-0070(i) 允许并须标注）；**测量**仍走真实感知通道（噪声 / 丢包 / FOV 照常生效）。
    {'arm': 'p1_coop', 'group': 'R1', 'scene': P1_SCENE, 'command_source': 'rl', 'shield': 'true',
     'backend': 'full', 'target_source': 'air_fusion', 'observe_yaw_mode': 'target',
     'note': 'P1 协同臂：空中机**对准目标**观测（显式 oracle 指向臂，ADR-0070(i)）'},
    {'arm': 'p1_noncoop', 'group': 'R1', 'scene': P1_SCENE, 'command_source': 'rl', 'shield': 'true',
     'backend': 'full', 'target_source': 'air_fusion', 'observe_yaw_mode': 'velocity',
     'air_tracking_ref': 'off',
     'note': 'P1 非协同臂：空中机**固定航迹**（不订阅跟踪参考）+ **机头随速度**（不指向目标）'},
    # —— ADR-0087：P1 机制消融（指向 × 跟踪参考 2×2）的另外两个 cell ——
    #   ①p1_coop = target+on ｜ ②p1_noncoop = velocity+off（上）｜ ③p1_point_only = target+off
    #   ④p1_track_only = velocity+on（已知风险：ADR-0073 实证 C10 hold 自锁可能使该 cell 退化——预登记）。
    {'arm': 'p1_point_only', 'group': 'R1x', 'scene': P1_SCENE, 'command_source': 'rl', 'shield': 'true',
     'backend': 'full', 'target_source': 'air_fusion', 'observe_yaw_mode': 'target',
     'air_tracking_ref': 'off',
     'note': 'P1 消融③仅指向：机头对准目标（oracle 指向），但飞**固定 figure-8**（不订阅跟踪参考）'},
    {'arm': 'p1_track_only', 'group': 'R1x', 'scene': P1_SCENE, 'command_source': 'rl', 'shield': 'true',
     'backend': 'full', 'target_source': 'air_fusion', 'observe_yaw_mode': 'velocity',
     'air_tracking_ref': 'on',
     'note': 'P1 消融④仅跟踪：跟随目标航迹（订阅跟踪参考），但机头**随速度**（不指向目标）；'
             'ADR-0073 已实证该组合可能触发 C10 hold 自锁 → 若退化照实报告'},
]


def paired_deltas(results):
    """CRN 配对（R1）：按 seed 对齐 P1 两臂，给**逐 seed 配对差**（协同 − 非协同）+ 汇总。
    判定（显著性 / 置信区间 / 阈值）**不在**此处——由步 3 按 `w7b_prereg.yaml` 执行。"""
    by = {(r['arm'], r['seed']): r for r in results}
    seeds = sorted({r['seed'] for r in results if r['group'] == 'R1'})
    rows = []
    for s in seeds:
        c, n = by.get(('p1_coop', s)), by.get(('p1_noncoop', s))
        if not c or not n:
            continue

        def d(k):
            a, b = c.get(k), n.get(k)
            return None if (a is None or b is None) else round(a - b, 6)

        rows.append({
            'seed': s,
            'coop_terminal': c['terminal_state'], 'noncoop_terminal': n['terminal_state'],
            'd_fire_control_opportunity_rate': d('fire_control_opportunity_rate'),
            'd_visibility_rate': d('visibility_rate'),
            'd_min_clearance_m': d('min_clearance_m'),
        })

    def mean(key):
        vals = [r[key] for r in rows if r[key] is not None]
        return round(sum(vals) / len(vals), 6) if vals else None

    return rows, {
        'n_paired': len(rows), 'seeds': seeds,
        'd_definition': '协同(p1_coop) − 非协同(p1_noncoop)，同 seed 配对（CRN）',
        'mean_d_fire_control_opportunity_rate': mean('d_fire_control_opportunity_rate'),
        'mean_d_visibility_rate': mean('d_visibility_rate'),
        'mean_d_min_clearance_m': mean('d_min_clearance_m'),
        'adjudication': 'not_executed（步 3：按 src/agc_rl/config/w7b_prereg.yaml 判定）',
    }


def paired_deltas_p2(results):
    """CRN 配对（P2）：按 seed 对齐两臂，给**逐 seed 配对差**（臂 B − 臂 A）+ 汇总。
    判定（显著性 / 置信区间 / 阈值）**不在**此处——由 `scripts/w7b_p2_adjudicate.py`
    按预注册 `statistics.p2` 执行（ADR-0082 / 预注册 v2）。"""
    by = {(r['arm'], r['seed']): r for r in results}
    seeds = sorted({r['seed'] for r in results if r['group'] == 'P2'})
    rows = []
    for s in seeds:
        a, b = by.get(('p2_c2_low_level', s)), by.get(('p2_rl_high', s))
        if not a or not b:
            continue

        def d(k):
            x, y = a.get(k), b.get(k)
            return None if (x is None or y is None) else round(y - x, 6)

        rows.append({
            'seed': s,
            'arm_a_terminal': a['terminal_state'], 'arm_b_terminal': b['terminal_state'],
            'd_fire_control_opportunity_rate': d('fire_control_opportunity_rate'),
            'd_fire_control_opportunity_rate_active': d('fire_control_opportunity_rate_active'),
            'd_visibility_rate': d('visibility_rate'),
            'd_min_clearance_m': d('min_clearance_m'),
            'd_wall_elapsed_s': d('wall_elapsed_s'),
            'd_rtf': d('rtf'),
        })

    def mean(key):
        vals = [r[key] for r in rows if r[key] is not None]
        return round(sum(vals) / len(vals), 6) if vals else None

    return rows, {
        'n_paired': len(rows), 'seeds': seeds,
        'd_definition': '臂 B(p2_rl_high) − 臂 A(p2_c2_low_level)，同 seed 配对（CRN）',
        'mean_d_fire_control_opportunity_rate': mean('d_fire_control_opportunity_rate'),
        'mean_d_fire_control_opportunity_rate_active': mean('d_fire_control_opportunity_rate_active'),
        'mean_d_visibility_rate': mean('d_visibility_rate'),
        'mean_d_min_clearance_m': mean('d_min_clearance_m'),
        'mean_d_wall_elapsed_s': mean('d_wall_elapsed_s'),
        'mean_d_rtf': mean('d_rtf'),
        'adjudication': 'not_executed（由 scripts/w7b_p2_adjudicate.py 按 statistics.p2 判定）',
    }


def perception_health(results, out_dir):
    """ADR-0080：汇总 `air_perception` 的 B1 门控计数（逐最终尝试日志）→ 批次产物。

    计数口径：每 5 s 一条诊断，**逐通道**累加（本项目 2 通道）→ 绝对数不是"拍数"，
    比值 = 该原因计数 / (gate_ok + 各拒绝原因计数)。无诊断行的批次（轻量栈 / 旧日志）标 available=false。
    """
    keys = ('gate_ok', 'no_pose', 'behind', 'rmin', 'rmax', 'fov_az', 'fov_el', 'blocked', 'dropped')
    episodes = []
    for r in results:
        tag = r['arm']
        if r.get('infra_attempts'):
            tag = r['infra_attempts'][-1]['log_tag']
        log_path = os.path.join(out_dir, 'log', 'ep_%s_%s_%d.log' % (tag, r['scene'], r['seed']))
        sums = {k: 0 for k in keys}
        windows = 0
        present_sum = 0.0
        if os.path.exists(log_path):
            with open(log_path, errors='ignore') as f:
                for line in f:
                    if 'B1 诊断' not in line:
                        continue
                    mp = re.search(r'target_present=([\d.]+)', line)
                    if not mp:
                        continue
                    windows += 1
                    present_sum += float(mp.group(1))
                    for k in keys:
                        mk = re.search(r'\b%s=(\d+)' % k, line)
                        if mk:
                            sums[k] += int(mk.group(1))
        reject_keys = ('no_pose', 'behind', 'rmin', 'rmax', 'fov_az', 'fov_el')
        rejects = sum(sums[k] for k in reject_keys)
        gate_total = sums['gate_ok'] + rejects
        ratios = {k: (round(sums[k] / gate_total, 6) if gate_total else None) for k in reject_keys}
        episodes.append({
            'arm': r['arm'], 'seed': r['seed'], 'log_tag': tag,
            'available': windows > 0,
            'windows_5s': windows,
            'target_present_mean': round(present_sum / windows, 4) if windows else None,
            'counts': sums,
            'gate_total': gate_total,
            'reject_ratios': ratios,
        })

    def agg(rows):
        sums = {k: sum(e['counts'][k] for e in rows) for k in keys}
        rejects = sum(sums[k] for k in ('no_pose', 'behind', 'rmin', 'rmax', 'fov_az', 'fov_el'))
        gate_total = sums['gate_ok'] + rejects
        windows = sum(e['windows_5s'] for e in rows)
        present = [e['target_present_mean'] for e in rows if e['target_present_mean'] is not None]
        return {
            'n_episodes': len(rows), 'windows_5s': windows,
            'target_present_mean': round(sum(present) / len(present), 4) if present else None,
            'counts': sums, 'gate_total': gate_total,
            'reject_ratios': {
                k: (round(sums[k] / gate_total, 6) if gate_total else None)
                for k in ('no_pose', 'behind', 'rmin', 'rmax', 'fov_az', 'fov_el')},
        }

    by_arm = {arm: agg([e for e in episodes if e['arm'] == arm])
              for arm in sorted({e['arm'] for e in episodes})}
    report = {
        'definition_version': 'b1_gate_counters_v1',
        'note': 'air_perception 每 5 s 一条；逐通道累加（计数非拍数）。比值 = 原因计数 / (gate_ok + 拒绝计数)；'
                'blocked = 门控通过但被遮挡，dropped = 通道丢包/突发（另行单列，不并入 gate_total）。',
        'episodes': episodes, 'by_arm': by_arm, 'totals': agg(episodes),
    }
    with open(os.path.join(out_dir, 'metrics', 'perception_health.json'), 'w') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    md = ['# 感知健康度 · 目标门控计数', '',
          '- 口径：`b1_gate_counters_v1`（每 5 s、逐通道；比值分母 = gate_ok + 拒绝）',
          '- 说明：`rmax` 高 = 目标超出感知量程（几何盲区）；`no_pose` 高 = 空中位姿断供；'
          '`fov_az/el` 高 = 指向/姿态导致偏出视场。', '',
          '| arm | seed | windows | target_present | gate_ok | no_pose | rmax | fov | blocked | dropped |',
          '|---|---|---|---|---|---|---|---|---|---|']
    for e in episodes:
        ratio_txt = lambda k: ('%.1f%%' % (100.0 * e['reject_ratios'][k])) if e['reject_ratios'][k] is not None else '-'
        fov_ratio = (e['reject_ratios']['fov_az'] or 0.0) + (e['reject_ratios']['fov_el'] or 0.0)
        md.append('| %s | %d | %d | %s | %d | %s | %s | %.1f%% | %d | %d |' % (
            e['arm'], e['seed'], e['windows_5s'],
            e['target_present_mean'] if e['target_present_mean'] is not None else '-',
            e['counts']['gate_ok'], ratio_txt('no_pose'), ratio_txt('rmax'), 100.0 * fov_ratio,
            e['counts']['blocked'], e['counts']['dropped']))
    md += ['', '## 按臂汇总', '',
           '| arm | episodes | target_present | no_pose | rmax | fov_az+el |',
           '|---|---|---|---|---|---|']
    for arm, a in by_arm.items():
        fov = (a['reject_ratios']['fov_az'] or 0.0) + (a['reject_ratios']['fov_el'] or 0.0)
        def pct(v):
            return ('%.2f%%' % (100.0 * v)) if v is not None else '-'
        md.append('| %s | %d | %s | %s | %s | %.2f%% |' % (
            arm, a['n_episodes'], a['target_present_mean'], pct(a['reject_ratios']['no_pose']),
            pct(a['reject_ratios']['rmax']), 100.0 * fov))
    with open(os.path.join(out_dir, 'metrics', 'perception_health.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='headless', choices=['headless', 'gui'])
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--ready-timeout-s', type=float, default=180.0)
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--seeds', default='0', help='逗号分隔 seed（CRN：同 seed 逐臂配对跑）')
    ap.add_argument('--arms', default=None, help='逗号分隔 arm 名（缺省全部）')
    # ADR-0085：场景覆盖（pilot / 新固定条件用；缺省 '' = 用 ARMS 内各臂自带场景）。
    ap.add_argument('--scene', default='', help='覆盖所选臂的场景（缺省沿用 ARMS 定义）')
    # ADR-0074：终态后继续采到**固定观测时长**（自 start 起、仿真钟秒）；须与 run_logger 的
    # `obs_window_s` 同值（w6_base.yaml，缺省 60.0），否则 visibility_metrics 窗口采不满。
    ap.add_argument('--obs-horizon-s', type=float, default=60.0)
    # 放宽（2026-10-20）：/agc/task/start 等待上限（s；须 ≥ task_manager 的 air_hover_timeout_s=45）
    ap.add_argument('--start-timeout-s', type=float, default=60.0)
    # ADR-0075：**infra 重试协议**——本机 uXRCE 实体创建**间歇**超时（PX4 侧硬编码 1 s 预算）导致的
    #   `air_hover_timeout` / `odom_invalid` 等 `classification=='infra'` 结果按本上限**重跑**；
    #   mission 结果（含 TIMEOUT）**一律不重试**。0 = 关闭。逐次产物（日志 .rN / run_id）全留。
    ap.add_argument('--infra-retries', type=int, default=3)
    args = ap.parse_args()

    run_id = args.run_id or ('%s-%s-%s-%04d' % (
        datetime.now().strftime('%Y%m%d'), args.campaign,
        datetime.now().strftime('%H%M%S'), random.randint(0, 9999)))
    out_dir = os.path.join(ROOT, 'experiments', args.campaign, run_id)
    if os.path.exists(out_dir):
        print('拒绝写入已有 run 目录：%s' % out_dir)
        return 2
    for sub in ('metrics', 'log'):
        os.makedirs(os.path.join(out_dir, sub), exist_ok=True)

    registry, carriers = _registry(), _carriers()
    arms = ARMS if not args.arms else [a for a in ARMS if a['arm'] in args.arms.split(',')]
    # ADR-0085：场景覆盖——**复制**后改 scene，不改 ARMS 本体（缺省行为不变）。
    if args.scene:
        arms = [dict(a, scene=args.scene) for a in arms]
    seeds = [int(s) for s in args.seeds.split(',') if s.strip()]

    rclpy.init()   # run_episode 依赖已初始化的 rclpy（与 episode_runner.main 同约定）
    results = []
    t0 = time.monotonic()
    for a in arms:
        backend = a.get('backend', 'lightweight')
        ts = a.get('target_source', '')
        oym = a.get('observe_yaw_mode', '')
        atr = a.get('air_tracking_ref', '')
        for seed in seeds:
            print('=== arm=%s group=%s scene=%s seed=%d source=%s shield=%s backend=%s oym=%s atr=%s ===' % (
                a['arm'], a['group'], a['scene'], seed, a['command_source'], a['shield'],
                backend, oym or 'default', atr or 'on'))
            r = run_episode_with_infra_retry(
                            a['scene'], seed, registry, args.mode, args.timeout,
                            out_dir, carriers, ready_timeout_s=args.ready_timeout_s,
                            command_source=a['command_source'], shield=a['shield'],
                            backend=backend, target_source=ts, observe_yaw_mode=oym,
                            air_tracking_ref=atr, log_tag=a['arm'],
                            goal_follow_target=a.get('goal_follow_target', ''),
                            obs_horizon_s=args.obs_horizon_s,
                            start_timeout_s=args.start_timeout_s,
                            infra_retries=args.infra_retries)
            sm = {}
            if r.get('summary_path') and os.path.exists(r['summary_path']):
                with open(r['summary_path']) as f:
                    sm = json.load(f)
            vis = sm.get('visibility_metrics') or {}
            ew = sm.get('episode_window_metrics') or {}
            attr = sm.get('command_attribution') or {}
            # W7-b P2 v3（ADR-0083）：active-window FCO——分母 = GROUND_APPROACH→终态 的有效拍，
            # 不含终态后停泊（规避 window_fixed_v1 的终态后污染）。
            fco_active = (ew['ticks_fire_control_opportunity'] / ew['vis_ticks_total']
                          if ew.get('vis_ticks_total') else None)
            row = {
                'arm': a['arm'], 'group': a['group'], 'note': a['note'],
                'seed': seed, 'backend': backend, 'target_source': ts or None,
                'observe_yaw_mode': oym or None, 'air_tracking_ref': atr or 'on',
                'goal_follow_target': a.get('goal_follow_target', '') or None,
                'obs_horizon_s': args.obs_horizon_s,
                'scene': a['scene'], 'command_source': a['command_source'], 'shield': a['shield'],
                'terminal_state': r.get('terminal_state'), 'reason_code': r.get('reason_code'),
                'expected_terminal': r.get('expected_terminal'),
                'expected_match': r.get('expected_match'), 'pass': r.get('pass'),
                'layer_integrity': r.get('layer_integrity'), 'outcome_class': r.get('outcome_class'),
                'rtf': r.get('rtf'), 'wall_elapsed_s': r.get('wall_elapsed_s'),
                'run_id': r.get('run_id'), 'summary_path': r.get('summary_path'),
                # DV（分层多指标，ADR-0070(b)）+ P1 判据（有效火控机会率 / 最小净空）
                'shield_infeasible_rate': r.get('shield_infeasible_rate'),
                'intervened_ticks': attr.get('intervened_ticks'),
                'visibility_rate': vis.get('visibility_rate'),
                'fire_control_opportunity_rate': vis.get('fire_control_opportunity_rate'),
                # P2 v3 primary（ADR-0083）：active 窗口（不含终态后停泊）
                'fire_control_opportunity_rate_active': fco_active,
                'active_window_ticks': ew.get('ticks_total'),
                'longest_outage_s': vis.get('longest_outage_s'),
                'min_clearance_m': attr.get('clearance_min_m'),
                # ADR-0075：infra 重试协议证据——本 (arm, seed) 用了几次重试、逐次终态/原因/run_id
                'infra_retries_used': r.get('infra_retries_used'),
                'infra_attempts': r.get('infra_attempts'),
                'infra_retry_protocol': r.get('infra_retry_protocol'),
            }
            results.append(row)
            print('  terminal=%s expected_match=%s pass=%s vis_rate=%s fco_rate=%s min_clearance=%s '
                  'infra_retries_used=%s' % (
                row['terminal_state'], row['expected_match'], row['pass'],
                row['visibility_rate'], row['fire_control_opportunity_rate'], row['min_clearance_m'],
                row['infra_retries_used']))

    rclpy.shutdown()

    p1_rows, p1_summary = paired_deltas(results)
    p2_rows, p2_summary = paired_deltas_p2(results)
    backends = sorted({r['backend'] for r in results})
    p1_carried = any(r['group'] == 'R1' for r in results)
    report = {
        'run_id': run_id, 'campaign': args.campaign, 'mode': args.mode,
        'backend': backends[0] if len(backends) == 1 else 'mixed', 'backends': backends,
        'launch': 'w6.launch.py', 'seeds': seeds,
        'n_runs': len(results), 'wall_elapsed_s': time.monotonic() - t0,
        'arms': results,
        # ADR-0075：infra 重试协议（机器噪声处理，**不**改自变量 / 阈值 / 样本量）
        'infra_retry_protocol': {
            'rule': 'classification=="infra"（判据唯一来源 agc_rl.termination）→ 按 max_retries 重跑；'
                    'mission 结果（含 MISSION_FAILED(TIMEOUT)，契约 C4）**一律不重试**',
            'max_retries': args.infra_retries,
            'retried_runs': sum(1 for r in results if (r.get('infra_retries_used') or 0) > 0),
            'note': '逐次尝试产物全留（日志 ep_<arm>_<scene>_<seed>.log[.rN] + 各自 run_id）；'
                    '上报结果 = 最后一次尝试；重试耗尽的 infra 仍按预注册 keep_all_failures_and_infra 计入分母',
        },
        'carried_arms': sorted({r['arm'] for r in results}),
        'p1_pairing': {'rows': p1_rows, **p1_summary},
        'p2_pairing': {'rows': p2_rows, **p2_summary},
        'pending_arms': ([] if p1_carried else [
            {'arm': 'p1_coop_vs_noncoop', 'group': 'R1',
             'reason': '需**整链双 PX4**（`--backend full`）→ 本次未跑该臂'},
            {'arm': 'ablation_coop_observation / target_prediction', 'group': 'ABL',
             'reason': '依赖空中协同观测通道（同上）'}]),
        'proposition': 'R1–R3：各臂**可运行 + 同权限同约束同安全层同终态**；P1（R1）以 `--backend full` 承载，'
                       'CRN 配对结果见 `p1_pairing`（判定属步 3，按预注册）。P2（R2）以 full 链 + 固定协同承载，'
                       'CRN 配对结果见 `p2_pairing`（判定由 scripts/w7b_p2_adjudicate.py 按预注册 statistics.p2 执行）。',
        'diversity_note': '各臂仅自变量不同（命令源 / shield / observe_yaw_mode），余同'
                          '（同场景 / 同 seed / 同安全层 / 同终态语义；'
                          'P2 v3 两臂 goal_follow_target 同置 → 同源感知目标，见 ADR-0083）。',
    }
    with open(os.path.join(out_dir, 'metrics', 'arms.json'), 'w') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    md = ['# W7-b 臂与消融 · %s' % run_id, '',
          '- backend: `%s` · mode: `%s` · seeds: %s · runs: %d' % (
              report['backend'], args.mode, ','.join(str(s) for s in seeds), len(results)),
          '', '| arm | group | scene | seed | backend | source | shield | oym | terminal | expected '
          '| pass | vis_rate | fco_rate | fco_active | min_clearance | retries |',
          '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for r in results:
        md.append('| %s | %s | %s | %d | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %d |' % (
            r['arm'], r['group'], r['scene'], r['seed'], r['backend'], r['command_source'],
            r['shield'], r['observe_yaw_mode'] or '-', r['terminal_state'], r['expected_terminal'],
            r['pass'], r['visibility_rate'], r['fire_control_opportunity_rate'],
            r.get('fire_control_opportunity_rate_active'), r['min_clearance_m'],
            r.get('infra_retries_used') or 0))
    md += ['', '- infra 重试协议（ADR-0075）：上限 %d · 触发重试的 (arm, seed) 数 %d；'
           '逐次尝试产物全留（日志 `ep_<arm>_<scene>_<seed>.log[.rN]`），上报结果 = 最后一次尝试；'
           '`mission` 结果（含 TIMEOUT）**不**重试。' % (
               args.infra_retries,
               sum(1 for r in results if (r.get('infra_retries_used') or 0) > 0))]
    retried = [(r['arm'], r['seed']) for r in results if (r.get('infra_retries_used') or 0) > 0]
    if retried:
        md += ['- 触发过 infra 重试：' + '、'.join('%s/seed%d' % (a, s) for a, s in retried)]
    if p1_rows:
        md += ['', '## P1 CRN 配对（协同 − 非协同，同 seed）',
               '| seed | coop terminal | noncoop terminal | Δfco_rate | Δvis_rate | Δmin_clearance |',
               '|---|---|---|---|---|---|']
        for r in p1_rows:
            md.append('| %d | %s | %s | %s | %s | %s |' % (
                r['seed'], r['coop_terminal'], r['noncoop_terminal'],
                r['d_fire_control_opportunity_rate'], r['d_visibility_rate'], r['d_min_clearance_m']))
        md.append('- 均值：Δfco_rate=%s · Δvis_rate=%s · Δmin_clearance=%s（n_paired=%d）' % (
            p1_summary['mean_d_fire_control_opportunity_rate'],
            p1_summary['mean_d_visibility_rate'],
            p1_summary['mean_d_min_clearance_m'], p1_summary['n_paired']))
        md.append('- 判定：%s' % p1_summary['adjudication'])
    if p2_rows:
        md += ['', '## P2 CRN 配对（臂 B − 臂 A，同 seed）',
               '| seed | A terminal | B terminal | Δfco_rate | Δfco_active | Δvis_rate | Δmin_clearance | Δwall_elapsed_s | Δrtf |',
               '|---|---|---|---|---|---|---|---|---|']
        for r in p2_rows:
            md.append('| %d | %s | %s | %s | %s | %s | %s | %s | %s |' % (
                r['seed'], r['arm_a_terminal'], r['arm_b_terminal'],
                r['d_fire_control_opportunity_rate'], r['d_fire_control_opportunity_rate_active'],
                r['d_visibility_rate'], r['d_min_clearance_m'], r['d_wall_elapsed_s'], r['d_rtf']))
        md.append('- 均值：Δfco_rate=%s · Δfco_active=%s · Δvis_rate=%s · Δmin_clearance=%s · Δwall_elapsed_s=%s · Δrtf=%s（n_paired=%d）' % (
            p2_summary['mean_d_fire_control_opportunity_rate'],
            p2_summary['mean_d_fire_control_opportunity_rate_active'],
            p2_summary['mean_d_visibility_rate'],
            p2_summary['mean_d_min_clearance_m'],
            p2_summary['mean_d_wall_elapsed_s'],
            p2_summary['mean_d_rtf'], p2_summary['n_paired']))
        md.append('- 判定：%s' % p2_summary['adjudication'])
    md += ['', '## 未承载臂', '']
    for p in report['pending_arms']:
        md.append('- `%s`（%s）：%s' % (p['arm'], p['group'], p['reason']))
    with open(os.path.join(out_dir, 'metrics', 'arms.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')

    # ADR-0080：感知健康度（B1 门控计数）随批次落盘，供每批复核几何盲区 / 位姿断供占比。
    ph = perception_health(results, out_dir)
    n_avail = sum(1 for e in ph['episodes'] if e['available'])
    print('\n-> %s（感知健康度 available=%d/%d）' % (
        os.path.join(out_dir, 'metrics', 'perception_health.json'), n_avail, len(ph['episodes'])))
    print('\n-> %s' % os.path.join(out_dir, 'metrics', 'arms.json'))
    # 退出码：**可运行性**臂（R3/ABL）须全 pass；P1（R1）与 P2（R2）由预注册判定，
    # mission failure（含 TIMEOUT）是合法结果，不得作为 runner 失败门。
    gate = [r for r in results if r['group'] not in ('R1', 'P2')]
    return 0 if all(r['pass'] for r in gate) else 1


if __name__ == '__main__':
    sys.exit(main())
