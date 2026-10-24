#!/usr/bin/env python3
"""w7b_p2_prediction_chain —— P2 射击 `pred` 误差的预测链分解（W7-b T3 / ADR-0087(d)）。

**定位**：**诊断 / 描述性**，不改 P2 verdict；真值只用于**离线评估**，不进入运行时门。

分解（逐 shot，逐段核对）：
  1. **目标估计**：`est`（TargetMotionFilter 滤波后位置）− 真值；
  2. **火控提前量**：`pred` − `est`（ADR-0079 的 `p + clamp(v·t_est, max_lead_m)`）；
  3. **高层 lead**：`goal`（`/agc/ground/goal_hl`）− 高层滤波目标（`HL_DIAG est`）；
  4. **飞行 / 命中时序**：`t_est`（predictTarget 迭代后的到达时间）、实际 `flight`、`miss_dist`、
     `SHOT_MISS` / `CAPTURE` 结局。

**兼容旧日志**：v3（2026-10-22）及更早的 `SHOT_FIRED` 无 `est/t_est/target_id/conf/age`，
无 `HL_DIAG`。此时：
  * `pred` 误差仍可逐 shot 给出（相对静态真值）；
  * 估计项用**上界推理**：`|pred−truth| ≤ |est−truth| + max_lead_m` ⇒
    `est_err ≥ total − max_lead_m`（不依赖缺失的 `t_est`）；
  * 另给“由 `vel` 与 `dist/vh` 推断的 est / lead”作为**估计值**（明确标注，不作证据级结论）。

产物：`experiments/<campaign>/<run_id>/metrics/p2_prediction_chain.{json,md}`（只新增，不覆盖既有）。
用法：
  python3 scripts/w7b_p2_prediction_chain.py --run-id 20261022-w7-rl-b-v3-n20r
  python3 scripts/w7b_p2_prediction_chain.py --run-id 20261024-w7-rl-b-p2-predchain
"""

import argparse
import json
import math
import os
import re
import sys
from collections import Counter

ROOT = '/home/chen/AGC'
SCENE = 's08_static_none_nominal_capture'
TARGET_XY = (4.5, 3.0)          # s08 scenario_reset 目标真值（静止；只评估用）
NET_RADIUS_M = 0.5              # net_diameter_m=1.0 → 有效半径 0.5
MAX_LEAD_M = 1.5                # 默认 target_max_lead_m（上界推理用）
NET_VH_MPS = 6.5 * math.cos(math.radians(75.0)) * 0.9   # v0·cosθ·drag ≈ 1.514 m/s
FLIGHT_TIMEOUT_S = 3.0

NUM = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
RE_STAMP = re.compile(r'\[(\d{9,}\.\d+)\]')
RE_SHOT = re.compile(
    r'SHOT_FIRED shot=(\S+) t=(' + NUM + r') h=(' + NUM + r') dist=(' + NUM + r') '
    r'pred=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)')
RE_VEL = re.compile(r'\bvel=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)')
RE_EST = re.compile(
    r'\best=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\) t_est=(' + NUM + r') '
    r'target_id=(\S+) conf=(' + NUM + r') age=(' + NUM + r') '
    r'ntracks=(\d+) front_is_maxconf=(\d+) maxconf_target_id=(\S*)')
RE_MISS = re.compile(
    r'SHOT_MISS shot=(\S+) model_id=\S+ h=(' + NUM + r') dist=(' + NUM + r') '
    r'flight=(' + NUM + r') s miss_dist=(' + NUM + r') m')
RE_CAPTURE = re.compile(
    r'CAPTURE（发射式网动态命中）：shot=(\S+) model_id=\S+ h=(' + NUM + r') '
    r'dist=(' + NUM + r') flight=(' + NUM + r') s miss_dist=(' + NUM + r')')
RE_HL = re.compile(
    r'HL_DIAG target_id=(\S+) n_tracks=(\d+) est=\((' + NUM + r'),(' + NUM + r')\) '
    r'lead=\((' + NUM + r'),(' + NUM + r')\) goal=\((' + NUM + r'),(' + NUM + r')\)'
    r'(?: conf=(' + NUM + r'))?')


def hdist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def load_arms(run_dir, arms):
    path = os.path.join(run_dir, 'metrics', 'arms.json')
    if os.path.exists(path):
        with open(path) as f:
            rows = json.load(f)['arms']
        return [r for r in rows if r['arm'] in arms]
    # 部分 run（尚未写 arms.json 或中途停止）：直接按日志文件扫描每个 (arm, seed) 的最终尝试。
    log_dir = os.path.join(run_dir, 'log')
    best = {}
    if os.path.isdir(log_dir):
        for f in sorted(os.listdir(log_dir)):
            for arm in arms:
                m = re.match(
                    r'^ep_%s(?:\.r(\d+))?_%s_(\d+)\.log$' % (
                        re.escape(arm), re.escape(SCENE)), f)
                if not m:
                    continue
                attempt = int(m.group(1) or 0)
                seed = int(m.group(2))
                tag = arm if attempt == 0 else '%s.r%d' % (arm, attempt)
                if (arm, seed) not in best or attempt > best[(arm, seed)]['attempt']:
                    best[(arm, seed)] = {
                        'attempt': attempt, 'tag': tag,
                        'row': {
                            'arm': arm, 'seed': seed, 'terminal_state': None,
                            'reason_code': None, 'summary_path': None,
                            'infra_attempts': [{'log_tag': tag}]}}
    return [v['row'] for _, v in sorted(best.items())]


def log_name(row):
    tag = (row.get('infra_attempts') or [{}])[-1].get('log_tag') or row['arm']
    return 'ep_%s_%s_%d.log' % (tag, SCENE, row['seed'])


def parse_log(path):
    shots = {}
    order = []
    results = {}
    hl = []
    if not os.path.exists(path):
        return order, shots, results, hl
    for line in open(path, errors='replace'):
        ms = RE_SHOT.search(line)
        if ms:
            sid = ms.group(1)
            stamp_m = RE_STAMP.search(line)
            rec = {
                'shot_id': sid,
                'arm_stamp': float(stamp_m.group(1)) if stamp_m else None,
                't': float(ms.group(2)), 'h': float(ms.group(3)),
                'dist': float(ms.group(4)),
                'pred': (float(ms.group(5)), float(ms.group(6)), float(ms.group(7))),
                'vel': None, 'est': None, 't_est': None, 'target_id': None,
                'conf': None, 'age': None, 'n_tracks': None,
                'front_is_max_conf': None, 'max_conf_target_id': None,
                'outcome': None, 'flight': None, 'miss_dist': None,
            }
            mv = RE_VEL.search(line)
            if mv:
                rec['vel'] = (float(mv.group(1)), float(mv.group(2)), float(mv.group(3)))
            me = RE_EST.search(line)
            if me:
                rec['est'] = (float(me.group(1)), float(me.group(2)), float(me.group(3)))
                rec['t_est'] = float(me.group(4))
                rec['target_id'] = me.group(5)
                rec['conf'] = float(me.group(6))
                rec['age'] = float(me.group(7))
                rec['n_tracks'] = int(me.group(8))
                rec['front_is_max_conf'] = bool(int(me.group(9)))
                rec['max_conf_target_id'] = me.group(10)
            shots[sid] = rec
            order.append(sid)
            continue
        mm = RE_MISS.search(line)
        if mm:
            results[mm.group(1)] = {
                'outcome': 'miss', 'h_result': float(mm.group(2)),
                'result_dist': float(mm.group(3)),
                'flight': float(mm.group(4)), 'miss_dist': float(mm.group(5))}
            continue
        mc = RE_CAPTURE.search(line)
        if mc:
            results[mc.group(1)] = {
                'outcome': 'hit', 'h_result': float(mc.group(2)),
                'result_dist': float(mc.group(3)),
                'flight': float(mc.group(4)), 'miss_dist': float(mc.group(5))}
            continue
        mh = RE_HL.search(line)
        if mh:
            stamp_m = RE_STAMP.search(line)
            hl.append({
                'stamp': float(stamp_m.group(1)) if stamp_m else None,
                'target_id': mh.group(1), 'n_tracks': int(mh.group(2)),
                'est': (float(mh.group(3)), float(mh.group(4))),
                'lead': (float(mh.group(5)), float(mh.group(6))),
                'goal': (float(mh.group(7)), float(mh.group(8))),
                'conf': float(mh.group(9)) if mh.group(9) is not None else None,
            })
    for sid in order:
        if sid in results:
            shots[sid].update(results[sid])
    return order, shots, results, hl


def nearest_hl(hl, stamp):
    if stamp is None or not hl:
        return None
    best = None
    for h in hl:
        if h['stamp'] is None or h['stamp'] > stamp + 1e-6:
            continue
        d = stamp - h['stamp']
        if d > 2.5:   # HL 1 Hz；超过 2.5 s 视为无同拍样本
            continue
        if best is None or d < best[0]:
            best = (d, h)
    return best[1] if best else None


def enrich(rec, hls):
    """逐 shot 计算各段分量；缺失字段照实标注（None / 推断）。"""
    px, py = rec['pred'][0], rec['pred'][1]
    total = hdist(px, py, TARGET_XY[0], TARGET_XY[1])
    rec['pred_err_m'] = round(total, 3)
    rec['pred_gt_net_radius'] = total > NET_RADIUS_M
    rec['pred_gt_1m'] = total > 1.0

    # 估计项：新字段可得时直接算；否则给严格上界推理 + 旧日志推断。
    if rec['est'] is not None:
        ex, ey = rec['est'][0], rec['est'][1]
        rec['est_err_m'] = round(hdist(ex, ey, TARGET_XY[0], TARGET_XY[1]), 3)
        rec['fc_advance_m'] = round(hdist(px, py, ex, ey), 3)
        rec['est_err_lower_bound_m'] = rec['est_err_m']
        rec['derived_from_old_log'] = False
    else:
        rec['est_err_m'] = None
        rec['fc_advance_m'] = None
        rec['est_err_lower_bound_m'] = round(max(0.0, total - MAX_LEAD_M), 3)
        rec['derived_from_old_log'] = True
        if rec['vel'] is not None:
            t_guess = min(rec['dist'] / NET_VH_MPS, FLIGHT_TIMEOUT_S)
            lx, ly = rec['vel'][0] * t_guess, rec['vel'][1] * t_guess
            ln = math.hypot(lx, ly)
            if ln > MAX_LEAD_M and ln > 1e-9:
                lx, ly = lx * MAX_LEAD_M / ln, ly * MAX_LEAD_M / ln
            rec['implied_fc_advance_m'] = round(math.hypot(lx, ly), 3)
            rec['implied_est'] = (round(px - lx, 3), round(py - ly, 3))
            rec['implied_est_err_m'] = round(
                hdist(px - lx, py - ly, TARGET_XY[0], TARGET_XY[1]), 3)

    # 高层 lead：取最近的前置 HL_DIAG（同一 ROS 时基），与火控 est 同拍核对。
    h = nearest_hl(hls, rec.get('arm_stamp'))
    rec['hl_matched'] = h is not None
    if h is not None:
        hx, hy = h['est']
        rec['hl_est'] = (hx, hy)
        rec['hl_lead_m'] = round(hdist(h['goal'][0], h['goal'][1], hx, hy), 3)
        rec['hl_est_err_m'] = round(hdist(hx, hy, TARGET_XY[0], TARGET_XY[1]), 3)
        rec['hl_target_id'] = h['target_id']
        rec['hl_n_tracks'] = h['n_tracks']
        rec['hl_conf'] = h['conf']
        # 新息代理：最近一次 HL est 相对前一 HL 样本的跳变（1 Hz；非 TargetMotionFilter 原始新息）。
        prev = None
        for x in hls:
            if x['stamp'] is None or x['stamp'] >= h['stamp'] - 1e-9:
                break
            prev = x
        rec['hl_est_jump_m'] = (round(hdist(hx, hy, prev['est'][0], prev['est'][1]), 3)
                                if prev is not None else None)
        if rec['est'] is not None:
            rec['hl_vs_fc_est_m'] = round(hdist(hx, hy, rec['est'][0], rec['est'][1]), 3)
    else:
        rec['hl_lead_m'] = None
        rec['hl_est_err_m'] = None
        rec['hl_target_id'] = None
        rec['hl_est_jump_m'] = None

    # 飞行 / 时序：predictTarget 的到达时间 vs 实际飞行。
    if rec['t_est'] is not None and rec['flight'] is not None:
        rec['flight_minus_t_est_s'] = round(rec['flight'] - rec['t_est'], 3)
    else:
        rec['flight_minus_t_est_s'] = None
    rec['t_est_guess_s'] = round(min(rec['dist'] / NET_VH_MPS, FLIGHT_TIMEOUT_S), 3)
    rec['flight_over_timeout'] = bool(rec['flight'] is not None and rec['flight'] > FLIGHT_TIMEOUT_S + 0.5)
    return rec


def percentile(vals, q):
    if not vals:
        return None
    s = sorted(vals)
    k = (len(s) - 1) * q
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return round(s[lo], 3)
    return round(s[lo] + (s[hi] - s[lo]) * (k - lo), 3)


def mean(vals):
    vals = [v for v in vals if v is not None]
    return round(sum(vals) / len(vals), 3) if vals else None


def summarize(shots):
    n = len(shots)
    outcomes = Counter(s['outcome'] or 'no_result' for s in shots)
    pred = [s['pred_err_m'] for s in shots]
    est_new = [s['est_err_m'] for s in shots if s['est_err_m'] is not None]
    est_lb = [s['est_err_lower_bound_m'] for s in shots]
    adv_new = [s['fc_advance_m'] for s in shots if s['fc_advance_m'] is not None]
    implied_adv = [s.get('implied_fc_advance_m') for s in shots]
    implied_est = [s.get('implied_est_err_m') for s in shots]
    hl_lead = [s['hl_lead_m'] for s in shots if s['hl_lead_m'] is not None]
    hl_est = [s['hl_est_err_m'] for s in shots if s['hl_est_err_m'] is not None]
    hl_vs_fc = [s['hl_vs_fc_est_m'] for s in shots if s.get('hl_vs_fc_est_m') is not None]
    hl_jump = [s['hl_est_jump_m'] for s in shots if s.get('hl_est_jump_m') is not None]
    ages = [s['age'] for s in shots if s['age'] is not None]
    confs = [s['conf'] for s in shots if s['conf'] is not None]
    speeds = [math.hypot(s['vel'][0], s['vel'][1]) for s in shots if s['vel'] is not None]
    flights = [s['flight'] for s in shots if s['flight'] is not None]
    miss = [s['miss_dist'] for s in shots if s['miss_dist'] is not None]
    return {
        'n_shots': n,
        'outcomes': dict(outcomes),
        'pred_err_mean_m': mean(pred),
        'pred_err_median_m': percentile(pred, 0.5),
        'pred_err_p90_m': percentile(pred, 0.9),
        'pred_err_max_m': round(max(pred), 3) if pred else None,
        'pred_gt_net_radius_ratio': round(sum(1 for v in pred if v > NET_RADIUS_M) / n, 4) if n else None,
        'pred_gt_net_radius_count': sum(1 for v in pred if v > NET_RADIUS_M),
        'pred_gt_1m_ratio': round(sum(1 for v in pred if v > 1.0) / n, 4) if n else None,
        'est_err_mean_m': mean(est_new),
        'est_err_max_m': round(max(est_new), 3) if est_new else None,
        'est_err_lower_bound_mean_m': mean(est_lb),
        'est_err_lower_bound_max_m': round(max(est_lb), 3) if est_lb else None,
        'fc_advance_mean_m': mean(adv_new),
        'fc_advance_max_m': round(max(adv_new), 3) if adv_new else None,
        'implied_fc_advance_mean_m': mean(implied_adv),
        'implied_est_err_mean_m': mean(implied_est),
        'hl_lead_mean_m': mean(hl_lead),
        'hl_lead_max_m': round(max(hl_lead), 3) if hl_lead else None,
        'hl_est_err_mean_m': mean(hl_est),
        'hl_vs_fc_est_mean_m': mean(hl_vs_fc),
        'hl_est_jump_mean_m': mean(hl_jump),
        'hl_est_jump_max_m': round(max(hl_jump), 3) if hl_jump else None,
        'age_mean_s': mean(ages),
        'age_max_s': max(ages) if ages else None,
        'conf_mean': mean(confs),
        'conf_min': min(confs) if confs else None,
        'speed_mean_mps': mean(speeds),
        'speed_max_mps': round(max(speeds), 3) if speeds else None,
        'flight_mean_s': mean(flights),
        'flight_max_s': round(max(flights), 3) if flights else None,
        'flight_over_timeout_count': sum(1 for v in flights if v > FLIGHT_TIMEOUT_S + 0.5),
        'flight_over_timeout_shots': [
            {'shot_id': s['shot_id'], 'seed': s.get('seed'), 'flight_s': s['flight']}
            for s in shots if s['flight'] is not None and s['flight'] > FLIGHT_TIMEOUT_S + 0.5],
        'miss_dist_mean_m': mean(miss),
        'miss_dist_median_m': percentile(miss, 0.5),
        'miss_dist_max_m': round(max(miss), 3) if miss else None,
        'miss_dist_gt_10m_count': sum(1 for v in miss if v > 10.0),
        'fields_present': {
            'est_t_est_target_id_conf_age': bool(est_new),
            'hl_diag': bool(hl_lead),
            'old_log_inference': any(s['derived_from_old_log'] for s in shots),
        },
    }


def track_consistency(shots, hls):
    """高层（最高置信度）与火控 front() 的一致性 + 与离群 pred 的关联。"""
    have_shot_fields = any(s['target_id'] is not None for s in shots)
    mism = [s for s in shots if s['front_is_max_conf'] is False]
    id_mism = [s for s in shots if s['target_id'] and s['max_conf_target_id']
               and s['target_id'] != s['max_conf_target_id']]
    hl_ids = [h['target_id'] for h in hls]
    hl_switches = sum(1 for a, b in zip(hl_ids, hl_ids[1:]) if a != b)
    # 高层 vs 火控 target_id（逐 shot 最近 HL 样本）
    cross = []
    for s in shots:
        if s.get('hl_target_id') and s['target_id']:
            cross.append(s['hl_target_id'] != s['target_id'])
    out = {
        'evidence': 'available' if (have_shot_fields and hls) else 'insufficient',
        'n_tracks_mean': mean([s['n_tracks'] for s in shots if s['n_tracks'] is not None]),
        'front_is_max_conf_false_count': len(mism),
        'front_vs_maxconf_target_id_mismatch_count_at_shot': len(id_mism),
        'hl_target_id_switches': hl_switches,
        'hl_vs_fc_target_id_mismatch_count_at_shot': sum(cross),
        'n_hl_samples': len(hls),
    }
    if mism:
        out['front_is_max_conf_false_shots'] = [
            {'shot_id': s['shot_id'], 'seed': s.get('seed'), 'pred_err_m': s['pred_err_m']}
            for s in mism]
    if not have_shot_fields or not hls:
        out['note'] = ('旧日志无 target_id/front_is_maxconf/HL_DIAG 字段；'
                       '航迹选择一致性**证据不足**（不得据此断言 front() 是离群主因）。')
    return out


def gate_evaluation(shots):
    """运行时一致性门（只离线评估，不实现）：只用运行时可得信号；真值仅作标签。"""
    sig = []
    for s in shots:
        if s['est'] is None and s.get('hl_est') is None:
            continue
        sig.append({
            'shot_id': s['shot_id'], 'seed': s.get('seed'),
            'label_pred_outlier': s['pred_err_m'] > NET_RADIUS_M,
            'label_hit': s['outcome'] == 'hit',
            'age_s': s['age'], 'conf': s['conf'],
            'speed_mps': round(math.hypot(s['vel'][0], s['vel'][1]), 3) if s['vel'] else None,
            'hl_vs_fc_est_m': s.get('hl_vs_fc_est_m'),
            'hl_est_jump_m': s.get('hl_est_jump_m'),
            'pred_err_m': s['pred_err_m'],
        })
    if not sig:
        return {
            'evidence': 'insufficient',
            'note': ('旧日志无 age/conf/Age/HL_DIAG；且现有发射门已要求 '
                     'age≤0.5 s、conf≥0.5、settled=true——离群射击**已经通过这些门**，'
                     '说明“年龄 / 置信度 / 就绪”类门不能解释本轮离群。'),
        }
    # 候选门（仅为离线评估；阈值取运行时信息的保守值）
    cands = {
        'age_le_0.25s': lambda r: r['age_s'] is not None and r['age_s'] <= 0.25,
        'conf_ge_0.8': lambda r: r['conf'] is not None and r['conf'] >= 0.8,
        'speed_le_0.5mps': lambda r: r['speed_mps'] is not None and r['speed_mps'] <= 0.5,
        'hl_vs_fc_est_le_0.5m': lambda r: r['hl_vs_fc_est_m'] is not None and r['hl_vs_fc_est_m'] <= 0.5,
        'hl_est_jump_le_1.0m': lambda r: r['hl_est_jump_m'] is not None and r['hl_est_jump_m'] <= 1.0,
        'hl_est_jump_le_0.5m': lambda r: r['hl_est_jump_m'] is not None and r['hl_est_jump_m'] <= 0.5,
    }
    out = {'evidence': 'available', 'n_shots_with_signals': len(sig), 'candidates': {}}
    for name, fn in cands.items():
        passed = [r for r in sig if fn(r)]
        blocked = [r for r in sig if not fn(r)]
        out['candidates'][name] = {
            'pass_count': len(passed), 'block_count': len(blocked),
            'blocked_outlier_count': sum(1 for r in blocked if r['label_pred_outlier']),
            'blocked_hit_count': sum(1 for r in blocked if r['label_hit']),
            'pass_outlier_ratio': (round(sum(1 for r in passed if r['label_pred_outlier']) / len(passed), 4)
                                   if passed else None),
        }
    out['note'] = ('门只使用运行时信息；真值只作离线标签。`TargetMotionFilter::settled()` '
                   '是发射前置条件（全部 shot 均已 settled）；`新息/gate_m` 未直接日志化，'
                   '`hl_est_jump_m` 只是 1 Hz 高层滤波位置的逐秒跳变**代理**（非原始新息）。'
                   '本表不据此实现任何门。')
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--arms', default='p2_rl_high')
    args = ap.parse_args()
    run_dir = os.path.join(ROOT, 'experiments', args.campaign, args.run_id)
    arms = tuple(a.strip() for a in args.arms.split(',') if a.strip())
    rows = load_arms(run_dir, arms)

    all_shots, per_episode, all_hl = [], [], []
    for r in rows:
        path = os.path.join(run_dir, 'log', log_name(r))
        order, shots, _results, hls = parse_log(path)
        all_hl.extend(hls)
        enriched = []
        for sid in order:
            s = dict(shots[sid], arm=r['arm'], seed=r['seed'])
            s = enrich(s, hls)
            enriched.append(s)
        all_shots.extend(enriched)
        per_episode.append({
            'arm': r['arm'], 'seed': r['seed'], 'log': os.path.basename(path),
            'terminal': r['terminal_state'], 'reason': r.get('reason_code'),
            'n_shots': len(enriched), 'n_hl_samples': len(hls),
            'shots': enriched,
        })

    summary = summarize(all_shots)
    consistency = track_consistency(all_shots, all_hl)
    gate = gate_evaluation(all_shots)

    outliers = sorted([s for s in all_shots if s['pred_err_m'] > NET_RADIUS_M],
                      key=lambda s: s['pred_err_m'], reverse=True)
    top = outliers[0] if outliers else None
    outlier_report = None
    if top:
        seg = {
            'est_err_m': top['est_err_m'],
            'est_err_lower_bound_m': top['est_err_lower_bound_m'],
            'implied_est_err_m': top.get('implied_est_err_m'),
            'fc_advance_m': top['fc_advance_m'],
            'implied_fc_advance_m': top.get('implied_fc_advance_m'),
            'hl_lead_m': top['hl_lead_m'],
            'flight_minus_t_est_s': top['flight_minus_t_est_s'],
        }
        # 主要来源判定（证据足够时直接比；旧日志用上界 + 推断）
        if top['est_err_m'] is not None:
            main_seg = '目标估计' if (top['est_err_m'] or 0) >= (top['fc_advance_m'] or 0) else '火控提前量'
        elif top['est_err_lower_bound_m'] > NET_RADIUS_M:
            main_seg = '目标估计（旧日志上界推理：est_err ≥ total − max_lead_m = %.2f m）' % \
                       top['est_err_lower_bound_m']
        else:
            main_seg = '证据不足（旧日志无法分离估计项与提前量项）'
        outlier_report = {
            'shot': {k: top.get(k) for k in (
                'shot_id', 'seed', 'arm', 't', 'dist', 'pred', 'est', 'vel', 't_est',
                'target_id', 'conf', 'age', 'outcome', 'flight', 'miss_dist', 'pred_err_m',
                'est_err_m', 'fc_advance_m', 'est_err_lower_bound_m', 'implied_est_err_m',
                'implied_fc_advance_m', 'hl_lead_m', 'flight_minus_t_est_s')},
            'segments': seg, 'main_segment': main_seg,
            'reading': ('pred 来自 capture_adjudicator 的射击预测（滤波后 est + 有上限的提前量），'
                        '**不**直接归因于高层 lead；高层 lead 只在 HL_DIAG 可得时单独核对。'),
        }

    out = {
        'kind': 'prediction_chain_diagnosis（W7-b T3 / ADR-0087(d)；不改 verdict）',
        'run_id': args.run_id, 'campaign': args.campaign, 'scene': SCENE,
        'target_truth_xy': list(TARGET_XY),
        'net_radius_m': NET_RADIUS_M, 'max_lead_m': MAX_LEAD_M,
        'net_horizontal_speed_mps': round(NET_VH_MPS, 4),
        'note': ('真值仅用于离线评估，不进入运行时门；`pred` 是火控射击预测，'
                 '不是高层发布 goal 的直接结果。旧日志缺 est/t_est/HL_DIAG 时照实标注。'),
        'summary': summary, 'track_consistency': consistency,
        'runtime_gate_evaluation': gate, 'outlier': outlier_report,
        'episodes': per_episode,
    }

    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)
    with open(os.path.join(run_dir, 'metrics', 'p2_prediction_chain.json'), 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    md = ['# P2 预测链分解 · %s' % args.run_id, '',
          '> 诊断（W7-b T3 / ADR-0087(d)）：**不改** P2 verdict；真值只用于离线评估。',
          '> `pred` = `capture_adjudicator` 的**射击预测**，不直接归因于高层 lead。', '',
          '## 口径', '',
          '- 目标真值（s08 静止）：`(4.5, 3.0)`（水平；只评估）。网有效半径 %.2f m。' % NET_RADIUS_M,
          '- 分量：目标估计 `est−truth`；火控提前量 `pred−est`；高层 lead `goal−HL est`；'
          '时序 `t_est` / 实际 `flight`。',
          '- 字段可得性：est/t_est/target_id/conf/age=%s；HL_DIAG=%s；旧日志推断=%s。' % (
              summary['fields_present']['est_t_est_target_id_conf_age'],
              summary['fields_present']['hl_diag'],
              summary['fields_present']['old_log_inference']),
          '']
    if not summary['fields_present']['est_t_est_target_id_conf_age']:
        md += ['> **字段缺失**：本 run 的 `SHOT_FIRED` 无 `est/t_est/target_id/conf/age`，'
               '`HL_DIAG` 亦缺（旧日志）。估计项只能给**上界**（`est_err ≥ pred_err − max_lead_m`）'
               '或由 `vel` 推断；**不能**据此做证据级因果结论。', '']
    md += ['## 分量汇总', '',
           '| 量 | 值 |', '|---|---|',
           '| 射击数 / 结局 | %d / %s |' % (summary['n_shots'], summary['outcomes']),
           '| `pred` 误差 均值/中位/p90/max (m) | %s / %s / %s / %s |' % (
               summary['pred_err_mean_m'], summary['pred_err_median_m'],
               summary['pred_err_p90_m'], summary['pred_err_max_m']),
           '| `pred` > 网半径 占比 | %s (%s/%d) |' % (
               summary['pred_gt_net_radius_ratio'], summary['pred_gt_net_radius_count'],
               summary['n_shots']),
           '| `pred` > 1 m 占比 | %s |' % summary['pred_gt_1m_ratio'],
           '| 目标估计误差 均值/最大 (m) | %s / %s |' % (
               summary['est_err_mean_m'], summary['est_err_max_m']),
           '| 估计误差**下界** 均值/最大 (m) | %s / %s |' % (
               summary['est_err_lower_bound_mean_m'], summary['est_err_lower_bound_max_m']),
           '| 火控提前量 均值/最大 (m) | %s / %s |' % (
               summary['fc_advance_mean_m'], summary['fc_advance_max_m']),
           '| （旧日志推断）提前量 / 估计误差 均值 (m) | %s / %s |' % (
               summary['implied_fc_advance_mean_m'], summary['implied_est_err_mean_m']),
           '| 高层 lead 均值/最大 (m) | %s / %s |' % (
               summary['hl_lead_mean_m'], summary['hl_lead_max_m']),
           '| 高层 est 误差 均值 / 高层 est vs 火控 est 均值 (m) | %s / %s |' % (
               summary['hl_est_err_mean_m'], summary['hl_vs_fc_est_mean_m']),
           '| 高层 est 逐秒跳变 均值/最大 (m，新息代理) | %s / %s |' % (
               summary['hl_est_jump_mean_m'], summary['hl_est_jump_max_m']),
           '| 观测年龄 均值/最大 (s) | %s / %s |' % (
               summary['age_mean_s'], summary['age_max_s']),
           '| 置信度 均值/最小 | %s / %s |' % (summary['conf_mean'], summary['conf_min']),
           '| 滤波速度 均值/最大 (m/s) | %s / %s |' % (
               summary['speed_mean_mps'], summary['speed_max_mps']),
           '| 飞行时长 均值/最大 (s)；超时(>%.1fs)射击数 | %s / %s；%d |' % (
               FLIGHT_TIMEOUT_S, summary['flight_mean_s'], summary['flight_max_s'],
               summary['flight_over_timeout_count']),
           '| miss_dist 均值/中位/最大 (m)；>10 m 射击数 | %s / %s / %s；%d |' % (
               summary['miss_dist_mean_m'], summary['miss_dist_median_m'],
               summary['miss_dist_max_m'], summary['miss_dist_gt_10m_count']),
           '']
    if summary['flight_over_timeout_shots']:
        md += ['> **时序异常**：下列射击的 `flight` 超过 `net_flight_timeout_s=%.1f s`——'
               '目标估计短时跳出适用高度/有效位姿时，`step()` 在“在飞”分支之前提前返回，'
               '超时判定未执行（状态机时序，非弹道）。%s'
               % (FLIGHT_TIMEOUT_S, ', '.join(
                   '`%s`(seed %s, %.1f s)' % (s['shot_id'], s['seed'], s['flight_s'])
                   for s in summary['flight_over_timeout_shots'])), '']
    md += ['## 逐 shot 表', '',
           '| seed | shot | t (s) | h | pred 误差 | 估计误差 | 估计下界 | 提前量 | 高层 lead | t_est | flight | flight−t_est | miss_dist | 结局 |',
           '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for s in sorted(all_shots, key=lambda x: (x['seed'], x['t'])):
        md.append('| %d | %s | %.3f | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
            s['seed'], s['shot_id'], s['t'], _f(s['h']), s['pred_err_m'],
            _f(s['est_err_m']), _f(s['est_err_lower_bound_m']), _f(s['fc_advance_m']),
            _f(s['hl_lead_m']), _f(s['t_est']), _f(s['flight']),
            _f(s['flight_minus_t_est_s']), _f(s['miss_dist']),
            s['outcome'] or 'no_result'))
    if summary['fields_present']['old_log_inference']:
        md += ['', '> 估计误差 / 提前量在旧日志为 `—`（`est` 缺失）；估计下界 = max(0, pred误差 − %.1f)。' % MAX_LEAD_M,
               '']
    else:
        md += ['', '> 本 run 字段完整：估计误差 = `est−truth`，火控提前量 = `pred−est`，高层 lead 取最近前置 `HL_DIAG`。', '']
    if outlier_report:
        md += ['## 最大离群点分段解释', '',
               '- shot `%s`（seed %s）：`pred` 误差 **%.3f m**，结局 `%s`，`miss_dist=%s`，`flight=%s`。' % (
                   outlier_report['shot']['shot_id'], outlier_report['shot']['seed'],
                   outlier_report['shot']['pred_err_m'], outlier_report['shot']['outcome'],
                   _f(outlier_report['shot']['miss_dist']), _f(outlier_report['shot']['flight'])),
               '- 分段：估计误差 `%s`（下界 `%s`，旧日志推断 `%s`）、火控提前量 `%s`、高层 lead `%s`、'
               '`flight − t_est=%s`。' % (
                   _f(outlier_report['segments']['est_err_m']),
                   _f(outlier_report['segments']['est_err_lower_bound_m']),
                   _f(outlier_report['segments']['implied_est_err_m']),
                   _f(outlier_report['segments']['fc_advance_m']),
                   _f(outlier_report['segments']['hl_lead_m']),
                   _f(outlier_report['segments']['flight_minus_t_est_s'])),
               '- **主要来源**：%s。' % outlier_report['main_segment'],
               '- 读法：%s' % outlier_report['reading'], '']
    md += ['## 航迹选择一致性', '',
           '- 证据：`%s`；火控 `front()` 非最高置信度次数 **%d**；'
           '火控 front 与 max-conf `target_id` 不一致次数 **%d**；'
           '高层 target_id 切换 **%d** 次；高层 vs 火控 `target_id` 不一致 **%d** 次。' % (
               consistency['evidence'], consistency['front_is_max_conf_false_count'],
               consistency['front_vs_maxconf_target_id_mismatch_count_at_shot'],
               consistency['hl_target_id_switches'],
               consistency['hl_vs_fc_target_id_mismatch_count_at_shot']),
           '- 平均航迹数：%s。' % _f(consistency['n_tracks_mean'])]
    if consistency.get('note'):
        md += ['- %s' % consistency['note']]
    md += ['', '## 运行时一致性门（只评估，不实现）', '']
    if gate['evidence'] == 'insufficient':
        md += ['- 证据不足：%s' % gate['note'].replace('age/conf/Age/HL_DIAG', 'age/conf/settled/HL_DIAG')]
    else:
        md += ['- 候选门离线通过/拦截（真值只作标签）：', '',
               '| 候选门 | 通过 | 拦截 | 拦截中的离群数 | 拦截中的命中数 | 通过中离群占比 |',
               '|---|---|---|---|---|---|']
        for name, c in gate['candidates'].items():
            md.append('| %s | %d | %d | %d | %d | %s |' % (
                name, c['pass_count'], c['block_count'], c['blocked_outlier_count'],
                c['blocked_hit_count'], c['pass_outlier_ratio']))
        md += ['', '- %s' % gate['note']]
    md += ['', '## 结论边界', '',
           '- 本报告只做**预测链分解**；不据此重判 P2。',
           '- 旧日志分量缺失时结论以**上界**给出；上界不足以定位时明确写“证据不足”。',
           '- 若建议加门：须独立 ADR + 回归，且**不**使用真值。', '']

    with open(os.path.join(run_dir, 'metrics', 'p2_prediction_chain.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')
    print('-> metrics/p2_prediction_chain.{json,md}')
    print('n_shots=%d outliers=%d max=%.3f' % (
        summary['n_shots'], len(outliers), summary['pred_err_max_m'] or 0.0))
    return 0


def _f(v):
    return '—' if v is None else ('%.3f' % v if isinstance(v, float) else str(v))


if __name__ == '__main__':
    sys.exit(main())
