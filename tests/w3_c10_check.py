#!/usr/bin/env python3
"""w3_c10_check —— W3 C10 联调录制与判据（spec R6/R7，契约 #hard-constraints C10）

订阅归一化 `/agc/air/odom`、`/agc/targets`、`/agc/air/tracking_ref`、`/agc/air/control_conflict`，
按**相位**录制并联立判据（阈值一律从 `--params`（`config/air.yaml`）读取，不复制参数；
静态障碍**几何**从单一真源 `agc_sim/config/world_geometry.yaml` 读取，见 S-06 / ADR-0062）：

  | 相位     | 判据                                                              |
  |----------|-------------------------------------------------------------------|
  | `circle` | 跟踪常开（`tracking_ref` 连续、无长间断）+ 水平距离 ≥ `safe_min` + 无 `control_conflict` |
  | `pole`   | 全程不侵入 `pole_1`/`pole_2` 安全半径（`obstacle_safety_radius`）  |
  | `freeze` | 目标过期（`ref_max_age_s`）后**冻结不追**：末端位移 ≤ 阈值，且 `tracking_ref` 仍在发布 |

用法（落点须遵 ADR-0007）：
  python3 tests/w3_c10_check.py --phase circle --duration 60 \\
      --out experiments/w3-air-link/<run_id>/metrics/c10_p1_circle.csv
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from trajectory_msgs.msg import MultiDOFJointTrajectory

from agc_msgs.msg import TargetTrackArray

LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def load_air_params(path: str, geometry_path: str) -> dict:
    """阈值从 air.yaml 读（不复制）；静态障碍**几何**从单一真源 world_geometry.yaml 读
    （S-06 / ADR-0062：几何已上移，air.yaml 不再硬编码障碍坐标）。"""
    with open(path) as f:
        doc = yaml.safe_load(f)
    follower = doc['standoff_follower']['ros__parameters']
    safety = doc['trajectory_tracker']['ros__parameters']['safety']
    with open(geometry_path) as f:
        geom = yaml.safe_load(f)
    radii = list(safety['obstacle_safety_radius'])
    poles = [(o['id'], float(o['x']), float(o['y']), float(radii[i]))
             for i, o in enumerate(geom['obstacles'])]
    return {
        'safe_min': float(follower['safe_min']),
        'safe_max': float(follower['safe_max']),
        'ref_max_age_s': float(follower['ref_max_age_s']),
        'poles': poles,
    }


def _body_to_world(q, v):
    """机体系 FLU → 世界 ENU（R-003）：`/agc/air/odom` 的 twist 在 child frame（机体系），
    而本 CSV 口径为世界 ENU，故用姿态四元数旋转回去（v' = R(q)·v）。"""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    qv = (qx, qy, qz)
    vi = (v.x, v.y, v.z)
    t = cross(qv, vi)
    t2 = cross(qv, t)
    out = type(v)()
    out.x, out.y, out.z = (vi[k] + 2.0 * (qw * t[k] + t2[k]) for k in range(3))
    return out


class C10Checker(Node):
    def __init__(self, topic_prefix: str, out: str):
        super().__init__('w3_c10_check')
        self.out = out
        self.rows: list[dict] = []
        self.t0 = None
        self.drone = None
        self.target = None
        self.target_t = None          # 最近一次目标观测的本地单调时刻
        self.ref = None               # 最近一次 tracking_ref 的 XY（跟踪误差用）
        self.ref_t = None             # 最近一次 tracking_ref 的本地单调时刻
        self.ref_count = 0
        self.gap_max = 0.0
        self.conflict = False

        self.create_subscription(Odometry, f'{topic_prefix}/odom', self.on_odom, 20)
        self.create_subscription(TargetTrackArray, '/agc/targets', self.on_target, 10)
        self.create_subscription(MultiDOFJointTrajectory, f'{topic_prefix}/tracking_ref',
                                 self.on_ref, 20)
        self.create_subscription(Bool, f'{topic_prefix}/control_conflict', self.on_conflict,
                                 LATCHED)

    def on_target(self, msg: TargetTrackArray) -> None:
        if not msg.tracks:
            return
        p = msg.tracks[0].pose.pose.position
        self.target = (p.x, p.y, p.z)
        self.target_t = time.monotonic()

    def on_ref(self, msg: MultiDOFJointTrajectory) -> None:
        t = time.monotonic()
        if self.ref_t is not None:
            self.gap_max = max(self.gap_max, t - self.ref_t)
        self.ref_t = t
        self.ref_count += 1
        if msg.points and msg.points[0].transforms:
            tr = msg.points[0].transforms[0].translation
            vz = msg.points[0].velocities[0].linear.z if msg.points[0].velocities else float('nan')
            self.ref = (tr.x, tr.y, tr.z, vz)

    def on_conflict(self, msg: Bool) -> None:
        self.conflict = self.conflict or bool(msg.data)

    def on_odom(self, msg: Odometry) -> None:
        t = time.monotonic()
        if self.t0 is None:
            self.t0 = t
        p = msg.pose.pose.position
        v = _body_to_world(msg.pose.pose.orientation, msg.twist.twist.linear)
        self.drone = (p.x, p.y, p.z)
        tgt_age = (t - self.target_t) if self.target_t is not None else float('inf')
        ref_gap = (t - self.ref_t) if self.ref_t is not None else float('inf')
        err = (math.hypot(p.x - self.ref[0], p.y - self.ref[1]) if self.ref else float('nan'))
        self.rows.append({
            't': t - self.t0, 'x': p.x, 'y': p.y, 'z': p.z,
            'vx': v.x, 'vy': v.y,
            'tx': self.target[0] if self.target else float('nan'),
            'ty': self.target[1] if self.target else float('nan'),
            'dist_xy': (math.hypot(p.x - self.target[0], p.y - self.target[1])
                        if self.target else float('nan')),
            'rx': self.ref[0] if self.ref else float('nan'),
            'ry': self.ref[1] if self.ref else float('nan'),
            'rz': self.ref[2] if self.ref else float('nan'),
            'rvz': self.ref[3] if self.ref else float('nan'),
            'track_err': err,
            'tgt_age': tgt_age, 'ref_gap': ref_gap, 'conflict': int(self.conflict),
        })


def judge(args, rows, cfg, ref_count: int) -> bool:
    """按相位判定并打印；返回是否 PASS"""
    warm = [r for r in rows if r['t'] >= args.warmup]
    if len(warm) < 50:
        print(f'[FAIL] 采样不足：{len(warm)}')
        return False

    fresh = [r for r in warm if r['tgt_age'] < cfg['ref_max_age_s']]
    dist = [r['dist_xy'] for r in fresh if not math.isnan(r['dist_xy'])]
    min_dist = min(dist) if dist else float('nan')
    pole_d = {n: min(math.hypot(r['x'] - px, r['y'] - py) for r in warm)
              for n, px, py, _ in cfg['poles']}
    ref_gap = max(r['ref_gap'] for r in warm)
    conflicts = sum(r['conflict'] for r in warm)

    print(f'样本数        : {len(warm)}（{warm[0]["t"]:.1f}s → {warm[-1]["t"]:.1f}s；'
          f'其中目标新鲜 {len(fresh)}）')
    print(f'tracking_ref  : {ref_count} 条，采样窗内最大间断 {ref_gap:.3f} s')
    print(f'水平距离      : min {min_dist:.2f} m（判据 ≥ safe_min = {cfg["safe_min"]:.1f} m）')
    for n, _, _, rs in cfg['poles']:
        print(f'  距 {n:<7} : min {pole_d[n]:.2f} m（安全半径 {rs:.1f} m）')
    print(f'control_conflict 置位数: {conflicts}')
    errs = sorted(r['track_err'] for r in warm if not math.isnan(r['track_err']))
    if errs:
        print(f'跟踪误差 |drone−tracking_ref| 均值/P95/max: '
              f'{sum(errs) / len(errs):.2f} / {errs[int(0.95 * len(errs)) - 1]:.2f} / {errs[-1]:.2f} m')

    if args.phase == 'circle':
        ok = (min_dist >= cfg['safe_min'] - args.tol and
              ref_gap <= args.max_ref_gap and conflicts == 0)
        print(f'判定（circle）：距离 ≥ safe_min−{args.tol} 且 ref 无间断 且 无冲突 → '
              f'{"PASS" if ok else "FAIL"}')
        return ok

    if args.phase == 'pole':
        ok = all(pole_d[n] >= rs - args.tol for n, _, _, rs in cfg['poles'])
        print(f'判定（pole）：不侵入安全半径（容差 {args.tol} m）→ {"PASS" if ok else "FAIL"}')
        return ok

    # freeze：目标过期后机体应停住（hold），且 tracking_ref 仍连续发布
    stale = [r for r in warm if r['tgt_age'] > cfg['ref_max_age_s']]
    if not stale:
        print('判定（freeze）：采样窗内未出现目标过期 → FAIL（未形成冻结用例）')
        return False
    tail = [r for r in warm if r['t'] >= warm[-1]['t'] - args.hold_tail]
    disp = max(math.hypot(r['x'] - tail[0]['x'], r['y'] - tail[0]['y']) for r in tail)
    print(f'目标过期时长  : {stale[-1]["t"] - stale[0]["t"]:.1f} s')
    print(f'末端 {args.hold_tail:.0f} s 机体位移: {disp:.3f} m（判据 ≤ {args.hold_eps} m）')
    ok = disp <= args.hold_eps and ref_gap <= args.max_ref_gap
    print(f'判定（freeze）：冻结不追且 ref 连续 → {"PASS" if ok else "FAIL"}')
    return ok


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--phase', choices=['circle', 'pole', 'freeze'], required=True)
    ap.add_argument('--params', default='src/agc_air/config/air.yaml',
                    help='参数唯一来源（阈值从此读，不复制）')
    ap.add_argument('--geometry', default='src/agc_sim/config/world_geometry.yaml',
                    help='静态障碍几何单一真源（S-06 / ADR-0062；障碍坐标从此读，不复制）')
    ap.add_argument('--prefix', default='/agc/air')
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--warmup', type=float, default=3.0)
    ap.add_argument('--out', default='')
    ap.add_argument('--tol', type=float, default=0.30, help='距离/半径容差 m')
    ap.add_argument('--max-ref-gap', type=float, default=0.30,
                    help='tracking_ref 允许的最大间断 s')
    ap.add_argument('--hold-tail', type=float, default=5.0, help='freeze 位移考察窗 s')
    ap.add_argument('--hold-eps', type=float, default=0.50, help='freeze 允许末端位移 m')
    args = ap.parse_args()

    cfg = load_air_params(args.params, args.geometry)
    print(f'阈值来源 {args.params}: safe_min={cfg["safe_min"]} safe_max={cfg["safe_max"]} '
          f'ref_max_age_s={cfg["ref_max_age_s"]} poles={[p[0] for p in cfg["poles"]]}')

    rclpy.init(args=[])
    node = C10Checker(args.prefix, args.out)
    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    rows = node.rows
    ref_count = node.ref_count
    node.destroy_node()
    rclpy.shutdown()

    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ['t'])
            w.writeheader()
            w.writerows(rows)
        print(f'已写出        : {args.out}')

    sys.exit(0 if judge(args, rows, cfg, ref_count) else 1)


if __name__ == '__main__':
    main()
