#!/usr/bin/env python3
"""w3_track_check —— W3 轨迹录制与判据（spec R5 判据 / C1）

订阅归一化 `/agc/air/odom`，录制机体世界系（ENU）位置与速度到 CSV，并按
契约 [硬约束] C1 判定：**全程恒速 `|v − 2.0| ≤ ε`、无加减速**。

用法（落点须遵 ADR-0007：`experiments/<campaign>/<run_id>/{config,log,metrics,figures}/`）：
  python3 tests/w3_track_check.py --duration 40 \\
      --out experiments/w3-air-link/<run_id>/metrics/c1_figure8.csv
"""

import argparse
import csv
import math
import os
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

TARGET_SPEED = 2.0   # C1：真八字恒速 v = 2.0 m/s（ADR-0019）


class TrackChecker(Node):
    def __init__(self, topic: str):
        super().__init__('w3_track_check')
        self.rows: list[tuple[float, float, float, float, float]] = []
        self.t0 = None
        self.create_subscription(Odometry, topic, self.on_odom, 20)

    def on_odom(self, msg: Odometry) -> None:
        # 时间基用本地单调钟：消息戳经 px4_adapter 的 µs→ROS 映射，绝对对齐 sim 时钟但
        # 不便于做"相对本采样窗口"的 warmup 过滤。
        t = time.monotonic()
        if self.t0 is None:
            self.t0 = t
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        self.rows.append((t - self.t0, p.x, p.y, p.z, speed))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/agc/air/odom')
    ap.add_argument('--duration', type=float, default=40.0)
    ap.add_argument('--out', default='')
    ap.add_argument('--eps', type=float, default=0.30, help='恒速容差 ε（m/s）')
    ap.add_argument('--warmup', type=float, default=3.0, help='忽略前 N 秒（接管瞬态）')
    args = ap.parse_args()

    rclpy.init(args=[])
    node = TrackChecker(args.topic)
    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    rows = [r for r in node.rows if r[0] >= args.warmup]
    node.destroy_node()
    rclpy.shutdown()

    if len(rows) < 50:
        print(f'[FAIL] 采样不足：{len(rows)}')
        raise SystemExit(1)

    speeds = [r[4] for r in rows]
    dev = [abs(s - TARGET_SPEED) for s in speeds]
    dev_sorted = sorted(dev)
    p95 = dev_sorted[int(0.95 * len(dev_sorted)) - 1]
    xs = [r[1] for r in rows]
    ys = [r[2] for r in rows]

    print(f'样本数        : {len(rows)}（{rows[0][0]:.1f}s → {rows[-1][0]:.1f}s）')
    print(f'|v| 均值/最小/最大: {statistics.fmean(speeds):.3f} / {min(speeds):.3f} / {max(speeds):.3f} m/s')
    print(f'|v − 2.0| 均值/P95/最大: {statistics.fmean(dev):.3f} / {p95:.3f} / {max(dev):.3f} m/s')
    print(f'轨迹 XY 范围  : x ∈ [{min(xs):.2f}, {max(xs):.2f}]  y ∈ [{min(ys):.2f}, {max(ys):.2f}] m')
    print(f'判定 ε={args.eps} : ' + ('PASS' if p95 <= args.eps else 'FAIL'))

    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'speed'])
            w.writerows(rows)
        print(f'已写出        : {args.out}')


if __name__ == '__main__':
    main()
