#!/usr/bin/env python3
"""w4_perception_check —— W4 验收：话题频率（±容差）+ 帧名（spec R9）。

用法：
  ros2 run（或 python3）scripts/w4_perception_check.py --duration 8 --tol 0.10

检查六话题频率（air/obs 20、targets 10、obstacles 10、ground/odom 30、map 2、gt/* 30）
与 header.frame_id（须 world、无裸 base_link）。结果打印并落
experiments/w4-perception/<run_id>/metrics/frequency.json（ADR-0007）。
"""
import argparse
import json
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from agc_msgs.msg import AirObservationArray, TargetTrackArray, ObstacleTrackArray

# (topic, type, expected_hz, lenient)  lenient=True 表示非 W4 交付物（仅观测）
TOPICS = [
    ('/agc/air/obs', AirObservationArray, 20.0, False),
    ('/agc/targets', TargetTrackArray, 10.0, False),
    ('/agc/obstacles', ObstacleTrackArray, 10.0, False),
    ('/agc/ground/odom', Odometry, 30.0, False),
    ('/agc/map', OccupancyGrid, 2.0, False),
    ('/agc/gt/intruder_uav', PoseStamped, 30.0, False),
]


class Checker(Node):
    def __init__(self):
        super().__init__('w4_perception_check')
        self.counts = {t[0]: 0 for t in TOPICS}
        self.frames = {}
        for topic, typ, _, _ in TOPICS:
            self.create_subscription(typ, topic, self._mk(topic), 50)

    def _mk(self, topic):
        def cb(msg):
            self.counts[topic] += 1
            if topic not in self.frames:
                self.frames[topic] = getattr(msg.header, 'frame_id', '')
        return cb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=8.0)
    ap.add_argument('--warmup', type=float, default=3.0, help='发现/稳态预热（不计入）')
    ap.add_argument('--tol', type=float, default=0.10)
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()

    rclpy.init()
    node = Checker()
    # 预热：等发现完成与稳态（这一时期的消息不计入频率）
    tw = time.monotonic()
    while time.monotonic() - tw < args.warmup:
        rclpy.spin_once(node, timeout_sec=0.1)
    for k in node.counts:
        node.counts[k] = 0
    node.frames.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    dur = time.monotonic() - t0

    results = []
    all_ok = True
    for topic, _, exp, lenient in TOPICS:
        rate = node.counts[topic] / dur
        within = abs(rate - exp) <= args.tol * exp
        frame = node.frames.get(topic, '')
        frame_ok = (frame == 'world')
        ok = within and frame_ok
        if not lenient:
            all_ok = all_ok and ok
        results.append({'topic': topic, 'expected_hz': exp, 'measured_hz': round(rate, 3),
                        'within_tol': within, 'frame_id': frame, 'frame_ok': frame_ok, 'ok': ok})
        print(f"{topic:26s} {rate:7.3f} Hz (exp {exp:5.1f} ±{int(args.tol*100)}%) "
              f"frame='{frame}' {'OK' if ok else 'FAIL'}")
    node.destroy_node()
    rclpy.shutdown()

    run_id = args.run_id or datetime.now().strftime('%Y%m%d-%H%M%S')
    out_dir = os.path.join(args.experiments_root, 'w4-perception', run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'frequency.json')
    with open(path, 'w') as f:
        json.dump({'run_id': run_id, 'duration_s': round(dur, 2), 'tol': args.tol,
                   'results': results, 'pass': all_ok}, f, indent=2)
    print(f"\n{'PASS' if all_ok else 'FAIL'} → {path}")
    return 0 if all_ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
