#!/usr/bin/env python3
"""w5_scan_self_filter_check —— R-007 节点级确定性测（scan_self_filter）。

起 `agc_ground/scan_self_filter` 节点，喂合成 LaserScan（含"车体区自返回" + "外部障碍返回"），
断言：车体矩形内返回被置 +inf（自滤）、外部障碍返回与 header 原样保留。

用法：python3 scripts/w5_scan_self_filter_check.py [--run-id ...]
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan


def qos_sensor():
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.durability = DurabilityPolicy.VOLATILE
    return q


class Probe(Node):
    def __init__(self):
        super().__init__('scan_self_filter_probe')
        self.get = None
        self.pub = self.create_publisher(LaserScan, '/agc/ground/scan', qos_sensor())
        self.create_subscription(
            LaserScan, '/agc/ground/scan_filtered', self._on_f, qos_sensor())

    def _on_f(self, m):
        self.get = m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261015-w5-r007-0')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()

    rclpy.init()
    probe = Probe()

    n = 360
    inc = 2.0 * math.pi / n
    amin = -math.pi
    ranges = [float('inf')] * n
    stamp = time.time()
    # 车体矩形内自返回（应被置 inf）
    self_idx = {180: 0.45, 181: 0.52, 270: 0.30, 90: 0.28, 0: 0.50, 359: 0.52}
    # 外部障碍返回（应保留）
    obst_idx = {200: 3.00, 160: 0.70, 140: 2.00, 250: 1.50}
    for i, r in self_idx.items():
        ranges[i] = r
    for i, r in obst_idx.items():
        ranges[i] = r

    scan = LaserScan()
    scan.header.stamp.sec = int(stamp)
    scan.header.stamp.nanosec = int((stamp % 1) * 1e9)
    scan.header.frame_id = 'car/lidar_link'
    scan.angle_min = amin
    scan.angle_max = amin + (n - 1) * inc
    scan.angle_increment = inc
    scan.range_min = 0.2
    scan.range_max = 20.0
    scan.ranges = ranges

    # 发布到就绪（等订阅连接）
    t0 = time.monotonic()
    got = None
    while time.monotonic() - t0 < 8.0:
        probe.pub.publish(scan)
        rclpy.spin_once(probe, timeout_sec=0.05)
        if probe.get is not None:
            got = probe.get
            if got.header.frame_id == 'car/lidar_link':
                break

    result = {'run_id': args.run_id}
    checks = []
    if got is None:
        result['pass'] = False
        result['error'] = 'no filtered scan received'
    else:
        for i in self_idx:
            ok = not math.isfinite(got.ranges[i])
            checks.append({'beam': i, 'kind': 'self', 'expect': 'inf',
                           'value': got.ranges[i], 'ok': bool(ok)})
        for i, r in obst_idx.items():
            ok = math.isclose(got.ranges[i], r, rel_tol=1e-5)
            checks.append({'beam': i, 'kind': 'obstacle', 'expect': r,
                           'value': got.ranges[i], 'ok': bool(ok)})
        header_ok = (got.header.frame_id == scan.header.frame_id
                     and got.header.stamp.sec == scan.header.stamp.sec
                     and len(got.ranges) == n)
        result.update({
            'all_pass': all(c['ok'] for c in checks),
            'header_preserved': header_ok,
            'checks': checks,
        })
        result['pass'] = bool(result['all_pass'] and header_ok)

    out_dir = os.path.join(args.experiments_root, 'w5-ground-nav-safety',
                           args.run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'scan_self_filter.json')
    with open(path, 'w') as fh:
        json.dump(result, fh, indent=2)

    for c in checks:
        print('beam %3d %-8s value=%s expect=%s %s'
              % (c['beam'], c['kind'], c['value'], c['expect'], 'OK' if c['ok'] else 'FAIL'))
    print('header_preserved=%s' % result.get('header_preserved'))
    print('\n%s -> %s' % ('PASS' if result['pass'] else 'FAIL', path))

    probe.destroy_node()
    rclpy.shutdown()
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
