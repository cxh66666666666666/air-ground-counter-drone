#!/usr/bin/env python3
"""w6_full_check —— W6 双 PX4 整链验收（R0 复核 / R4 机体层 / R8 注入）。

在 `w6.launch.py backend:=full` 运行中执行：
  * 采样 `/agc/gt/intruder_uav`（世界真值）→ 机体层辨识目标运动（static/linear/maneuver）；
  * 读 `/px4_1/fmu/out/vehicle_status` → 入侵者 armed + OFFBOARD；
  * 可选 `--jam`：发 `/agc/task/jam_cmd=LAND`，观测 `nav_state` 切换（若目标在作用包络内）。

结果落 `experiments/w6-task-eval/<run_id>/metrics/full.json`。

用法：
  python3 scripts/w6_full_check.py --motion linear --duration 8 --run-id 20261014-w6-full-s02
"""

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus


def _qos():
    q = QoSProfile(depth=5)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    return q


class FullCheck(Node):
    def __init__(self):
        super().__init__('w6_full_check')
        self.samples = []
        self.nav_state = None
        self.armed = None
        self.create_subscription(PoseStamped, '/agc/gt/intruder_uav', self._on_gt, 20)
        self.create_subscription(VehicleStatus, '/px4_1/fmu/out/vehicle_status',
                                 self._on_status, _qos())
        self.jam_pub = self.create_publisher(String, '/agc/task/jam_cmd', 10)

    def _on_gt(self, m):
        self.samples.append((m.pose.position.x, m.pose.position.y, m.pose.position.z))

    def _on_status(self, m):
        self.nav_state = m.nav_state
        self.armed = (m.arming_state == 2)

    def spin_for(self, dur):
        tw = time.monotonic()
        while time.monotonic() - tw < dur:
            rclpy.spin_once(self, timeout_sec=0.1)


def classify(samples):
    if len(samples) < 3:
        return 'none', {}
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    dx = max(xs) - min(xs)
    dy = max(ys) - min(ys)
    if dx < 0.3 and dy < 0.3:
        kind = 'static'
    elif min(dx, dy) < 0.5 and max(dx, dy) > 1.0:
        kind = 'linear'
    elif dx > 1.0 and dy > 1.0:
        kind = 'maneuver'
    else:
        kind = 'unclassified'
    return kind, {'dx': round(dx, 3), 'dy': round(dy, 3), 'n': len(samples)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--motion', required=True, help='场景目标运动（static/linear/maneuver）')
    ap.add_argument('--duration', type=float, default=8.0)
    ap.add_argument('--jam', action='store_true', help='追加发 jam_cmd=LAND 并观测 nav_state 切换')
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-full-0')

    rclpy.init()
    node = FullCheck()
    node.spin_for(args.duration)
    kind, stats = classify(node.samples)
    expected = args.motion
    motion_ok = (kind == expected)
    nav_before = node.nav_state
    armed = node.armed
    print('motion=%s -> classified=%s %s' % (expected, kind, stats))
    print('intruder armed=%s nav_state=%s' % (armed, nav_before))

    jam_result = None
    if args.jam:
        node.jam_pub.publish(String(data='LAND'))
        node.spin_for(4.0)
        jam_result = {'nav_before': nav_before, 'nav_after': node.nav_state}
        print('jam LAND: nav_state %s -> %s' % (nav_before, node.nav_state))

    all_ok = motion_ok and (armed is True) and (nav_before == 14)  # OFFBOARD
    if args.jam:
        all_ok = all_ok and (jam_result['nav_after'] == 18)  # AUTO_LAND

    node.destroy_node()
    rclpy.shutdown()

    payload = {'run_id': run_id, 'motion': expected, 'classified': kind, 'stats': stats,
               'intruder_armed': armed, 'nav_state_before': nav_before,
               'jam': jam_result, 'pass': bool(all_ok)}
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'full_%s.json' % expected)
    with open(path, 'w') as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print('\n%s → %s' % ('PASS' if all_ok else 'FAIL', path))
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
