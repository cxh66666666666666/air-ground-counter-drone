#!/usr/bin/env python3
"""w6_intruder_check —— W6-a R4（目标 driver 三种运动）命令化验收。

不依赖 PX4：向 `/<ns>/fmu/out/{vehicle_odometry,vehicle_status}` 注入**伪造**的已解锁/
OFFBOARD 状态，使 `intruder_driver` 持续输出 `/<ns>/fmu/in/trajectory_setpoint`；采集
setpoint 位置随时间，按运动类型判据辨识：
  * `static`   —— setpoint 位置近似不变（dx≈0, dy≈0）；
  * `linear`   —— 沿 NED +y 匀速推进（dy 大、dx≈0）；
  * `maneuver` —— 圆周机动（dx 与 dy 均显著）。

**范围说明**：本检查验证 driver **运动指令逻辑**可辨；机体真实响应（gz `model -p`）属
"双 PX4 整链 launch 集成"（Task 5.3 的 GUI 观测部分，`static` 已由前置 P 实跑）。
结果落 `experiments/w6-task-eval/<run_id>/metrics/intruder_<motion>.json`（ADR-0007）。
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry, VehicleStatus, TrajectorySetpoint


def _qos():
    q = QoSProfile(depth=5)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    return q


class IntruderCheck(Node):
    def __init__(self, ns):
        super().__init__('w6_intruder_check')
        self.ns = ns
        p = f'/{ns}/fmu'
        self.pub_odom = self.create_publisher(VehicleOdometry, f'{p}/out/vehicle_odometry', _qos())
        self.pub_status = self.create_publisher(VehicleStatus, f'{p}/out/vehicle_status', _qos())
        self.setpoints = []  # (x, y, z)
        self.create_subscription(TrajectorySetpoint, f'{p}/in/trajectory_setpoint',
                                 self._on_sp, _qos())

    def _on_sp(self, m):
        self.setpoints.append((float(m.position[0]), float(m.position[1]), float(m.position[2])))

    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def spin_for(self, dur):
        tw = time.monotonic()
        while time.monotonic() - tw < dur:
            od = VehicleOdometry()
            od.timestamp = self._now_us()
            od.position = [0.0, 0.0, 0.0]  # NED 原点；driver 以此为 boot 原点
            od.q = [1.0, 0.0, 0.0, 0.0]
            self.pub_odom.publish(od)

            st = VehicleStatus()
            st.timestamp = self._now_us()
            st.arming_state = 2   # ARMED
            st.nav_state = 14     # OFFBOARD
            st.failsafe = False
            self.pub_status.publish(st)

            rclpy.spin_once(self, timeout_sec=0.01)


def classify(setpoints):
    if not setpoints:
        return 'none', {}
    xs = [p[0] for p in setpoints]
    ys = [p[1] for p in setpoints]
    dx = max(xs) - min(xs)
    dy = max(ys) - min(ys)
    dy_seq = ys[-1] - ys[0]
    if dx < 0.1 and dy < 0.1:
        kind = 'static'
    elif dx < 0.3 and abs(dy_seq) > 0.5:
        kind = 'linear'
    elif dx > 1.0 and dy > 1.0:
        kind = 'maneuver'
    else:
        kind = 'unclassified'
    return kind, {'dx': round(dx, 3), 'dy': round(dy, 3),
                  'dy_net': round(dy_seq, 3), 'n': len(setpoints)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--motion', required=True)
    ap.add_argument('--ns', default='px4_1')
    ap.add_argument('--duration', type=float, default=6.0)
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-intruder-0')

    rclpy.init()
    node = IntruderCheck(args.ns)
    node.spin_for(args.duration)
    kind, stats = classify(node.setpoints)
    expect = args.motion
    ok = (kind == expect) and stats.get('n', 0) > 5
    print('motion=%s -> classified=%s  %s  %s'
          % (expect, kind, stats, 'OK' if ok else 'MISMATCH'))
    first = node.setpoints[0] if node.setpoints else None
    last = node.setpoints[-1] if node.setpoints else None
    print('first=%s last=%s' % (first, last))

    node.destroy_node()
    rclpy.shutdown()

    payload = {'run_id': run_id, 'motion': expect, 'classified': kind, 'stats': stats,
               'first_setpoint': first, 'last_setpoint': last, 'pass': bool(ok)}
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'intruder_%s.json' % expect)
    with open(path, 'w') as fh:
        json.dump(payload, fh, indent=2, ensure_ascii=False)
    print('\n%s → %s' % ('PASS' if ok else 'FAIL', path))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
