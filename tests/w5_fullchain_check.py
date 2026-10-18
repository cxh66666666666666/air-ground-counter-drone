#!/usr/bin/env python3
"""w5_fullchain_check —— W5-c 全链回归（spec R10）。

在顶层 launch（`agc_task m0_skeleton.launch.py`，GUI 或 `-s` 无头）运行中执行，校验：
  * 门控：`/agc/task/start` 成功 → `GROUND_APPROACH`（M0 门控不退化）；
  * 频率：`/agc/ground/odom` 30、`/agc/map` 2、`/agc/targets` 10、`/agc/obstacles` 10、
          `/agc/ground/cmd_vel` 20、`/agc/ground/cmd_vel_raw` 20（±容差）；
  * 帧名：含 Header 的协同话题 `frame_id=world`（`cmd_vel`/`cmd_vel_raw` 为车体系）；
  * 终态：**不得**为 `TRACKING_SUCCESS`（契约 #terminal-semantics；W5 不裁定成功）。
结果落 `experiments/w5-ground-nav-safety/<run_id>/metrics/fullchain.json`（ADR-0007）。

用法：
  python3 tests/w5_fullchain_check.py --duration 8 --run-id <YYYYMMDD>-fullchain-0
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
from rclpy.qos import QoSProfile, DurabilityPolicy

from agc_msgs.msg import AgcTaskState, ObstacleTrackArray, TargetTrackArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_srvs.srv import Trigger

# (topic, type, 期望 Hz, lenient, 期望 frame)
TOPICS = [
    ('/agc/ground/odom', Odometry, 30.0, False, 'world'),
    ('/agc/map', OccupancyGrid, 2.0, False, 'world'),
    ('/agc/targets', TargetTrackArray, 10.0, False, 'world'),
    ('/agc/obstacles', ObstacleTrackArray, 10.0, False, 'world'),
    ('/agc/ground/cmd_vel', TwistStamped, 20.0, False, 'car/base_link'),
    ('/agc/ground/cmd_vel_raw', TwistStamped, 20.0, False, 'car/base_link'),
]


class FullChainChecker(Node):
    def __init__(self):
        super().__init__('w5_fullchain_check')
        self.counts = {t[0]: 0 for t in TOPICS}
        self.frames = {}
        for topic, typ, _, _, _ in TOPICS:
            qos = QoSProfile(depth=50)
            if topic == '/agc/map':
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(typ, topic, self._mk(topic), qos)
        self.state = 0
        self.state_name = ''
        self.create_subscription(AgcTaskState, '/agc/task/state', self._on_state, 20)
        # 运动证据：命令幅值 + 车实测位移（R8 基线需真实驱动）
        self.v_raw_max = 0.0
        self.create_subscription(TwistStamped, '/agc/ground/cmd_vel_raw', self._on_raw, 50)
        self.create_subscription(Odometry, '/agc/ground/odom', self._on_odom, 50)
        self.start_xy = None
        self.end_xy = None
        self.cli = self.create_client(Trigger, '/agc/task/start')

    def _on_raw(self, msg):
        self.v_raw_max = max(self.v_raw_max, abs(msg.twist.linear.x))

    def _on_odom(self, msg):
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.start_xy is None:
            self.start_xy = xy
        self.end_xy = xy

    def _mk(self, topic):
        def cb(msg):
            self.counts[topic] += 1
            if topic not in self.frames:
                self.frames[topic] = getattr(msg.header, 'frame_id', '')
        return cb

    def _on_state(self, msg):
        self.state = msg.state
        self.state_name = msg.state_name

    def start_task(self, timeout=15.0):
        if not self.cli.wait_for_service(timeout_sec=timeout):
            return False, 'service /agc/task/start unavailable'
        fut = self.cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False, 'start call timeout'
        res = fut.result()
        return res.success, res.message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=8.0)
    ap.add_argument('--warmup', type=float, default=4.0)
    ap.add_argument('--tol', type=float, default=0.15)
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()

    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-fullchain-0')

    rclpy.init()
    node = FullChainChecker()
    tw = time.monotonic()
    while time.monotonic() - tw < args.warmup:
        rclpy.spin_once(node, timeout_sec=0.1)

    start_ok, start_msg = node.start_task()
    print('task/start: %s (%s)' % (start_ok, start_msg))
    # 让门控后链路再运行数拍
    tw = time.monotonic()
    while time.monotonic() - tw < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    for k in node.counts:
        node.counts[k] = 0
    node.frames.clear()
    node.start_xy = node.end_xy          # 位移测量窗口起点
    node.v_raw_max = 0.0
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    dur = time.monotonic() - t0

    traveled = 0.0
    if node.start_xy and node.end_xy:
        traveled = math.hypot(node.end_xy[0] - node.start_xy[0],
                              node.end_xy[1] - node.start_xy[1])

    results = []
    all_ok = bool(start_ok) and node.state == 3  # GROUND_APPROACH
    all_ok = all_ok and traveled >= 0.5 and node.v_raw_max > 0.05
    for topic, _, exp, lenient, exp_frame in TOPICS:
        rate = node.counts[topic] / dur
        within = abs(rate - exp) <= args.tol * exp
        frame = node.frames.get(topic, '')
        frame_ok = (frame == exp_frame)
        ok = within and frame_ok
        if not lenient:
            all_ok = all_ok and ok
        results.append({'topic': topic, 'expected_hz': exp, 'measured_hz': round(rate, 3),
                        'within_tol': within, 'frame_id': frame,
                        'frame_ok': frame_ok, 'ok': ok})
        print('%-26s %7.3f Hz (exp %5.1f ±%d%%) frame=%s %s'
              % (topic, rate, exp, int(args.tol * 100), frame, 'OK' if ok else 'FAIL'))

    terminal_ok = (node.state_name != 'TRACKING_SUCCESS')
    all_ok = all_ok and terminal_ok
    print('state=%s (GROUND_APPROACH=%s) terminal_ok=%s'
          % (node.state_name, node.state == 3, terminal_ok))
    print('motion: traveled=%.2f m  v_cmd_raw_max=%.2f m/s' % (traveled, node.v_raw_max))

    node.destroy_node()
    rclpy.shutdown()

    payload = {'run_id': run_id, 'start_ok': bool(start_ok), 'start_message': start_msg,
               'state_name': node.state_name, 'duration_s': round(dur, 2), 'tol': args.tol,
               'traveled_m': round(traveled, 3), 'v_raw_max': round(node.v_raw_max, 3),
               'terminal_not_tracking_success': terminal_ok, 'results': results, 'pass': bool(all_ok)}
    out_dir = os.path.join(args.experiments_root, 'w5-ground-nav-safety', run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'fullchain.json')
    with open(path, 'w') as fh:
        json.dump(payload, fh, indent=2)
    print('\n%s → %s' % ('PASS' if all_ok else 'FAIL', path))
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
