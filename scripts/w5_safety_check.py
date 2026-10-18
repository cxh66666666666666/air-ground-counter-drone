#!/usr/bin/env python3
"""w5_safety_check —— W5-a/c 验收：C2 三档触发 + C6 真实逃逸（spec R4/R5/R9）。

在 `agc_task w5_safety_scenario.launch.py` 运行中执行：
  1. 调 `/agc/task/start`（复用 M0 门控）→ 期望 GROUND_APPROACH；
  2. 记录 `/agc/ground/cmd_vel`（安全层输出）、`/agc/ground/cmd_vel_raw`（命令源输出）、
     `/agc/ground/odom`（车真实位姿）、`/agc/task/state`；
  3. 判据：
     * **C2 顺序 + 承载**：车自巡航速降速（slowdown@安全档）→ 停车（stop@碰撞档）；
     * **C6 真实逃逸**：车**实测停住**（odom 位移极小）、且**未撞柱**（最近距离 ≥ 阈值）。
  结果落 `experiments/w5-ground-nav-safety/<run_id>/metrics/safety.json`（ADR-0007）。

用法：
  python3 scripts/w5_safety_check.py --duration 30 --run-id <YYYYMMDD>-pole-approach-0
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

from agc_msgs.msg import AgcTaskState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

POLE = (0.0, -6.0)          # pole_1（world），见 src/agc_sim/worlds/agc_world.sdf
GROUND_APPROACH = 3         # AgcTaskState.GROUND_APPROACH


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SafetyChecker(Node):
    def __init__(self, run_id):
        super().__init__('w5_safety_check')
        self.samples = []          # (t, v_cmd, v_raw, x, y, dist_pole, state)
        self.v_cmd = 0.0
        self.v_raw = 0.0
        self.x = self.y = 0.0
        self.state = 0
        self.ground_controller = ''
        self.t0 = None
        self.create_subscription(TwistStamped, '/agc/ground/cmd_vel', self._on_cmd, 50)
        self.create_subscription(TwistStamped, '/agc/ground/cmd_vel_raw', self._on_raw, 50)
        self.create_subscription(Odometry, '/agc/ground/odom', self._on_odom, 50)
        self.create_subscription(AgcTaskState, '/agc/task/state', self._on_state, 50)
        self.cli = self.create_client(Trigger, '/agc/task/start')

    def _on_cmd(self, m):
        self.v_cmd = m.twist.linear.x

    def _on_raw(self, m):
        self.v_raw = m.twist.linear.x

    def _on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y

    def _on_state(self, m):
        self.state = m.state
        self.ground_controller = m.ground_controller

    def sample(self):
        dist = math.hypot(self.x - POLE[0], self.y - POLE[1])
        self.samples.append((time.monotonic(), self.v_cmd, self.v_raw, self.x, self.y,
                             dist, self.state))

    def start_task(self, timeout=10.0):
        if not self.cli.wait_for_service(timeout_sec=timeout):
            return False, 'service /agc/task/start unavailable'
        req = Trigger.Request()
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False, 'start call timeout'
        res = fut.result()
        return res.success, res.message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=30.0, help='记录时长（s）')
    ap.add_argument('--warmup', type=float, default=3.0, help='发现/预热（s）')
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    ap.add_argument('--collision-dist', type=float, default=1.0,
                    help='判为撞柱的最近距离阈值（m）')
    args = ap.parse_args()

    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-pole-approach-0')

    rclpy.init()
    node = SafetyChecker(run_id)

    # 预热：等待发现与稳态
    tw = time.monotonic()
    while time.monotonic() - tw < args.warmup:
        rclpy.spin_once(node, timeout_sec=0.05)

    ok, msg = node.start_task()
    print('task/start: %s (%s)' % (ok, msg))
    node.t0 = time.monotonic()

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
        node.sample()

    node.destroy_node()
    rclpy.shutdown()

    samples = node.samples
    result = {'run_id': run_id, 'start_ok': bool(ok), 'start_message': msg,
              'duration_s': args.duration, 'ground_controller': node.ground_controller}

    if not samples:
        result['pass'] = False
        result['error'] = 'no samples'
    else:
        v_cmd = [s[1] for s in samples]
        v_raw = [s[2] for s in samples]
        dist = [s[5] for s in samples]
        v_start = max(v_cmd[:max(1, len(v_cmd) // 5)])          # 起始巡航速（前 20% 窗口）
        elapsed = [s[0] - samples[0][0] for s in samples]
        # 降速档：命令被安全层压到 0.05..0.7×巡航速（SlowSafety ratio=0.3）
        slowdown = any(0.05 < v < max(0.7 * v_start, 0.05) for v in v_cmd)
        # 停车档：出现持续 >= 0.5 s 的 v≈0
        stop_run = 0.0
        stop_seen = False
        prev_t = samples[0][0]
        for t, v, _, _, _, _, _ in samples:
            if v < 0.02:
                stop_run += (t - prev_t)
                if stop_run >= 0.5:
                    stop_seen = True
            else:
                stop_run = 0.0
            prev_t = t
        min_dist = min(dist)
        # 车在**末段静止窗口**内的实测位移（C6：真实停住）——自末尾向前取连续 v≈0 的区间
        i0 = len(samples)
        while i0 > 0 and samples[i0 - 1][1] < 0.02:
            i0 -= 1
        tail_move = 0.0
        if i0 < len(samples):
            tail = samples[i0:]
            tail_move = max(math.hypot(s[3] - tail[0][3], s[4] - tail[0][4]) for s in tail)
        rest_window_s = round(samples[-1][0] - samples[i0][0], 2) if i0 < len(samples) else 0.0

        result.update({
            'v_cruise_max': round(v_start, 3),
            'v_cmd_min': round(min(v_cmd), 3),
            'v_cmd_max': round(max(v_cmd), 3),
            'v_raw_max': round(max(v_raw), 3),
            'slowdown_seen': slowdown,
            'stop_seen': stop_seen,
            'min_dist_to_pole_m': round(min_dist, 3),
            'tail_move_m': round(tail_move, 3),
            'rest_window_s': rest_window_s,
            'final_state': samples[-1][6],
            'samples': len(samples),
            't_series': {'t': [round(e, 2) for e in elapsed[::max(1, len(elapsed) // 60)]],
                         'v_cmd': [round(v_cmd[i], 3) for i in range(0, len(v_cmd), max(1, len(v_cmd) // 60))],
                         'dist_pole': [round(dist[i], 3) for i in range(0, len(dist), max(1, len(dist) // 60))]},
        })
        result['pass'] = bool(ok and v_start > 0.2 and slowdown and stop_seen
                              and min_dist >= args.collision_dist
                              and tail_move <= 0.3 and rest_window_s >= 0.5)

    out_dir = os.path.join(args.experiments_root, 'w5-ground-nav-safety', run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'safety.json')
    with open(path, 'w') as fh:
        json.dump(result, fh, indent=2)

    print('\n[C2] slowdown_seen=%s stop_seen=%s  v_cruise=%.2f v_cmd_min=%.2f'
          % (result.get('slowdown_seen'), result.get('stop_seen'),
             result.get('v_cruise_max', 0.0), result.get('v_cmd_min', 0.0)))
    print('[C6] min_dist_to_pole=%.2f m  tail_move=%.2f m' %
          (result.get('min_dist_to_pole_m', 0.0), result.get('tail_move_m', 0.0)))
    print('\n%s → %s' % ('PASS' if result['pass'] else 'FAIL', path))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
