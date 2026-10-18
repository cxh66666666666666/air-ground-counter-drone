#!/usr/bin/env python3
"""w6_r6_check —— W6-b R6（出生净空 / 复位重试，C7）命令化验收。

在链路就绪、状态 IDLE（可复位）时，连续调用 `/agc/scenario/reset` 两次，比较
`/agc/scenario/car_start` —— 由 `scenario_reset` 以 `spawn_seed` 生成的 spawn：
  * randomize=true 时同 seed 两次生成应一致（可复现，C7）；
  * randomize=false 且 spawn 过近（< 下限）时 reset 应被拒（success=false）。

用法（launch 就绪后）：python3 tests/w6_r6_check.py --run-id ... --expect-random
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


class R6Checker(Node):
    def __init__(self):
        super().__init__('w6_r6_check')
        self.msg = None
        self.create_subscription(PoseStamped, '/agc/scenario/car_start', self._on, 10)
        self.cli = self.create_client(Trigger, '/agc/scenario/reset')

    def _on(self, m):
        self.msg = m

    def reset_and_capture(self, timeout=10.0):
        # 先清空缓存，再发起 reset（call_reset 内 spin 时可能已投递 car_start），随后补齐等待
        self.msg = None
        fut = self.cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = fut.done() and fut.result().success
        msg_txt = fut.result().message if fut.done() else 'timeout'
        tw = time.monotonic()
        while time.monotonic() - tw < 3.0 and self.msg is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        return ok, msg_txt, self.msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--expect-random', action='store_true',
                    help='randomize=true：期待两次 spawn 一致（可复现）')
    ap.add_argument('--expect-reject', action='store_true',
                    help='randomize=false 且过近：期待 reset 被拒')
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r6-0')

    rclpy.init()
    node = R6Checker()
    if not node.cli.wait_for_service(timeout_sec=15.0):
        print('FAIL: /agc/scenario/reset 服务不可用')
        node.destroy_node(); rclpy.shutdown(); return 1

    ok1, m1, p1 = node.reset_and_capture()
    ok2, m2, p2 = node.reset_and_capture()

    def xy(p):
        return None if p is None else (round(p.pose.position.x, 6), round(p.pose.position.y, 6))

    xy1, xy2 = xy(p1), xy(p2)
    reproducible = (xy1 is not None and xy1 == xy2)
    print('reset1=%s %s  car_start=%s' % (ok1, m1, xy1))
    print('reset2=%s %s  car_start=%s' % (ok2, m2, xy2))

    if args.expect_reject:
        all_ok = (not ok1) and (not ok2)
        print('expect_reject -> %s' % all_ok)
    else:
        all_ok = ok1 and ok2 and reproducible
        print('expect_random reproducible -> %s' % reproducible)

    node.destroy_node()
    rclpy.shutdown()

    payload = {'run_id': run_id, 'reset1': {'ok': bool(ok1), 'msg': m1, 'car_start': xy1},
               'reset2': {'ok': bool(ok2), 'msg': m2, 'car_start': xy2},
               'reproducible': reproducible, 'pass': bool(all_ok)}
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'r6.json')
    with open(path, 'w') as fh:
        json.dump(payload, fh, indent=2, ensure_ascii=False)
    print('\n%s → %s' % ('PASS' if all_ok else 'FAIL', path))
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
