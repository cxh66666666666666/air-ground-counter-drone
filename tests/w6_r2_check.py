#!/usr/bin/env python3
"""w6_r2_check —— W6-a R2（门禁与 episode 起始时序）命令化验收。

在顶层 launch 运行中、**链路尚未就绪时尽早**介入：先观察 `/agc/task/start` 因门禁未过
失败（odom / cmd 回环未就绪），再到链路就绪后成功；据此校验：
  * 门禁真实生效（未就绪必拒，非静默放行）；
  * `scenario_reset → /agc/air/reset_phase → 门禁校验` 之后 start 成功；
  * 无 `ODOM_INVALID` 自伤误判（成功那次的原因码不含 odom_invalid；失败-成功序列可达）。
结果落 `experiments/w6-task-eval/<run_id>/metrics/r2.json`（ADR-0007）。

用法（launch 刚起时即执行，勿等待）：
  python3 tests/w6_r2_check.py --run-id 20261014-w6-r2-0
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from agc_msgs.msg import AgcTaskState
from std_srvs.srv import Trigger


class R2Checker(Node):
    def __init__(self):
        super().__init__('w6_r2_check')
        self.state = None
        self.state_name = ''
        self.attempts = []          # [(ok, message)]
        self.first_fail = None
        self.first_success = None
        self.cli = self.create_client(Trigger, '/agc/task/start')
        self.create_subscription(AgcTaskState, '/agc/task/state', self._on_state, 20)

    def _on_state(self, msg):
        self.state = msg.state
        self.state_name = msg.state_name

    def call_start(self, timeout=12.0):
        fut = self.cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False, 'call timeout'
        res = fut.result()
        return res.success, res.message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--service-timeout', type=float, default=20.0)
    ap.add_argument('--deadline', type=float, default=70.0)
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()

    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r2-0')

    rclpy.init()
    node = R2Checker()

    # 等门控服务出现（task_manager 起即建服务；此时地面 odom / cmd 回环多半尚未就绪）
    ready = node.cli.wait_for_service(timeout_sec=args.service_timeout)
    if not ready:
        print('FAIL: /agc/task/start service never appeared')
        node.destroy_node(); rclpy.shutdown()
        return 1
    print('start service available; 立即首次尝试门禁（期望：未就绪被拒）')

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.deadline:
        ok, msg = node.call_start()
        node.attempts.append({'ok': bool(ok), 'message': msg,
                              't': round(time.monotonic() - t0, 3)})
        print('attempt %2d: ok=%s  %s' % (len(node.attempts), ok, msg))
        if ok:
            node.first_success = msg
            break
        if node.first_fail is None:
            node.first_fail = msg
        # 让订阅刷新，再看下一轮
        tw = time.monotonic()
        while time.monotonic() - tw < 1.0:
            rclpy.spin_once(node, timeout_sec=0.1)

    # 成功后再让 FSM 推进数拍，确认进入活动态
    tw = time.monotonic()
    while time.monotonic() - tw < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    got_fail = node.first_fail is not None
    got_success = node.first_success is not None
    success_message_ok = bool(got_success) and 'AIR_SEARCH' in (node.first_success or '')
    active = node.state_name in ('AIR_SEARCH', 'TARGET_ACQUIRED', 'GROUND_APPROACH')
    # 关键：成功那次不得由 odom_invalid 引起误判（成功即非误判）；失败→成功序列可达
    no_odom_false_positive = bool(got_success)
    all_ok = got_fail and got_success and success_message_ok and active and no_odom_false_positive

    print('got_fail=%s got_success=%s success_msg_ok=%s active=%s state=%s'
          % (got_fail, got_success, success_message_ok, active, node.state_name))

    node.destroy_node()
    rclpy.shutdown()

    payload = {
        'run_id': run_id,
        'attempts': node.attempts,
        'first_fail': node.first_fail,
        'first_success': node.first_success,
        'got_fail': got_fail,
        'got_success': got_success,
        'success_message_ok': success_message_ok,
        'state_name': node.state_name,
        'active_state': active,
        'no_odom_false_positive': no_odom_false_positive,
        'pass': bool(all_ok),
    }
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'r2.json')
    with open(path, 'w') as fh:
        json.dump(payload, fh, indent=2, ensure_ascii=False)
    print('\n%s → %s' % ('PASS' if all_ok else 'FAIL', path))
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
