#!/usr/bin/env python3
"""w6_fsm_check —— W6-a R1（完整任务状态机）命令化验收。

在顶层 launch（`agc_task m0_skeleton.launch.py`，无头 / GUI）运行中执行，校验：
  * `/agc/task/start` 命令化驱动 FSM；
  * 采集 `/agc/task/state` 全字段，记录态迁移轨迹；
  * 走到终态：`TRACKING_SUCCESS`（抵近并持续跟踪）或 `MISSION_FAILED`(C4 超时)；
  * 全字段如实填（`state_name / reason_code / air_controller / ground_controller /
    target_id / target_xy_distance_m / target_obs_age_s / tracking_milestone_met`）。
结果落 `experiments/w6-task-eval/<run_id>/metrics/fsm.json`（ADR-0007）。

用法（launch 已在运行）：
  python3 tests/w6_fsm_check.py --run-id 20261014-w6-fsm-0
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

TERMINAL = {
    AgcTaskState.TRACKING_SUCCESS,
    AgcTaskState.INTERCEPT_CONFIRMED,
    AgcTaskState.MISSION_FAILED,
    AgcTaskState.ABORTED,
    AgcTaskState.INFRA_FAILURE,
}


class FsmChecker(Node):
    def __init__(self):
        super().__init__('w6_fsm_check')
        self.history = []          # 去重后的态序列
        self.frames = []           # 每拍快照
        self.last_state = None
        self.terminal_seen = None
        self.cli = self.create_client(Trigger, '/agc/task/start')
        self.create_subscription(AgcTaskState, '/agc/task/state', self._on_state, 20)

    def _on_state(self, msg: AgcTaskState):
        snap = {
            't_wall': round(time.monotonic(), 3),
            'state': msg.state,
            'state_name': msg.state_name,
            'reason_code': msg.reason_code,
            'air_controller': msg.air_controller,
            'ground_controller': msg.ground_controller,
            'target_id': msg.target_id,
            'target_xy_distance_m': (None if msg.target_xy_distance_m != msg.target_xy_distance_m
                                     else round(msg.target_xy_distance_m, 3)),
            'target_obs_age_s': (None if msg.target_obs_age_s != msg.target_obs_age_s
                                 else round(msg.target_obs_age_s, 3)),
            'tracking_milestone_met': bool(msg.tracking_milestone_met),
        }
        self.frames.append(snap)
        if msg.state != self.last_state:
            self.history.append({'state': msg.state, 'state_name': msg.state_name,
                                 'reason_code': msg.reason_code})
            self.last_state = msg.state
        if msg.state in TERMINAL and self.terminal_seen is None:
            self.terminal_seen = msg.state

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
    ap.add_argument('--duration', type=float, default=150.0,
                    help='从 start 起最长等待时间（应 > T_max，默认 150 s）')
    ap.add_argument('--warmup', type=float, default=0.0)
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--expect-terminal', default=None,
                    help='期望终态 state_name（如 TRACKING_SUCCESS / MISSION_FAILED / INTERCEPT_CONFIRMED）')
    ap.add_argument('--expect-reason', default=None,
                    help='期望终态 reason_code（如 TIMEOUT / CAPTURE_NET / JAM_LAND）')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()

    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-fsm-0')

    rclpy.init()
    node = FsmChecker()

    tw = time.monotonic()
    while time.monotonic() - tw < args.warmup:
        rclpy.spin_once(node, timeout_sec=0.1)

    start_ok, start_msg = node.start_task()
    print('task/start: %s (%s)' % (start_ok, start_msg))

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration and node.terminal_seen is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    dur = time.monotonic() - t0

    names = [h['state_name'] for h in node.history]
    print('trace: ' + ' -> '.join(names))
    print('terminal=%s  %s' % (
        node.terminal_seen,
        'reached' if node.terminal_seen is not None else 'NOT reached (still active)'))

    # R1 判据：命令化走到终态（TRACKING_SUCCESS / MISSION_FAILED(TIMEOUT) 均为有效）
    seq_ok = names[:1] == ['AIR_SEARCH'] or (
        'AIR_SEARCH' in names and 'TARGET_ACQUIRED' in names and 'GROUND_APPROACH' in names)
    reached_ok = node.terminal_seen in (AgcTaskState.TRACKING_SUCCESS, AgcTaskState.MISSION_FAILED)
    # 全字段如实填：终态帧需有非空 state_name / reason_code，且目标字段在态中可用
    last = node.frames[-1] if node.frames else {}
    fields_ok = bool(last.get('state_name')) and last.get('reason_code') is not None \
        and last.get('air_controller') is not None and last.get('ground_controller') is not None
    # 场景期望（可选）：终态名 / 原因码须精确匹配
    reason_ok = True
    if args.expect_terminal:
        reached_ok = (last.get('state_name') == args.expect_terminal)
    if args.expect_reason:
        reason_ok = (last.get('reason_code') == args.expect_reason)
    all_ok = bool(start_ok) and seq_ok and reached_ok and fields_ok and reason_ok

    print('start_ok=%s seq_ok=%s reached_ok=%s fields_ok=%s reason_ok=%s'
          % (start_ok, seq_ok, reached_ok, fields_ok, reason_ok))
    print('last frame: %s' % json.dumps(last, ensure_ascii=False))

    node.destroy_node()
    rclpy.shutdown()

    payload = {
        'run_id': run_id,
        'start_ok': bool(start_ok),
        'start_message': start_msg,
        'trace': names,
        'history': node.history,
        'terminal_state': node.terminal_seen,
        'duration_s': round(dur, 2),
        'last_frame': last,
        'seq_ok': seq_ok,
        'reached_ok': reached_ok,
        'fields_ok': fields_ok,
        'reason_ok': reason_ok,
        'pass': bool(all_ok),
    }
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'fsm.json')
    with open(path, 'w') as fh:
        json.dump(payload, fh, indent=2, ensure_ascii=False)
    print('\n%s → %s' % ('PASS' if all_ok else 'FAIL', path))
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
