#!/usr/bin/env python3
"""w7b_e2_check —— W7-b T2（E2 / F-11）验收：最小动态火控「完整射击闭环」取证。

起 `w6.launch.py backend:=lightweight scene:=s08_static_none_nominal_capture command_source:=rl`
（可选 gui）→ 调 `/agc/task/start` → 追踪 `/agc/task/state` 至终态或超时，**同时**记录：
  * `/agc/task/fire_control`（`SHOT_FIRED|SHOT_HIT|SHOT_MISS shot=<id> …`）
  * `/agc/task/capture_evidence`（标注串）
  * `/agc/task/capture_hit`（命中值恰为 `CAPTURE_NET`）

判据（[ADR-0070](docs/adr/0070-w7b-research-propositions-and-rl-role-adjudication.md)(c) / [F-11]）：
  * **≥1 次完整射击闭环**：出现 `SHOT_FIRED` **且**随后出现 `SHOT_HIT` 或 `SHOT_MISS`（同一 `shot_id`）；
  * 命中时 `/agc/task/capture_hit` 值**恰为** `CAPTURE_NET`，终态 `INTERCEPT_CONFIRMED`；
  * 证据串携带 `shot=` / `flight_s=` / `miss_dist_m=`（逐帧比对可复核）；
  * 结果落 `experiments/w7-rl-b/<run_id>/metrics/e2.json`（ADR-0007）。

用法：
  python3 tests/w7b_e2_check.py --mode headless --campaign w7-rl-b
"""

import argparse
import glob
import json
import os
import random
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from agc_msgs.msg import AgcTaskState

sys.path.insert(0, '/home/chen/AGC/scripts')
from episode_runner import TERMINALS, _reap_process_group  # noqa: E402

ROOT = '/home/chen/AGC'
LAUNCH = 'w6.launch.py'
SCENE = 's08_static_none_nominal_capture'
GT_CAR, GT_TARGET, GT_FRESH_S = '/agc/gt/agc_car', '/agc/gt/intruder_uav', 1.0
CMD_TOPIC, CMD_FRESH_S = '/agc/ground/cmd_vel', 1.0


class Mon(Node):
    def __init__(self):
        super().__init__('w7b_e2_check')
        self.state = 0
        self.state_name = ''
        self.reason = ''
        self.gt_car_rx = None
        self.gt_tgt_rx = None
        self.cmd_rx = None
        self.fire = []       # 火控事件原始串
        self.evidence = []   # 捕获证据原始串
        self.hit_vals = []
        self.create_subscription(AgcTaskState, '/agc/task/state', self._on_state, 20)
        self.create_subscription(Clock, '/clock', lambda m: None, 1)
        self.create_subscription(PoseStamped, GT_CAR, lambda m: setattr(self, 'gt_car_rx', time.monotonic()), 10)
        self.create_subscription(PoseStamped, GT_TARGET, lambda m: setattr(self, 'gt_tgt_rx', time.monotonic()), 10)
        self.create_subscription(TwistStamped, CMD_TOPIC, lambda m: setattr(self, 'cmd_rx', time.monotonic()), 20)
        self.create_subscription(String, '/agc/task/fire_control', lambda m: self.fire.append(m.data), 20)
        self.create_subscription(String, '/agc/task/capture_evidence', lambda m: self.evidence.append(m.data), 20)
        self.create_subscription(String, '/agc/task/capture_hit', lambda m: self.hit_vals.append(m.data), 20)
        self.cli = self.create_client(Trigger, '/agc/task/start')

    def _on_state(self, m):
        self.state = m.state
        self.state_name = m.state_name
        self.reason = m.reason_code

    def fresh(self):
        t = time.monotonic()
        ok = lambda v: v is not None and (t - v) < GT_FRESH_S
        return ok(self.gt_car_rx) and ok(self.gt_tgt_rx) and \
            (self.cmd_rx is not None and (t - self.cmd_rx) < CMD_FRESH_S)

    def start(self, timeout=20.0):
        if not self.cli.wait_for_service(timeout_sec=timeout):
            return False, 'service unavailable'
        fut = self.cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return (fut.result().success, fut.result().message) if fut.done() else (False, 'timeout')

    def wait_terminal(self, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state in TERMINALS:
                return True
        return False


def shot_of(line):
    for tok in line.split():
        if tok.startswith('shot='):
            return tok[len('shot='):]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='headless', choices=['headless', 'gui'])
    ap.add_argument('--scene', default=SCENE)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--ready-timeout-s', type=float, default=180.0)
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--command-source', default='rl')
    args = ap.parse_args()

    run_id = args.run_id or ('%s-%s-%s-%04d' % (
        datetime.now().strftime('%Y%m%d'), args.campaign,
        datetime.now().strftime('%H%M%S'), random.randint(0, 9999)))
    out_dir = os.path.join(ROOT, 'experiments', args.campaign, run_id)
    if os.path.exists(out_dir):
        print('拒绝写入已有 run 目录：%s' % out_dir)
        return 2
    for sub in ('metrics', 'log'):
        os.makedirs(os.path.join(out_dir, sub), exist_ok=True)

    world = os.path.join(ROOT, 'install/agc_sim/share/agc_sim/worlds/agc_world.sdf')
    gz = ('-r -v 3 ' if args.mode == 'gui' else '-s -r -v 3 ') + world
    log = os.path.join(out_dir, 'log', 'e2_%s_%d.log' % (args.scene, args.seed))

    rclpy.init()
    mon = Mon()
    wall_start = time.monotonic()
    env = dict(os.environ)
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = env.get(
        'GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/jazzy/opt/gz_sim_vendor/lib')
    proc = __import__('subprocess').Popen(
        ['ros2', 'launch', 'agc_task', LAUNCH, 'backend:=lightweight',
         'scene:=%s' % args.scene, 'seed:=%d' % args.seed, 'gz_args:=%s' % gz,
         'command_source:=%s' % args.command_source, 'shield:=true'],
        stdout=open(log, 'w'), stderr=__import__('subprocess').STDOUT,
        start_new_session=True, env=env)
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        pgid = None

    result = {'run_id': run_id, 'scene': args.scene, 'seed': args.seed,
              'mode': args.mode, 'command_source': args.command_source,
              'backend': 'lightweight', 'launch': LAUNCH}
    try:
        t0 = time.monotonic()
        ready = False
        while time.monotonic() - t0 < args.ready_timeout_s:
            rclpy.spin_once(mon, timeout_sec=0.1)
            if mon.cli.service_is_ready() and mon.fresh():
                ready = True
                break
        result['ready'] = ready
        t_settle = time.monotonic() + 2.0
        while time.monotonic() < t_settle:
            rclpy.spin_once(mon, timeout_sec=0.1)
        ok, msg = mon.start()
        result['started'] = bool(ok)
        result['start_message'] = msg
        remaining = max(args.timeout - (time.monotonic() - wall_start), 1.0)
        reached = mon.wait_terminal(remaining)
        result['terminal_reached'] = reached
        result['terminal_state'] = mon.state_name if reached else 'ACTIVE'
        result['reason_code'] = mon.reason
    finally:
        mon.destroy_node()
        rclpy.shutdown()
        result['proc_exit_confirmed'], result['proc_returncode'] = _reap_process_group(proc, pgid)

    fired = [l for l in mon.fire if l.startswith('SHOT_FIRED')]
    hit = [l for l in mon.fire if l.startswith('SHOT_HIT')]
    miss = [l for l in mon.fire if l.startswith('SHOT_MISS')]
    fired_ids = {shot_of(l) for l in fired}
    resolved_ids = {shot_of(l) for l in hit + miss}
    closed = sorted(fired_ids & resolved_ids)

    result.update({
        'fire_control_events': mon.fire,
        'capture_evidence': mon.evidence,
        'capture_hit_values': mon.hit_vals,
        'n_shot_fired': len(fired), 'n_shot_hit': len(hit), 'n_shot_miss': len(miss),
        'closed_shot_ids': closed,
        'wall_elapsed_s': time.monotonic() - wall_start,
    })
    checks = {
        'shot_fired_present': len(fired) >= 1,
        'shot_resolved_same_id': len(closed) >= 1,
        'evidence_carries_shot_and_flight': all(
            ('shot=' in e and 'flight_s=' in e and 'miss_dist_m=' in e)
            for e in mon.evidence if e.startswith(('CAPTURE_HIT', 'SHOT_MISS')) or e.startswith(('SHOT_FIRED',))),
        'capture_hit_is_exact_CAPTURE_NET': (
            len(hit) == 0 or all(v == 'CAPTURE_NET' for v in mon.hit_vals)),
    }
    if hit:
        checks['terminal_is_intercept_confirmed'] = (result['terminal_state'] == 'INTERCEPT_CONFIRMED')
    result['checks'] = checks
    result['overall_pass'] = all(checks.values())
    result['proposition'] = ('E2 / F-11：最小动态火控——≥1 次完整射击闭环（SHOT_FIRED→SHOT_HIT|SHOT_MISS，'
                             '同一 shot_id）；命中时 capture_hit 恰为 CAPTURE_NET 且终态 INTERCEPT_CONFIRMED；'
                             '证据串携带 shot/flight_s/miss_dist_m。')

    with open(os.path.join(out_dir, 'metrics', 'e2.json'), 'w') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    print('run_id=%s mode=%s terminal=%s(%s) fired=%d hit=%d miss=%d closed=%s' % (
        run_id, args.mode, result['terminal_state'], result['reason_code'],
        len(fired), len(hit), len(miss), closed))
    for k, v in checks.items():
        print('  %-42s %s' % (k, 'PASS' if v else 'FAIL'))
    for l in mon.evidence:
        print('    evidence: %s' % l)
    print('\n%s -> %s' % ('PASS' if result['overall_pass'] else 'FAIL',
                          os.path.join(out_dir, 'metrics', 'e2.json')))
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
