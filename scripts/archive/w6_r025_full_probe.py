#!/usr/bin/env python3
"""w6_r025_full_probe —— R-025 整链**同一 run 因果证据收集器**。

在**已运行**的 `w6.launch.py backend:=full`（双 PX4 整链）上执行（与 `w6_full_check.py` 同风格）：

  ① 等 `/agc/air/active_controller` 出现（`takeoff_controller`），再等若干秒（己方起飞到悬停）；
  ② 调用 `/agc/air/release_control`（Trigger）→ 等 `active_controller` 变为 `trajectory_tracker`
     （= **己方接管证据**；全仓无节点自动调用该服务，需本收集器显式调用）；
  ③ 调用 `/agc/task/start`（Trigger）→ 记录 `success` + `message`（`started: AIR_SEARCH` = start OK）；
  ④ 墙钟看门狗等待**任一终态**（超时则如实记 `terminal_reached=false`，**不**伪造）；
  ⑤ 全程记录时间线（各事件墙钟时刻 + 仿真时刻 `/clock`）。

输出 `experiments/w6-task-eval/<run_id>/same_run.json`：在同一 run（同一 run_id / 同一进程时间线）
回答全部 5 问——`start OK` / 己方接管(release_control) / 目标运动 / 捕获或干扰 / 真实终态。
**所有 bool 均为真实观测推导**；拿不到即如实标 false（`full_chain_executed` 亦然）。

用法：
  python3 scripts/archive/w6_r025_full_probe.py --run-id 20261016-w6-r025-0 --scene s08_static_none_nominal_capture
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from agc_msgs.msg import AgcTaskState
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleStatus
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT = '/home/chen/AGC'

STATE_TOPIC = '/agc/task/state'
GT_TOPIC = '/agc/gt/intruder_uav'
CAPTURE_EVENT_TOPIC = '/agc/task/capture_event'
JAM_CONFIRM_TOPIC = '/agc/task/jam_confirm'
ACTIVE_CTRL_TOPIC = '/agc/air/active_controller'
INTRUDER_STATUS_TOPIC = '/px4_1/fmu/out/vehicle_status'
CLOCK_TOPIC = '/clock'
START_SERVICE = '/agc/task/start'
RELEASE_SERVICE = '/agc/air/release_control'

TERMINAL_NAMES = {
    AgcTaskState.TRACKING_SUCCESS: 'TRACKING_SUCCESS',
    AgcTaskState.MISSION_FAILED: 'MISSION_FAILED',
    AgcTaskState.ABORTED: 'ABORTED',
    AgcTaskState.INTERCEPT_CONFIRMED: 'INTERCEPT_CONFIRMED',
    AgcTaskState.INFRA_FAILURE: 'INFRA_FAILURE',
}
TERMINALS = set(TERMINAL_NAMES.keys())
JAM_POSITIVE = ('JAM_RTL', 'JAM_LAND', 'JAM_HOLD', 'JAM_LOST')


def _tl_qos():
    """对齐 takeoff_controller / capture_adjudicator 发布端：depth=1 + RELIABLE + TRANSIENT_LOCAL。"""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _be_qos():
    """对齐 /px4_*/fmu/out/*（px4 用 BEST_EFFORT）。"""
    q = QoSProfile(depth=5)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    return q


def _registry_scene(scene):
    with open(os.path.join(ROOT, 'src/agc_task/config/scenarios.yaml')) as f:
        scenes = yaml.safe_load(f)['scenes']
    return scenes.get(scene, {}) or {}


def classify_motion(samples):
    """目标运动分类（与 w6_full_check.py 同判据）：static / linear / maneuver / unclassified。"""
    if len(samples) < 3:
        return 'unclassified', {'dx': None, 'dy': None, 'n': len(samples)}
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


class FullProbe(Node):
    def __init__(self):
        super().__init__('w6_r025_full_probe')
        self.t0 = time.monotonic()
        self.timeline = []

        self.state = None
        self.state_name = ''
        self.reason = ''
        self.active_controller = None
        self.px4_armed = None
        self.px4_nav_state = None
        self.px4_status_seen = False
        self.capture_event = None
        self.jam_event = None
        self.gt_samples = []
        self.sim_s = None

        self.create_subscription(AgcTaskState, STATE_TOPIC, self._on_state, 20)
        self.create_subscription(PoseStamped, GT_TOPIC, self._on_gt, 20)
        self.create_subscription(String, CAPTURE_EVENT_TOPIC, self._on_capture, _tl_qos())
        self.create_subscription(String, JAM_CONFIRM_TOPIC, self._on_jam, 10)
        self.create_subscription(String, ACTIVE_CTRL_TOPIC, self._on_active, _tl_qos())
        self.create_subscription(VehicleStatus, INTRUDER_STATUS_TOPIC, self._on_px4, _be_qos())
        self.create_subscription(Clock, CLOCK_TOPIC, self._on_clock, 1)

        self.start_cli = self.create_client(Trigger, START_SERVICE)
        self.release_cli = self.create_client(Trigger, RELEASE_SERVICE)

    # ---- callbacks ----
    def _on_state(self, m):
        self.state = int(m.state)
        self.state_name = m.state_name
        self.reason = m.reason_code

    def _on_gt(self, m):
        self.gt_samples.append((m.pose.position.x, m.pose.position.y, m.pose.position.z))

    def _on_capture(self, m):
        self.capture_event = m.data

    def _on_jam(self, m):
        self.jam_event = m.data

    def _on_active(self, m):
        self.active_controller = m.data

    def _on_px4(self, m):
        self.px4_status_seen = True
        self.px4_armed = (m.arming_state == 2)
        self.px4_nav_state = m.nav_state

    def _on_clock(self, m):
        self.sim_s = m.clock.sec + m.clock.nanosec * 1e-9

    # ---- helpers ----
    def record(self, event, **extra):
        self.timeline.append({
            'event': event, 'wall_s': round(time.monotonic() - self.t0, 3),
            'sim_s': None if self.sim_s is None else round(self.sim_s, 3), **extra})

    def spin_for(self, dur):
        t = time.monotonic()
        while time.monotonic() - t < dur:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_until(self, cond, max_dur):
        t = time.monotonic()
        while time.monotonic() - t < max_dur:
            rclpy.spin_once(self, timeout_sec=0.1)
            if cond():
                return True
        return False

    def call(self, cli, timeout=15.0):
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--scene', default='s08_static_none_nominal_capture')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--mode', default='headless', choices=['headless', 'gui'])
    ap.add_argument('--wait-active-timeout', type=float, default=60.0,
                    help='等 active_controller 出现的上限（秒）')
    ap.add_argument('--hover-wait', type=float, default=10.0, help='接管前额外等待（己方起飞到悬停）')
    ap.add_argument('--takeover-timeout', type=float, default=20.0, help='等 active_controller 翻转的上限')
    ap.add_argument('--terminal-timeout', type=float, default=120.0, help='等终态的墙钟看门狗上限')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r025-0')

    sc = _registry_scene(args.scene)
    expected_terminal = sc.get('expected_terminal')
    expected_reason = sc.get('expected_reason')

    rclpy.init()
    node = FullProbe()
    res = {'run_id': run_id, 'backend': 'full', 'launch': 'w6.launch.py',
           'scene': args.scene, 'seed': args.seed, 'mode': args.mode,
           'expected_terminal': expected_terminal, 'expected_reason': expected_reason}
    causal = {'own_takeover': {}, 'target_motion': {}, 'capture': {}, 'jam': {}, 'terminal': {}}

    try:
        # ① 等 active_controller 出现（takeoff_controller），再额外等若干秒（己方起飞到悬停）
        node.record('probe_start')
        appeared = node.run_until(lambda: node.active_controller is not None, args.wait_active_timeout)
        node.record('active_controller_appeared' if appeared else 'active_controller_timeout',
                    value=node.active_controller)
        if appeared:
            node.spin_for(args.hover_wait)
        active_before = node.active_controller
        causal['own_takeover']['active_controller_before'] = active_before

        # ② 调用 release_control → 等 active_controller 变为 trajectory_tracker
        released = False
        takeover_ok = False
        takeover_elapsed = None
        if node.release_cli.wait_for_service(timeout_sec=10.0):
            t_rel = time.monotonic()
            node.record('release_control_requested')
            r = node.call(node.release_cli, timeout=15.0)
            released = r is not None
            node.record('release_control_response',
                        success=(r.success if r is not None else None),
                        message=(r.message if r is not None else 'service unavailable/timeout'))
            if released:
                takeover_ok = node.run_until(
                    lambda: node.active_controller == 'trajectory_tracker', args.takeover_timeout)
                takeover_elapsed = round(time.monotonic() - t_rel, 3)
        else:
            node.record('release_control_service_unavailable')
        causal['own_takeover'].update({
            'release_requested': bool(released),
            'active_controller_after': node.active_controller,
            'takeover_ok': bool(takeover_ok),
            'takeover_elapsed_s': takeover_elapsed,
        })

        # ③ 调用 /agc/task/start → 记录 success + message
        start_ok = False
        start_message = ''
        if node.start_cli.wait_for_service(timeout_sec=20.0):
            node.record('task_start_requested')
            r = node.call(node.start_cli, timeout=20.0)
            if r is not None:
                start_message = r.message
                start_ok = bool(r.success) and str(r.message).startswith('started: AIR_SEARCH')
            node.record('task_start_response', success=(r.success if r is not None else None),
                        message=start_message or 'service unavailable/timeout')
        else:
            node.record('task_start_service_unavailable')

        # ④ 墙钟看门狗等待任一终态
        reached = node.run_until(lambda: node.state in TERMINALS, args.terminal_timeout)
        node.record('terminal_reached' if reached else 'terminal_timeout',
                    state_name=node.state_name, reason=node.reason)

        # ⑤ 汇总真实观测（可能为 false）
        causal['start_ok'] = bool(start_ok)
        causal['start_message'] = start_message
        motion_kind, motion_stats = classify_motion(node.gt_samples)
        causal['target_motion'] = {'classified': motion_kind, 'stats': motion_stats}
        causal['capture'] = {'observed': node.capture_event is not None,
                             'event': node.capture_event}
        jam_positive = bool(node.jam_event in JAM_POSITIVE)
        causal['jam'] = {'observed': jam_positive, 'event': node.jam_event}
        causal['terminal'] = {'state_name': (node.state_name if reached else None),
                              'reason_code': (node.reason if reached else None), 'reached': bool(reached)}

        res['causal_record'] = causal
        res['timeline'] = node.timeline
        res['equipment'] = {
            'dual_px4_configured': True,   # 源事实：w6.launch.py backend:=full 起 2 个 PX4 实例
            'px4_agents': 2,               # 源事实：8888 / 8889 两个 uXRCE-DDS Agent
            'intruder_armed': node.px4_armed,
            'intruder_nav_state': node.px4_nav_state,
            'intruder_status_seen': bool(node.px4_status_seen),
        }

        # full_chain_executed：整链 launch + probe 是否真的跑起来（真实观测推导）
        active_seen = active_before is not None or node.active_controller is not None
        full_chain_executed = bool(active_seen or node.px4_status_seen)
        res['full_chain_executed'] = full_chain_executed

        # 命题 + pass（真值来自真实观测）
        chain_desc = '整链 w6.launch.py backend:=full（双 PX4，%s）' % args.mode
        term_desc = expected_terminal or '?'
        if expected_reason:
            term_desc = '%s(%s)' % (term_desc, expected_reason)
        capture_desc = '捕获可观测' if node.capture_event is not None else (
            '干扰可观测' if jam_positive else '捕获/干扰未可观')
        res['proposition'] = ('%s %s 同一 run 达 %s，且 start OK + 己方接管 + 目标运动(%s) + %s'
                              % (chain_desc, args.scene, term_desc, motion_kind, capture_desc))

        term_match = bool(reached and node.state_name == expected_terminal and
                          (not expected_reason or node.reason == expected_reason))
        evidence_ok = bool(node.capture_event is not None or jam_positive)
        res['pass'] = bool(full_chain_executed and start_ok and takeover_ok and
                           (motion_kind != 'unclassified') and evidence_ok and term_match)
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['pass'] = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'same_run.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)

    print('scene=%s start_ok=%s takeover=%s motion=%s capture=%s jam=%s terminal=%s reached=%s'
          % (args.scene, res.get('causal_record', {}).get('start_ok'),
             res.get('causal_record', {}).get('own_takeover', {}).get('takeover_ok'),
             res.get('causal_record', {}).get('target_motion', {}).get('classified'),
             res.get('causal_record', {}).get('capture', {}).get('observed'),
             res.get('causal_record', {}).get('jam', {}).get('observed'),
             res.get('causal_record', {}).get('terminal', {}).get('state_name'),
             res.get('causal_record', {}).get('terminal', {}).get('reached')))
    print('\n%s → %s' % ('PASS' if res.get('pass') else 'FAIL', path))
    return 0 if res.get('pass') else 1


if __name__ == '__main__':
    sys.exit(main())
