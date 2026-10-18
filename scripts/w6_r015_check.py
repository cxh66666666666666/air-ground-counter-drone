#!/usr/bin/env python3
"""w6_r015_check —— 评审 R-015 复位受控事务与失效安全验收。

在轻量 W6 栈（`agc_task w6.launch.py backend:=lightweight`，无 PX4）运行中执行：
先等 `/agc/scenario/reset` 服务出现，再**杀掉 launch 起的 `task_manager`（[t]ask_manager）与
`scenario_reset`（[s]cenario_reset）**（同名服务不能并存两个 server）。此后由本 check 节点
**发布 `/agc/task/state`（10 Hz，注入的 state 可切换）**，并逐用例起一个独立
`install/lib/agc_task/scenario_reset`（`--ros-args --params-file`）后调 `/agc/scenario/reset`。

订阅（rclpy Node）：`/agc/scenario/car_start`（PoseStamped）、`/agc/scenario/reset_epoch`（Int32）、
`/agc/gt/agc_car`、`/agc/gt/intruder_uav`（PoseStamped）、`/agc/ground/odom`（Odometry）。

用例：
  a. 各终态可复位 + 多回合：INTERCEPT_CONFIRMED(7) / TRACKING_SUCCESS(4) / MISSION_FAILED(5) /
     ABORTED(6) / INFRA_FAILURE(8)；每态**同进程连续两次**均 success=True 且 car_start 到达、epoch 递增。
  b. active 拒绝：GROUND_APPROACH(3) → success=False，reason=not_resettable。
  c. 目标写入失败 → 无中间态：`target_entity_name=no_such_uav`（GZ 对不存在实体仍返回 success，
     实测结论见 ADR-0030）→ 写目标"成功"但**读回**（/agc/gt/intruder_uav 未到新位姿）超时判失败 →
     回滚车到写入前位姿；断言 success=False、car_start 未到达、车 gt ≈ 写入前 且 ≠ 新 spawn。
  d. 读回不一致 → 失败：`readback_tolerance_m=0.0` + `car_entity_name=no_such_car`（GZ 复位写入
     不存在实体仍返回 success；tol=0.0 下真实车 gt 与写入目标不一致）→ success=False 且 reason 含 readback。
  e. 启动初始复位走门控：注入 IDLE(0) 起节点 → ~4 s 后 car_start 到达（含 epoch 发布）；
     注入 GROUND_APPROACH(3) 起节点 → 6 s 内无 car_start。

注：用例 a–d 起节点后先用 active 态**抑制**启动初始复位（4 s 定时器门控拒绝），再切到目标态调用，
避免启动复位污染 epoch / car_start 计数。结果落
`experiments/w6-task-eval/<run_id>/metrics/r015.json`（ADR-0007）。

注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
"""

import argparse
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from agc_msgs.msg import AgcTaskState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

SERVICE = '/agc/scenario/reset'
CAR_START = '/agc/scenario/car_start'
RESET_EPOCH = '/agc/scenario/reset_epoch'
CAR_GT = '/agc/gt/agc_car'
TARGET_GT = '/agc/gt/intruder_uav'
GROUND_ODOM = '/agc/ground/odom'
STATE = '/agc/task/state'

# 终态集合（R-015 / tasks 3.4：含 INTERCEPT_CONFIRMED）
TERMINAL_STATES = [
    AgcTaskState.INTERCEPT_CONFIRMED,   # 7
    AgcTaskState.TRACKING_SUCCESS,      # 4
    AgcTaskState.MISSION_FAILED,        # 5
    AgcTaskState.ABORTED,               # 6
    AgcTaskState.INFRA_FAILURE,         # 8
]


def run_cli(args, timeout=25.0):
    try:
        out = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return out.returncode, (out.stdout or '') + (out.stderr or '')
    except subprocess.TimeoutExpired:
        return -1, 'timeout'


def service_present(name):
    rc, txt = run_cli(['ros2', 'service', 'list', '--no-daemon'], timeout=12.0)
    if rc != 0:
        return False
    return any(line.strip() == name for line in txt.splitlines())


def wait_service(name, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if service_present(name):
            return True
        time.sleep(1.0)
    return False


def wait_service_gone(name, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if not service_present(name):
            return True
        time.sleep(0.5)
    return False


def parse_reply(txt):
    m = re.search(r'success=(True|False)', txt)
    success = (m.group(1) == 'True') if m else None
    m2 = re.search(r"message='([^']*)'", txt)
    message = m2.group(1) if m2 else ''
    reason = None
    if message.startswith('reset rejected:'):
        reason = message[len('reset rejected:'):].strip()
    return success, message, reason


def call_reset():
    """调 /agc/scenario/reset（经 ros2 CLI，规避同名节点重启后的客户端端点缓存）。
    应答偶发不可解析（CLI 发现抖动 / 超时）时重试，取首个可解析结果。"""
    last = None
    for _ in range(3):
        rc, txt = run_cli(['ros2', 'service', 'call', SERVICE, 'std_srvs/srv/Trigger'],
                          timeout=30.0)
        success, message, reason = parse_reply(txt)
        last = {'rc': rc, 'success': success, 'message': message, 'reason': reason,
                'raw': txt.strip()[-300:]}
        if success is not None:
            return last
        time.sleep(0.5)
    return last


class R15Checker(Node):
    def __init__(self):
        super().__init__('w6_r015_check')
        self.injected_state = AgcTaskState.IDLE
        self.car_start_count = 0
        self.last_car_start = None
        self.last_epoch = None
        self.epoch_count = 0
        self.car_gt = None
        self.target_gt = None
        self.odom_speed = None
        self.create_subscription(PoseStamped, CAR_START, self._on_car_start, 20)
        self.create_subscription(Int32, RESET_EPOCH, self._on_epoch, 20)
        self.create_subscription(PoseStamped, CAR_GT, self._on_car_gt, 30)
        self.create_subscription(PoseStamped, TARGET_GT, self._on_target_gt, 30)
        self.create_subscription(Odometry, GROUND_ODOM, self._on_odom, 20)
        self.pub_state = self.create_publisher(AgcTaskState, STATE, 10)
        self.create_timer(0.1, self._publish_state)

    def _publish_state(self):
        m = AgcTaskState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.state = self.injected_state
        m.state_name = 'INJECTED'
        self.pub_state.publish(m)

    def _on_car_start(self, m):
        self.car_start_count += 1
        self.last_car_start = m

    def _on_epoch(self, m):
        self.last_epoch = int(m.data)
        self.epoch_count += 1

    def _on_car_gt(self, m):
        self.car_gt = m

    def _on_target_gt(self, m):
        self.target_gt = m

    def _on_odom(self, m):
        t = m.twist.twist.linear
        self.odom_speed = math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)

    def set_state(self, s):
        self.injected_state = s


def write_params(share, path, overrides):
    base = yaml.safe_load(open(os.path.join(share, 'config', 'w6_base.yaml')))
    params = dict(base['scenario_reset']['ros__parameters'])
    params.update(overrides)
    params['use_sim_time'] = True
    doc = {'scenario_reset': {'ros__parameters': params}}
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
    return params


def exe_path(share):
    return str(Path(share).parents[1] / 'lib' / 'agc_task' / 'scenario_reset')


def start_reset_node(share, out_dir, name, overrides, log_dir):
    params_path = os.path.join(out_dir, 'r015_%s.yaml' % name)
    write_params(share, params_path, overrides)
    log = open(os.path.join(log_dir, 'case_%s.log' % name), 'w')
    proc = subprocess.Popen(
        [exe_path(share), '--ros-args', '--params-file', params_path],
        stdout=log, stderr=subprocess.STDOUT)
    return proc, log, params_path, time.monotonic()


def suppress_startup(node, t_start):
    """起节点后先用 active 态抑制启动初始复位（4 s 定时器门控拒绝），避免污染 epoch / car_start 计数。"""
    node.set_state(AgcTaskState.GROUND_APPROACH)
    while time.monotonic() - t_start < 5.0:
        time.sleep(0.1)


def stop_reset_node(proc, log):
    try:
        proc.terminate()
        proc.wait(timeout=5)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass
    log.close()
    wait_service_gone(SERVICE, 15.0)


def pose_xy(p):
    return (p.pose.position.x, p.pose.position.y)


def wait_until(cond, timeout, step=0.05):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True, time.monotonic() - t0
        time.sleep(step)
    return False, time.monotonic() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r015-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    share = get_package_share_directory('agc_task')
    res = {'run_id': run_id, 'service': SERVICE, 'tests': {}}

    rclpy.init()
    node = R15Checker()
    stop_event = threading.Event()

    def _spin_loop():
        while not stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin_loop, daemon=True)
    spin_thread.start()

    try:
        # 1. 等 launch 起的 /agc/scenario/reset 出现（栈就绪）
        if not wait_service(SERVICE, 30.0):
            res['overall_pass'] = False
            res['error'] = '/agc/scenario/reset service never appeared (stack not ready)'
            return dump(res, out_dir)

        # 等 gt 就绪（车 / 目标真值旁路发布）
        wait_until(lambda: node.car_gt is not None and node.target_gt is not None, 15.0)

        # 2. 杀掉 launch 起的 task_manager + scenario_reset（同名服务不可并存；状态改由 check 注入）
        run_cli(['pkill', '-TERM', '-f', '[t]ask_manager'])
        run_cli(['pkill', '-TERM', '-f', '[s]cenario_reset'])
        if not wait_service_gone(SERVICE, 15.0):
            res['overall_pass'] = False
            res['error'] = 'launch scenario_reset did not deregister'
            return dump(res, out_dir)
        # 确认 check 的 /agc/task/state 发布生效（无其它发布者）

        # ---------------- a. 各终态可复位 + 多回合 ----------------
        a = {'terminals': {}}
        proc, log, _, t_start = start_reset_node(share, out_dir, 'a', {}, out_dir + '/log')
        try:
            if not wait_service(SERVICE, 25.0):
                a['pass'] = False
                a['error'] = 'service never appeared'
            else:
                # 抑制启动复位：先置 active，等启动定时器（4 s）被门控拒绝
                suppress_startup(node, t_start)
                all_ok = True
                ep_prev = None
                for s in TERMINAL_STATES:
                    node.set_state(s)
                    time.sleep(1.0)
                    rounds = []
                    for _ in range(2):
                        cs_before = node.car_start_count
                        r = call_reset()
                        arrived, _ = wait_until(lambda: node.car_start_count > cs_before, 2.0)
                        ep = node.last_epoch
                        rounds.append({'success': r['success'], 'reason': r['reason'],
                                       'epoch': ep, 'car_start_arrived': bool(arrived)})
                        ep_ok = (ep is not None and (ep_prev is None or ep > ep_prev))
                        if not (r['success'] is True and arrived and ep_ok):
                            all_ok = False
                        if ep is not None:
                            ep_prev = ep
                    a['terminals'][str(s)] = rounds
                a['epoch_end'] = node.last_epoch
                a['pass'] = bool(all_ok and node.last_epoch is not None and node.last_epoch >= 2)
        finally:
            stop_reset_node(proc, log)
        res['tests']['a_terminal_multiround'] = a

        # ---------------- b. active 拒绝 ----------------
        b = {}
        proc, log, _, t_start = start_reset_node(share, out_dir, 'b', {}, out_dir + '/log')
        try:
            if not wait_service(SERVICE, 25.0):
                b['pass'] = False
                b['error'] = 'service never appeared'
            else:
                suppress_startup(node, t_start)
                node.set_state(AgcTaskState.GROUND_APPROACH)
                time.sleep(1.0)
                cs_before = node.car_start_count
                b = call_reset()
                b['car_start_arrived'] = bool(node.car_start_count > cs_before)
                b['pass'] = bool(b['success'] is False and b['reason'] == 'not_resettable' and
                                 not b['car_start_arrived'])
        finally:
            stop_reset_node(proc, log)
        res['tests']['b_active_reject'] = b

        # ---------------- c. 目标写入失败 → 无中间态 ----------------
        c = {}
        over_c = {'car_x': -6.0, 'car_y': -3.0, 'target_x': 10.0, 'target_y': -2.0,
                  'target_entity_name': 'no_such_uav'}
        proc, log, _, t_start = start_reset_node(share, out_dir, 'c', over_c, out_dir + '/log')
        try:
            if not wait_service(SERVICE, 25.0):
                c['pass'] = False
                c['error'] = 'service never appeared'
            else:
                suppress_startup(node, t_start)
                node.set_state(AgcTaskState.INTERCEPT_CONFIRMED)
                time.sleep(1.5)
                pre = node.car_gt
                c['pre_car_xy'] = pose_xy(pre) if pre else None
                cs_before = node.car_start_count
                r = call_reset()
                c.update({'success': r['success'], 'reason': r['reason'], 'message': r['message']})
                c['car_start_arrived'] = bool(node.car_start_count > cs_before)
                time.sleep(1.5)
                post = node.car_gt
                c['post_car_xy'] = pose_xy(post) if post else None
                new_spawn = (-6.0, -3.0)
                c['new_spawn_xy'] = new_spawn
                rolled_back = far_from_new = None
                if pre and post:
                    pxy = pose_xy(pre)
                    qxy = pose_xy(post)
                    rolled_back = math.hypot(qxy[0] - pxy[0], qxy[1] - pxy[1])
                    far_from_new = math.hypot(qxy[0] - new_spawn[0], qxy[1] - new_spawn[1])
                c['rollback_dist_m'] = rolled_back
                c['dist_from_new_spawn_m'] = far_from_new
                c['pass'] = bool(
                    c['success'] is False and 'readback' in (c['reason'] or '') and
                    not c['car_start_arrived'] and rolled_back is not None and
                    rolled_back <= 0.4 and far_from_new is not None and far_from_new >= 1.0)
        finally:
            stop_reset_node(proc, log)
        res['tests']['c_target_write_fail_rollback'] = c

        # ---------------- d. 读回不一致 → 失败 ----------------
        d = {}
        over_d = {'readback_tolerance_m': 0.0, 'car_entity_name': 'no_such_car', 'car_x': -8.0}
        proc, log, _, t_start = start_reset_node(share, out_dir, 'd', over_d, out_dir + '/log')
        try:
            if not wait_service(SERVICE, 25.0):
                d['pass'] = False
                d['error'] = 'service never appeared'
            else:
                suppress_startup(node, t_start)
                node.set_state(AgcTaskState.INTERCEPT_CONFIRMED)
                time.sleep(1.5)
                cs_before = node.car_start_count
                r = call_reset()
                d.update({'success': r['success'], 'reason': r['reason'], 'message': r['message']})
                d['car_start_arrived'] = bool(node.car_start_count > cs_before)
                d['pass'] = bool(d['success'] is False and 'readback' in (d['reason'] or '') and
                                 not d['car_start_arrived'])
        finally:
            stop_reset_node(proc, log)
        res['tests']['d_readback_mismatch'] = d

        # ---------------- e. 启动初始复位走门控 ----------------
        e = {}
        # e1：IDLE → 启动复位成功（~4 s 后 car_start + epoch）；**不抑制**（令启动定时器经门控通过）
        node.set_state(AgcTaskState.IDLE)
        cs0 = node.car_start_count
        ec0 = node.epoch_count
        proc, log, _, _t = start_reset_node(share, out_dir, 'e1', {}, out_dir + '/log')
        try:
            wait_service(SERVICE, 25.0)
            arrived, dt = wait_until(lambda: node.car_start_count > cs0, 6.5)
            e['idle_car_start_arrived'] = bool(arrived)
            e['idle_car_start_delay_s'] = round(dt, 3)
            e['idle_epoch_published'] = bool(node.epoch_count > ec0)
        finally:
            stop_reset_node(proc, log)
        # e2：GROUND_APPROACH → 启动复位被门控拒绝（6 s 内无 car_start）
        node.set_state(AgcTaskState.GROUND_APPROACH)
        cs1 = node.car_start_count
        ec1 = node.epoch_count
        proc, log, _, _t = start_reset_node(share, out_dir, 'e2', {}, out_dir + '/log')
        try:
            wait_service(SERVICE, 25.0)
            time.sleep(6.0)
            e['active_car_start_arrived'] = bool(node.car_start_count > cs1)
            e['active_epoch_published'] = bool(node.epoch_count > ec1)
        finally:
            stop_reset_node(proc, log)
        e['pass'] = bool(
            e.get('idle_car_start_arrived') and e.get('idle_epoch_published') and
            not e.get('active_car_start_arrived') and not e.get('active_epoch_published'))
        res['tests']['e_startup_gated'] = e

        res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
        return dump(res, out_dir)
    finally:
        stop_event.set()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def dump(res, out_dir):
    path = os.path.join(out_dir, 'metrics', 'r015.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-28s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
        for k in ('success', 'reason', 'car_start_arrived', 'rollback_dist_m',
                  'dist_from_new_spawn_m', 'idle_car_start_delay_s', 'active_car_start_arrived'):
            if k in t:
                print('      %s = %s' % (k, t[k]))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


if __name__ == '__main__':
    sys.exit(main())
