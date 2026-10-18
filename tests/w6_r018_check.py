#!/usr/bin/env python3
"""w6_r018_check —— 评审 R-018（干扰真实行为确认）自包含 ROS 级验收 harness。

结构拷贝自 `tests/w6_r017_check.py`：**本脚本不起 launch**（自包含），自己充当「假 PX4」与
「task_manager 桩环境」，以 subprocess 直接起 `install/agc_task/lib/agc_task/{jammer, task_manager}`
+ 临时参数文件，无需 gz / launch。

假 PX4（本 harness 节点）：
  * 发布 `/px4_1/fmu/out/vehicle_status`（VehicleStatus，BEST_EFFORT，20 Hz；nav_state 由用例
    可控行为 `px4_behavior` 决定：`none` 不发 / `offboard` 恒 14 / `reach_hold` 收到命令后达命令模式
    并保持 / `reach_brief` 达命令模式仅 `brief_s` 后退出）。
  * 订 `/px4_1/fmu/in/vehicle_command`（BEST_EFFORT）观测 jammer 注入（`target_system` 不匹配时
    可选忽略 → 模式不达）。
  * 发布 `/agc/task/envelope_in`（Bool，可控）+ 订 `/agc/task/jam_state`（授权）/`jam_confirm`
    （确认）/`jam_evidence`（证据）。
  * task_manager 桩环境：host `/agc/scenario/reset`（Trigger，恒成功）；发布
    `/agc/ground/odom_valid`（Bool true）/`/agc/ground/odom`（Odometry）/
    `/agc/ground/cmd_vel`（TwistStamped）/`/agc/targets`（TargetTrackArray）；订 `/agc/task/state`；
    client `/agc/task/start`（`air_present=false`，跳过相位复位）。

用例（结果落 `experiments/w6-task-eval/<run_id>/metrics/r018.json`，ADR-0007）：
  a 无 VehicleStatus → 不确认，负例 `JAM_FAIL_TARGET_ABSENT`（或 `JAM_FAIL_NO_FEEDBACK`）。
  b 命令被拒 / 无状态流：
    b1 `target_system` 不匹配（假 PX4 仅认 sysid=2，jammer 发 99）→ 模式不达 → `MODE_UNREACHED`。
    b2 无状态流（jammer `vehicle_status_topic` 指无发布者话题）→ `TARGET_ABSENT`/`NO_FEEDBACK`。
  c 窗口内有状态但未达模式（恒 14）→ `JAM_FAIL_MODE_UNREACHED`。
  d 模式短暂进入后退出（0.4 s < jam_sustain_s）→ `JAM_FAIL_NOT_SUSTAINED`。
  e 模式持续 ≥ jam_sustain_s → `jam_confirm` 出 `JAM_RTL` **且** `task_manager` 进入
    `INTERCEPT_CONFIRMED`（`reason_code=JAM_RTL`）。
  f 仅发 `/agc/task/jam_state`（授权）无真实状态证据 → `task_manager` **不**进入
    `INTERCEPT_CONFIRMED`。

注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身 → 自杀）；用 "[x]xx" 写法规避或 Popen 句柄 kill。
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from ament_index_python.packages import get_package_share_directory

from agc_msgs.msg import AgcTaskState, TargetTrack, TargetTrackArray
from geometry_msgs.msg import PoseWithCovariance, TwistStamped, TwistWithCovariance
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommand, VehicleStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

NS = 'px4_1'
STATUS_TOPIC = f'/{NS}/fmu/out/vehicle_status'
JAM_CMD_TOPIC = '/agc/task/jam_cmd'
JAM_STATE_TOPIC = '/agc/task/jam_state'
JAM_CONFIRM_TOPIC = '/agc/task/jam_confirm'
JAM_EVIDENCE_TOPIC = '/agc/task/jam_evidence'
ENV_TOPIC = '/agc/task/envelope_in'
STATE_TOPIC = '/agc/task/state'
START_SRV = '/agc/task/start'
RESET_SRV = '/agc/scenario/reset'
ODOM_VALID = '/agc/ground/odom_valid'
ODOM = '/agc/ground/odom'
CMD = '/agc/ground/cmd_vel'
TARGETS = '/agc/targets'
NAV_STATE_CMD = 100001   # VEHICLE_CMD_SET_NAV_STATE
JAM_NAV_PARAM1 = (5.0, 18.0, 4.0)   # RTL / LAND / HOLD
POSITIVE = ('JAM_RTL', 'JAM_LAND', 'JAM_HOLD', 'JAM_LOST')
NO_FEEDBACK_TOKENS = ('JAM_FAIL_TARGET_ABSENT', 'JAM_FAIL_NO_FEEDBACK')
PX4_SYSID = 2
DT = 0.05


def _qos():
    q = QoSProfile(depth=5)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    return q


class R18Harness(Node):
    """假 PX4 + task_manager 桩环境（单节点，timer 20 Hz 持续发布可控输入）。"""

    def __init__(self):
        super().__init__('w6_r018_check')
        p = f'/{NS}/fmu'
        self.pub_status = self.create_publisher(VehicleStatus, STATUS_TOPIC, _qos())
        self.create_subscription(VehicleCommand, f'{p}/in/vehicle_command', self._on_px4_cmd, _qos())
        self.pub_env = self.create_publisher(Bool, ENV_TOPIC, 10)
        self.pub_jam_cmd = self.create_publisher(String, JAM_CMD_TOPIC, 10)
        self.pub_jam_state = self.create_publisher(String, JAM_STATE_TOPIC, 10)
        # task_manager 桩环境
        self.pub_ov = self.create_publisher(Bool, ODOM_VALID, 10)
        self.pub_odom = self.create_publisher(Odometry, ODOM, 10)
        self.pub_cmd = self.create_publisher(TwistStamped, CMD, 10)
        self.pub_tgt = self.create_publisher(TargetTrackArray, TARGETS, 10)
        self.create_service(Trigger, RESET_SRV, self._on_reset)
        self.start_cli = self.create_client(Trigger, START_SRV)

        self.create_subscription(String, JAM_STATE_TOPIC, self._on_jam_state, 10)
        self.create_subscription(String, JAM_CONFIRM_TOPIC, self._on_jam_confirm, 10)
        self.create_subscription(String, JAM_EVIDENCE_TOPIC, self._on_jam_evidence, 10)
        self.create_subscription(AgcTaskState, STATE_TOPIC, self._on_state, 20)

        self.create_timer(DT, self._pub_tick)

        # 可控开关
        self.ground_on_ = False
        self.px4_on_ = False
        self.env_on_ = False
        self.env_value_ = True
        # PX4 行为
        self.px4_behavior = 'offboard'      # none|offboard|reach_hold|reach_brief
        self.px4_honor_only_matching = False
        self.px4_sysid = PX4_SYSID
        self.brief_s = 0.4
        self.cmd_t = None
        self.cmd_nav = 14
        # 观测缓冲
        self.jam_states = []      # (t, data)
        self.jam_confirms = []    # (t, data)
        self.jam_evidence = []    # (t, data)
        self.px4_cmds = []        # (t, command, param1, target_system)
        self.state = 0
        self.state_name = ''
        self.reason_code = ''
        self.target_xy = (30.0, 30.0)   # 远 → 不触 C8 跟踪里程碑

    # ---- callbacks ----
    def _on_reset(self, _req, res):
        res.success = True
        res.message = 'stub reset ok'
        return res

    def _on_px4_cmd(self, m):
        self.px4_cmds.append((time.monotonic(), int(m.command), float(m.param1),
                              int(m.target_system)))
        if int(m.command) != NAV_STATE_CMD:
            return
        if not any(abs(float(m.param1) - v) < 0.5 for v in JAM_NAV_PARAM1):
            return
        if self.px4_honor_only_matching and int(m.target_system) != self.px4_sysid:
            return   # target_system 不匹配 → 忽略，模式不达
        self.cmd_t = time.monotonic()
        self.cmd_nav = int(round(float(m.param1)))

    def _on_jam_state(self, m):
        self.jam_states.append((time.monotonic(), m.data.strip().upper()))

    def _on_jam_confirm(self, m):
        self.jam_confirms.append((time.monotonic(), m.data.strip().upper()))

    def _on_jam_evidence(self, m):
        self.jam_evidence.append((time.monotonic(), m.data.strip()))

    def _on_state(self, m):
        self.state = m.state
        self.state_name = m.state_name
        self.reason_code = m.reason_code

    # ---- 发布 ----
    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _px4_status(self):
        if self.px4_behavior == 'none':
            return None
        if self.px4_behavior in ('reach_hold', 'reach_brief') and self.cmd_t is not None:
            if self.px4_behavior == 'reach_brief' and (time.monotonic() - self.cmd_t) >= self.brief_s:
                return (2, 14)
            return (2, self.cmd_nav)
        return (2, 14)

    def _pub_tick(self):
        if self.px4_on_:
            st = self._px4_status()
            if st is not None:
                a, n = st
                msg = VehicleStatus()
                msg.timestamp = self._now_us()
                msg.arming_state = a
                msg.nav_state = n
                msg.system_id = self.px4_sysid
                msg.failsafe = False
                self.pub_status.publish(msg)
        if self.env_on_:
            self.pub_env.publish(Bool(data=bool(self.env_value_)))
        if self.ground_on_:
            ts = self.get_clock().now().to_msg()
            self.pub_ov.publish(Bool(data=True))
            o = Odometry()
            o.header.stamp = ts
            o.header.frame_id = 'world'
            o.child_frame_id = 'car/base_link'
            o.pose.pose.orientation.w = 1.0
            self.pub_odom.publish(o)
            c = TwistStamped()
            c.header.stamp = ts
            c.header.frame_id = 'world'
            self.pub_cmd.publish(c)
            arr = TargetTrackArray()
            arr.header.stamp = ts
            arr.header.frame_id = 'world'
            t = TargetTrack()
            t.header = arr.header
            t.target_id = 'intruder_uav'
            t.pose = PoseWithCovariance()
            t.pose.pose.position.x = self.target_xy[0]
            t.pose.pose.position.y = self.target_xy[1]
            t.pose.pose.position.z = 2.0
            t.pose.pose.orientation.w = 1.0
            t.twist = TwistWithCovariance()
            t.confidence = 0.9
            t.source = 'stub'
            arr.tracks = [t]
            self.pub_tgt.publish(arr)

    # ---- 驱动 ----
    def run(self, dur):
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_until(self, cond, max_dur):
        t0 = time.monotonic()
        while time.monotonic() - t0 < max_dur:
            rclpy.spin_once(self, timeout_sec=0.02)
            if cond():
                return True
        return False

    def publish_jam_cmd(self, s):
        self.pub_jam_cmd.publish(String(data=s))
        return time.monotonic()

    def call_start(self, timeout=20.0):
        fut = self.start_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return None
        return fut.result()

    # ---- 缓冲清理 ----
    def reset_buffers(self):
        self.jam_states.clear()
        self.jam_confirms.clear()
        self.jam_evidence.clear()
        self.px4_cmds.clear()
        self.cmd_t = None
        self.cmd_nav = 14


# ---- 进程管理 ----
def _exe(share, name):
    return str(Path(share).parents[1] / 'lib' / 'agc_task' / name)


def _write_params(share, path, node_name, overrides):
    base = yaml.safe_load(open(os.path.join(share, 'config', 'w6_base.yaml')))
    params = dict((base.get(node_name) or {}).get('ros__parameters', {}))
    params.update(overrides)
    params['use_sim_time'] = False   # 墙钟，规避无 /clock 时间冻结
    doc = {node_name: {'ros__parameters': params}}
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
    return params


def _start_node(share, out_dir, exe_name, node_name, prefix, overrides):
    path = os.path.join(out_dir, 'r018_%s_%s.yaml' % (prefix, node_name))
    _write_params(share, path, node_name, overrides)
    log = open(os.path.join(out_dir, 'log', 'case_%s_%s.log' % (prefix, node_name)), 'w')
    proc = subprocess.Popen(
        [_exe(share, exe_name), '--ros-args', '--params-file', path],
        stdout=log, stderr=subprocess.STDOUT)
    return proc, log, path


def _stop_proc(proc, log):
    if proc is None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=3)
    except Exception:
        try:
            proc.terminate()
            proc.wait(timeout=3)
        except Exception:
            try:
                proc.kill()
                proc.wait(timeout=3)
            except Exception:
                pass
    try:
        log.close()
    except Exception:
        pass


def _auth(node, since=0.0):
    return [d for t, d in node.jam_states
            if t >= since and d.startswith('JAM_') and d != 'JAM_NONE']


def _confirms(node, since=0.0):
    return [d for t, d in node.jam_confirms if t >= since and d in POSITIVE]


def _evidence(node, since=0.0):
    return [d for t, d in node.jam_evidence if t >= since]


def _has_neg(ev, tokens):
    return any(e.split()[0] in tokens for e in ev)


def _inject_until_auth(node, cmd, timeout=6.0):
    """反复发干扰命令直至观测到授权事件（JAM_*）——规避 DDS 发现延迟导致的丢包。

    返回首个授权事件的到达墙钟时刻；超时返回 None。授权事件仅由 jammer 对「合法 + 包络新鲜」
    命令产出，故其到达即证明命令已送达且包络有效（窗口从此命令起算）。
    """
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        node.publish_jam_cmd(cmd)
        node.run(0.15)
        if node.jam_states:
            return node.jam_states[0][0]
    return None


# ---- 用例 ----
def case_a(node, share, out_dir):
    """无 VehicleStatus（假 PX4 不发状态）→ 不确认，负例 TARGET_ABSENT/NO_FEEDBACK。"""
    res = {}
    node.reset_buffers()
    node.ground_on_ = False
    node.px4_on_ = False
    node.px4_behavior = 'none'
    node.env_on_ = True
    node.env_value_ = True
    proc, log, _ = _start_node(share, out_dir, 'jammer', 'jammer', 'a', {})
    try:
        node.run(1.5)                       # 发现 + 包络新鲜
        t = _inject_until_auth(node, 'RTL')
        node.run(3.8)                       # > 确认窗口 3.0 s
        auth = _auth(node, t if t else 0.0)
        cf = _confirms(node, t if t else 0.0)
        ev = _evidence(node, t if t else 0.0)
        res['authorized_events'] = auth
        res['confirm_events'] = cf
        res['evidence'] = ev
        res['pass'] = bool(
            t is not None and 'JAM_RTL' in auth and not cf and
            _has_neg(ev, NO_FEEDBACK_TOKENS))
    finally:
        _stop_proc(proc, log)
    return res


def case_b(node, share, out_dir):
    """命令被拒（target_system 不匹配）/ 无状态流 → 不确认。"""
    res = {}
    # b1：target_system 不匹配（假 PX4 仅认 sysid=2；jammer 发 99 → 命令被忽略 → 模式不达）
    node.reset_buffers()
    node.ground_on_ = False
    node.px4_on_ = True
    node.px4_behavior = 'offboard'
    node.px4_honor_only_matching = True
    node.env_on_ = True
    node.env_value_ = True
    proc, log, _ = _start_node(share, out_dir, 'jammer', 'jammer', 'b1',
                               {'target_system': 99, 'jam_confirm_window_s': 2.0})
    try:
        node.run(1.5)
        t = _inject_until_auth(node, 'RTL')
        node.run(2.8)
        auth = _auth(node, t if t else 0.0)
        cf = _confirms(node, t if t else 0.0)
        ev = _evidence(node, t if t else 0.0)
        res['b1_target_system_mismatch'] = {
            'authorized_events': auth, 'confirm_events': cf, 'evidence': ev,
            'pass': bool(t is not None and 'JAM_RTL' in auth and not cf and
                         _has_neg(ev, ('JAM_FAIL_MODE_UNREACHED',)))}
    finally:
        _stop_proc(proc, log)
        node.px4_honor_only_matching = False

    # b2：无状态流（jammer 订无发布者话题）
    node.reset_buffers()
    node.px4_on_ = False
    node.px4_behavior = 'none'
    node.env_on_ = True
    node.env_value_ = True
    proc, log, _ = _start_node(share, out_dir, 'jammer', 'jammer', 'b2',
                               {'vehicle_status_topic': '/px4_1/fmu/out/vehicle_status_none',
                                'jam_confirm_window_s': 2.0})
    try:
        node.run(1.5)
        t = _inject_until_auth(node, 'RTL')
        node.run(2.8)
        auth = _auth(node, t if t else 0.0)
        cf = _confirms(node, t if t else 0.0)
        ev = _evidence(node, t if t else 0.0)
        res['b2_no_status_stream'] = {
            'authorized_events': auth, 'confirm_events': cf, 'evidence': ev,
            'pass': bool(t is not None and 'JAM_RTL' in auth and not cf and
                         _has_neg(ev, NO_FEEDBACK_TOKENS))}
    finally:
        _stop_proc(proc, log)

    res['pass'] = bool(res['b1_target_system_mismatch']['pass'] and
                       res['b2_no_status_stream']['pass'])
    return res


def case_c(node, share, out_dir):
    """窗口内有状态但未达模式（恒 14）→ MODE_UNREACHED。"""
    res = {}
    node.reset_buffers()
    node.ground_on_ = False
    node.px4_on_ = True
    node.px4_behavior = 'offboard'
    node.env_on_ = True
    node.env_value_ = True
    proc, log, _ = _start_node(share, out_dir, 'jammer', 'jammer', 'c',
                               {'jam_confirm_window_s': 2.0})
    try:
        node.run(1.5)
        t = _inject_until_auth(node, 'RTL')
        node.run(2.8)
        auth = _auth(node, t if t else 0.0)
        cf = _confirms(node, t if t else 0.0)
        ev = _evidence(node, t if t else 0.0)
        res['authorized_events'] = auth
        res['confirm_events'] = cf
        res['evidence'] = ev
        res['pass'] = bool(
            t is not None and 'JAM_RTL' in auth and not cf and
            _has_neg(ev, ('JAM_FAIL_MODE_UNREACHED',)))
    finally:
        _stop_proc(proc, log)
    return res


def case_d(node, share, out_dir):
    """模式短暂进入后退出（0.4 s < jam_sustain_s）→ NOT_SUSTAINED。"""
    res = {}
    node.reset_buffers()
    node.ground_on_ = False
    node.px4_on_ = True
    node.px4_behavior = 'reach_brief'
    node.brief_s = 0.4
    node.env_on_ = True
    node.env_value_ = True
    proc, log, _ = _start_node(share, out_dir, 'jammer', 'jammer', 'd',
                               {'jam_confirm_window_s': 2.0, 'jam_sustain_s': 1.0})
    try:
        node.run(1.5)
        t = _inject_until_auth(node, 'RTL')
        node.run(2.8)
        auth = _auth(node, t if t else 0.0)
        cf = _confirms(node, t if t else 0.0)
        ev = _evidence(node, t if t else 0.0)
        res['authorized_events'] = auth
        res['confirm_events'] = cf
        res['evidence'] = ev
        res['pass'] = bool(
            t is not None and 'JAM_RTL' in auth and not cf and
            _has_neg(ev, ('JAM_FAIL_NOT_SUSTAINED',)))
    finally:
        _stop_proc(proc, log)
        node.brief_s = 0.4
    return res


def _drive_task_manager(node, share, out_dir, prefix):
    """起 task_manager（air_present=false）→ start → 驱动至 GROUND_APPROACH。"""
    proc, log, _ = _start_node(share, out_dir, 'task_manager', 'task_manager', prefix,
                               {'air_present': False})
    node.ground_on_ = True
    node.px4_on_ = False
    ok = node.run_until(lambda: node.start_cli.service_is_ready(), 15.0)
    if not ok:
        return proc, log, False, 'start service never ready'
    res = node.call_start(20.0)
    if res is None:
        return proc, log, False, 'start call timeout'
    if not res.success:
        return proc, log, False, 'start rejected: %s' % res.message
    reached = node.run_until(lambda: node.state_name == 'GROUND_APPROACH', 15.0)
    return proc, log, reached, ('GROUND_APPROACH' if reached else
                                'did not reach GROUND_APPROACH (state=%s)' % node.state_name)


def case_e(node, share, out_dir):
    """模式持续 ≥ jam_sustain_s → jam_confirm 出 JAM_RTL 且 task_manager → INTERCEPT_CONFIRMED。"""
    res = {}
    node.reset_buffers()
    node.env_on_ = False
    node.px4_on_ = False
    node.px4_behavior = 'none'
    tm, tlog, reached, note = _drive_task_manager(node, share, out_dir, 'e')
    res['task_manager_reached'] = reached
    res['drive_note'] = note
    proc, log = None, None
    try:
        if not reached:
            res['pass'] = False
            return res
        # 起 jammer + 假 PX4「收到命令后达目标模式并保持」
        node.px4_behavior = 'reach_hold'
        node.px4_on_ = True
        node.env_on_ = True
        node.env_value_ = True
        proc, log, _ = _start_node(share, out_dir, 'jammer', 'jammer', 'e', {})
        node.run(1.5)
        t = _inject_until_auth(node, 'RTL')
        node.run_until(
            lambda: _confirms(node, t if t else 0.0) and
            node.state_name == 'INTERCEPT_CONFIRMED', 8.0)
        cf = _confirms(node, t if t else 0.0)
        ev = _evidence(node, t if t else 0.0)
        res['authorized_events'] = _auth(node, t if t else 0.0)
        res['confirm_events'] = cf
        res['evidence'] = ev
        res['task_state_name'] = node.state_name
        res['task_reason_code'] = node.reason_code
        res['pass'] = bool(
            t is not None and 'JAM_RTL' in cf and
            node.state_name == 'INTERCEPT_CONFIRMED' and
            node.reason_code == 'JAM_RTL' and
            any(e.startswith('JAM_CONFIRMED') for e in ev))
    finally:
        _stop_proc(proc, log)
        _stop_proc(tm, tlog)
        node.env_on_ = False
        node.px4_on_ = False
    return res


def case_f(node, share, out_dir):
    """仅发授权事件 /agc/task/jam_state（无真实状态证据）→ task_manager 不进入 INTERCEPT_CONFIRMED。"""
    res = {}
    node.reset_buffers()
    node.env_on_ = False
    node.px4_on_ = False
    node.px4_behavior = 'none'
    tm, tlog, reached, note = _drive_task_manager(node, share, out_dir, 'f')
    res['task_manager_reached'] = reached
    res['drive_note'] = note
    try:
        if not reached:
            res['pass'] = False
            return res
        # 仅发布授权事件（模拟 R-017 授权 / jammer 已发命令但无真实状态反馈）
        for _ in range(10):
            node.pub_jam_state.publish(String(data='JAM_RTL'))
            node.run(0.1)
        node.run(1.0)
        res['task_state_name'] = node.state_name
        res['task_reason_code'] = node.reason_code
        res['pass'] = bool(node.state_name != 'INTERCEPT_CONFIRMED' and
                           node.reason_code not in POSITIVE)
    finally:
        _stop_proc(tm, tlog)
    return res


def dump(res, out_dir):
    path = os.path.join(out_dir, 'metrics', 'r018.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-32s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
        for k, v in t.items():
            if k != 'pass':
                print('      %s = %s' % (k, v))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r018-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    share = get_package_share_directory('agc_task')
    res = {'run_id': run_id, 'ns': NS, 'tests': {}}

    rclpy.init()
    node = R18Harness()
    try:
        res['tests']['a_no_status_no_confirm'] = case_a(node, share, out_dir)
        res['tests']['b_rejected_or_no_stream'] = case_b(node, share, out_dir)
        res['tests']['c_mode_unreached'] = case_c(node, share, out_dir)
        res['tests']['d_not_sustained'] = case_d(node, share, out_dir)
        res['tests']['e_sustained_confirmed'] = case_e(node, share, out_dir)
        res['tests']['f_auth_only_no_confirm'] = case_f(node, share, out_dir)
        res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    except Exception as exc:   # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return dump(res, out_dir)


if __name__ == '__main__':
    sys.exit(main())
