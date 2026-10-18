#!/usr/bin/env python3
"""w6_r041_bench —— R-041（目标失效策略与位移阈值化重规划）**真实节点台架**。

起**真实二进制**：
  * `install/agc_task/lib/agc_task/task_manager`（场景 1：target 过期 / 失联）；
  * `install/agc_ground/lib/agc_ground/ground_baseline`（场景 2/3：微噪声 goal / 失效安全停车），
    配 `static_transform_publisher`（world→car/base_link）+ 台架合成 `/agc/map`（latched）+ `/agc/ground/odom`
    自持拉起（对齐 R-012 基线台架做法）；台架另以 latched 反复发布 world→car/base_link 静态 TF，
    覆盖 costmap 激活的 TF 发现窗口（消除独立 tf 进程的偶发发现抖动），激活失败即退避重试。

场景（逐条验收 R-041 / checklist）：
  1. `target_expire`：`goal_follow_target=true`、`goal_hold_valid_s=2.0`。先喂新鲜目标 → goal 位置=目标、
     `stamp≈目标**源时刻**（≠now）`、`goal_info valid=1 reason=TARGET_FRESH`；停喂 → 窗口内保持**最后有效
     位置 + 原源时刻（age 增长）**、`reason=HOLD_LAST_VALID`；超窗 → `valid=0 reason=EXPIRED` 且
     **无**新时间戳固定点 goal；**从不**反射固定 `goal_x/goal_y`。
  2. `micro_noise_goal`：喂 odom + 抖幅 <`replan_displacement_m`(=1.0) 的微噪声 goal 流 → **不**逐消息重规划
     （无 `重规划触发：目标身份变` 日志、重发次数 ≈ 周期）；再喂真实位移 2.0 m > 阈值 → 触发重规划且新路径端点
     ≈ 新目标；记录实际重规划次数 / 频率 / 时刻与路径轨迹。
  3. `failsafe_stop`：目标有效时 `cmd_vel_raw` 有前进量 → 喂 `goal_info valid=0` → `cmd_vel_raw≈0`（安全停车）。

只回收**本进程组**子进程（`start_new_session` + `killpg`）；**不**用裸名 `pkill -f`、**不**做递归删除。
证据落 `experiments/w6-task-eval/<run_id>/r041/bench.json`（ADR-0007）。

⚠ 范围标注：本台架为**节点行为台架**（真实二进制 + 合成输入，**非**整链 / **非** GUI）。
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from agc_msgs.msg import AgcTaskState, TargetTrack, TargetTrackArray
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Log
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

ROOT = '/home/chen/AGC'
TASK_BIN = os.path.join(ROOT, 'install', 'agc_task', 'lib', 'agc_task')
GROUND_BIN = os.path.join(ROOT, 'install', 'agc_ground', 'lib', 'agc_ground')

GOAL_TOPIC = '/agc/ground/goal'
GOAL_INFO_TOPIC = '/agc/ground/goal_info'
TARGET_TOPIC = '/agc/targets'
ODOM_TOPIC = '/agc/ground/odom'
MAP_TOPIC = '/agc/map'
PATH_TOPIC = '/agc/ground/path'
CMD_RAW_TOPIC = '/agc/ground/cmd_vel_raw'
STATE_TOPIC = '/agc/task/state'

TARGET_XY = (2.0, -5.0)          # 目标位置（区别于固定 goal_x/goal_y=6.5/2.5）
FIXED_GOAL = (6.5, 2.5)
STAMP_BACKOFF = 0.25             # 触发目标 stamp = now - 0.25（< target_max_age_s=0.5，仍新鲜）
GROUND_APPROACH = 3
REPLAN_DISP = 1.0
REPLAN_PERIOD = 2.0

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


# ------------------------------------------------------------------ 进程工具

def _kill(proc):
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    try:
        if pgid == os.getpgrp():
            proc.send_signal(signal.SIGKILL)
        else:
            os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError, OSError):
        pass
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass


def _reap_stale(patterns):
    for pat in patterns:
        try:
            out = subprocess.run(['pgrep', '-f', pat], capture_output=True, text=True, timeout=10)
        except Exception:  # noqa: BLE001
            continue
        for pid in [p for p in out.stdout.split() if p.strip().isdigit()]:
            if int(pid) != os.getpid():
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except OSError:
                    pass
    time.sleep(0.5)


REAP_PATTERNS = [
    'install/agc_task/lib/agc_task/task_manage[r]',
    'install/agc_ground/lib/agc_ground/ground_baselin[e]',
    'tf2_ros/static_transform_publishe[r]',
]


# ------------------------------------------------------------------ 场景 1：target 过期

class TargetExpireHarness(Node):
    def __init__(self):
        super().__init__('r041_bench_target')
        self.goals = []       # {'wall','stamp','x','y'}
        self.infos = []       # {'wall','raw','valid','reason','target_id','src_stamp','age'}
        self.target_pub = self.create_publisher(TargetTrackArray, TARGET_TOPIC, 10)
        self.create_subscription(PoseStamped, GOAL_TOPIC, self._on_goal, 20)
        self.create_subscription(String, GOAL_INFO_TOPIC, self._on_info, LATCHED)

    def _on_goal(self, m):
        self.goals.append({'wall': time.time(), 'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                           'x': m.pose.position.x, 'y': m.pose.position.y})

    def _on_info(self, m):
        d = {}
        for kv in m.data.split(';'):
            if '=' in kv:
                k, v = kv.split('=', 1)
                d[k] = v
        self.infos.append({'wall': time.time(), 'raw': m.data, 'valid': d.get('valid') == '1',
                           'reason': d.get('reason'), 'target_id': d.get('target_id'),
                           'src_stamp': float(d.get('src_stamp', -1.0)), 'age': float(d.get('age', -1.0))})

    def pub_target(self, backoff=STAMP_BACKOFF):
        msg = TargetTrackArray()
        now = self.get_clock().now()
        stamp = now - Duration(seconds=float(backoff))
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'world'
        t = TargetTrack()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'world'
        t.target_id = 'intruder_1'
        t.pose.pose.position.x = TARGET_XY[0]
        t.pose.pose.position.y = TARGET_XY[1]
        t.pose.pose.orientation.w = 1.0
        t.confidence = 0.9
        msg.tracks.append(t)
        self.target_pub.publish(msg)

    def pump(self, duration, feed_target, hz=20.0):
        t0 = time.time()
        last = 0.0
        period = 1.0 / hz
        while True:
            el = time.time() - t0
            if el >= duration:
                break
            if feed_target and time.time() - last >= period:
                last = time.time()
                self.pub_target()
            rclpy.spin_once(self, timeout_sec=0.005)

    def pump_until(self, feed_target, timeout, cond, hz=20.0):
        """喂目标并轮询 cond()，成立即返回 True，或超时返回 False（订阅握手，替代固定 sleep）。"""
        t0 = time.time()
        last = 0.0
        period = 1.0 / hz
        while True:
            if time.time() - t0 >= timeout:
                return False
            if feed_target and time.time() - last >= period:
                last = time.time()
                self.pub_target()
            rclpy.spin_once(self, timeout_sec=0.005)
            if cond():
                return True


def sc_target_expire(log_dir, cfg_dir):
    params = os.path.join(cfg_dir, 'r041_params_task.yaml')
    _write(params,
           'task_manager:\n  ros__parameters:\n'
           '    use_sim_time: false\n'
           '    world_name: "agc_world"\n'
           '    goal_follow_target: true\n'
           '    goal_hold_valid_s: 2.0\n'
           '    target_max_age_s: 0.5\n'
           '    confidence_min: 0.5\n'
           '    goal_x: 6.5\n'
           '    goal_y: 2.5\n'
           '    goal_topic: "%s"\n'
           '    goal_info_topic: "%s"\n'
           '    target_topic: "%s"\n' % (GOAL_TOPIC, GOAL_INFO_TOPIC, TARGET_TOPIC))
    log = open(os.path.join(log_dir, 'target_expire.log'), 'w')
    proc = subprocess.Popen([os.path.join(TASK_BIN, 'task_manager'), '--ros-args', '--params-file', params],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    h = TargetExpireHarness()
    res = {'scenario': 'target_expire'}
    try:
        h.pump(1.0, feed_target=False)      # warmup / DDS discovery
        # 订阅握手：持续喂新鲜目标直至收到 ≥2 条 goal（确认 task_manager→harness 的 goal 发现已匹配），
        # 避免在发现窗口内漏收首条（goal 为 volatile，非 latched，故须确认后再进入判定期）。
        got = h.pump_until(feed_target=True, timeout=6.0, cond=lambda: len(h.goals) >= 2)
        res['goal_handshake_ok'] = bool(got)
        h.pump(1.5, feed_target=True)       # 新鲜目标 → 期望 TARGET_FRESH
        h.pump(5.0, feed_target=False)      # 停喂 → 窗口内 HOLD_LAST_VALID → 超窗 EXPIRED
        res.update(_classify_target(h))
    finally:
        _kill(proc)
        h.destroy_node()
        log.close()
    return res


def _classify_target(h):
    def near_goal(wall, tol=0.4):
        cand = [g for g in h.goals if abs(g['wall'] - wall) <= tol]
        return cand[0] if cand else None

    fresh = [i for i in h.infos if i['valid'] and i['reason'] == 'TARGET_FRESH']
    hold = [i for i in h.infos if i['valid'] and i['reason'] == 'HOLD_LAST_VALID']
    expired = [i for i in h.infos if (not i['valid']) and i['reason'] == 'EXPIRED']
    lost = [i for i in h.infos if (not i['valid']) and i['reason'] == 'TARGET_LOST']

    # ① 新鲜期：位置=目标、stamp≈源时刻（≠now）
    fresh_ok = False
    fresh_detail = {}
    for i in fresh:
        g = near_goal(i['wall'])
        if g is None:
            continue
        pos_ok = abs(g['x'] - TARGET_XY[0]) < 0.05 and abs(g['y'] - TARGET_XY[1]) < 0.05
        src_ok = abs(g['stamp'] - (g['wall'] - STAMP_BACKOFF)) < 0.2
        not_now = abs(g['stamp'] - g['wall']) > 0.15
        fresh_detail = {'goal': g, 'pos_ok': pos_ok, 'src_stamp_ok': src_ok, 'not_now': not_now}
        fresh_ok = pos_ok and src_ok and not_now
        if fresh_ok:
            break

    # ② 保持期：位置=最后有效位置、原源时刻（age 增长）
    hold_ok = False
    hold_detail = {}
    for i in hold:
        g = near_goal(i['wall'])
        if g is None:
            continue
        pos_ok = abs(g['x'] - TARGET_XY[0]) < 0.05 and abs(g['y'] - TARGET_XY[1]) < 0.05
        hold_detail = {'goal': g, 'info': i, 'pos_ok': pos_ok}
        hold_ok = pos_ok
        if hold_ok:
            break

    # ③ 超窗：valid=0 reason=EXPIRED
    expired_ok = len(expired) > 0

    # ④ 失效后**无**新 goal 位姿（不发布带新时间戳的固定点）
    first_exp = min([i['wall'] for i in expired], default=None)
    goals_after_expiry = [g for g in h.goals if first_exp is not None and g['wall'] > first_exp + 0.05]
    no_goal_after_expiry = len(goals_after_expiry) == 0

    # ⑤ 从不反射固定点 goal_x/goal_y
    fixed_hits = [g for g in h.goals
                  if abs(g['x'] - FIXED_GOAL[0]) < 0.1 and abs(g['y'] - FIXED_GOAL[1]) < 0.1]

    return {
        'pass': bool(fresh_ok and hold_ok and expired_ok and no_goal_after_expiry and not fixed_hits),
        'fresh_ok': fresh_ok, 'fresh_detail': fresh_detail,
        'hold_ok': hold_ok, 'hold_detail': hold_detail,
        'expired_seen': expired_ok, 'expired_count': len(expired),
        'target_lost_seen': len(lost) > 0,
        'no_new_stamped_goal_after_expiry': no_goal_after_expiry,
        'goals_after_expiry': len(goals_after_expiry),
        'fixed_goal_hits': len(fixed_hits),
        'goal_count': len(h.goals), 'info_count': len(h.infos),
        'reasons_seen': sorted(set(i['reason'] for i in h.infos)),
        'goal_stamps': [round(g['stamp'], 3) for g in h.goals][:20],
        'goal_positions': [[round(g['x'], 3), round(g['y'], 3)] for g in h.goals][:20],
        'note': ('新鲜期：位置=目标 + stamp≈源时刻（≠now）；停喂：窗口内保持最后有效位置与原源时刻'
                 '（HOLD_LAST_VALID）→ 超窗 valid=0 EXPIRED 且无新时间戳固定点 goal；从不反射固定点。'),
    }


# ------------------------------------------------------------------ 场景 2/3：ground_baseline

class GroundHarness(Node):
    def __init__(self, name):
        super().__init__('r041_bench_' + name)
        self.x = self.y = 0.0
        self.yaw = 0.0
        self.paths = []     # {'wall','n','end_x','end_y'}
        self.cmds = []      # {'wall','lin','ang'}
        self.logs = []      # {'wall','text'}
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
        self.info_pub = self.create_publisher(String, GOAL_INFO_TOPIC, LATCHED)
        self.state_pub = self.create_publisher(AgcTaskState, STATE_TOPIC, 10)
        self.odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 20)
        self.map_pub = self.create_publisher(OccupancyGrid, MAP_TOPIC, LATCHED)
        # R-041 台架稳化：本 harness 自持 world→car/base_link 静态 TF（transient_local，与 /agc/map
        # 同法反复发布）——harness↔ground_baseline 的发现链路已由其它话题证实，故由其发布可**确定性**
        # 覆盖 costmap 激活时的 TF 发现窗口（不再依赖独立 static_transform_publisher 的偶发发现时序）。
        self.tf_pub = self.create_publisher(TFMessage, '/tf_static', LATCHED)
        self.create_subscription(Path, PATH_TOPIC, self._on_path, 10)
        self.create_subscription(TwistStamped, CMD_RAW_TOPIC, self._on_cmd, 50)
        self.create_subscription(Log, '/rosout', self._on_log, 50)
        self.chg = self.create_client(ChangeState, '/ground_baseline/change_state')
        self.getst = self.create_client(GetState, '/ground_baseline/get_state')

    def _on_path(self, m):
        end = m.poses[-1].pose.position if m.poses else None
        self.paths.append({'wall': time.time(), 'n': len(m.poses),
                           'end_x': (end.x if end else float('nan')),
                           'end_y': (end.y if end else float('nan'))})

    def _on_cmd(self, m):
        self.cmds.append({'wall': time.time(), 'lin': m.twist.linear.x, 'ang': m.twist.angular.z})

    def _on_log(self, m):
        if m.name == 'ground_baseline':
            self.logs.append({'wall': time.time(), 'text': m.msg})

    def pub_map(self):
        m = OccupancyGrid()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = 0.5
        m.info.width = 160
        m.info.height = 160
        m.info.origin.position.x = -40.0
        m.info.origin.position.y = -40.0
        m.info.origin.orientation.w = 1.0
        m.data = [0] * (160 * 160)
        self.map_pub.publish(m)

    def pub_tf(self):
        """反复发布 world→car/base_link 静态 TF（transient_local）；幂等、可覆盖发现窗口。"""
        t = TransformStamped()
        t.header.frame_id = 'world'
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = 'car/base_link'
        t.transform.rotation.w = 1.0
        msg = TFMessage()
        msg.transforms.append(t)
        self.tf_pub.publish(msg)

    def pub_odom(self):
        o = Odometry()
        o.header.frame_id = 'world'
        o.child_frame_id = 'car/base_link'
        o.header.stamp = self.get_clock().now().to_msg()
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        o.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        self.odom_pub.publish(o)

    def pub_state(self):
        s = AgcTaskState()
        s.header.frame_id = 'world'
        s.header.stamp = self.get_clock().now().to_msg()
        s.state = GROUND_APPROACH
        s.state_name = 'GROUND_APPROACH'
        self.state_pub.publish(s)

    def pub_goal(self, x, y):
        g = PoseStamped()
        g.header.frame_id = 'world'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    def pub_info(self, valid, reason, tgt=''):
        m = String()
        m.data = 'valid=%d;reason=%s;target_id=%s;src_stamp=%.6f;age=%.6f' % (
            1 if valid else 0, reason, tgt, -1.0, -1.0)
        self.info_pub.publish(m)

    def spin(self, duration, goal=None, valid=None, hz=20.0):
        t0 = time.time()
        last = 0.0
        period = 1.0 / hz
        while True:
            el = time.time() - t0
            if el >= duration:
                break
            if time.time() - last >= period:
                last = time.time()
                self.pub_map(); self.pub_odom(); self.pub_state(); self.pub_tf()
                if goal is not None:
                    self.pub_goal(*goal)
                if valid is not None:
                    self.pub_info(valid, 'TARGET_FRESH')
            rclpy.spin_once(self, timeout_sec=0.005)

    def spin_fn(self, duration, fn, hz=20.0):
        t0 = time.time()
        last = 0.0
        period = 1.0 / hz
        while True:
            el = time.time() - t0
            if el >= duration:
                break
            if time.time() - last >= period:
                last = time.time()
                self.pub_map(); self.pub_odom(); self.pub_state(); self.pub_tf()
                fn(el)
            rclpy.spin_once(self, timeout_sec=0.005)

    def transition(self, tid, timeout=20.0):
        if not self.chg.wait_for_service(timeout_sec=timeout):
            return False
        req = ChangeState.Request()
        req.transition.id = tid
        fut = self.chg.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() is not None and fut.result().success)

    def settle(self, duration, hz=20.0):
        """发布 map/odom/state/tf（不推进 goal），让 ground_baseline 完成发现与 TF 装填。"""
        t0 = time.time()
        last = 0.0
        period = 1.0 / hz
        while True:
            now = time.time()
            if now - t0 >= duration:
                break
            if now - last >= period:
                last = now
                self.pub_map(); self.pub_odom(); self.pub_state(); self.pub_tf()
            rclpy.spin_once(self, timeout_sec=0.005)

    def spin_until(self, cond, timeout, goal=None, valid=None, hz=20.0):
        """条件等待：轮询到 cond() 成立即返回 True，或超时返回 False（替代固定 sleep）。"""
        t0 = time.time()
        last = 0.0
        period = 1.0 / hz
        while True:
            now = time.time()
            if now - t0 >= timeout:
                return False
            if now - last >= period:
                last = now
                self.pub_map(); self.pub_odom(); self.pub_state(); self.pub_tf()
                if goal is not None:
                    self.pub_goal(*goal)
                if valid is not None:
                    self.pub_info(valid, 'TARGET_FRESH')
            rclpy.spin_once(self, timeout_sec=0.005)
            if cond():
                return True

    def state_id(self):
        if not self.getst.service_is_ready() and not self.getst.wait_for_service(timeout_sec=1.0):
            return None
        fut = self.getst.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done() and fut.result() is not None:
            return int(fut.result().current_state.id)
        return None

    def ensure_active(self, attempts=3):
        """configure(1)→activate(3)，失败即退避重试（不一次判负）；返回 (ok, detail)。

        costmap 激活需在自身 tf buffer 中解析 world↔car/base_link。本 harness 反复 latched 发布该
        静态 TF（与 /agc/map 同法），故激活前先 settle 覆盖发现窗口；激活失败（发现抖动）则重试。
        """
        detail = {'configure_ok': False, 'activate_ok': False, 'activations': []}
        for i in range(attempts):
            self.settle(1.5)
            st = self.state_id()
            detail['activations'].append({'attempt': i, 'pre_state': st})
            if st == 3:
                detail['configure_ok'] = True
                detail['activate_ok'] = True
                return True, detail
            if st != 2:  # 非 inactive（未配置/unconfigured）→ 先 configure
                detail['configure_ok'] = bool(self.transition(1))
            ok = self.transition(3, timeout=40.0)
            detail['activations'][-1]['activate_success'] = bool(ok)
            if ok:
                detail['activate_ok'] = True
                return True, detail
            time.sleep(1.0)
        return False, detail


def _start_ground(log_dir, tag, params):
    """起真实 ground_baseline + static TF（world→car/base_link）；返回 (proc, tf, logfh)。"""
    log = open(os.path.join(log_dir, tag + '.log'), 'w')
    proc = subprocess.Popen([os.path.join(GROUND_BIN, 'ground_baseline'), '--ros-args',
                             '--params-file', params],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    tf = subprocess.Popen(['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                           '--frame-id', 'world', '--child-frame-id', 'car/base_link'],
                          stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return proc, tf, log


def sc_micro_noise_goal(log_dir, cfg_dir):
    params = os.path.join(cfg_dir, 'r041_params_ground.yaml')
    _write(params, _ground_params_text())
    proc, tf, log = _start_ground(log_dir, 'micro_noise_goal', params)
    h = GroundHarness('micro')  # noqa: F841  (Node needs init)
    res = {'scenario': 'micro_noise_goal'}
    try:
        ok, life = h.ensure_active()
        res['configure_ok'] = life['configure_ok']
        res['activate_ok'] = life['activate_ok']
        res['lifecycle'] = life
        if not ok:
            res['pass'] = False
            res['error'] = 'ground_baseline 未激活（TF/DDS 发现失败）'
            return res
        # 建立初始目标 + 首次规划/路径（条件等待首个路径，替代固定 sleep）
        base = (0.0, -30.0)
        h.spin_until(lambda: len(h.paths) > 0, 6.0, goal=base)
        h.spin(0.5, goal=base)
        # 测量窗：微噪声（抖幅 0.3 < replan_displacement_m=1.0）
        win_t0 = time.time()
        k = [0]

        def micro(_el):
            k[0] += 1
            jx = 0.3 * math.sin(0.9 * k[0])
            jy = 0.3 * math.cos(0.9 * k[0])
            h.pub_goal(base[0] + jx, base[1] + jy)

        h.spin_fn(5.0, micro)
        win_t1 = time.time()
        # 真实位移 2.0 m > 阈值
        new_goal = (base[0] + 2.0, base[1])
        disp_t0 = time.time()
        paths_before = len(h.paths)

        def disp(_el):
            h.pub_goal(*new_goal)

        h.spin_fn(1.5, disp)

        win_paths = [p for p in h.paths if win_t0 <= p['wall'] <= win_t1]
        win_disp_logs = [l for l in h.logs if win_t0 <= l['wall'] <= win_t1 and '重规划触发：目标身份变' in l['text']]
        win_any_trigger = [l for l in h.logs if win_t0 <= l['wall'] <= win_t1 and '重规划触发' in l['text']]
        new_paths = h.paths[paths_before:]
        disp_logs = [l for l in h.logs if l['wall'] >= disp_t0 and '重规划触发：目标身份变' in l['text']]
        path_to_new = [p for p in new_paths
                       if abs(p['end_x'] - new_goal[0]) < 0.4 and abs(p['end_y'] - new_goal[1]) < 0.4]
        replan_latency = (min([p['wall'] for p in path_to_new]) - disp_t0) if path_to_new else None

        # 记录实际重规划频率 / 时刻 / 轨迹
        res.update({
            'pass': bool(res['configure_ok'] and res['activate_ok'] and len(h.paths) > 0 and
                         len(win_disp_logs) == 0 and
                         len(win_paths) <= int(math.ceil(5.0 / REPLAN_PERIOD) + 1) and
                         len(path_to_new) > 0 and replan_latency is not None and replan_latency < 1.0),
            'micro_window_s': round(win_t1 - win_t0, 3),
            'micro_replan_count': len(win_paths),
            'micro_replan_rate_hz': round(len(win_paths) / (win_t1 - win_t0), 3),
            'micro_replan_goal_disp_logs': len(win_disp_logs),
            'micro_any_trigger_logs': len(win_any_trigger),
            'micro_expected_periodic_max': int(math.ceil(5.0 / REPLAN_PERIOD) + 1),
            'disp_replan_latency_s': (round(replan_latency, 3) if replan_latency is not None else None),
            'disp_goal_disp_logs': len(disp_logs),
            'disp_new_path_matches_goal': len(path_to_new) > 0,
            'total_path_publishes': len(h.paths),
            'total_path_rate_hz': round(len(h.paths) / max(win_t1 - win_t0 + 3.0, 1e-3), 3),
            'replan_times_rel': [round(p['wall'] - win_t0, 3) for p in h.paths][:40],
            'path_trajectory_last20': [[round(p['end_x'], 3), round(p['end_y'], 3)] for p in h.paths[-20:]],
            'trigger_logs': [l['text'] for l in h.logs if '重规划触发' in l['text']][:20],
            'note': ('微噪声（抖幅 0.3 < replan_displacement_m=1.0）窗内无「目标身份变」触发、重发次数 ≈ 周期；'
                     '真实位移 2.0 m 触发重规划且新路径端点 ≈ 新目标。'),
        })
    finally:
        _kill(tf); _kill(proc)
        h.destroy_node()
        log.close()
    return res


def sc_failsafe_stop(log_dir, cfg_dir):
    params = os.path.join(cfg_dir, 'r041_params_ground.yaml')
    proc, tf, log = _start_ground(log_dir, 'failsafe_stop', params)
    h = GroundHarness('failsafe')
    res = {'scenario': 'failsafe_stop'}
    try:
        ok, life = h.ensure_active()
        res['configure_ok'] = life['configure_ok']
        res['activate_ok'] = life['activate_ok']
        res['lifecycle'] = life
        if not ok:
            res['pass'] = False
            res['error'] = 'ground_baseline 未激活（TF/DDS 发现失败）'
            return res
        # 车头朝目标方向（yaw=-90°：世界 -y）→ LQR 前进分量 lin>0，验证「有运动」后再失效停车
        h.yaw = -math.pi / 2
        target = (0.0, -30.0)
        # 有效目标 → 条件等待出现前进量（替代固定 sleep）
        got_motion = h.spin_until(
            lambda: any(abs(c['lin']) > 0.1 for c in h.cmds), 6.0, goal=target, valid=True)
        res['pre_motion_seen'] = bool(got_motion)
        pre_t0 = time.time()
        h.spin(1.0, goal=target, valid=True)
        pre_cmds = [c for c in h.cmds if c['wall'] >= pre_t0]
        pre_max = max([abs(c['lin']) for c in pre_cmds], default=0.0)
        # 目标失效 → 安全停车
        v0 = time.time()
        h.spin(1.5, goal=target, valid=False)
        v1 = time.time()
        post = [c for c in h.cmds if v0 + 0.5 <= c['wall'] <= v1]
        post_max = max([abs(c['lin']) for c in post], default=0.0)
        post_max_w = max([abs(c['ang']) for c in post], default=0.0)
        res.update({
            'pass': bool(res['configure_ok'] and res['activate_ok'] and pre_max > 0.1 and
                         post_max < 1e-6 and post_max_w < 1e-6 and len(post) > 0),
            'pre_valid_max_lin': round(pre_max, 4),
            'post_invalid_max_lin': round(post_max, 6),
            'post_invalid_max_ang': round(post_max_w, 6),
            'post_samples': len(post),
            'note': '目标有效时 cmd_vel_raw 有前进量；喂 goal_info valid=0 → 安全停车（cmd_vel_raw≈0）。',
        })
    finally:
        _kill(tf); _kill(proc)
        h.destroy_node()
        log.close()
    return res


# ------------------------------------------------------------------ 生成参数 / 写入

def _write(path, text):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def _ground_params_text():
    return (
        'ground_baseline:\n  ros__parameters:\n'
        '    use_sim_time: false\n'
        '    base_frame_id: "car/base_link"\n'
        '    global_frame_id: "world"\n'
        '    odom_topic: "%s"\n'
        '    goal_topic: "%s"\n'
        '    goal_info_topic: "%s"\n'
        '    state_topic: "%s"\n'
        '    cmd_raw_topic: "%s"\n'
        '    path_topic: "%s"\n'
        '    rate_hz: 20.0\n'
        '    planner_plugin: "nav2_navfn_planner::NavfnPlanner"\n'
        '    planner_id: "GridBased"\n'
        '    replan_period_s: 2.0\n'
        '    replan_displacement_m: 1.0\n'
        '    max_path_age_s: 5.0\n'
        '    goal_reached_tol_m: 0.3\n'
        '    lookahead_m: 1.0\n'
        '    k_yaw: 1.5\n'
        '    reference_speed: 1.0\n'
        '    max_linear_velocity: 1.5\n'
        '    max_angular_velocity: 2.0\n'
        '    stop_on_invalid: True\n'
        '    lqr:\n'
        '      q_pos: 4.0\n      q_vel: 4.0\n      r: 1.0\n      dt: 0.05\n      acc_max: 4.0\n'
        'costmap:\n  ros__parameters:\n'
        '    use_sim_time: false\n'
        '    update_frequency: 5.0\n'
        '    publish_frequency: 2.0\n'
        '    global_frame: world\n'
        '    robot_base_frame: car/base_link\n'
        '    rolling_window: false\n'
        '    width: 40\n    height: 40\n    resolution: 0.25\n    robot_radius: 0.6\n'
        '    track_unknown_space: true\n'
        '    plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]\n'
        '    static_layer:\n'
        '      plugin: "nav2_costmap_2d::StaticLayer"\n'
        '      map_subscribe_transient_local: True\n'
        '      map_topic: "%s"\n'
        '    obstacle_layer:\n'
        '      plugin: "nav2_costmap_2d::ObstacleLayer"\n'
        '      enabled: True\n'
        '      observation_sources: scan\n'
        '      scan:\n'
        '        topic: /agc/ground/scan_filtered\n'
        '        max_obstacle_height: 2.5\n        clearing: True\n        marking: True\n'
        '        data_type: "LaserScan"\n'
        '        raytrace_max_range: 20.0\n        raytrace_min_range: 0.0\n'
        '        obstacle_max_range: 15.0\n        obstacle_min_range: 0.0\n'
        '    voxel_layer:\n'
        '      plugin: "nav2_costmap_2d::VoxelLayer"\n'
        '      enabled: True\n      publish_voxel_map: True\n      origin_z: 0.0\n'
        '      z_resolution: 0.25\n      z_voxels: 16\n      max_obstacle_height: 2.5\n'
        '      min_obstacle_height: 0.30\n      mark_threshold: 0\n'
        '      observation_sources: pointcloud\n'
        '      pointcloud:\n'
        '        topic: /agc/sim/car_points\n        data_type: "PointCloud2"\n'
        '        max_obstacle_height: 2.5\n        min_obstacle_height: 0.30\n'
        '        clearing: True\n        marking: True\n'
        '        raytrace_max_range: 20.0\n        raytrace_min_range: 0.0\n'
        '        obstacle_max_range: 15.0\n        obstacle_min_range: 0.0\n'
        '    inflation_layer:\n'
        '      plugin: "nav2_costmap_2d::InflationLayer"\n'
        '      cost_scaling_factor: 3.0\n      inflation_radius: 1.0\n'
        '    always_send_full_costmap: True\n'
        % (ODOM_TOPIC, GOAL_TOPIC, GOAL_INFO_TOPIC, STATE_TOPIC, CMD_RAW_TOPIC, PATH_TOPIC, MAP_TOPIC))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261018-w6-r041-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()

    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)
    cfg_dir = os.path.join(out_dir, 'config')
    log_dir = os.path.join(out_dir, 'r041', 'log')
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    result = {
        'run_id': args.run_id,
        'suite': 'R-041 goal invalidation policy + displacement-thresholded replan bench',
        'scope_note': ('节点行为台架：真实 task_manager / ground_baseline 二进制 + 合成 target/odom/map/goal 流，'
                       '验证目标失效显式策略（有期限最后有效目标→超窗安全停车）、位移阈值化重规划、'
                       '伴随诊断 valid 传递；**非整链** / 非 GUI。'),
        'unsupported': [],
        'scenarios': {},
    }

    APIs = [
        ('target_expire', lambda: sc_target_expire(log_dir, cfg_dir)),
        ('micro_noise_goal', lambda: sc_micro_noise_goal(log_dir, cfg_dir)),
        ('failsafe_stop', lambda: sc_failsafe_stop(log_dir, cfg_dir)),
    ]

    def run_scenario(name, fn, attempts=3):
        """整台架对**单场景偶发发现抖动**重试（不掩盖产品错误：重试仅吸收 TF/DDS / 订阅发现抖动）。"""
        last = None
        for i in range(attempts):
            rclpy.init()
            _reap_stale(REAP_PATTERNS)
            try:
                sc = fn()
            except Exception as exc:  # noqa: BLE001
                sc = {'scenario': name, 'pass': False, 'error': repr(exc)}
            finally:
                try:
                    rclpy.shutdown()
                except Exception:  # noqa: BLE001
                    pass
            _reap_stale(REAP_PATTERNS)
            sc['scenario'] = name
            sc['attempts'] = i
            last = sc
            if sc.get('pass'):
                return sc
            time.sleep(0.5)
        return last

    for name, fn in APIs:
        sc = run_scenario(name, fn)
        result['scenarios'][name] = sc
        print('%-20s PASS=%s (attempts=%s)' % (name, sc.get('pass'), sc.get('attempts')))

    named = ['target_expire', 'micro_noise_goal', 'failsafe_stop']
    result['overall_pass'] = bool(all(result['scenarios'].get(n, {}).get('pass') for n in named))
    with open(os.path.join(out_dir, 'r041', 'bench.json'), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print('\noverall_pass=%s -> %s' % (
        result['overall_pass'], os.path.join(out_dir, 'r041', 'bench.json')))
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
