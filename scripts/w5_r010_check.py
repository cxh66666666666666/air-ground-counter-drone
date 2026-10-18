#!/usr/bin/env python3
"""w5_r010_check —— W5 R-010 RL 推理节点「输入许可与失效停车」验收。

背景（评审风险 R-010）：`ground_policy_rl` 原对 odom / goal / scan / task 状态只记「到达过即**永久有效**」
（have_odom / have_goal 布尔永不失效），且 `interface` 把 NaN / <=0 一并当「无遮挡」；`task_manager`
若死在 `GROUND_APPROACH`，策略仍每拍发**新时间戳**的运动命令。契约真源：
docs/ROS2迁移交接文档.md#execution-port（唯一执行口）、#params（参数单一注入）。

两个场景（`--scenario`）：
  node —— 独立进程（不依赖 GZ）：拉起 `ground_policy_rl`，本脚本向四类输入发合成消息，逐一注入
          失效并观测 `cmd_vel_raw` 是否归零（含 `interface` 单元判据）。
  sim  —— 整仿真（GUI，硬约束 C3）内集成验收：W5 场景以 RL 为主控，对**真实节点**注入故障
          （`SIGSTOP ground_odom_adapter` / `SIGKILL task_manager`），观测 `cmd_vel_raw` 归零与真值停车。

node 场景子测（PASS 判据见各 test 的 `pass` 字段）：
  U1 `interface` 单元：NaN → 扇区按遮挡（0.0）；`+inf` → 无遮挡（1.0）；all-NaN → `scan_has_valid_reading` 假。
  N0 **从未收到 scan** → 零速。 N1 正常链路 → 非零且输出 ~20 Hz。
  N2 **全 NaN scan** → 零速（不当作无遮挡）。 N3 停 task/state → 超授权寿命后零速。
  N4 停 odom → 超寿命后零速。 N5 停 goal → 超寿命后零速。

sim 场景子测：
  S1 正常：`cmd_vel_raw` 非零（RL 为主控在驱动）。
  S2 `SIGSTOP ground_odom_adapter`（odom 断源）→ `cmd_vel_raw` 归零；`SIGCONT` 后恢复非零。
  S3 `SIGKILL task_manager`（task 授权 + goal 双断）→ 归零，且随后**真值位移**≈0（真实停车）。

退出码：0 PASS / 1 断言 FAIL / 2 基础设施错误（供 shell 决定是否重试）。
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
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from agc_msgs.msg import AgcTaskState
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.srv import SetEntityPose
from std_srvs.srv import Trigger

from agc_rl import interface

RAW = '/agc/ground/cmd_vel_raw'
ODOM = '/agc/ground/odom'
SCAN = '/agc/ground/scan'
GOAL = '/agc/ground/goal'
STATE = '/agc/task/state'
GT = '/agc/gt/agc_car'

GROUND_APPROACH = AgcTaskState.GROUND_APPROACH   # = 3
ZERO_V = 0.05          # 归零判据（线速 m/s）
MOVE_V = 0.30          # 「在动」判据（线速 m/s）
N_PUMP = 72            # 合成扫描束数


class Probe(Node):
    """共享探针：合成发布四类输入 + 订阅 cmd_vel_raw / odom / gt / state + 调 task/start 服务。"""

    def __init__(self):
        super().__init__('w5_r010_check')
        self.raw = []                 # (t_monotonic, v, w)
        self.odom_vx = 0.0
        self.have_odom = False
        self.state = None
        self.have_state = False
        self.gx = self.gy = 0.0
        self.have_gt = False

        self._pub_odom = self.create_publisher(Odometry, ODOM, 10)
        self._pub_goal = self.create_publisher(PoseStamped, GOAL, 10)
        self._pub_state = self.create_publisher(AgcTaskState, STATE, 10)
        sq = QoSProfile(depth=10)
        sq.reliability = ReliabilityPolicy.BEST_EFFORT
        sq.durability = DurabilityPolicy.VOLATILE
        self._pub_scan = self.create_publisher(LaserScan, SCAN, sq)

        self.create_subscription(TwistStamped, RAW, self._on_raw, 50)
        self.create_subscription(Odometry, ODOM, self._on_odom, 50)
        self.create_subscription(AgcTaskState, STATE, self._on_state, 50)
        self.create_subscription(PoseStamped, GT, self._on_gt, 50)
        self._start_cli = self.create_client(Trigger, '/agc/task/start')
        self._pose_cli = self.create_client(SetEntityPose, '/world/agc_world/set_pose')

    # ---- 订阅 ----
    def _on_raw(self, m):
        self.raw.append((time.monotonic(), float(m.twist.linear.x), float(m.twist.angular.z)))

    def _on_odom(self, m):
        self.odom_vx = float(m.twist.twist.linear.x)
        self.have_odom = True

    def _on_state(self, m):
        self.state = int(m.state)
        self.have_state = True

    def _on_gt(self, m):
        self.gx = m.pose.position.x
        self.gy = m.pose.position.y
        self.have_gt = True

    def set_pose(self, x, y, z, yaw, timeout=6.0):
        """把 `agc_car` 传送至指定位姿（开阔区移入，使真实停车可观测；与 R-009 同法）。"""
        if not self._pose_cli.wait_for_service(timeout_sec=timeout):
            return False
        req = SetEntityPose.Request()
        req.entity.name = 'agc_car'
        req.entity.type = 2
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.z = math.sin(yaw * 0.5)
        req.pose.orientation.w = math.cos(yaw * 0.5)
        fut = self._pose_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() and fut.result().success)

    # ---- 合成消息 ----
    def _mk_odom(self, x=0.0, y=0.0, yaw=0.0):
        m = Odometry()
        m.header.frame_id = 'world'
        m.pose.pose.position.x = float(x)
        m.pose.pose.position.y = float(y)
        m.pose.pose.orientation.z = math.sin(yaw * 0.5)
        m.pose.pose.orientation.w = math.cos(yaw * 0.5)
        return m

    def _mk_goal(self, x=5.0, y=0.0):
        m = PoseStamped()
        m.header.frame_id = 'world'
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.w = 1.0
        return m

    def _mk_state(self, state):
        m = AgcTaskState()
        m.state = int(state)
        return m

    def _mk_scan(self, ranges):
        m = LaserScan()
        n = len(ranges)
        m.angle_min = -math.pi
        m.angle_increment = 2.0 * math.pi / n
        m.angle_max = math.pi - m.angle_increment
        m.range_min = 0.2
        m.range_max = 20.0
        m.ranges = [float(r) for r in ranges]
        return m

    # ---- 驱动 ----
    def pump(self, sec, odom=True, goal=True, state=GROUND_APPROACH, scan='valid'):
        """在 `sec` 秒内按链路频率发布选定的输入（`state=None` / `scan=None` 即停发该项）。"""
        t0 = time.monotonic()
        last = {'odom': -1.0, 'goal': -1.0, 'state': -1.0, 'scan': -1.0}
        while time.monotonic() - t0 < sec:
            now = time.monotonic()
            if odom and now - last['odom'] >= 0.05:
                self._pub_odom.publish(self._mk_odom())
                last['odom'] = now
            if goal and now - last['goal'] >= 0.20:
                self._pub_goal.publish(self._mk_goal())
                last['goal'] = now
            if state is not None and now - last['state'] >= 0.20:
                self._pub_state.publish(self._mk_state(state))
                last['state'] = now
            if scan is not None and now - last['scan'] >= 0.10:
                ranges = [8.0] * N_PUMP if scan == 'valid' else [float('nan')] * N_PUMP
                self._pub_scan.publish(self._mk_scan(ranges))
                last['scan'] = now
            rclpy.spin_once(self, timeout_sec=0.005)
        return time.monotonic()

    def sample(self, sec):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.005)
        return time.monotonic()

    # ---- 观测 ----
    def window(self, t_from, t_to=None):
        return [s for s in self.raw if s[0] >= t_from and (t_to is None or s[0] <= t_to)]

    def clear(self):
        self.raw.clear()

    def tail_max(self, sec):
        """采样 `sec` 秒，返回该窗口内 |v| 最大值。"""
        t_end = self.sample(sec)
        return max((abs(s[1]) for s in self.raw if s[0] >= t_end - sec), default=0.0)

    def tail_rate(self, sec):
        t_end = self.sample(sec)
        n = len([s for s in self.raw if s[0] >= t_end - sec])
        return n / sec


def pgrep(pattern):
    try:
        out = subprocess.check_output(['pgrep', '-f', pattern], text=True)
    except subprocess.CalledProcessError:
        return []
    return [int(p) for p in out.split() if p.strip()]


def test_interface():
    """U1：`interface` 区分未知（NaN）与合法无回波（+inf）。"""
    s_nan = interface.sectors_from_angles([0.0], [float('nan')], 12, 10.0)
    s_inf = interface.sectors_from_angles([0.0], [float('inf')], 12, 10.0)
    s_hit = interface.sectors_from_angles([0.0], [2.0], 12, 10.0)
    detail = {
        'nan_sector_front': s_nan[0],       # 应 0.0（保守按遮挡）
        'inf_sector_front': s_inf[0],       # 应 1.0（确证无遮挡）
        'hit_sector_front': s_hit[0],       # 应 0.2
        'valid_all_nan': interface.scan_has_valid_reading([float('nan')] * 8),
        'valid_inf_only': interface.scan_has_valid_reading([float('inf')]),
        'valid_hit': interface.scan_has_valid_reading([float('nan'), 3.0]),
    }
    detail['pass'] = bool(
        abs(detail['nan_sector_front'] - 0.0) < 1e-9
        and abs(detail['inf_sector_front'] - 1.0) < 1e-9
        and abs(detail['hit_sector_front'] - 0.2) < 1e-6
        and detail['valid_all_nan'] is False
        and detail['valid_inf_only'] is True
        and detail['valid_hit'] is True)
    return detail


def start_node(args, out_dir):
    log = open(os.path.join(out_dir, 'rl_node.log'), 'w')
    proc = subprocess.Popen(
        [args.node_exe, '--ros-args', '--params-file', args.params_file, '-p', 'use_sim_time:=false'],
        stdout=log, stderr=subprocess.STDOUT)
    return proc, log


# =====================================================================================
# 场景 node（U1 + N0–N5；独立进程，不依赖 GZ）
# =====================================================================================
def run_node(args):
    out_dir = metrics_dir(args)
    res = {'run_id': args.run_id, 'scenario': 'node', 'tests': {}}
    rclpy.init()
    p = Probe()
    proc, log = start_node(args, out_dir)
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0 and not p.raw:
            rclpy.spin_once(p, timeout_sec=0.05)
        if not p.raw:
            res['error'] = 'infra: ground_policy_rl 未在 15 s 内上线（cmd_vel_raw 无消息）'
            return dump(res, out_dir, 'r010.json', infra=True)

        res['tests']['U1_interface_nan_vs_inf'] = test_interface()

        # ---- N0 从未收到 scan → 零速 ----
        p.clear()
        p.pump(1.5, scan=None)
        v = p.tail_max(1.0)
        res['tests']['N0_never_scan_zero'] = {
            'cmd': '只发 odom/goal/state(GROUND_APPROACH)，**不发 scan**',
            'max_v_recent': round(v, 4), 'pass': bool(v < ZERO_V)}

        # ---- N1 正常链路 → 非零且 ~20 Hz ----
        p.clear()
        p.pump(1.2)
        rate = p.tail_rate(1.0)
        v1 = max((abs(s[1]) for s in p.raw), default=0.0)
        res['tests']['N1_normal_nonzero'] = {
            'cmd': '四类输入齐备（scan 有效）',
            'max_v': round(v1, 3), 'rate_hz': round(rate, 2),
            'pass': bool(v1 > MOVE_V and 14.0 <= rate <= 26.0)}

        # ---- N2 全 NaN scan → 零速 ----
        p.clear()
        p.pump(1.2, scan='nan')
        v2 = p.tail_max(1.0)
        res['tests']['N2_all_nan_scan_zero'] = {
            'cmd': 'scan 全 NaN（其余输入正常）',
            'max_v_recent': round(v2, 4), 'pass': bool(v2 < ZERO_V)}

        # ---- N3 停 task/state → 零速（授权寿命）----
        p.clear()
        p.pump(1.2)
        p.clear()
        p.pump(args.auth_settle, state=None)
        v3 = p.tail_max(1.0)
        res['tests']['N3_stop_task_state_zero'] = {
            'cmd': '停 /agc/task/state（GROUND_APPROACH 中）',
            'settle_s': round(args.auth_settle, 2),
            'max_v_recent': round(v3, 4), 'pass': bool(v3 < ZERO_V)}

        # ---- N4 停 odom → 零速 ----
        p.clear()
        p.pump(1.2)
        p.clear()
        p.pump(args.fast_settle, odom=False)
        v4 = p.tail_max(1.0)
        res['tests']['N4_stop_odom_zero'] = {
            'cmd': '停 /agc/ground/odom', 'settle_s': round(args.fast_settle, 2),
            'max_v_recent': round(v4, 4), 'pass': bool(v4 < ZERO_V)}

        # ---- N5 停 goal → 零速 ----
        p.clear()
        p.pump(1.2)
        p.clear()
        p.pump(args.auth_settle, goal=False)
        v5 = p.tail_max(1.0)
        res['tests']['N5_stop_goal_zero'] = {
            'cmd': '停 /agc/ground/goal', 'settle_s': round(args.auth_settle, 2),
            'max_v_recent': round(v5, 4), 'pass': bool(v5 < ZERO_V)}
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=5)
        except Exception:  # noqa: BLE001
            proc.kill()
        log.close()
        p.destroy_node()
        rclpy.shutdown()

    res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    return dump(res, out_dir, 'r010_node.json')


# =====================================================================================
# 场景 sim（GUI 集成：真实节点故障注入）
# =====================================================================================
def call_start(p, timeout=10.0):
    if not p._start_cli.wait_for_service(timeout_sec=timeout):
        return False, 'no /agc/task/start'
    fut = p._start_cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(p, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return False, 'start timeout'
    return bool(fut.result().success), fut.result().message


def wait_state(p, want, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rclpy.spin_once(p, timeout_sec=0.05)
        if p.have_state and p.state == want:
            return True
    return False


def run_sim(args):
    out_dir = metrics_dir(args)
    res = {'run_id': args.run_id, 'scenario': 'sim', 'tests': {}}
    rclpy.init()
    p = Probe()
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 25.0 and not (p.raw and p.have_state):
            rclpy.spin_once(p, timeout_sec=0.05)
        if not (p.raw and p.have_state):
            res['error'] = 'infra: 链路未就绪（cmd_vel_raw=%d state=%s）' % (len(p.raw), p.have_state)
            return dump(res, out_dir, 'r010.json', infra=True)

        # 门控：/agc/task/start → AIR_SEARCH → … → GROUND_APPROACH（RL 授权前置）
        ok, msg = call_start(p)
        res['start'] = {'success': ok, 'message': msg}
        if not ok:
            res['error'] = 'infra: /agc/task/start 失败：%s' % msg
            return dump(res, out_dir, 'r010.json', infra=True)
        if not wait_state(p, GROUND_APPROACH, 45.0):
            res['error'] = 'infra: 45 s 内未达 GROUND_APPROACH（state=%s）' % p.state
            return dump(res, out_dir, 'r010.json', infra=True)

        # 移入开阔区（正前方 26 m 无遮挡）：使「真实停车」可观测（避免安全层先于故障停车）
        moved = p.set_pose(0.0, 20.0, 0.02, -math.pi / 2.0)
        res['teleport'] = {'ok': moved, 'pose': '(0, 20, yaw=-pi/2)'}
        if not moved:
            res['error'] = 'infra: set_pose 失败（开阔区移入）'
            return dump(res, out_dir, 'r010.json', infra=True)
        p.sample(2.0)

        # ---- S1 正常：RL 为主控在驱动 ----
        p.clear()
        v1 = p.tail_max(2.0)
        res['tests']['S1_normal_nonzero'] = {
            'cmd': 'GROUND_APPROACH 中（RL 主控）', 'max_v': round(v1, 3),
            'pass': bool(v1 > MOVE_V)}

        # ---- S2 odom 断源（SIGSTOP）→ 超寿命后零速；SIGCONT 恢复 ----
        pids = pgrep('[g]round_odom_adapter')
        s2 = {'odom_adapter_pids': pids}
        if not pids:
            s2['pass'] = False
            s2['error'] = 'ground_odom_adapter 进程未找到'
        else:
            for q in pids:
                os.kill(q, signal.SIGSTOP)
            p.sample(args.fast_settle)          # 让 odom 寿命（0.5 s）失效
            p.clear()
            v_zero = p.tail_max(1.5)
            for q in pids:
                os.kill(q, signal.SIGCONT)
            p.sample(2.0)                        # odom 恢复、RL 重新驱动
            p.clear()
            v_resume = p.tail_max(1.5)
            s2.update({'v_zero_recent': round(v_zero, 4), 'v_after_resume': round(v_resume, 3),
                       'pass': bool(v_zero < ZERO_V and v_resume > MOVE_V)})
        res['tests']['S2_odom_stop_zero_then_resume'] = s2

        # ---- S3 task 授权断源（SIGKILL task_manager）+ 真值停车 ----
        pids = pgrep('[t]ask_manager')
        s3 = {'task_manager_pids': pids}
        if not pids:
            s3['pass'] = False
            s3['error'] = 'task_manager 进程未找到'
        else:
            v_before = p.odom_vx
            for q in pids:
                os.kill(q, signal.SIGKILL)
            p.sample(args.auth_settle)           # 让授权 / goal 寿命（2.0 s）失效
            p.clear()
            v_zero = p.tail_max(1.5)
            p.sample(3.0)                        # 制动时间
            g0 = (p.gx, p.gy)
            p.sample(2.0)
            disp = math.hypot(p.gx - g0[0], p.gy - g0[1])
            s3.update({'v_before_kill': round(v_before, 3), 'v_zero_recent': round(v_zero, 4),
                       'odom_speed_after': round(p.odom_vx, 3),
                       'gt_displacement_after_brake_m': round(disp, 3),
                       'pass': bool(v_zero < ZERO_V and disp < 0.20)})
        res['tests']['S3_task_stop_zero_and_real_stop'] = s3
    finally:
        p.destroy_node()
        rclpy.shutdown()

    res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    return dump(res, out_dir, 'r010.json')

def metrics_dir(args):
    d = os.path.join(args.experiments_root, 'w5-ground-nav-safety', args.run_id, 'metrics')
    os.makedirs(d, exist_ok=True)
    return d


def dump(res, out_dir, filename, infra=False):
    path = os.path.join(out_dir, filename)
    with open(path, 'w') as fh:
        json.dump(res, fh, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-34s %s  %s' % (name, 'PASS' if t.get('pass') else 'FAIL',
                                {k: v for k, v in t.items() if k != 'pass'}))
    if 'error' in res:
        print('error:', res['error'])
        return 2 if infra else 1
    ok = bool(res.get('overall_pass'))
    print('\n%s -> %s' % ('PASS' if ok else 'FAIL', path))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='sim', choices=['node', 'sim'])
    ap.add_argument('--run-id', default='20261015-w5-r010-0')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    ap.add_argument('--node-exe',
                    default='/home/chen/AGC/install/agc_rl/lib/agc_rl/ground_policy_rl')
    ap.add_argument('--params-file',
                    default='/home/chen/AGC/install/agc_rl/share/agc_rl/config/rl.yaml')
    ap.add_argument('--auth-settle', type=float, default=2.5,
                    help='task 授权 / goal 寿命（2.0 s）+ 余量的收尾观察时长')
    ap.add_argument('--fast-settle', type=float, default=1.0,
                    help='odom / scan 寿命（0.5 s）+ 余量的收尾观察时长')
    args = ap.parse_args()
    if args.scenario == 'node':
        return run_node(args)
    return run_sim(args)


if __name__ == '__main__':
    sys.exit(main())
