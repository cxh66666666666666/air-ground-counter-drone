#!/usr/bin/env python3
"""w5_r007_check —— W5 R-007 真实仿真验收（自滤 / 旋转误停 / 旋转停 / 后向 / 薄障碍 / min_points 标定）。

在 `agc_task w5_safety_scenario.launch.py` 运行中执行（先杀掉 RL 命令源，改由本脚本作脚本化命令源
直接发 `/agc/ground/cmd_vel_raw`），观察 意图（cmd_vel_raw）/ 实际执行（cmd_vel）/ 物理真值（odom）。

子测：
  CAL  min_points 标定：立柱(0.30 m) 与薄柱(0.08 m) 在各距离的命中束数。
  A.   openspace          ：开阔处，自滤后扫描无车体返回 → 不误停。
  B.   rotation_free      ：开阔处 (0, ω=2.0) → cmd_vel ω≈2.0 不误停，odom yaw 实际转过 ≈ω·t。
  C.   rotation_blocked   ：障碍在旋转区 → 命令旋转 → stop（cmd_vel→0）且停窗内 yaw 几乎不变。
  D.   backward_blocked   ：障碍在车后 → 命令 linear=-1.0 → 后向区触发 stop（车真实停）。
  E.   thin_obstacle      ：薄障碍（细柱）进入停车区（满足 min_points）→ stop。
结果落 experiments/w5-ground-nav-safety/<run_id>/metrics/r007.json。
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import LaserScan

POLE = (0.0, -6.0)
POLE_HALF = 0.15


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def qos_sensor():
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.durability = DurabilityPolicy.VOLATILE
    return q


class R007(Node):
    def __init__(self):
        super().__init__('w5_r007_check')
        self.cmd_pub = self.create_publisher(TwistStamped, '/agc/ground/cmd_vel_raw', 10)
        self.v_out = 0.0
        self.w_out = 0.0
        self.x = self.y = 0.0
        self.yaw = 0.0
        self.scan_f = None
        self.scan_raw = None
        self.n_cmd = 0
        self.create_subscription(TwistStamped, '/agc/ground/cmd_vel', self._on_cmd, 50)
        self.create_subscription(Odometry, '/agc/ground/odom', self._on_odom, 50)
        self.create_subscription(LaserScan, '/agc/ground/scan_filtered', self._on_sf, qos_sensor())
        self.create_subscription(LaserScan, '/agc/ground/scan', self._on_sr, qos_sensor())
        self.cli = self.create_client(SetEntityPose, '/world/agc_world/set_pose')
        self.state_cli = self.create_client(GetState, '/collision_monitor/get_state')

    def _on_cmd(self, m):
        self.v_out = m.twist.linear.x
        self.w_out = m.twist.angular.z
        self.n_cmd += 1

    def _on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_from_quat(m.pose.pose.orientation)

    def _on_sf(self, m):
        self.scan_f = m

    def _on_sr(self, m):
        self.scan_raw = m

    def wait_active(self, timeout=70.0):
        if not self.state_cli.wait_for_service(timeout_sec=15.0):
            return False, 'no /collision_monitor/get_state'
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            fut = self.state_cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.done() and fut.result() is not None:
                if fut.result().current_state.id == 3:  # ACTIVE
                    return True, 'active'
            time.sleep(1.0)
        return False, 'collision_monitor not active in %.0fs' % timeout

    def set_pose(self, x, y, z, yaw, timeout=6.0):
        if not self.cli.wait_for_service(timeout_sec=timeout):
            return False
        req = SetEntityPose.Request()
        req.entity.name = 'agc_car'
        req.entity.type = 2  # MODEL
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.z = math.sin(yaw * 0.5)
        req.pose.orientation.w = math.cos(yaw * 0.5)
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() and fut.result().success)

    def publish_cmd(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'car/base_link'
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.cmd_pub.publish(m)

    def settle(self, seconds=1.5):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.publish_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_window(self, v, w, duration):
        samples = []
        t0 = time.monotonic()
        while True:
            now = time.monotonic()
            if now - t0 >= duration:
                break
            self.publish_cmd(v, w)
            rclpy.spin_once(self, timeout_sec=0.01)
            samples.append((now - t0, self.v_out, self.w_out, self.x, self.y, self.yaw))
        return samples

    def finite_points(self, scan):
        if scan is None:
            return []
        pts = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            th = scan.angle_min + i * scan.angle_increment
            pts.append((r * math.cos(th), r * math.sin(th)))
        return pts

    def forward_hits(self, half_ang=0.35):
        """自滤后扫描中，车前方 ±half_ang 内的有限返回数。"""
        pts = self.finite_points(self.scan_f)
        return sum(1 for p in pts if abs(math.atan2(p[1], p[0])) < half_ang)


def total_yaw(samples):
    """累计转角（逐样本增量积分，避免 yaw 归一化到 (-pi,pi] 造成的 >pi 别名）。"""
    tot = 0.0
    for i in range(1, len(samples)):
        tot += wrap(samples[i][5] - samples[i - 1][5])
    return abs(tot)


def dist_to_pole(s):
    return math.hypot(s[3] - POLE[0], s[4] - POLE[1])


def stop_run_s(samples, key, thresh):
    run = best = 0.0
    prev = samples[0][0] if samples else 0.0
    for s in samples:
        if abs(s[key]) < thresh:
            run += (s[0] - prev)
            best = max(best, run)
        else:
            run = 0.0
        prev = s[0]
    return best


def tail_yaw_move(samples, key=2, thresh=0.5):
    i0 = len(samples)
    while i0 > 0 and abs(samples[i0 - 1][key]) < thresh:
        i0 -= 1
    if i0 >= len(samples):
        return 0.0
    return abs(wrap(samples[-1][5] - samples[i0][5]))


def spawn_thin(x, y, z):
    sdf = ("<sdf version='1.8'><model name='r007_thin'><static>true</static>"
           "<link name='link'><collision name='c'><geometry><box><size>0.08 0.08 2</size>"
           "</box></geometry></collision><visual name='v'><geometry><box><size>0.08 0.08 2</size>"
           "</box></geometry></visual></link></model></sdf>")
    cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'agc_world',
           '-name', 'r007_thin', '-x', str(x), '-y', str(y), '-z', str(z), '-string', sdf]
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=25)
    return p.returncode == 0, (p.stdout + p.stderr)[-200:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261015-w5-r007-0')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()

    rclpy.init()
    n = R007()
    res = {'run_id': args.run_id, 'tests': {}, 'calibration': {}}

    t0 = time.monotonic()
    while time.monotonic() - t0 < 12.0 and (n.scan_f is None or n.scan_raw is None):
        rclpy.spin_once(n, timeout_sec=0.05)
    active, amsg = n.wait_active()
    res['collision_monitor_active'] = active
    res['collision_monitor_state_msg'] = amsg
    if not active:
        res['overall_pass'] = False
        res['error'] = amsg
        _dump(res, args)
        return 1

    # ---- CAL：min_points 标定（立柱 0.30 m / 薄柱 0.08 m 各距离命中束数）----
    cal_pole = []
    for d in [4.0, 3.0, 2.0, 1.5, 1.0]:
        n.set_pose(0.0, -6.0 + POLE_HALF + d, 0.02, -math.pi / 2)
        n.settle(1.4)
        cal_pole.append({'dist_m': d, 'hits': n.forward_hits()})
    ok_spawn, smsg = spawn_thin(12.0, 7.0, 1.0)
    res['thin_spawn'] = {'ok': ok_spawn, 'msg': smsg}
    cal_thin = []
    for d in [4.0, 3.0, 2.0, 1.5, 1.0]:
        n.set_pose(12.0, 7.0 + d, 0.02, -math.pi / 2)
        n.settle(1.4)
        cal_thin.append({'dist_m': d, 'hits': n.forward_hits()})
    res['calibration'] = {'pole_0.30m': cal_pole, 'thin_0.08m': cal_thin}

    # ---- A. openspace 自滤（真开阔处，无 20 m 内障碍）----
    n.set_pose(30.0, 30.0, 0.02, 0.0)
    n.settle(1.6)
    raw_pts = n.finite_points(n.scan_raw)
    f_pts = n.finite_points(n.scan_f)
    res['tests']['openspace'] = {
        'pose': (30, 30),
        'raw_finite_returns': len(raw_pts),
        'filtered_finite_returns': len(f_pts),
        'filtered_near_le_1m': sum(1 for p in f_pts if math.hypot(p[0], p[1]) <= 1.0),
        'raw_near_le_1m': sum(1 for p in raw_pts if math.hypot(p[0], p[1]) <= 1.0),
        'pass': bool(len(f_pts) == 0),
    }

    # ---- B. rotation_free ----
    n.set_pose(30.0, 30.0, 0.02, 0.0)
    n.settle(1.5)
    s = n.run_window(0.0, 2.0, 4.0)
    yaw_total = total_yaw(s)
    max_w = max((abs(x[2]) for x in s), default=0.0)
    tail = s[5:] if len(s) > 5 else s     # 跳过起步若干样本
    zero_frac = (sum(1 for x in tail if abs(x[2]) < 0.5) / len(tail)) if tail else 1.0
    res['tests']['rotation_free'] = {
        'cmd': '(0, w=2.0) x4s（开阔处，无 20 m 内障碍）',
        'max_w_out': round(max_w, 3),
        'zero_frac_w_out': round(zero_frac, 3),
        'yaw_total_rad': round(yaw_total, 3),
        'expected_yaw_rad': 8.0,
        'pass': bool(max_w >= 1.6 and zero_frac <= 0.05 and yaw_total >= 4.0),
    }

    # ---- C. rotation_blocked ----
    n.set_pose(0.0, -4.7, 0.02, -math.pi / 2)
    n.settle(1.5)
    s = n.run_window(0.0, 2.0, 5.0)
    stop_w = stop_run_s(s, 2, 0.5)
    tail = tail_yaw_move(s, 2, 0.5)
    min_d = min((dist_to_pole(x) for x in s), default=99.0)
    res['tests']['rotation_blocked'] = {
        'cmd': '(0, w=2.0) x5s; obstacle 1.3 m ahead',
        'stop_run_w_s': round(stop_w, 3),
        'tail_yaw_move_rad': round(tail, 4),
        'min_dist_to_pole_m': round(min_d, 3),
        'pass': bool(stop_w >= 0.5 and tail <= 0.10 and min_d >= 0.8),
    }

    # ---- D. backward_blocked ----
    n.set_pose(0.0, -3.4, 0.02, math.pi / 2)
    n.settle(1.5)
    d0 = math.hypot(n.x - POLE[0], n.y - POLE[1])
    s = n.run_window(-1.0, 0.0, 6.0)
    stop_v = stop_run_s(s, 1, 0.05)
    min_d = min((dist_to_pole(x) for x in s), default=99.0)
    moved = math.hypot(s[-1][3] - s[0][3], s[-1][4] - s[0][4]) if s else 0.0
    res['tests']['backward_blocked'] = {
        'cmd': '(v=-1.0, 0) x6s; obstacle 2.6 m behind',
        'dist_at_start_m': round(d0, 3),
        'min_dist_to_pole_m': round(min_d, 3),
        'stop_run_v_s': round(stop_v, 3),
        'travel_m': round(moved, 3),
        'pass': bool(stop_v >= 0.5 and min_d >= 1.0),
    }

    # ---- E. thin_obstacle（前向）----
    n.set_pose(12.0, 10.0, 0.02, -math.pi / 2)   # 薄柱在正前 3.0 m
    n.settle(1.6)
    hits3 = n.forward_hits()
    s = n.run_window(1.0, 0.0, 8.0)
    stop_v = stop_run_s(s, 1, 0.02)
    i0 = len(s)
    while i0 > 0 and abs(s[i0 - 1][1]) < 0.02:
        i0 -= 1
    tail = s[i0:] if i0 < len(s) else []
    tail_move = max((math.hypot(t[3] - tail[0][3], t[4] - tail[0][4]) for t in tail), default=0.0)
    min_thin = min((abs(t[4] - 7.0) for t in s), default=99.0)
    res['tests']['thin_obstacle'] = {
        'cmd': '(v=1.0, 0) x8s; thin post 0.08m, start 3.0 m ahead',
        'hits_at_3m': hits3,
        'stop_run_v_s': round(stop_v, 3),
        'tail_move_m': round(tail_move, 3),
        'min_dist_to_thin_m': round(min_thin, 3),
        'pass': bool(hits3 >= 2 and stop_v >= 0.5 and tail_move <= 0.3 and min_thin >= 0.5),
    }

    res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    rc = _dump(res, args)
    return rc


def _dump(res, args):
    out_dir = os.path.join(args.experiments_root, 'w5-ground-nav-safety',
                           args.run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'r007.json')
    with open(path, 'w') as fh:
        json.dump(res, fh, indent=2)
    for name, t in res['tests'].items():
        print('%-20s %s  %s' % (name, 'PASS' if t['pass'] else 'FAIL',
                               {k: v for k, v in t.items() if k != 'pass'}))
    print('calibration:', res.get('calibration'))
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


if __name__ == '__main__':
    sys.exit(main())
