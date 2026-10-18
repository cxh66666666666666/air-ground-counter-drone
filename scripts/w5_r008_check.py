#!/usr/bin/env python3
"""w5_r008_check —— W5 R-008 失源失效停车故障注入验收（scan 断源 / TF 断源）。

在 `agc_task w5_safety_scenario.launch.py` 运行中执行（先杀掉 RL 命令源，改由本脚本作"脚本化命令源"
直接发 `/agc/ground/cmd_vel_raw`），观察 意图（cmd_vel_raw）/ 实际执行（cmd_vel）/ 物理真值（odom / gt）。

语义取证（本机 Jazzy nav2_collision_monitor）：`Source::getData` 对失效源（源超 `source_timeout` 未更新，
或源→base 变换不可解）**返回 false → 封锁机器人/停车**，**非静默透传**原命令
（`/opt/ros/jazzy/include/nav2_collision_monitor/source.hpp`：`@return false if an invalid source should block the robot`）。
本脚本以「持续发 v_max 且 cmd_vel_out 归零 + 真值速度→0」实证该语义。

标定值（ADR-0021 追加节 · R-008）：T = source_timeout = 0.3 s；本车实测 v_max = 1.5 m/s、a ≈ 0.9 m/s²。
  失源盲行 d_blind = v_max·T = 0.45 m；制动 s_brake = v_max²/(2a) = 1.25 m；总停距 1.70 m < 碰撞档 2.0 m。

子测：
  LAT  延迟实测：cmd_raw → cmd_out（安全层链路）、cmd_raw → 真值运动（执行链路）；扫描 range 元数据。
  A.   normal_obstacle   ：车前 ~3 m 立柱，满速前进 → 碰撞档真实停车（记录停距/最近距离）。
  B.   scan_source_lost  ：开阔区满速稳态 → SIGSTOP scan_self_filter（/agc/ground/scan_filtered 停更）
                           → 车在 d_blind+s_brake 有界窗口内真实停住；PASS 判据：真值停住 且 盲行位移 ≤ 上界。
  C.   tf_source_lost    ：开阔区满速稳态 → SIGSTOP ground_odom_adapter（world↔car/base_link TF 停更）
                           → collision_monitor 判源无效并停车（非透传）。用 /agc/gt/agc_car 观测真值位移。
结果落 experiments/w5-ground-nav-safety/<run_id>/metrics/r008.json。
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

from geometry_msgs.msg import PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import LaserScan

POLE = (0.0, -6.0)
POLE_HALF = 0.15

V_MAX = 1.5
A_BRAKE = 0.9
T_CAL = 0.3
S_BRAKE = V_MAX * V_MAX / (2.0 * A_BRAKE)
D_BLIND = V_MAX * T_CAL
BLIND_BOUND = D_BLIND + S_BRAKE + 0.5
TF_BOUND = V_MAX * (T_CAL + 0.3) + S_BRAKE + 0.5


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def qos_sensor():
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.durability = DurabilityPolicy.VOLATILE
    return q


def pgrep(pattern):
    try:
        out = subprocess.check_output(['pgrep', '-f', pattern], text=True)
    except subprocess.CalledProcessError:
        return []
    return [int(p) for p in out.split() if p.strip()]


class R008(Node):
    def __init__(self):
        super().__init__('w5_r008_check')
        self.cmd_pub = self.create_publisher(TwistStamped, '/agc/ground/cmd_vel_raw', 10)
        self.v_out = 0.0
        self.w_out = 0.0
        self.n_out = 0
        self.x = self.y = self.yaw = 0.0
        self.ovx = self.ovy = 0.0
        self.gx = self.gy = 0.0
        self.have_gt = False
        self.scan_f = None
        self.create_subscription(TwistStamped, '/agc/ground/cmd_vel', self._on_out, 50)
        self.create_subscription(Odometry, '/agc/ground/odom', self._on_odom, 50)
        self.create_subscription(LaserScan, '/agc/ground/scan_filtered', self._on_sf, qos_sensor())
        self.create_subscription(PoseStamped, '/agc/gt/agc_car', self._on_gt, 50)
        self.cli = self.create_client(SetEntityPose, '/world/agc_world/set_pose')
        self.state_cli = self.create_client(GetState, '/collision_monitor/get_state')

    def _on_out(self, m):
        self.v_out = m.twist.linear.x
        self.w_out = m.twist.angular.z
        self.n_out += 1

    def _on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_from_quat(m.pose.pose.orientation)
        self.ovx = m.twist.twist.linear.x
        self.ovy = m.twist.twist.linear.y

    def _on_sf(self, m):
        self.scan_f = m

    def _on_gt(self, m):
        self.gx = m.pose.position.x
        self.gy = m.pose.position.y
        self.have_gt = True

    def odom_speed(self):
        return math.hypot(self.ovx, self.ovy)

    def wait_active(self, timeout=70.0):
        if not self.state_cli.wait_for_service(timeout_sec=15.0):
            return False, 'no /collision_monitor/get_state'
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            fut = self.state_cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.done() and fut.result() is not None:
                if fut.result().current_state.id == 3:
                    return True, 'active'
            time.sleep(1.0)
        return False, 'collision_monitor not active in %.0fs' % timeout

    def set_pose(self, x, y, z, yaw, timeout=6.0):
        if not self.cli.wait_for_service(timeout_sec=timeout):
            return False
        req = SetEntityPose.Request()
        req.entity.name = 'agc_car'
        req.entity.type = 2
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

    def spin_for(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)

    def get_param(self, node, name):
        for _ in range(2):
            try:
                out = subprocess.check_output(
                    ['ros2', 'param', 'get', node, name], text=True,
                    timeout=30, stderr=subprocess.STDOUT)
                return out.strip()
            except Exception:
                time.sleep(2)
        return None


def stop_detect_loop(node, sig_t, duration, key_fn, zero_thresh=0.05, quiet_s=0.3):
    samples = []
    last_move = sig_t
    t_stop = None
    prev_g = (node.gx, node.gy)
    while True:
        now = time.monotonic()
        if now - sig_t > duration:
            break
        node.publish_cmd(V_MAX, 0.0)
        rclpy.spin_once(node, timeout_sec=0.005)
        now = time.monotonic()
        spd = key_fn(node)
        moved_g = math.hypot(node.gx - prev_g[0], node.gy - prev_g[1]) > 0.002
        prev_g = (node.gx, node.gy)
        if spd > zero_thresh or moved_g:
            last_move = now
        samples.append({'t': round(now - sig_t, 3), 'out': round(node.v_out, 3),
                        'speed': round(spd, 3), 'x': round(node.x, 3), 'y': round(node.y, 3),
                        'gx': round(node.gx, 3), 'gy': round(node.gy, 3)})
        if now - last_move > quiet_s:
            t_stop = now
            break
    return t_stop, samples


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261015-w5-r008-0')
    ap.add_argument('--mode', default='gui')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()

    rclpy.init()
    n = R008()
    res = {'run_id': args.run_id, 'mode': args.mode, 'tests': {}, 'latency': {},
           'calibration': {'v_max': V_MAX, 'a_brake': A_BRAKE, 'source_timeout_T': T_CAL,
                           'd_blind': round(D_BLIND, 3), 's_brake': round(S_BRAKE, 3),
                           'total_stop': round(D_BLIND + S_BRAKE, 3),
                           'blind_bound': round(BLIND_BOUND, 3), 'tf_bound': round(TF_BOUND, 3)}}

    t0 = time.monotonic()
    while time.monotonic() - t0 < 12.0 and n.scan_f is None:
        rclpy.spin_once(n, timeout_sec=0.05)
    active, amsg = n.wait_active()
    res['collision_monitor_active'] = active
    res['collision_monitor_state_msg'] = amsg
    res['source_timeout_param'] = n.get_param('/collision_monitor', 'source_timeout')
    res['scan_source_timeout_param'] = n.get_param('/collision_monitor', 'scan.source_timeout')
    if not active:
        res['overall_pass'] = False
        res['error'] = amsg
        _dump(res, args)
        return 1

    if n.scan_f is not None:
        res['scan_meta'] = {'range_min': n.scan_f.range_min, 'range_max': n.scan_f.range_max,
                            'angle_min': round(n.scan_f.angle_min, 4),
                            'angle_increment': round(n.scan_f.angle_increment, 6),
                            'n_beams': len(n.scan_f.ranges)}

    # ---- LAT ----
    n.set_pose(30.0, 30.0, 0.02, 0.0)
    n.settle(2.0)
    t_start = time.monotonic()
    first_out = first_motion = None
    while time.monotonic() - t_start < 2.5:
        n.publish_cmd(0.8, 0.0)
        rclpy.spin_once(n, timeout_sec=0.005)
        now = time.monotonic()
        if first_out is None and n.v_out > 0.4:
            first_out = now
        if first_motion is None and n.odom_speed() > 0.05:
            first_motion = now
    lat_raw_to_out = (first_out - t_start) if first_out else None
    lat_raw_to_motion = (first_motion - t_start) if first_motion else None
    res['latency'] = {
        'cmd_raw_to_cmd_out_ms': round(lat_raw_to_out * 1000, 1) if lat_raw_to_out else None,
        'cmd_raw_to_motion_ms': round(lat_raw_to_motion * 1000, 1) if lat_raw_to_motion else None,
        'note': '量级观测（单调钟）：raw→安全层链路 ms 级；raw→真值运动含 DiffDrive 加速启动',
    }
    n.settle(1.5)

    # ---- A ----
    n.set_pose(0.0, -6.0 + POLE_HALF + 3.0, 0.02, -math.pi / 2)
    n.settle(1.6)
    d0 = math.hypot(n.x - POLE[0], n.y - POLE[1])
    samples = []
    tA = time.monotonic()
    stop_out_t = stop_t = None
    last_move = tA
    while time.monotonic() - tA < 12.0:
        n.publish_cmd(V_MAX, 0.0)
        rclpy.spin_once(n, timeout_sec=0.005)
        now = time.monotonic()
        d = math.hypot(n.x - POLE[0], n.y - POLE[1])
        samples.append((now - tA, n.v_out, n.odom_speed(), d, n.x, n.y))
        if stop_out_t is None and abs(n.v_out) < 0.05:
            stop_out_t = now
        if n.odom_speed() > 0.05:
            last_move = now
        if stop_out_t is not None and now - last_move > 0.3:
            stop_t = now
            break
    min_d = min((s[3] for s in samples), default=99.0)
    end_speed = samples[-1][2] if samples else 99.0
    travel_after = math.hypot(samples[-1][4] - samples[0][4], samples[-1][5] - samples[0][5]) if samples else 0.0
    stop_run_s = (stop_t - stop_out_t) if (stop_t and stop_out_t) else 0.0
    res['tests']['A_normal_obstacle'] = {
        'cmd': 'v=1.5 直行，立柱正前 ~3 m',
        'dist_at_start_m': round(d0, 3),
        'min_dist_to_pole_m': round(min_d, 3),
        'cmd_out_zero_reached': bool(stop_out_t is not None),
        'stop_run_s_after_cmd_zero': round(stop_run_s, 3),
        'end_odom_speed': round(end_speed, 3),
        'travel_m': round(travel_after, 3),
        'pass': bool(stop_out_t is not None and end_speed < 0.05 and min_d >= 0.5),
    }
    n.settle(1.5)

    # ---- B ----
    n.set_pose(30.0, 30.0, 0.02, 0.0)
    n.settle(1.6)
    tB = time.monotonic()
    while time.monotonic() - tB < 4.0:
        n.publish_cmd(V_MAX, 0.0)
        rclpy.spin_once(n, timeout_sec=0.005)
        if n.odom_speed() > 1.35:
            break
    v_steady = n.odom_speed()
    p_start = (n.x, n.y, n.gx, n.gy)
    pids = pgrep('[s]can_self_filter')
    b = {'n_pid': len(pids), 'pids': pids}
    if not pids:
        b['pass'] = False
        b['error'] = 'scan_self_filter 进程未找到'
        res['tests']['B_scan_source_lost'] = b
    else:
        for p in pids:
            os.kill(p, signal.SIGSTOP)
        t_sig = time.monotonic()
        try:
            t_stop, samples = stop_detect_loop(n, t_sig, 6.0, lambda nd: nd.odom_speed())
        finally:
            for p in pids:
                os.kill(p, signal.SIGCONT)
        end_speed = samples[-1]['speed'] if samples else 99.0
        blind_dist = math.hypot(n.x - p_start[0], n.y - p_start[1])
        min_out_after = min((s['out'] for s in samples), default=99.0)
        b.update({
            'v_steady_mps': round(v_steady, 3),
            'blind_displacement_m': round(blind_dist, 3),
            'blind_window_s': round((t_stop - t_sig), 3) if t_stop else None,
            'cmd_out_min_after_loss': round(min_out_after, 3),
            'end_odom_speed': round(end_speed, 3),
            'bound_m': round(BLIND_BOUND, 3),
            'pass': bool(t_stop is not None and end_speed < 0.05
                         and blind_dist <= BLIND_BOUND and min_out_after < 0.05),
        })
        res['tests']['B_scan_source_lost'] = b
        n.settle(1.0)

    # ---- C ----
    n.set_pose(34.0, 34.0, 0.02, 0.0)
    n.settle(1.6)
    tC = time.monotonic()
    while time.monotonic() - tC < 4.0:
        n.publish_cmd(V_MAX, 0.0)
        rclpy.spin_once(n, timeout_sec=0.005)
        if n.odom_speed() > 1.35:
            break
    c = {}
    pids = pgrep('[g]round_odom_adapter')
    c['n_pid'] = len(pids)
    c['pids'] = pids
    if not pids:
        c['pass'] = False
        c['error'] = 'ground_odom_adapter 进程未找到（TF 发布者）'
        res['tests']['C_tf_source_lost'] = c
    else:
        g0 = (n.gx, n.gy)
        for p in pids:
            os.kill(p, signal.SIGSTOP)
        t_sig = time.monotonic()
        try:
            t_stop, samples = stop_detect_loop(n, t_sig, 6.0, lambda nd: 0.0)
        finally:
            for p in pids:
                os.kill(p, signal.SIGCONT)
        gt_dist = math.hypot(n.gx - g0[0], n.gy - g0[1])
        min_out_after = min((s['out'] for s in samples), default=99.0)
        c.update({
            'gt_displacement_m': round(gt_dist, 3),
            'blind_window_s': round((t_stop - t_sig), 3) if t_stop else None,
            'cmd_out_min_after_loss': round(min_out_after, 3),
            'bound_m': round(TF_BOUND, 3),
            'pass': bool(t_stop is not None and min_out_after < 0.05 and gt_dist <= TF_BOUND),
        })
        res['tests']['C_tf_source_lost'] = c
        n.settle(1.0)
        n.spin_for(1.0)

    res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    return _dump(res, args)


def _dump(res, args):
    out_dir = os.path.join(args.experiments_root, 'w5-ground-nav-safety',
                           args.run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'r008.json')
    with open(path, 'w') as fh:
        json.dump(res, fh, indent=2, ensure_ascii=False)
    for name, t in res['tests'].items():
        print('%-22s %s  %s' % (name, 'PASS' if t.get('pass') else 'FAIL',
                                {k: v for k, v in t.items() if k not in ('pass', 'pids')}))
    print('latency:', res.get('latency'))
    print('scan_meta:', res.get('scan_meta'))
    print('source_timeout_param:', res.get('source_timeout_param'))
    print('scan_source_timeout_param:', res.get('scan_source_timeout_param'))
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


if __name__ == '__main__':
    sys.exit(main())
