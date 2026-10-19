#!/usr/bin/env python3
"""w7_d1_check —— W7 入口前置 D1（时空与帧正确：R-002 / R-003）自包含对拍。

拉起 `agc_air` 的 **px4_adapter**（生命周期节点），以**合成 `px4_msgs/VehicleOdometry`** 驱动，
逐项断言：
  a 时间单位（R-002）  —— 两条 msg 的 `timestamp` 差 1e6 µs → `/agc/air/odom` Header 差 ≈ 1e9 ns
                          （误用 `toRosMicros` 会得 1e6 ns，缩小 1000 倍）
  b 时间锚定           —— Header ≈ 发送时刻墙钟（唯一 ROS 时钟）
  c 位置原点（R-003）  —— `origin_pose=10,-5,0.2` 时，世界 ENU = 原点 + nedToEnu(2,3,-1) = (13,-3,1.2)
  d 速度帧（R-003）    —— 机头朝北 + vel NED=(1,0,0) → twist.linear（child frame=机体系 FLU）=(1,0,0)
  e 角速度帧（R-003）  —— ang FRD=(0.1,-0.2,0.3) → FLU=(0.1,0.2,-0.3)
  f 帧名（契约 #namespace）—— frame_id=world、child_frame_id=drone/base_link
  g 非 NED 帧丢弃（R-003） —— pose_frame=FRD 的 msg 不产出 `/agc/air/odom`

对拍向量取自既有单测 `src/agc_air/test/test_px4_conversions.cpp`（**不**另造期望值）。

证据落 `experiments/w7-rl/<run_id>/`：`config/run_identity.json`、`d1/{bench.json,check.json,curves.csv}`、
`metrics/d1.json`。退出码 0 = PASS。须由 `scripts/archive/run_w7_d1.sh` 拉起（已 source ROS + 工作区）。
"""

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

ROOT = '/home/chen/AGC'
CAMPAIGN = 'w7-rl'
NODE_NAME = 'px4_adapter'
TOPIC_IN = '/fmu/out/vehicle_odometry'
ODOM_OUT = '/agc/air/odom'

# 对拍向量（同 src/agc_air/test/test_px4_conversions.cpp）
ORIGIN = '10,-5,0.2,0,0,0'
POS_NED = (2.0, 3.0, -1.0)
Q_XYZW = (0.0, 0.0, 0.0, 1.0)      # 机头朝北、水平
VEL_NED = (1.0, 0.0, 0.0)          # 北向 1 m/s
ANG_FRD = (0.1, -0.2, 0.3)
EXP_POSE = (13.0, -3.0, 1.2)
EXP_TWIST = (1.0, 0.0, 0.0)
EXP_ANG = (0.1, 0.2, -0.3)
POSE_FRAME_NED = 1
POSE_FRAME_FRD = 2
VELOCITY_FRAME_NED = 1

TOL_UNIT_RATIO = 0.02          # |Δ − 1e9| / 1e9
TOL_ANCHOR_S = 0.20            # Header 与墙钟差（s）
TOL_POSE_M = 1e-5
TOL_TWIST = 1e-5
TOL_ANG = 1e-5


class Probe(Node):
    def __init__(self):
        super().__init__('w7_d1_probe')
        self.last = None
        self.count = 0
        self.rows = []
        self.pub = self.create_publisher(VehicleOdometry, TOPIC_IN, qos_profile_sensor_data)
        self.sub = self.create_subscription(Odometry, ODOM_OUT, self._on_odom, 10)
        self.cli = self.create_client(ChangeState, '/' + NODE_NAME + '/change_state')

    def _on_odom(self, m):
        self.last = m
        self.count += 1

    def spin_until(self, pred, timeout, period=0.02):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=period)
            if pred():
                return True
        return False

    def call(self, cli, req, timeout=10.0):
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not fut.done():
            return None
        return fut.result()

    def change_state(self, tid, timeout=10.0):
        req = ChangeState.Request()
        req.transition.id = tid
        return self.call(self.cli, req, timeout)


def make_msg(t_us, pose_frame=POSE_FRAME_NED):
    m = VehicleOdometry()
    m.timestamp = int(t_us)
    m.timestamp_sample = int(t_us)
    m.pose_frame = int(pose_frame)
    m.velocity_frame = int(VELOCITY_FRAME_NED)
    m.position = [float(v) for v in POS_NED]
    m.q = [float(v) for v in Q_XYZW]
    m.velocity = [float(v) for v in VEL_NED]
    m.angular_velocity = [float(v) for v in ANG_FRD]
    m.reset_counter = 0
    return m


def stamp_ns(odom):
    return int(odom.header.stamp.sec) * 10**9 + int(odom.header.stamp.nanosec)


def close3(got, exp, tol):
    return all(abs(float(got[i]) - exp[i]) <= tol for i in range(3))


def near(got, exp, tol):
    return abs(float(got) - exp) <= tol


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()

    run_dir = os.path.join(args.experiments_root, CAMPAIGN, args.run_id)
    for sub in ('config', 'd1', 'metrics', 'log'):
        os.makedirs(os.path.join(run_dir, sub), exist_ok=True)
    log_path = os.path.join(run_dir, 'log', 'px4_adapter.log')

    res = {'run_id': args.run_id, 'run_dir': run_dir, 'origin_pose': ORIGIN,
           'node': NODE_NAME, 'tests': {}, 'numbers': {}}
    proc = None
    node = None
    try:
        res['numbers']['git_commit'] = subprocess.run(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True).stdout.strip()
        with open(os.path.join(run_dir, 'config', 'run_identity.json'), 'w', encoding='utf-8') as f:
            json.dump({'campaign': CAMPAIGN, 'run_id': args.run_id,
                       'git_commit': res['numbers']['git_commit'],
                       'probe': 'tests/w7_d1_check.py',
                       'node_params': {'odom_in_topic': TOPIC_IN, 'origin_pose': ORIGIN,
                                       'bond_heartbeat_period': 0.0},
                       'check_vectors': {'pos_ned': list(POS_NED), 'q_xyzw': list(Q_XYZW),
                                         'vel_ned': list(VEL_NED), 'ang_frd': list(ANG_FRD),
                                         'expect_pose_enu': list(EXP_POSE)},
                       'env': {'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID'),
                               'ROS_AUTOMATIC_DISCOVERY_RANGE':
                                   os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')}},
                      f, indent=2, ensure_ascii=False)

        logf = open(log_path, 'w', encoding='utf-8')
        proc = subprocess.Popen(
            ['ros2', 'run', 'agc_air', NODE_NAME, '--ros-args',
             '-p', 'odom_in_topic:=' + TOPIC_IN,
             '-p', 'origin_pose:=' + ORIGIN,
             '-p', 'bond_heartbeat_period:=0.0'],
            stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)

        rclpy.init()
        node = Probe()
        # 本机 DDS：**graph 查询不可靠**（`get_node_names` / `wait_for_service` 常空或"假真"），
        # 且**首个服务调用常超时**（发现预热），其后即通 —— 实测本机已验证可用序列为
        # 「先发一次 get_state（忽略结果）→ 再以长超时 configure/activate」；此处复刻该序列。
        gs = node.create_client(GetState, '/' + NODE_NAME + '/get_state')
        node.spin_until(lambda: False, 4.0)
        node.call(gs, GetState.Request(), 20.0)   # 预热（结果忽略）

        for tid, nm in ((Transition.TRANSITION_CONFIGURE, 'configure'),
                        (Transition.TRANSITION_ACTIVATE, 'activate')):
            ok = False
            for _ in range(4):
                r = node.change_state(tid, timeout=20.0)
                if r is not None and getattr(r, 'success', False):
                    ok = True
                    break
            res['numbers'][nm + '_ok'] = ok
            if not ok:
                raise RuntimeError('px4_adapter 未成功 %s（见 log/px4_adapter.log）' % nm)

        # 预热：确认 pub/sub 已匹配（并锚定 mapper 的偏移）
        warm_ok = False
        for _ in range(60):
            node.pub.publish(make_msg(int(time.time() * 1e6) - 10_000_000))
            if node.spin_until(lambda: node.count > 0, 0.2):
                warm_ok = True
                break
        res['numbers']['warmup_matched'] = warm_ok
        if not warm_ok:
            raise RuntimeError('预热未收到 /agc/air/odom：pub/sub 未匹配（QoS 或 DDS？）')

        node.spin_until(lambda: False, 1.0)   # 稳定 1 s

        # 时间对：P1 与 P2 的 `timestamp` 差 1 s，且**墙钟同样间隔 1 s**（保持 `ros−px4` 偏移不变）；
        # 否则指数滤波偏移会被平滑拉到新值，Δstamp ≠ ΔT（那是设计问题，不是 R-002 的单位问题）。
        t1 = int(time.time() * 1e6) - 10_000_000
        t2 = t1 + 1_000_000
        n0 = node.count
        wall1 = time.time()
        node.pub.publish(make_msg(t1))
        got1 = node.spin_until(lambda: node.count > n0, 3.0)
        o1 = node.last
        n1 = node.count
        node.spin_until(lambda: False, 1.0)     # 墙钟推进 1 s，与 ΔT 对齐
        node.pub.publish(make_msg(t2))
        got2 = node.spin_until(lambda: node.count > n1, 3.0)
        o2 = node.last

        res['numbers'].update({'t1_us': t1, 't2_us': t2, 'got_odom_p1': got1, 'got_odom_p2': got2})
        if got1 and got2 and o1 is not None and o2 is not None:
            s1, s2 = stamp_ns(o1), stamp_ns(o2)
            d_ns = s2 - s1
            res['numbers'].update({'stamp1_ns': s1, 'stamp2_ns': s2, 'delta_ns': d_ns,
                                   'delta_expected_ns': 10**9,
                                   'stamp1_minus_wall_s': (s1 / 1e9) - wall1})
            res['tests']['a_time_unit_ns'] = {
                'pass': near(d_ns, 10**9, TOL_UNIT_RATIO * 10**9),
                'delta_ns': d_ns, 'expected_ns': 10**9,
                'ratio': d_ns / 1e9, 'tol_ratio': TOL_UNIT_RATIO}
            res['tests']['b_time_anchor_wallclock'] = {
                'pass': near(s1 / 1e9, wall1, TOL_ANCHOR_S),
                'stamp1_s': s1 / 1e9, 'wall_send_s': wall1, 'tol_s': TOL_ANCHOR_S}
        else:
            res['tests']['a_time_unit_ns'] = {'pass': False, 'reason': 'P1/P2 未各自收到 odom'}
            res['tests']['b_time_anchor_wallclock'] = {'pass': False, 'reason': 'P1/P2 未各自收到 odom'}

        o = o2 if o2 is not None else o1
        if o is None:
            for k in ('c_pose_origin_enu', 'd_twist_body_flu', 'e_angular_body_flu', 'f_frame_ids'):
                res['tests'][k] = {'pass': False, 'reason': '未收到 odom'}
        else:
            p = o.pose.pose.position
            vl = o.twist.twist.linear
            va = o.twist.twist.angular
            res['numbers'].update({
                'pose_enu': [p.x, p.y, p.z], 'twist_body_flu': [vl.x, vl.y, vl.z],
                'angular_body_flu': [va.x, va.y, va.z],
                'frame_id': o.header.frame_id, 'child_frame_id': o.child_frame_id})
            res['tests']['c_pose_origin_enu'] = {
                'pass': close3((p.x, p.y, p.z), EXP_POSE, TOL_POSE_M),
                'got': [p.x, p.y, p.z], 'expected': list(EXP_POSE)}
            res['tests']['d_twist_body_flu'] = {
                'pass': close3((vl.x, vl.y, vl.z), EXP_TWIST, TOL_TWIST),
                'got': [vl.x, vl.y, vl.z], 'expected': list(EXP_TWIST)}
            res['tests']['e_angular_body_flu'] = {
                'pass': close3((va.x, va.y, va.z), EXP_ANG, TOL_ANG),
                'got': [va.x, va.y, va.z], 'expected': list(EXP_ANG)}
            res['tests']['f_frame_ids'] = {
                'pass': (o.header.frame_id == 'world' and o.child_frame_id == 'drone/base_link'),
                'got': [o.header.frame_id, o.child_frame_id],
                'expected': ['world', 'drone/base_link']}

        # 非 NED 帧应被丢弃（R-003）
        n2 = node.count
        node.pub.publish(make_msg(int(time.time() * 1e6) - 10_000_000, pose_frame=POSE_FRAME_FRD))
        node.spin_until(lambda: node.count > n2, 0.8)
        res['tests']['g_non_ned_dropped'] = {
            'pass': node.count == n2, 'odom_before': n2, 'odom_after': node.count}

    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        if proc is not None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=8)
            except Exception:  # noqa: BLE001
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:  # noqa: BLE001
                    pass

    res['overall_pass'] = bool(res['tests']) and all(t.get('pass') for t in res['tests'].values())
    for name in sorted(res['tests']):
        print('%-26s %s' % (name, 'PASS' if res['tests'][name].get('pass') else 'FAIL'))
    if 'error' in res:
        print('error:', res['error'])
    with open(os.path.join(run_dir, 'd1', 'bench.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'd1', 'check.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'metrics', 'd1.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    with open(os.path.join(run_dir, 'd1', 'curves.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'stamp_ns', 'pose_x', 'pose_y', 'pose_z',
                    'twist_x', 'twist_y', 'twist_z', 'ang_x', 'ang_y', 'ang_z'])
    print('\n%s -> %s' % ('PASS' if res['overall_pass'] else 'FAIL',
                          os.path.join(run_dir, 'd1', 'check.json')))
    return 0 if res['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
