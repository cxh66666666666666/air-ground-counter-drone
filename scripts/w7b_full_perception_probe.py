#!/usr/bin/env python3
"""w7b_full_perception_probe —— W7-b 步 1「整链评价可用」**内省取证**。

目的：定位 full 整链「观测链」在哪一段断供（实测现象：run_logger `visibility_rate = 0`
→ 地面拿不到目标 → 任务跑满 t_max 超时）。链路：

    /agc/gt/intruder_uav  ┐
    /agc/air/odom         ┴→ air_perception → /agc/air/obs ┐
                                                           ┴→ air_fusion → /agc/targets
                                                                          → /agc/obstacles
    /agc/task/state · /agc/task/envelope_in · /agc/ground/cmd_vel   （阶段 / 交战门 / 执行口）

窗口内如实记录：到达计数 / 速率、`header.stamp` 是否**负 sec**（上游负 stamp 的证据）、
数组长度（`sensors` / `tracks` / 目标条数）、阶段名、`envelope_in` 为真拍数。

air_perception 被守卫**拒收**的证据（一次性 WARN）在**节点日志**里，由调用方 grep，不在本脚本。

用法（配合已起的 full 整链）：
  python3 scripts/w7b_full_perception_probe.py --seconds 20 --tag during --start
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from agc_msgs.msg import (
    AgcTaskState,
    AirObservationArray,
    ObstacleTrackArray,
    TargetTrackArray,
)
from std_srvs.srv import Trigger

PROBE = [
    ('/agc/gt/intruder_uav', PoseStamped, 'stamp'),
    ('/agc/air/odom', Odometry, 'stamp'),
    ('/agc/air/obs', AirObservationArray, 'stamp'),
    ('/agc/targets', TargetTrackArray, 'stamp'),
    ('/agc/obstacles', ObstacleTrackArray, 'stamp'),
    ('/agc/task/state', AgcTaskState, 'stamp'),
    ('/agc/task/envelope_in', Bool, 'none'),
    ('/agc/ground/cmd_vel', TwistStamped, 'stamp'),
]


class Probe(Node):
    def __init__(self):
        super().__init__('w7b_full_perception_probe')
        self.n = {name: 0 for name, _t, _h in PROBE}
        self.smin = {name: None for name, _t, _h in PROBE}
        self.smax = {name: None for name, _t, _h in PROBE}
        self.neg = {name: 0 for name, _t, _h in PROBE}
        self.lens = {name: [] for name, _t, _h in PROBE}
        self.tgt_n = {name: [] for name, _t, _h in PROBE}
        self.last_pos = {}
        self.last_yaw = {}
        self.states = []
        self.last_state = None
        self.env_true = 0
        for name, typ, kind in PROBE:
            self.create_subscription(typ, name, self._mk(name, kind), 20)

    def _mk(self, name, kind):
        def cb(m):
            self.n[name] += 1
            if kind == 'stamp':
                s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                if self.smin[name] is None or s < self.smin[name]:
                    self.smin[name] = s
                if self.smax[name] is None or s > self.smax[name]:
                    self.smax[name] = s
                if m.header.stamp.sec < 0:
                    self.neg[name] += 1
            if name == '/agc/air/obs':
                self.lens[name].append(len(getattr(m, 'sensors', [])))
                # 关键区分：观测**确实带了目标**吗（sensor 里 TargetObservation 条数）
                self.tgt_n[name].append(
                    sum(len(getattr(so, 'targets', [])) for so in getattr(m, 'sensors', [])))
            if name in ('/agc/targets', '/agc/obstacles'):
                self.lens[name].append(len(getattr(m, 'tracks', [])))
            if name in ('/agc/air/odom', '/agc/gt/intruder_uav'):
                # 几何内省：己方机（Odometry） / 目标（PoseStamped）最近位置（世界 ENU）
                q = (m.pose.pose.orientation if name == '/agc/air/odom'
                     else m.pose.orientation)
                p = m.pose.pose.position if name == '/agc/air/odom' else m.pose.position
                self.last_pos[name] = (p.x, p.y, p.z)
                self.last_yaw[name] = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            if name == '/agc/task/state':
                if m.state_name != self.last_state:
                    self.last_state = m.state_name
                    self.states.append(m.state_name)
            if name == '/agc/task/envelope_in' and bool(m.data):
                self.env_true += 1
        return cb


def call_start(node, tries=3, per_try_s=8.0):
    cli = node.create_client(Trigger, '/agc/task/start')
    if not cli.wait_for_service(timeout_sec=per_try_s):
        print('[start] 服务不可用')
        return
    for i in range(tries):
        # 本机首个服务调用常超时（发现预热）——重试并忽略前次结果
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=per_try_s)
        if fut.done() and fut.result() is not None:
            print('[start] try=%d success=%s msg=%s' % (
                i + 1, fut.result().success, (fut.result().message or '')[:80]))
            return
    print('[start] %d 次尝试均超时' % tries)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--tag', default='probe')
    ap.add_argument('--start', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    node = Probe()
    if args.start:
        call_start(node)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    dt = max(time.monotonic() - t0, 1e-6)

    print('=== w7b 观测链内省 tag=%s 窗口=%.1fs ===' % (args.tag, dt))
    for name, _t, kind in PROBE:
        n = node.n[name]
        stamp = ''
        if kind == 'stamp' and node.smin[name] is not None:
            stamp = ' stamp[min=%.3f max=%.3f neg*=%d]' % (
                node.smin[name], node.smax[name], node.neg[name])
        lens = node.lens[name]
        ltxt = ''
        if lens:
            ltxt = ' len[max=%d min=%d]' % (max(lens), min(lens))
        tn = node.tgt_n[name]
        if tn:
            ltxt += ' 目标条数[max=%d]' % max(tn)
        pos = node.last_pos.get(name)
        if pos:
            ltxt += ' 位置=(%.2f,%.2f,%.2f)' % pos
        print('  %-26s n=%-5d rate=%6.2f Hz%s%s' % (name, n, n / dt, stamp, ltxt))
    print('  阶段序列: %s' % (' → '.join(node.states) or '（无 /agc/task/state）'))
    print('  envelope_in 为真拍数: %d' % node.env_true)
    # 航向 × 视场：己方机偏航 vs 目标方位（机体系前视 hfov ±50° → |Δ| ≤ 50° 才可能看见）
    uav_p = node.last_pos.get('/agc/air/odom')
    tgt_p = node.last_pos.get('/agc/gt/intruder_uav')
    if uav_p and tgt_p:
        yaw = node.last_yaw.get('/agc/air/odom')
        brg = math.atan2(tgt_p[1] - uav_p[1], tgt_p[0] - uav_p[0])
        d = math.degrees((brg - yaw + math.pi) % (2 * math.pi) - math.pi)
        print('  己方机 yaw=%.1f° 目标方位=%.1f° Δ=%+.1f°（机体系前视 ±50° 才可见）| 斜距=%.2f m | 高度差=%+.2f m'
              % (math.degrees(yaw), math.degrees(brg), d,
                 math.hypot(tgt_p[0] - uav_p[0], tgt_p[1] - uav_p[1]), tgt_p[2] - uav_p[2]))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
