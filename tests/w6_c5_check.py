#!/usr/bin/env python3
"""w6_c5_check —— W6-a R3/C5（相位复位触发）轻量验收。

不依赖 PX4：单独起 `trajectory_tracker`（lifecycle），由本脚本提供
  * `/agc/air/odom`（假 odom，50 Hz，使 offboard odom 有效）；
  * `/agc/air/active_controller`（latched = "trajectory_tracker"，使 tracker 接管并发布 ref）；
经 `configure`→`activate` 后，读取 `/agc/air/ref` 轨迹点（相位推进），再调用
`/agc/air/reset_phase`，校验相位归零（ref 回到接管起点、出现回跳）。

**注意**：本脚本与 tracker 均不使用仿真时钟（默认 use_sim_time=false，走墙钟），
否则无 /clock 时 now() 冻结、相位不推进。

结果落 `experiments/w6-task-eval/<run_id>/metrics/c5.json`（ADR-0007）。
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import MultiDOFJointTrajectory


def _pos(msg):
    if not msg.points:
        return None
    tf = msg.points[0].transforms
    if not tf:
        return None
    t = tf[0].translation
    return (t.x, t.y, t.z)


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


class C5Checker(Node):
    def __init__(self):
        super().__init__('w6_c5_check')
        self.pub_odom = self.create_publisher(Odometry, '/agc/air/odom', 20)
        # tracker 以 transient_local 订阅 active_controller（latched 语义），发布端须匹配，
        # 否则 QoS 不兼容、消息不投递（tracker 不接管 → ref 恒为 phase 0）。
        active_qos = QoSProfile(depth=1)
        active_qos.reliability = ReliabilityPolicy.RELIABLE
        active_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_active = self.create_publisher(String, '/agc/air/active_controller', active_qos)
        self.refs = []  # (t_wall, pos)
        self.create_subscription(MultiDOFJointTrajectory, '/agc/air/ref', self._on_ref, 20)
        self.cli = self.create_client(Trigger, '/agc/air/reset_phase')
        self._t0 = time.monotonic()
        self.last_ref = None

    def _on_ref(self, msg):
        p = _pos(msg)
        if p is not None:
            self.refs.append((time.monotonic() - self._t0, p))
            self.last_ref = p

    def spin_for(self, dur):
        tw = time.monotonic()
        while time.monotonic() - tw < dur:
            self._publish_inputs()
            rclpy.spin_once(self, timeout_sec=0.02)

    def _publish_inputs(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'world'
        o.child_frame_id = 'drone/base_link'
        o.pose.pose.position.x = 0.0
        o.pose.pose.position.y = 0.0
        o.pose.pose.position.z = 2.5
        o.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(o)
        s = String()
        s.data = 'trajectory_tracker'
        self.pub_active.publish(s)


def _lifecycle(node_name, transition):
    r = subprocess.run(
        ['ros2', 'lifecycle', 'set', '/' + node_name, transition],
        capture_output=True, text=True, timeout=20)
    return (r.returncode == 0), (r.stdout + r.stderr).strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-c5-0')

    rclpy.init()
    node = C5Checker()

    transitions = {}
    node.spin_for(1.5)
    for tr in ('configure', 'activate'):
        ok, out = _lifecycle('trajectory_tracker', tr)
        transitions[tr] = {'ok': ok, 'out': out}
        print('lifecycle %s: %s' % (tr, 'ok' if ok else 'FAIL -> ' + out))
        node.spin_for(0.5)

    # 采集相位推进段
    node.spin_for(3.0)
    before = list(node.refs)
    before_last = node.last_ref

    # 调相位复位
    if not node.cli.wait_for_service(timeout_sec=5.0):
        print('FAIL: /agc/air/reset_phase 服务不可用')
        node.destroy_node(); rclpy.shutdown()
        return 1
    fut = node.cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result() if fut.done() else None
    reset_ok = bool(res and res.success)
    print('reset_phase: success=%s msg=%s' % (reset_ok, res.message if res else 'timeout'))

    node.refs.clear()
    node.spin_for(2.5)
    after = list(node.refs)

    p_first = before[0][1] if before else None
    p_pre = before_last if before_last else None
    p_post = after[0][1] if after else None

    moved_away = bool(p_first and p_pre and _dist(p_first, p_pre) > 0.5)
    jump = bool(p_pre and p_post and _dist(p_pre, p_post) > 0.5)
    returned = bool(p_first and p_post and _dist(p_first, p_post) < 0.4)
    all_ok = reset_ok and moved_away and jump and returned and transitions['activate']['ok']

    print('moved_away=%s jump=%s returned_to_start=%s' % (moved_away, jump, returned))
    print('p_first=%s p_pre=%s p_post=%s' % (p_first, p_pre, p_post))

    node.destroy_node()
    rclpy.shutdown()

    payload = {
        'run_id': run_id,
        'transitions': transitions,
        'reset_phase_ok': reset_ok,
        'reset_message': (res.message if res else None),
        'n_refs_before': len(before), 'n_refs_after': len(after),
        'p_start': p_first, 'p_pre_reset': p_pre, 'p_post_reset': p_post,
        'moved_away': moved_away, 'jump_on_reset': jump, 'returned_to_start': returned,
        'pass': bool(all_ok),
    }
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'c5.json')
    with open(path, 'w') as fh:
        json.dump(payload, fh, indent=2, ensure_ascii=False)
    print('\n%s → %s' % ('PASS' if all_ok else 'FAIL', path))
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
