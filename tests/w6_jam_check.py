#!/usr/bin/env python3
"""w6_jam_check —— W6 R8 干扰链（作用包络门控 + 命令输出）轻量验收。

不依赖 PX4：向 `capture_adjudicator` 提供车/目标位姿，令目标**进入作用包络**；随后
  * 先发 `jam_cmd`（包络外/未就绪时）→ `jammer` 应**拒绝**（`jam_state` 不置 JAM_*）；
  * 再在包络内发 `jam_cmd=LAND` → `jammer` 应发 `/px4_1/fmu/in/vehicle_command`
    （`VEHICLE_CMD_SET_NAV_STATE`，param1=18=AUTO_LAND）并置 `jam_state=JAM_LAND`。

PX4 侧响应（`nav_state 14→18`）已由前置 P 实跑；本条补"包络门控 + 命令发射"路径。

用法（capture_adjudicator + jammer 已在运行）：
  python3 tests/w6_jam_check.py --run-id 20261014-w6-jam-0
"""

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from agc_msgs.msg import TargetTrackArray, TargetTrack
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommand
from std_msgs.msg import Bool, String


def _qos():
    q = QoSProfile(depth=5)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    return q


class JamCheck(Node):
    def __init__(self, target_xy):
        super().__init__('w6_jam_check')
        self.target_xy = target_xy
        self.envelope = False
        self.jam_state = 'NONE'
        self.cmds = []
        self.pub_odom = self.create_publisher(Odometry, '/agc/ground/odom', 20)
        self.pub_tgt = self.create_publisher(TargetTrackArray, '/agc/targets', 20)
        self.pub_jam = self.create_publisher(String, '/agc/task/jam_cmd', 10)
        self.create_subscription(Bool, '/agc/task/envelope_in', self._on_env, 10)
        self.create_subscription(String, '/agc/task/jam_state', self._on_state, 10)
        self.create_subscription(VehicleCommand, '/px4_1/fmu/in/vehicle_command',
                                 self._on_cmd, _qos())

    def _on_env(self, m):
        self.envelope = bool(m.data)

    def _on_state(self, m):
        self.jam_state = m.data

    def _on_cmd(self, m):
        self.cmds.append((int(m.command), float(m.param1)))

    def _publish_inputs(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'world'
        o.child_frame_id = 'car/base_link'
        o.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(o)

        arr = TargetTrackArray()
        arr.header.stamp = o.header.stamp
        arr.header.frame_id = 'world'
        t = TargetTrack()
        t.header = arr.header
        t.target_id = 'intruder_uav'
        t.pose = PoseWithCovariance()
        t.pose.pose.position.x = self.target_xy[0]
        t.pose.pose.position.y = self.target_xy[1]
        t.pose.pose.position.z = 0.5
        t.pose.pose.orientation.w = 1.0
        t.twist = TwistWithCovariance()
        t.confidence = 0.9
        t.source = 'stub'
        arr.tracks = [t]
        self.pub_tgt.publish(arr)

    def spin_for(self, dur):
        tw = time.monotonic()
        while time.monotonic() - tw < dur:
            self._publish_inputs()
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-jam-0')

    rclpy.init()
    # 目标远（包络外）→ 先测"门控拒绝"；随后移近（包络内）→ 测"命中发射"
    node = JamCheck(target_xy=(30.0, 30.0))
    node.spin_for(2.0)
    out_of_env = node.envelope
    node.pub_jam.publish(String(data='LAND'))
    node.spin_for(1.5)
    rejected = (node.jam_state != 'JAM_LAND') and (not node.cmds)
    print('out-of-envelope: env=%s jam_state=%s cmds=%d -> rejected=%s'
          % (out_of_env, node.jam_state, len(node.cmds), rejected))

    # 移入包络内
    node.target_xy = (0.8, 0.0)
    tw = time.monotonic()
    while time.monotonic() - tw < 3.0 and not node.envelope:
        node.spin_for(0.2)
    in_env = node.envelope
    node.cmds.clear()
    node.jam_state = 'NONE'
    node.pub_jam.publish(String(data='LAND'))
    node.spin_for(2.0)
    nav_cmds = [(c, p) for (c, p) in node.cmds if c == VehicleCommand.VEHICLE_CMD_SET_NAV_STATE]
    emitted = bool(nav_cmds) and abs(nav_cmds[-1][1] - 18.0) < 0.5
    state_ok = (node.jam_state == 'JAM_LAND')
    print('in-envelope: env=%s jam_state=%s nav_cmds=%s emitted=%s'
          % (in_env, node.jam_state, nav_cmds[-1:], emitted))

    all_ok = (not out_of_env) and rejected and in_env and emitted and state_ok
    node.destroy_node()
    rclpy.shutdown()

    payload = {'run_id': run_id, 'out_of_envelope_rejected': rejected,
               'in_envelope': in_env, 'jam_state': node.jam_state,
               'nav_state_cmd': nav_cmds[-1] if nav_cmds else None,
               'command_emitted': emitted, 'pass': bool(all_ok)}
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'jam.json')
    with open(path, 'w') as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print('\n%s → %s' % ('PASS' if all_ok else 'FAIL', path))
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
