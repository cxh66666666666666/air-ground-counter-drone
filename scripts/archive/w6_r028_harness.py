#!/usr/bin/env python3
"""w6_r028_harness —— R-028（episode 窗口统计 / 终态锁存 / 有效样本掩码）确定性合成 harness。

为 `run_logger` 提供**合成诊断数据**（**不**命令任何车体运动，**不**发布 `/agc/ground/cmd_vel`，
只用 `/clock`、`/agc/ground/odom`、`/agc/ground/scan`、`/collision_monitor_state`、`/agc/task/state`），
以可控复现「陈旧 scan / 预热 / 终态锁存 / 无 scan 拒绝 / 无任务拒绝」。

用法：`python3 scripts/archive/w6_r028_harness.py --out-dir <RUN_ID>/summaries [--cases a,b,...]`

每个 case：起一个 `run_logger`（`use_sim_time:=true`，`contact_topic:=` 空），等 1.5s 后按脚本发布；
发布完成后对 logger 进程 SIGINT，待其 finalize 落 `summary.json`。用 glob + mtime 定位该 case 的
`summary.json`（**不**硬编码日期前缀 —— R-029 起 run_id 唯一不可覆盖），拷为
`<out_dir>/harness-<case>.json`，并打印关键数字。

时钟：harness 自身 `use_sim_time=false`；20ms wall timer 为拍；每拍发布 `/clock`
（sec 从 1.0 起，每拍 +0.02），并发布 odom（`twist.twist.linear.x=0.3`）与 cm；按 case 计划发布
scan 与 state。roscale 与墙钟差异见 ADR-0043。
"""

import argparse
import glob
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import CollisionMonitorState
from agc_msgs.msg import AgcTaskState

ROOT = '/home/chen/AGC'
EXP = os.path.join(ROOT, 'experiments', 'w6-task-eval')
TICK_DT = 0.02      # 每拍 20ms（wall）
CLOCK_T0 = 1.0      # 仿真时钟起点（s）；每拍 +TICK_DT

# 6 个 case 的发布计划：scenario 名即 run_id 中的场景段。
#   ticks  : 总拍数
#   states : {tick: (state(uint8), state_name, reason_code)}（仅命中该拍时发布）
#   scan   : fn(tick) -> bool（是否发布 scan 帧）
#   cm     : fn(tick) -> action_type（每拍都发布）
CASES = {
    'r028-no-task': dict(
        ticks=60,
        states={},
        scan=lambda t: t % 2 == 0,
        cm=lambda t: 0),
    'r028-noscan': dict(
        ticks=60,
        states={10: (1, 'AIR_SEARCH', 'ok'), 50: (5, 'MISSION_FAILED', 'TIMEOUT')},
        scan=lambda t: False,
        cm=lambda t: 0 if t < 12 else 1),
    'r028-stale-scan': dict(
        ticks=180,
        states={10: (1, 'AIR_SEARCH', 'ok'), 160: (5, 'MISSION_FAILED', 'TIMEOUT')},
        scan=lambda t: 10 <= t <= 40 and t % 2 == 0,
        cm=lambda t: 0 if t < 12 else 1),
    'r028-warmup-short': dict(
        ticks=100,
        states={20: (1, 'AIR_SEARCH', 'ok'), 80: (5, 'MISSION_FAILED', 'TIMEOUT')},
        scan=lambda t: t >= 20 and t % 2 == 0,
        cm=lambda t: 0 if t < 22 else 1),
    'r028-warmup-long': dict(
        ticks=140,
        states={60: (1, 'AIR_SEARCH', 'ok'), 120: (5, 'MISSION_FAILED', 'TIMEOUT')},
        scan=lambda t: t >= 60 and t % 2 == 0,
        cm=lambda t: 0 if t < 62 else 1),
    'r028-terminal-latch': dict(
        ticks=60,
        states={10: (1, 'AIR_SEARCH', 'ok'),
                30: (4, 'TRACKING_SUCCESS', 'tracking_milestone'),
                40: (5, 'MISSION_FAILED', 'TIMEOUT'),
                50: (1, 'AIR_SEARCH', 'ok')},
        scan=lambda t: t >= 10 and t % 2 == 0,
        cm=lambda t: 0 if t < 12 else 1),
}


class Harness(Node):

    def __init__(self, plan, scenario):
        super().__init__('w6_r028_harness')
        self.plan = plan
        self.scenario = scenario
        self.tick = 0
        self.done = False
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_odom = self.create_publisher(Odometry, '/agc/ground/odom', 50)
        self.pub_scan = self.create_publisher(
            LaserScan, '/agc/ground/scan', qos_profile_sensor_data)
        self.pub_cm = self.create_publisher(
            CollisionMonitorState, '/collision_monitor_state', 20)
        self.pub_state = self.create_publisher(AgcTaskState, '/agc/task/state', 10)
        self.timer = None

    def start(self):
        """发现 / 匹配稳定后再启动发布拍（避免首发消息在 DDS 匹配前丢弃）。"""
        self.timer = self.create_timer(TICK_DT, self._on_tick)

    def _on_tick(self):
        t = self.tick
        if t >= self.plan['ticks']:
            self.done = True
            return
        sec = CLOCK_T0 + t * TICK_DT
        total_ns = int(round(sec * 1e9))
        clk = Clock()
        clk.clock.sec = total_ns // 10 ** 9
        clk.clock.nanosec = total_ns % 10 ** 9
        self.pub_clock.publish(clk)

        od = Odometry()
        od.twist.twist.linear.x = 0.3
        self.pub_odom.publish(od)

        if self.plan['scan'](t):
            ls = LaserScan()
            ls.header.frame_id = 'car/base_link'
            ls.angle_min = -1.0
            ls.angle_increment = 0.02
            ls.range_min = 0.05
            ls.range_max = 100.0
            ranges = [float('inf')] * 101
            ranges[50] = 1.0
            ls.ranges = ranges
            self.pub_scan.publish(ls)

        cm = CollisionMonitorState()
        cm.action_type = int(self.plan['cm'](t))
        cm.polygon_name = 'forward_cone'
        self.pub_cm.publish(cm)

        if t in self.plan['states']:
            state_val, name, reason = self.plan['states'][t]
            st = AgcTaskState()
            st.state = state_val
            st.state_name = name
            st.reason_code = reason
            self.pub_state.publish(st)

        self.tick += 1


def latest_summary(scenario):
    pat = os.path.join(EXP, '*-%s-*/metrics/summary.json' % scenario)
    files = glob.glob(pat)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def run_case(case, out_dir, log_dir):
    plan = CASES[case]
    scenario = case
    log_path = os.path.join(log_dir, 'harness-%s.log' % case)
    with open(log_path, 'w') as logf:
        proc = subprocess.Popen(
            ['ros2', 'run', 'agc_task', 'run_logger', '--ros-args',
             '-p', 'campaign:=w6-task-eval', '-p', 'scenario:=%s' % scenario,
             '-p', 'seed:=0', '-p', 'use_sim_time:=true', '-p', 'contact_topic:=""',
             '-p', 'scan_max_age_s:=0.5', '-p', 'action_max_age_s:=0.5'],
            stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(1.5)  # 等 logger 订阅就绪 / 发现

    node = Harness(plan, scenario)
    # 发现 / 匹配稳定：先空转 2.0s（不发布），避免首发消息在 DDS 匹配前被丢弃。
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.start()
    while not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    time.sleep(0.3)  # 让最后几条消息送达

    # 对**进程组**发 SIGINT（ros2 run 会派生/包一层，单发 pid 可能到不了节点；
    # 组信号确保 run_logger 收到信号 → finalize 落 summary）。
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        pgid = None
    if pgid is not None:
        os.killpg(pgid, signal.SIGINT)
    try:
        proc.wait(timeout=6)
    except subprocess.TimeoutExpired:
        if pgid is not None:
            os.killpg(pgid, signal.SIGKILL)
        proc.wait(timeout=3)
    time.sleep(2)  # 等 finalize 落盘

    src = latest_summary(scenario)
    dst = os.path.join(out_dir, 'harness-%s.json' % case)
    if not src:
        print('[%s] ERROR 未找到 summary（%s）' % (case, os.path.join(EXP, '*-%s-*' % scenario)))
        return None
    with open(src, encoding='utf-8') as f:
        sm = json.load(f)  # 顺带自检 JSON 合法性
    with open(dst, 'w', encoding='utf-8') as f:
        json.dump(sm, f, indent=2, ensure_ascii=False)
    ewm = sm.get('episode_window_metrics', {})
    print('[%s] <- %s' % (case, src))
    print('    window_count=%s applicable=%s na=%s terminal_state=%s latched=%s src=%s'
          % (ewm.get('window_count'), sm.get('evaluation_applicable'),
             sm.get('evaluation_na_reasons'), sm.get('terminal_state'),
             sm.get('terminal_state_latched'), sm.get('terminal_state_source')))
    print('    proc ticks_total=%s | window ticks_total=%s valid=%s invalid_scan=%s '
          'triggered=%s infeasible=%s scan_samples=%s rate=%s'
          % (sm.get('ticks_total'), ewm.get('ticks_total'), ewm.get('ticks_valid'),
             ewm.get('ticks_invalid_scan'), ewm.get('triggered_ticks'),
             ewm.get('infeasible_ticks'), ewm.get('scan_samples'),
             ewm.get('shield_infeasible_rate')))
    return dst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', required=True, help='<RUN_ID>/summaries（绝对路径）')
    ap.add_argument('--cases', default=','.join(CASES.keys()))
    args = ap.parse_args()

    out_dir = os.path.abspath(args.out_dir)
    log_dir = os.path.abspath(os.path.join(out_dir, '..', 'log'))
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    cases = [c.strip() for c in args.cases.split(',') if c.strip()]
    for c in cases:
        if c not in CASES:
            print('unknown case: %s' % c)
            return 2

    rclpy.init()
    rc = 0
    try:
        for c in cases:
            if run_case(c, out_dir, log_dir) is None:
                rc = 1
    finally:
        rclpy.shutdown()
    print('harness done rc=%d' % rc)
    return rc


if __name__ == '__main__':
    sys.exit(main())
