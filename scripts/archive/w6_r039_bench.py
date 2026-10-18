#!/usr/bin/env python3
"""w6_r039_bench —— R-039（地面 odom 源时刻分离 + 复位 epoch 速度历史）**真实节点台架**。

起**真实**二进制 `install/agc_perception/lib/agc_perception/ground_odom_adapter`，
用 Python 发布**合成真值**（`geometry_msgs/PoseStamped` → `/agc/gt/agc_car`）与 **reset epoch**
（`std_msgs/Int32` → `/agc/scenario/reset_epoch`，transient_local），**同一时间轴**采集：
真值位姿/速度、odom 的 `header.stamp` 与位姿/twist、`/agc/ground/odom_valid`、诊断 `/agc/ground/odom_diag`。

四场景（逐条验收 R-039 的需求）：
  1. `disconnect`  断源：停止发布 GT → 期望 `valid=false`，年龄如实增大（不把缓存当真值新测量）。
  2. `stale_sample` 旧样本：冻结 stamp 反复发布 → 期望 odom stamp **保持**源时刻（**不**被 now() 重盖）、
                    年龄增大、`valid` 按其时效阈值（`odom_max_age_s`）翻转。
  3. `teleport`    先正常运行建立速度，再发布新 epoch 并以跳变位置发布 GT → 期望**无速度尖峰**
                    （重建 ≥2 新样本前速度 0，首对差分不跨 teleport）。
  4. `time_jump`   发布 future stamp（age<0）→ 期望 `clock_anomaly` → `valid=false`（**不**钳 0）。

只回收**本进程组**子进程（`start_new_session` + `killpg`）；**不**用裸名 `pkill -f`、**不**做递归删除。
证据落 `experiments/w6-task-eval/<run_id>/r039/bench.json`（ADR-0007）。

⚠ 范围标注：本台架为**节点行为台架**（真实 odom 发布二进制 + 合成真值流，**非**整链 / **非**双 PX4 / **非** GUI）。
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
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, String

ROOT = '/home/chen/AGC'
PERC_BIN = os.path.join(ROOT, 'install', 'agc_perception', 'lib', 'agc_perception')
GT_TOPIC = '/agc/gt/agc_car'
EPOCH_TOPIC = '/agc/scenario/reset_epoch'
ODOM_TOPIC = '/agc/ground/odom'
VALID_TOPIC = '/agc/ground/odom_valid'
DIAG_TOPIC = '/agc/ground/odom_diag'


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


def _reap_stale():
    for pat in ('agc_perception/lib/agc_perception/ground_odom_adapte[r]',):
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


class Harness:
    def __init__(self, name):
        self.node = Node('r039_bench_' + name)
        self.gt_pub = self.node.create_publisher(PoseStamped, GT_TOPIC, 10)
        self.epoch_pub = self.node.create_publisher(
            Int32, EPOCH_TOPIC, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom = []
        self.valid = []
        self.diag = []
        self.gt = []          # 命令真值（含 stamp / 位置 / 速度）
        self.node.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        self.node.create_subscription(Bool, VALID_TOPIC, self._on_valid, 10)
        self.node.create_subscription(
            String, DIAG_TOPIC, self._on_diag,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on_odom(self, m):
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.odom.append({
            'wall': time.time(), 'stamp': s,
            'x': m.pose.pose.position.x, 'y': m.pose.pose.position.y,
            'speed': math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y),
            'wz': m.twist.twist.angular.z,
            'frame': m.header.frame_id, 'child': m.child_frame_id,
        })

    def _on_valid(self, m):
        self.valid.append({'wall': time.time(), 'data': bool(m.data)})

    def _on_diag(self, m):
        self.diag.append({'wall': time.time(), 'data': m.data})

    def pub_epoch(self, e):
        msg = Int32()
        msg.data = e
        self.epoch_pub.publish(msg)

    def pub_gt(self, stamp, x, y, vx=0.0, vy=0.0, z=0.0):
        m = PoseStamped()
        m.header.frame_id = 'world'
        m.header.stamp.sec = int(stamp)
        m.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        self.gt_pub.publish(m)
        self.gt.append({'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                        'x': float(x), 'y': float(y), 'vx': float(vx), 'vy': float(vy)})

    def pump(self, duration, cb, hz=30.0):
        """驱动循环：按 hz 调 cb(elapsed, wall_now)（cb 内自行发布），同时 spin 收消息。"""
        t0 = time.time()
        last = 0.0
        period = 1.0 / hz
        while True:
            now = time.time()
            el = now - t0
            if el >= duration:
                break
            if now - last >= period:
                last = now
                cb(el, now)
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def drain(self, duration):
        """只 spin，不发布（用于断源 / 冻结观察）。"""
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)


def _dump(harness):
    return {
        'odom': harness.odom,
        'valid': harness.valid,
        'diag': [d['data'] for d in harness.diag],
        'diag_t': [d['wall'] for d in harness.diag],
        'gt': harness.gt,
    }


def _nearest_gt_speed(gt, stamp):
    best, bd = None, 1e9
    for g in gt:
        d = abs(g['stamp'] - stamp)
        if d < bd:
            bd, best = d, g
    if best is None:
        return None, bd
    return math.hypot(best['vx'], best['vy']), bd


def _valid_flips(valid, after=None):
    """返回 `valid` 序列中在 `after`（wall）之后**首次**出现 False 的 wall。"""
    for v in valid:
        if not v['data'] and (after is None or v['wall'] >= after):
            return v['wall']
    return None


def run_scenario(name, params_file, log_dir, script_fn):
    log = os.path.join(log_dir, name + '.log')
    proc = subprocess.Popen(
        [os.path.join(PERC_BIN, 'ground_odom_adapter'),
         '--ros-args', '--params-file', params_file],
        stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    h = Harness(name)
    res = {'scenario': name}
    try:
        # 等节点 / DDS 发现：最多 4 s；已收到任一消息（valid/diag）即视为已连通。
        end = time.time() + 4.0
        while time.time() < end:
            rclpy.spin_once(h.node, timeout_sec=0.05)
            if h.valid or h.diag:
                break
        connected = bool(h.valid or h.diag)
        res = script_fn(h)
        res['scenario'] = name
        res['discovery_connected'] = connected
        # 未与节点连通（DDS 发现失败）不得判 PASS（避免零消息的假阳）。
        res['pass'] = bool(res.get('pass') and connected)
        res['records'] = _dump(h)
    finally:
        _kill(proc)
        h.node.destroy_node()
    return res


def run_scenario_with_retry(name, params_file, log_dir, script_fn, attempts=3):
    """DDS 发现偶发失败：每场景用**全新 ROS context**重试，直到连通或尝试耗尽。"""
    last = None
    for i in range(attempts):
        rclpy.init()
        try:
            last = run_scenario(name + ('' if i == 0 else '_r%d' % i),
                                params_file, log_dir, script_fn)
        finally:
            rclpy.shutdown()
        if last.get('discovery_connected'):
            last['scenario'] = name
            last.setdefault('retries', i)
            return last
        _reap_stale()
        time.sleep(0.5)
    last['scenario'] = name
    return last


# ------------------------------------------------------------------ 四场景

def sc_disconnect(h):
    """断源：正常发布 → 停止 → 期望 valid=false 且年龄增大。"""
    h.pump(0.9, lambda el, now: h.pub_gt(now, x=el * 1.0, y=0.0, vx=1.0))
    t_stop = time.time()
    h.drain(1.3)
    t_end = time.time()
    off = _valid_flips(h.valid, after=t_stop)
    stale_seen = any('state=stale' in d['data'] for d in h.diag)
    # 断源 + 时效阈值越过（跳过停止瞬间的边界样本）后不应再产生 odom
    odom_after = [o for o in h.odom if o['wall'] >= t_stop + 0.2]
    return {
        'pass': bool(off is not None and stale_seen and len(odom_after) == 0),
        'valid_off_wall': off,
        'valid_off_after_stop_s': (off - t_stop) if off else None,
        'diag_has_stale': stale_seen,
        'odom_count_after_stop': len(odom_after),
        't_stop': t_stop, 't_end': t_end,
        'note': '断源：无新样本 → 不发布 odom；valid 由时效阈值翻转；诊断 state=stale。',
    }


def sc_stale_sample(h):
    """旧样本：冻结 stamp / 位姿反复发布 → odom stamp 保持源时刻，年龄增大，valid 翻转。"""
    h.pump(0.9, lambda el, now: h.pub_gt(now, x=el * 1.0, y=0.0, vx=1.0))
    frozen = h.gt[-1]['stamp']
    fx, fy = h.gt[-1]['x'], h.gt[-1]['y']
    t_freeze = time.time()
    h.pump(1.2, lambda el, now: h.pub_gt(frozen, x=fx, y=fy, vx=0.0, vy=0.0))
    t_end = time.time()
    # 冻结窗（跳过起始边界样本）：stamp 必须恒等于源采样时刻
    odom_frozen = [o for o in h.odom if o['wall'] >= t_freeze + 0.1]
    stamps_kept = all(abs(o['stamp'] - frozen) < 1e-9 for o in odom_frozen) if odom_frozen else True
    # 反例：不得有 odom 的 stamp 逼近 wall-now（旧实现 now() 重盖）
    now_covered = [o for o in odom_frozen if abs(o['wall'] - o['stamp']) < 0.05]
    off = _valid_flips(h.valid, after=t_freeze)
    stale_seen = any('state=stale' in d['data'] for d in h.diag)
    return {
        'pass': bool(stamps_kept and (len(now_covered) == 0) and (off is not None) and stale_seen),
        'frozen_stamp': frozen,
        'odom_frozen_count': len(odom_frozen),
        'stamps_kept_source_time': stamps_kept,
        'odom_stamp_near_now_count': len(now_covered),
        'valid_off_wall': off,
        'diag_has_stale': stale_seen,
        't_freeze': t_freeze, 't_end': t_end,
        'note': '旧样本：发布 stamp 恒等于源采样时刻（非 now()）；年龄增大、valid 翻转。',
    }


def sc_teleport(h):
    """teleport：先建立速度 → 新 epoch + 跳变位置 → 期望无速度尖峰。"""
    h.pump(1.0, lambda el, now: h.pub_gt(now, x=el * 1.0, y=0.0, vx=1.0))
    v_before = [o['speed'] for o in h.odom]
    h.pub_epoch(1)
    t_tp = time.time()
    h.pump(0.8, lambda el, now: h.pub_gt(now, x=100.0 + el * 1.0, y=0.0, vx=1.0))
    t_end = time.time()
    odom_after = [o for o in h.odom if o['wall'] >= t_tp]
    speeds_after = [o['speed'] for o in odom_after]
    max_speed_after = max(speeds_after) if speeds_after else 0.0
    # 逐拍与同时间轴真值比对（仅比较**已恢复速度**的拍；rebuild 期速度为 0 单独断言）
    paired = []
    for o in odom_after:
        vg, d = _nearest_gt_speed(h.gt, o['stamp'])
        if vg is not None and d < 1e-6 and o['speed'] > 0.1:
            paired.append({'stamp': o['stamp'], 'v_odom': o['speed'], 'v_gt': vg,
                           'abs_err': abs(o['speed'] - vg)})
    max_err = max((p['abs_err'] for p in paired), default=None)
    rebuild0_first = all(abs(o['speed']) < 1e-6 for o in odom_after[:1]) if odom_after else False
    # 尖峰阈值：真值 1.0 m/s，容许 ≤ 2.0（旧实现跨 teleport 差分约 ~600 m/s）
    spike_free = max_speed_after <= 2.0
    return {
        'pass': bool(spike_free and max_err is not None and max_err <= 0.35 and rebuild0_first),
        'v_before_max': (max(v_before) if v_before else 0.0),
        'max_speed_after_teleport': max_speed_after,
        'spike_threshold': 2.0,
        'spike_free': spike_free,
        'first_after_speed_zero': rebuild0_first,
        'max_abs_err_odom_vs_gt': max_err,
        'paired_count': len(paired),
        'paired': paired[:80],
        't_teleport': t_tp, 't_end': t_end,
        'note': 'teleport：epoch 清历史 + 跳变守卫 → 重建前速度 0；无尖峰；与真值速度同轴误差小。',
    }


def sc_time_jump(h):
    """time jump：future stamp（age<0）→ clock_anomaly + valid=false（不钳 0）。"""
    h.pump(0.6, lambda el, now: h.pub_gt(now, x=el * 1.0, y=0.0, vx=1.0))
    t_jump = time.time()
    h.pump(0.9, lambda el, now: h.pub_gt(now + 8.0, x=el * 1.0, y=0.0, vx=1.0))
    t_end = time.time()
    anomaly = any('state=clock_anomaly' in d['data'] for d in h.diag)
    neg_ages = [float(d['data'].split('age=')[1].split()[0]) for d in h.diag
                if 'state=clock_anomaly' in d['data'] and 'age=' in d['data']]
    min_age = min(neg_ages) if neg_ages else None
    off = _valid_flips(h.valid, after=t_jump)
    odom_in_jump = [o for o in h.odom if o['wall'] >= t_jump]
    return {
        'pass': bool(anomaly and (min_age is not None and min_age < -1.0) and
                     (off is not None) and (len(odom_in_jump) == 0)),
        'diag_has_clock_anomaly': anomaly,
        'min_reported_age': min_age,
        'valid_off_wall': off,
        'odom_count_in_jump_window': len(odom_in_jump),
        't_jump': t_jump, 't_end': t_end,
        'note': 'future stamp：age<0 → clock_anomaly → valid=false，且不发布 odom（**不**钳 0）。',
    }


def _write(path, text):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261018-w6-r039-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()

    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)
    cfg_dir = os.path.join(out_dir, 'config')
    log_dir = os.path.join(out_dir, 'r039', 'log')
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    params_file = os.path.join(cfg_dir, 'r039_ground_odom_adapter.yaml')
    _write(params_file,
           'ground_odom_adapter:\n  ros__parameters:\n'
           f'    gt_topic: "{GT_TOPIC}"\n'
           f'    odom_topic: "{ODOM_TOPIC}"\n'
           f'    valid_topic: "{VALID_TOPIC}"\n'
           f'    diag_topic: "{DIAG_TOPIC}"\n'
           f'    reset_epoch_topic: "{EPOCH_TOPIC}"\n'
           '    world_frame: "world"\n'
           '    base_frame: "car/base_link"\n'
           '    rate_hz: 30.0\n'
           '    diff_window_s: 0.15\n'
           '    odom_max_age_s: 0.5\n'
           '    reset_jump_clear_m: 0.5\n')

    rclpy.init()
    _reap_stale()
    rclpy.shutdown()

    scenarios = {
        'disconnect': sc_disconnect,
        'stale_sample': sc_stale_sample,
        'teleport': sc_teleport,
        'time_jump': sc_time_jump,
    }
    result = {
        'run_id': args.run_id,
        'suite': 'R-039 ground odom source-time + reset-epoch velocity bench',
        'scope_note': ('节点行为台架：真实 ground_odom_adapter 二进制 + 合成真值 / epoch 流，'
                       '验证源时刻=发布时刻 / 负年龄无效 / teleport 无尖峰；'
                       '**非整链** / 非双 PX4 / 非 GUI。'),
        'unsupported': [],
        'scenarios': {},
    }
    for name, fn in scenarios.items():
        result['scenarios'][name] = run_scenario_with_retry(name, params_file, log_dir, fn)
        print('%-14s PASS=%s' % (name, result['scenarios'][name].get('pass')))
        _reap_stale()

    result['overall_pass'] = bool(
        result['scenarios'] and all(s.get('pass') for s in result['scenarios'].values()))
    with open(os.path.join(out_dir, 'r039', 'bench.json'), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print('\noverall_pass=%s -> %s' % (
        result['overall_pass'], os.path.join(out_dir, 'r039', 'bench.json')))
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
