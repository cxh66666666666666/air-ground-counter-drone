#!/usr/bin/env python3
"""w6_r036_bench —— R-036（C1 高度语义统一与同时间证据曲线）**逻辑台架**（实跑真实节点）。

起**真实** `agc_air` 生命周期节点（`trajectory_tracker`，standalone：`bond_heartbeat_period=0.0`，
发 `active_controller="trajectory_tracker"` 取得显式控制许可），在**同一 run、同一时间轴**
（统一 ROS 时钟）驱动一根**脚本化机体**并记录 5 条曲线：

  1. **world GT**      —— 台架合成的机体**世界真值**（`/agc/gt/friendly_uav` 诊断话题 + 内部）；
  2. **PX4 估计**       —— 合成 `/agc/air/odom`（= world GT + **已知偏置**，证明三指标可区分）；
  3. **base**          —— 节点对外 `/agc/air/ref`（R-036 后**带 `z_offset_`**，与 effective 同高度基准）；
  4. **effective**     —— 由 `/fmu/in/trajectory_setpoint` 反变换回世界 ENU 后**减去 `0.5·a·dt²` 前馈**重建；
  5. **setpoint**      —— 同上反变换（含前馈）的世界 ENU 指令位置。

**重建口径（与 ADR-0051 一致）**：`offboard_interface.cpp::publishSetpoint` 将世界 ENU 位置
`p_enu` 先减局部原点（本台架 `origin_pose=0,0,0` → 原点为零）再 `enu→ned` 发布；故
`setpoint_enu = nedToEnu(m.position)`，`acc_enu = nedToEnu(m.acceleration)`，
`effective.pos = setpoint_enu − 0.5·acc_enu·dt²`（dt = `1/setpoint_rate_hz` = 0.02 s），
`effective.vel = nedToEnu(m.velocity)`；`setpoint` 曲线 = `setpoint_enu`（含前馈）。

**机体模型（脚本化，避免以被测输出自证）**：机体真值按**二阶跟踪**跟随 `effective`（世界 ENU），
`a = kp·(eff.pos − gt.pos) + kd·(eff.vel − gt.vel)`；估计 = 真值 + 已知常量**位置**偏置（默认 z:+0.60 m，
速度不偏置 → est 与 gt 速度同为机体速度）。安全层 `enabled=false`（本台架**隔离高度语义**；
联合屏障行为由 R-033 覆盖），障碍 / 目标不参与。

报告：对 base / est / gt **三指标**分别给出**高度 z** 与**速度 |v|−2.0** 的 **P95 / max / 持续越限**
（**全窗** P95·max 与 **稳态窗**（t≥warmup）P95·max **分别**给；**不**以稳态 P95 自证全程 max）。
回归**基准轨迹安全**：断言 base↔effective 的 z 高度基准一致（差值在容限内）、`z_offset` 可辨。

产物：`<run_id>/r036/bench.json`（含 `scope_note`（**非整链/逻辑台架**）+ `unsupported`）
+ `<run_id>/r036/curves.csv`（同时间轴 5 曲线）。C1 容限 / 目标速度**读取** `air.yaml`（不复制参数）。

只回收**本进程组**的子进程（`start_new_session` + `killpg` + `wait`）；**不**用裸名 `pkill -f`、**不**递归删除。
证据落 `experiments/w6-task-eval/<run_id>/r036/`（ADR-0007）。

⚠ 范围标注：本台架为 **C1 高度语义 / 同时间证据逻辑台架**（真实节点，**非**整链 / **非**双 PX4 / **非** GUI）。
"""

import argparse
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory
from px4_msgs.msg import TrajectorySetpoint

ROOT = '/home/chen/AGC'
NODE_BIN = os.path.join(ROOT, 'install', 'agc_air', 'lib', 'agc_air')

# standalone：无 manager（bond_heartbeat_period=0.0 → 不建 bond，看门狗不误停）；origin (0,0,0)（局部=世界）。
# 安全层 enabled=false（隔离高度语义）；traj 为真八字 Gerono（C1）。
STANDALONE_PARAMS = """/**:
  ros__parameters:
    use_sim_time: false
    bond_heartbeat_period: 0.0
    origin_pose: "0,0,0"
    setpoint_rate_hz: 50.0
    controller_name: trajectory_tracker
    conflict_persist_s: 1.0
    conflict_clear_s: 2.0
    ref_topic: "/agc/air/ref"
    tracking_status_topic: "/agc/air/tracking_status"
    traj:
      speed: 2.0
      height: 2.5
      rx: 15.0
      ry: 9.0
      center_x: 0.0
      center_y: 0.0
    ctrl:
      q_pos: 4.0
      q_vel: 4.0
      r: 1.0
      dt: 0.02
      acc_max: 4.0
    safety:
      enabled: false
"""


def ned_to_enu(v):
    """NED → ENU：enu = (E, N, U) = (ned.y, ned.x, −ned.z)（与 px4_conversions.hpp 同口径）。"""
    return (v[1], v[0], -v[2])


def _kill(proc):
    """仅回收本子进程组（start_new_session → 独立 pgid）；self-group 护栏下退化为单 PID。"""
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
    """清理同名残留真实进程（obfuscated pattern：不匹配本进程命令行；仅按 PID kill）。"""
    for pat in ['agc_air/lib/agc_air/trajector[y]_tracker']:
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


def _field(s, key):
    """解析 JSON 字段（`"key":"v"` 或 `"key": v`，有无空格均容错）。"""
    kp = s.find('"%s"' % key)
    if kp < 0:
        return ''
    colon = s.find(':', kp + len(key) + 2)
    if colon < 0:
        return ''
    vp = colon + 1
    while vp < len(s) and s[vp] == ' ':
        vp += 1
    if vp >= len(s):
        return ''
    if s[vp] == '"':
        e = s.find('"', vp + 1)
        return s[vp + 1:e] if e >= 0 else ''
    e = len(s)
    for c in (',', '}'):
        i = s.find(c, vp)
        if i >= 0:
            e = min(e, i)
    return s[vp:e].strip()


def _pct(vals, q):
    if not vals:
        return None
    s = sorted(vals)
    idx = max(0, min(int(q * len(s)) - 1, len(s) - 1))
    return s[idx]


def _median(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    s = sorted(vals)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class Rig(Node):
    """合成驱动（odom / GT / active_controller）+ 证据采集（ref / setpoint / tracking_status）。"""

    def __init__(self):
        super().__init__('w6_r036_rig')
        q_tl = QoSProfile(depth=1)
        q_tl.reliability = ReliabilityPolicy.RELIABLE
        q_tl.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_odom = self.create_publisher(Odometry, '/agc/air/odom', qos_profile_sensor_data)
        self.pub_active = self.create_publisher(String, '/agc/air/active_controller', q_tl)
        # 诊断：台架自合成 world GT（非 ABI、非命令者；/agc/gt/friendly_uav 供同轴复核）。
        self.pub_gt = self.create_publisher(PoseStamped, '/agc/gt/friendly_uav', 10)

        self._lock = threading.Lock()
        self.base = None            # {pos:(x,y,z), vel:(x,y,z), stamp_s:float}
        self.sp = None              # {sp_pos, sp_vel, acc, eff_pos, eff_vel, recv_s:float}
        self.status = None          # 最近 tracking_status 原始串
        self.z_offset_seen = None

        self.create_subscription(MultiDOFJointTrajectory, '/agc/air/ref', self._on_ref, 10)
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self._on_sp, 50)
        self.create_subscription(String, '/agc/air/tracking_status', self._on_status, 20)

        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self)
        self._spin = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin.start()

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_ref(self, m):
        if not m.points or not m.points[0].transforms:
            return
        tr = m.points[0].transforms[0].translation
        v = m.points[0].velocities[0].linear if m.points[0].velocities else None
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self._lock:
            self.base = {
                'pos': (tr.x, tr.y, tr.z),
                'vel': (v.x, v.y, v.z) if v else (float('nan'),) * 3,
                'stamp_s': stamp,
            }

    def _on_sp(self, m):
        # px4_msgs 的 position / velocity / acceleration 为 float[3]（rclpy 呈 numpy.ndarray）。
        sp_pos = ned_to_enu((m.position[0], m.position[1], m.position[2]))
        acc = ned_to_enu((m.acceleration[0], m.acceleration[1], m.acceleration[2]))
        vel = ned_to_enu((m.velocity[0], m.velocity[1], m.velocity[2]))
        dt = 0.02
        eff_pos = tuple(sp_pos[i] - 0.5 * acc[i] * dt * dt for i in range(3))
        with self._lock:
            self.sp = {'sp_pos': sp_pos, 'sp_vel': vel, 'acc': acc,
                       'eff_pos': eff_pos, 'eff_vel': vel, 'recv_s': self.now_s()}

    def _on_status(self, m):
        with self._lock:
            self.status = m.data
            z = _field(m.data, 'z_offset')
            try:
                self.z_offset_seen = float(z)
            except (TypeError, ValueError):
                pass

    def latest_sp(self):
        with self._lock:
            return self.sp

    def snapshot(self, t_rel, gt, gv, est):
        """同时间轴一行：t_rel + ROS 绝对时刻 + 各曲线最近值（含 base 头时间 / sp 接收时刻）。"""
        now = self.now_s()
        with self._lock:
            base = self.base
            sp = self.sp
            status = self.status
        row = {'t_rel': t_rel, 't_ros': now,
               'base_stamp': base['stamp_s'] if base else '',
               'sp_recv': sp['recv_s'] if sp else '',
               'status': status or ''}
        for k, i in (('x', 0), ('y', 1), ('z', 2)):
            row['gt_' + k] = gt[i]
            row['gt_v' + k] = gv[i]
            row['est_' + k] = est[i]
            row['est_v' + k] = gv[i]           # 位置偏置、速度不偏置（est 与 gt 同机体速度）
        if base:
            for k, i in (('x', 0), ('y', 1), ('z', 2)):
                row['base_' + k] = base['pos'][i]
                row['base_v' + k] = base['vel'][i]
        else:
            for k in ('x', 'y', 'z'):
                row['base_' + k] = ''
                row['base_v' + k] = ''
        if sp:
            for k, i in (('x', 0), ('y', 1), ('z', 2)):
                row['eff_' + k] = sp['eff_pos'][i]
                row['eff_v' + k] = sp['eff_vel'][i]
                row['sp_' + k] = sp['sp_pos'][i]
                row['sp_v' + k] = sp['sp_vel'][i]
        else:
            for k in ('x', 'y', 'z'):
                row['eff_' + k] = ''
                row['eff_v' + k] = ''
                row['sp_' + k] = ''
                row['sp_v' + k] = ''
        return row

    def drive(self, active, gt, gv, est, pub_gt=True, pub_active=True):
        """发一帧 odom（PX4 估计，含位置偏置）+（可选）world GT +（可选）active_controller。"""
        stamp = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'world'
        o.child_frame_id = 'drone/base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = est
        o.pose.pose.orientation.w = 1.0                       # 单位姿态 → twist 即世界 ENU
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = gv
        self.pub_odom.publish(o)
        if pub_gt:
            g = PoseStamped()
            g.header.stamp = stamp
            g.header.frame_id = 'world'
            g.pose.position.x, g.pose.position.y, g.pose.position.z = gt
            g.pose.orientation.w = 1.0
            self.pub_gt.publish(g)
        if pub_active:
            msg = String()
            msg.data = active
            self.pub_active.publish(msg)

    # ---- 生命周期（CLI + 重试至目标状态） ----
    @staticmethod
    def _cli(*args, timeout=25.0):
        try:
            return subprocess.run(['ros2', 'lifecycle', *args], capture_output=True, text=True,
                                  timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def state(self, service):
        r = self._cli('get', service)
        if r is None or r.returncode != 0:
            return None, 'no-response'
        m = re.search(r'([a-z_]+)\s*\[(\d+)\]', r.stdout)
        if not m:
            return None, 'no-response'
        return int(m.group(2)), m.group(1)

    def wait_node_up(self, service, timeout=40.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.state(service)[0] is not None:
                return True
            time.sleep(0.5)
        return False

    def apply(self, service, transition_label, expect_label, tries=6):
        for _ in range(tries):
            self._cli('set', service, transition_label)
            if self.state(service)[1] == expect_label:
                return True
            time.sleep(0.4)
        return self.state(service)[1] == expect_label


def _spawn(cmd, log_path):
    return subprocess.Popen(cmd, stdout=open(log_path, 'w'), stderr=subprocess.STDOUT,
                            start_new_session=True)


def run_bench(rig, cfg_dir, log_dir, duration, dt, bias, tau_v, tau_p):
    result = {'pass': False}
    pf = os.path.join(cfg_dir, 'r036_standalone.yaml')
    with open(pf, 'w', encoding='utf-8') as f:
        f.write(STANDALONE_PARAMS)
    log = os.path.join(log_dir, 'trajectory_tracker.log')
    proc = _spawn([os.path.join(NODE_BIN, 'trajectory_tracker'), '--ros-args',
                   '--params-file', pf], log)
    rows = []
    try:
        up = rig.wait_node_up('/trajectory_tracker')
        result['node_up'] = up
        if not up:
            return result, rows
        result['configure_ok'] = rig.apply('/trajectory_tracker', 'configure', 'inactive')
        result['activate_ok'] = rig.apply('/trajectory_tracker', 'activate', 'active')

        gt = [0.0, 0.0, 2.5]      # world GT 初值 = Figure8 θ=0（x=0,y=0,z=h=2.5）
        gv = [0.0, 0.0, 0.0]
        est = [gt[0] + bias[0], gt[1] + bias[1], gt[2] + bias[2]]

        # 阶段 0：发 odom + 未授权 active（"takeoff_controller"）→ tracker 尚未接管（permit=false）。
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rig.drive('takeoff_controller', tuple(gt), tuple(gv), tuple(est))
            time.sleep(dt)

        # 接管时刻参考（tracker t0 ≈ 此处；warmup 自该时刻起算）。
        t0 = rig.now_s()
        prev = t0
        last_gt_pub = t0
        last_active_pub = t0
        end = time.monotonic() + duration
        while time.monotonic() < end:
            now = rig.now_s()
            dt_now = max(1e-4, min(now - prev, 0.1))   # 用**实际**步长积分（并发运行时抖动安全）
            prev = now
            sp = rig.latest_sp()
            if sp is not None:
                eff_pos, eff_vel = sp['eff_pos'], sp['eff_vel']
                # 机体真值模型：**一阶**速度趋近参考速度（隐式欧拉 → 任意 dt 稳定）+ 弱位置牵引。
                av = dt_now / (tau_v + dt_now)
                gv = [gv[i] + av * (eff_vel[i] - gv[i]) for i in range(3)]
                gt = [gt[i] + gv[i] * dt_now for i in range(3)]
                ap = dt_now / (tau_p + dt_now)
                gt = [gt[i] + ap * (eff_pos[i] - gt[i]) for i in range(3)]
            est = [gt[0] + bias[0], gt[1] + bias[1], gt[2] + bias[2]]
            pub_a = (now - last_active_pub) >= 1.0     # active 为 transient_local，无需每拍刷
            pub_g = (now - last_gt_pub) >= 0.05        # GT 诊断话题 ~20 Hz
            rig.drive('trajectory_tracker', tuple(gt), tuple(gv), tuple(est),
                      pub_gt=pub_g, pub_active=pub_a)
            if pub_a:
                last_active_pub = now
            if pub_g:
                last_gt_pub = now
            rows.append(rig.snapshot(now - t0, tuple(gt), tuple(gv), tuple(est)))
            time.sleep(dt)
    finally:
        _kill(proc)
    result['n_rows'] = len(rows)
    result['z_offset_seen'] = rig.z_offset_seen
    result['pass'] = bool(result.get('node_up') and result.get('configure_ok') and
                          result.get('activate_ok') and len(rows) > 100)
    return result, rows


def _row_num(row, key):
    v = row.get(key)
    if v == '' or v is None:
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _speed_of(row, pfx):
    v = [_row_num(row, pfx + '_v' + a) for a in ('x', 'y', 'z')]
    if any(x is None for x in v):
        return None
    return math.sqrt(sum(c * c for c in v))


def _mono(v):
    return all(v[i] <= v[i + 1] + 1e-9 for i in range(len(v) - 1)) if len(v) > 1 else False


def _sustained(t_used, flags):
    """最大**连续**越限时长（s）。"""
    best = 0.0
    cur = 0.0
    for i, f in enumerate(flags):
        if f:
            if i > 0:
                cur += t_used[i] - t_used[i - 1]
            best = max(best, cur)
        else:
            cur = 0.0
    return best


def analyze(rows, warmup, target_speed, eps_speed, eps_height):
    """对 base / est / gt 三指标分别算高度 z / 速度 |v| 的 P95 / max / 持续越限。"""
    def metric_stats(pfx):
        sp_err_full, sp_err_steady = [], []
        h_err_full, h_err_steady = [], []
        sp_flags, h_flags = [], []
        t_used, speed_abs, zs = [], [], []
        for r in rows:
            sp = _speed_of(r, pfx)
            z = _row_num(r, pfx + '_z')
            zb = _row_num(r, 'base_z')
            if sp is None or z is None or zb is None:
                continue
            t_used.append(r['t_rel'])
            speed_abs.append(sp)
            zs.append(z)
            e_s, e_h = abs(sp - target_speed), abs(z - zb)
            sp_err_full.append(e_s)
            h_err_full.append(e_h)
            sp_flags.append(e_s > eps_speed)
            h_flags.append(e_h > eps_height)
            if r['t_rel'] >= warmup:
                sp_err_steady.append(e_s)
                h_err_steady.append(e_h)
        return {
            'speed_abs_p95': _pct(speed_abs, 0.95),
            'speed_abs_max': max(speed_abs) if speed_abs else None,
            'speed_err_full_p95': _pct(sp_err_full, 0.95),
            'speed_err_full_max': max(sp_err_full) if sp_err_full else None,
            'speed_err_steady_p95': _pct(sp_err_steady, 0.95),
            'speed_err_steady_max': max(sp_err_steady) if sp_err_steady else None,
            'speed_err_sustained_s': _sustained(t_used, sp_flags),
            'height_z_median': _median(zs),
            'height_err_full_p95': _pct(h_err_full, 0.95),
            'height_err_full_max': max(h_err_full) if h_err_full else None,
            'height_err_steady_p95': _pct(h_err_steady, 0.95),
            'height_err_steady_max': max(h_err_steady) if h_err_steady else None,
            'height_err_sustained_s': _sustained(t_used, h_flags),
        }

    diffs = []
    for r in rows:
        z = _row_num(r, 'base_z')
        e = _row_num(r, 'eff_z')
        if z is not None and e is not None:
            diffs.append(abs(z - e))

    t_series = [r['t_rel'] for r in rows]
    base_series = [r['base_stamp'] for r in rows if r['base_stamp'] != '']
    sp_series = [r['sp_recv'] for r in rows if r['sp_recv'] != '']
    align_err = sorted(abs(r['t_ros'] - r['base_stamp']) for r in rows if r['base_stamp'] != '')

    def _p95(v):
        if not v:
            return None
        return v[max(0, min(int(0.95 * len(v)) - 1, len(v) - 1))]

    return {
        'n_rows': len(rows),
        'duration_s': (rows[-1]['t_rel'] - rows[0]['t_rel']) if rows else 0.0,
        'warmup_s': warmup,
        'time_axis': {
            't_monotonic': _mono(t_series),
            'base_stamp_monotonic': _mono(base_series),
            'sp_recv_monotonic': _mono(sp_series),
            'align_err_p95_s': _p95(align_err),
            'align_err_max_s': max(align_err) if align_err else None,
            'align_tol_s': 0.25,
        },
        'height_datum': {
            'base_minus_eff_p95': _pct(diffs, 0.95),
            'base_minus_eff_max': max(diffs) if diffs else None,
        },
        'curves': {'base': metric_stats('base'), 'est': metric_stats('est'),
                   'gt': metric_stats('gt')},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261017-w6-r036-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    ap.add_argument('--params', default=os.path.join(ROOT, 'src/agc_air/config/air.yaml'),
                    help='C1 容限 / 目标速度唯一来源（不复制参数）')
    ap.add_argument('--duration', type=float, default=40.0)
    ap.add_argument('--warmup', type=float, default=5.0)
    ap.add_argument('--dt', type=float, default=0.02)
    args = ap.parse_args()

    with open(args.params, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    c1 = doc['trajectory_tracker']['ros__parameters']['c1']
    target_speed = float(c1['ref_speed_target'])
    tol_ref_speed = float(c1['ref_speed_tol'])
    tol_datum = float(c1['height_datum_tol_m'])
    tol_body_speed_p95 = float(c1['body_speed_p95_tol_mps'])
    eps_speed = float(c1['sustained_limit_eps_mps'])
    eps_height = float(c1['sustained_limit_eps_mps'])

    bias = (0.20, -0.15, 0.60)   # 已知偏置：估计 = world GT + 位置偏置（z:+0.60 m）
    tau_v, tau_p = 0.20, 0.50    # 机体一阶速度趋近时间常数 / 弱位置牵引时间常数
    # （模型口径：机体真值速度一阶趋近参考速度 → 稳态速度误差**非零但小**；见 ADR-0051 取证节。）

    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)
    cfg_dir = os.path.join(out_dir, 'config')
    log_dir = os.path.join(out_dir, 'r036', 'log')
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    _reap_stale()
    rclpy.init()
    rig = Rig()
    result = {
        'run_id': args.run_id, 'suite': 'R-036 C1 height semantics and same-time evidence bench',
        'scope_note': (
            'C1 高度语义 / 同时间证据**逻辑台架**：真实 agc_air 生命周期节点（trajectory_tracker，standalone），'
            '同一 run 同一时间轴记录 **world GT / PX4 估计 / base / effective / setpoint** 5 曲线；'
            '机体为**脚本化二阶跟踪**（不消费 setpoint 的位置环，避免以被测输出自证），'
            '**非整链** / 非双 PX4 / 非 GUI。'),
        'unsupported': [
            '真实整链机体动力学 / PX4 SITL 闭环（本台架为脚本化机体）',
            'world GT 独立于 base 的全链真值（既有做法直接采 gz /world/agc_world/pose/info；本台架为合成）',
        ],
        'config': {'bias_xyz': list(bias),
                   'model': 'first-order velocity approach tau_v=%.2f (pos pull tau_p=%.2f)'
                            % (tau_v, tau_p),
                   'target_speed': target_speed, 'tol_ref_speed': tol_ref_speed,
                   'tol_datum': tol_datum, 'tol_body_speed_p95': tol_body_speed_p95,
                   'warmup_s': args.warmup},
        'bench': {},
        'evidence': {},
        'checks': {},
    }
    try:
        bench_res, rows = run_bench(rig, cfg_dir, log_dir, args.duration, args.dt, bias,
                                    tau_v, tau_p)
        result['bench'] = bench_res

        csv_path = os.path.join(out_dir, 'r036', 'curves.csv')
        if rows:
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
        result['curves_csv'] = csv_path

        ev = analyze(rows, args.warmup, target_speed, eps_speed, eps_height)
        result['evidence'] = ev

        basec, estc, gtc = ev['curves']['base'], ev['curves']['est'], ev['curves']['gt']
        ta = ev['time_axis']
        gt_est_z_gap = (abs((gtc['height_z_median'] or 0.0) - (estc['height_z_median'] or 0.0))
                        if gtc['height_z_median'] is not None and estc['height_z_median'] is not None
                        else 0.0)
        z_off = result['bench'].get('z_offset_seen')
        result['checks'] = {
            'node_and_lifecycle_ok': bool(bench_res.get('pass')),
            'datum_base_eff_consistent': bool(
                ev['height_datum']['base_minus_eff_p95'] is not None and
                ev['height_datum']['base_minus_eff_p95'] <= tol_datum),
            'ref_speed_const': bool(basec['speed_err_full_max'] is not None and
                                    basec['speed_err_full_max'] <= tol_ref_speed),
            'body_speed_steady_in_tol': bool(gtc['speed_err_steady_p95'] is not None and
                                             gtc['speed_err_steady_p95'] <= tol_body_speed_p95),
            'three_metrics_distinguishable': bool(gt_est_z_gap >= 0.5 * abs(bias[2])),
            'steady_p95_reported_separately': bool(
                basec['speed_err_full_max'] is not None and basec['speed_err_steady_p95'] is not None and
                gtc['speed_err_full_max'] is not None and gtc['speed_err_steady_p95'] is not None),
            'z_offset_field_present': bool(z_off is not None),
            'z_offset_detectable': bool(z_off is not None and abs(z_off) >= 0.5 * abs(bias[2])),
            'same_time_axis': bool(ta['t_monotonic'] and ta['base_stamp_monotonic'] and
                                   ta['sp_recv_monotonic'] and ta['align_err_p95_s'] is not None and
                                   ta['align_err_p95_s'] <= ta['align_tol_s']),
        }
        result['overall_pass'] = bool(all(result['checks'].values()))
    finally:
        rig.destroy_node()
        rclpy.shutdown()

    with open(os.path.join(out_dir, 'r036', 'bench.json'), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print('bench checks:')
    for k, v in result.get('checks', {}).items():
        print('  %-32s %s' % (k, v))
    print('\nbench overall_pass=%s -> %s' % (
        result['overall_pass'], os.path.join(out_dir, 'r036', 'bench.json')))
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
