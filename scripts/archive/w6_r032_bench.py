#!/usr/bin/env python3
"""w6_r032_bench —— R-032（C10 冻结下游封闭）**逻辑台架**（Phase A，实跑 trajectory_tracker）。

对每个场景：本进程起一个**合成驱动**（发布 `/agc/air/odom`、`/agc/air/active_controller`、
`/agc/air/tracking_ref`、`/agc/targets`、`/agc/air/rl_offset`），并 `ros2 run agc_air
trajectory_tracker` 起**真实** tracker（configure/activate），订阅
`/fmu/in/trajectory_setpoint`（重建 tracker 实际下发的**有效参考**）与
`/agc/air/tracking_status`（R-032 配套诊断）。

只回收**本进程组的 tracker 子进程**（terminate → wait → kill）；**不** 用裸名模式批杀、**不**做递归删除。
证据落 `experiments/w6-task-eval/<run_id>/r032/bench.json`（ADR-0007）。

场景（与评审 R-032 / spec 场景一一对应）：
  a_follower_death  —— 已获目标后停发 tracking_ref → 显式 hold（保持位置、**不**回落八字）
  b_old_ref_replay  —— 收到源时间为过去的 tracking_ref → 按**源时间**拒（保持基础八字，未被旧包推移）
  c_low_confidence  —— 目标消息持续但 confidence < 门槛 → 冻结（**不**跟随持续变化的 tracking_ref）
  d_offset_lifetime —— rl_offset 超幅 → 限幅；源停止（超寿命）→ 归零
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
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Vector3Stamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist

from agc_msgs.msg import TargetTrackArray, TargetTrack
from px4_msgs.msg import TrajectorySetpoint

ROOT = '/home/chen/AGC'

# R-032 逻辑台架参数：隔离安全层（safety.enabled=false），聚焦 tracker 的**参考裁定**逻辑。
# origin_pose=0,0,0 使 setpoint 的局部 ENU ≡ 世界 ENU（便于从 /fmu/in/trajectory_setpoint 重建有效参考）。
BENCH_PARAMS = """trajectory_tracker:
  ros__parameters:
    trajectory_plugin: "agc_control/Figure8Trajectory"
    controller_plugin: "agc_control/LqrController"
    trajectory_param_ns: "traj"
    controller_param_ns: "ctrl"
    controller_name: "trajectory_tracker"
    setpoint_rate_hz: 50.0
    max_yaw_rate: 2.0
    tracking_ref_timeout_s: 0.5
    target_max_age_s: 0.5
    confidence_min: 0.5
    rl_offset_timeout_s: 0.5
    rl_offset_max_m: 2.0
    conflict_persist_s: 1.0
    use_sim_time: false
    origin_pose: "0,0,0"
    odom_topic: "/agc/air/odom"
    status_topic: "/agc/air/status"
    ref_topic: "/agc/air/ref"
    tracking_ref_topic: "/agc/air/tracking_ref"
    target_topic: "/agc/targets"
    active_controller_topic: "/agc/air/active_controller"
    conflict_topic: "/agc/air/control_conflict"
    rl_offset_topic: "/agc/air/rl_offset"
    tracking_status_topic: "/agc/air/tracking_status"
    traj: {speed: 2.0, height: 2.5, rx: 15.0, ry: 9.0, center_x: 0.0, center_y: 0.0}
    ctrl: {q_pos: 4.0, q_vel: 4.0, r: 1.0, dt: 0.02, acc_max: 4.0}
    safety: {enabled: false}
"""

HOLD_XY = (5.0, -3.0)      # 台架约定的跟踪参考位置（XY）
TARGET_XY = (-10.0, 5.0)   # 目标位置（与参考水平距离 > 安全间距，避免与安全层耦合）



def _to_stamp(sec_float):
    """float 秒 → builtin_interfaces 时间戳（整数纳秒，避免 rclpy.time 对 float 秒的严格类型）。"""
    return rclpy.time.Time(nanoseconds=int(sec_float * 1e9)).to_msg()


class Rig(Node):
    """合成驱动 + 证据采集。"""

    def __init__(self):
        super().__init__('w6_r032_rig')
        self.latest_status = None
        self.statuses = []
        self.eff = []            # (t, ex, ey, ez) 有效参考（由 setpoint 重建）

        q_tl = QoSProfile(depth=1)
        q_tl.reliability = ReliabilityPolicy.RELIABLE
        q_tl.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_odom = self.create_publisher(Odometry, '/agc/air/odom', qos_profile_sensor_data)
        self.pub_active = self.create_publisher(
            String, '/agc/air/active_controller', q_tl)
        self.pub_ref = self.create_publisher(
            MultiDOFJointTrajectory, '/agc/air/tracking_ref', 10)
        self.pub_target = self.create_publisher(TargetTrackArray, '/agc/targets', 10)
        self.pub_offset = self.create_publisher(Vector3Stamped, '/agc/air/rl_offset', 10)

        self.create_subscription(String, '/agc/air/tracking_status', self._on_status, 10)
        self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self._on_setpoint, 10)

    # ---- 回调 ----
    def _on_status(self, m):
        self.latest_status = self._parse(m.data)
        self.statuses.append(self.latest_status)

    def _on_setpoint(self, m):
        # enuToNed: ned=(enu.y, enu.x, -enu.z) → enu=(ned.y, ned.x, -ned.z)；pos_local_ff≈pos（ff 极小）
        ex, ey, ez = m.position[1], m.position[0], -m.position[2]
        self.eff.append((time.time(), ex, ey, ez))

    @staticmethod
    def _parse(s):
        try:
            return json.loads(s)
        except Exception:  # noqa: BLE001
            return {'source': None}

    # ---- 发布 ----
    def pub_odom_now(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'world'
        o.child_frame_id = 'drone/base_link'
        o.pose.pose.position.z = 2.5
        o.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(o)

    def pub_active_now(self):
        m = String()
        m.data = 'trajectory_tracker'
        self.pub_active.publish(m)

    def pub_ref_at(self, x, y, z, stamp_s, vel=(0.0, 0.0, 0.0)):
        m = MultiDOFJointTrajectory()
        m.header.frame_id = 'world'
        m.header.stamp = _to_stamp(stamp_s)
        p = MultiDOFJointTrajectoryPoint()
        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z = x, y, z
        tf.rotation.w = 1.0
        p.transforms.append(tf)
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = vel
        p.velocities.append(tw)
        p.accelerations.append(Twist())
        m.points.append(p)
        self.pub_ref.publish(m)

    def pub_target_at(self, x, y, z, conf, stamp_s):
        arr = TargetTrackArray()
        arr.header.frame_id = 'world'
        arr.header.stamp = _to_stamp(stamp_s)
        tr = TargetTrack()
        tr.header.frame_id = 'world'
        tr.header.stamp = _to_stamp(stamp_s)  # 目标**源时间**
        tr.target_id = 'intruder_uav'
        tr.pose.pose.position.x, tr.pose.pose.position.y, tr.pose.pose.position.z = x, y, z
        tr.confidence = float(conf)
        tr.source = 'bench'
        arr.tracks.append(tr)
        self.pub_target.publish(arr)

    def pub_offset_at(self, x, y, z, stamp_s):
        m = Vector3Stamped()
        m.header.frame_id = 'world'
        m.header.stamp = _to_stamp(stamp_s)
        m.vector.x, m.vector.y, m.vector.z = x, y, z
        self.pub_offset.publish(m)

    def spin_for(self, dur_s, publish_fn):
        """以 ~20 Hz 运行 publish_fn 并 spin，持续 dur_s；返回本段采集到的有效参考 XY。"""
        t0 = time.time()
        n0 = len(self.eff)
        while time.time() - t0 < dur_s:
            publish_fn()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        return self.eff[n0:]


def _spawn_tracker(params_file, log_path):
    proc = subprocess.Popen(
        ['ros2', 'run', 'agc_air', 'trajectory_tracker',
         '--ros-args', '--params-file', params_file],
        stdout=open(log_path, 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    return proc


def _lifecycle_run(args, timeout=25.0):
    """带超时运行 `ros2 lifecycle ...`（DDS 发现间歇失效时**不**无限阻塞）。"""
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None


def _wait_node_up(timeout_s=25.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        r = _lifecycle_run(['ros2', 'lifecycle', 'get', '/trajectory_tracker'], timeout=8.0)
        if r is not None and r.returncode == 0 and r.stdout.strip():
            return True
        time.sleep(0.5)
    return False


def _transition(tr, tries=2):
    r = None
    for _ in range(tries):
        r = _lifecycle_run(['ros2', 'lifecycle', 'set', '/trajectory_tracker', tr], timeout=25.0)
        if r is not None and r.returncode == 0:
            return True, (r.stdout + r.stderr).strip()
    msg = '' if r is None else (r.stdout + r.stderr).strip()
    return False, msg


def _kill_tracker(proc):
    """仅回收 tracker 子进程组：SIGINT → wait → SIGKILL；不匹配裸名、不做递归删除。

    安全护栏：若 pgid 意外等于**本进程组**（绝不 killpg 自身会话），退回按 PID 发信号。
    """
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    same_group = (pgid == os.getpgrp())
    print('  [reap] tracker pid=%s pgid=%s self_pgrp=%s same_group=%s'
          % (proc.pid, pgid, os.getpgrp(), same_group))

    def _sig(sig):
        try:
            if same_group:
                proc.send_signal(sig)
            else:
                os.killpg(pgid, sig)
        except (ProcessLookupError, PermissionError, OSError):
            pass

    _sig(signal.SIGINT)
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass
    _sig(signal.SIGKILL)
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass


def _xy_var(pts):
    """点集在 XY 上的最大两两距离（离散度指标）。"""
    if len(pts) < 2:
        return 0.0
    mx = 0.0
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            d = math.hypot(pts[i][1] - pts[j][1], pts[i][2] - pts[j][2])
            mx = max(mx, d)
    return mx


def _xy_mean(pts):
    if not pts:
        return (None, None)
    return (sum(p[1] for p in pts) / len(pts), sum(p[2] for p in pts) / len(pts))


def _tail(pts, frac=0.6):
    if len(pts) < 2:
        return pts
    k = max(2, int(len(pts) * frac))
    return pts[-k:]


def _source_seen(statuses, name):
    return any(s.get('source') == name for s in statuses)


def scenario_follower_death(rig):
    """a：已获目标后停发 tracking_ref → 显式 hold（保持位置、不回落八字）。"""
    xt = time.time()

    def drive_acquire():
        rig.pub_odom_now()
        rig.pub_active_now()
        rig.pub_ref_at(HOLD_XY[0], HOLD_XY[1], 2.5, time.time())
        rig.pub_target_at(TARGET_XY[0], TARGET_XY[1], 2.5, 0.9, time.time())

    def drive_lost_ref():
        rig.pub_odom_now()
        rig.pub_active_now()
        # 停发 tracking_ref（follower 死亡）；目标仍新鲜，以**隔离**"失参考"单一因子
        rig.pub_target_at(TARGET_XY[0], TARGET_XY[1], 2.5, 0.9, time.time())

    rig.statuses, rig.eff = [], []
    seg1 = rig.spin_for(2.0, drive_acquire)
    st1 = list(rig.statuses)
    rig.statuses = []
    seg2 = rig.spin_for(2.5, drive_lost_ref)
    st2 = list(rig.statuses)
    tail = _tail(seg2)
    mean = _xy_mean(tail)
    res = {
        'phase1_tracking_seen': _source_seen(st1, 'tracking'),
        'phase2_hold_seen': _source_seen(st2, 'hold'),
        'phase2_base_seen': _source_seen(st2, 'base'),
        'phase2_xy_variation': round(_xy_var(tail), 4),
        'phase2_mean_xy': [None if mean[0] is None else round(mean[0], 3), None if mean[1] is None else round(mean[1], 3)],
        'n_samples_phase2': len(seg2),
    }
    res['pass'] = bool(
        res['phase1_tracking_seen'] and res['phase2_hold_seen'] and
        not res['phase2_base_seen'] and res['n_samples_phase2'] >= 5 and
        res['phase2_xy_variation'] < 0.25 and
        mean[0] is not None and math.hypot(mean[0] - HOLD_XY[0], mean[1] - HOLD_XY[1]) < 0.4)
    res['elapsed_s'] = round(time.time() - xt, 2)
    return res


def scenario_old_ref_replay(rig):
    """b：tracking_ref 源时间为**过去**（旧包重放）→ 按源时间拒 → 保持基础八字（未被旧包推移）。"""
    xt = time.time()

    def drive():
        rig.pub_odom_now()
        rig.pub_active_now()
        # header.stamp = 3 s 之前（源时间过期）；内容与 hold 位置相同，若被采用会**恒停**在该点
        rig.pub_ref_at(HOLD_XY[0], HOLD_XY[1], 2.5, time.time() - 3.0)
        rig.pub_target_at(TARGET_XY[0], TARGET_XY[1], 2.5, 0.9, time.time())

    rig.statuses, rig.eff = [], []
    seg = rig.spin_for(2.5, drive)
    st = list(rig.statuses)
    tail = _tail(seg)
    mean = _xy_mean(tail)
    res = {
        'tracking_seen': _source_seen(st, 'tracking'),
        'hold_seen': _source_seen(st, 'hold'),
        'base_seen': _source_seen(st, 'base'),
        'xy_variation': round(_xy_var(tail), 4),
        'mean_xy': [None if mean[0] is None else round(mean[0], 3), None if mean[1] is None else round(mean[1], 3)],
        'n_samples': len(seg),
    }
    res['pass'] = bool(
        not res['tracking_seen'] and not res['hold_seen'] and res['base_seen'] and
        res['n_samples'] >= 5 and res['xy_variation'] > 0.8)
    res['elapsed_s'] = round(time.time() - xt, 2)
    return res


def scenario_low_confidence(rig):
    """c：目标消息持续但 confidence < 门槛 → 冻结（**不**跟随持续变化的 tracking_ref）。"""
    xt = time.time()

    def drive_acquire():
        rig.pub_odom_now()
        rig.pub_active_now()
        rig.pub_ref_at(HOLD_XY[0], HOLD_XY[1], 2.5, time.time())
        rig.pub_target_at(TARGET_XY[0], TARGET_XY[1], 2.5, 0.9, time.time())

    t_phase2 = [time.time()]

    def drive_low_conf():
        rig.pub_odom_now()
        rig.pub_active_now()
        # tracking_ref **持续发布且位置不断变化**（若 confidence 门未生效，tracker 会跟随它）
        dt = time.time() - t_phase2[0]
        rig.pub_ref_at(HOLD_XY[0] + 2.0 * dt, HOLD_XY[1], 2.5, time.time())
        # 目标消息**持续**（新鲜）但 confidence 低 → 目标门应判过期 / 冻结
        rig.pub_target_at(TARGET_XY[0], TARGET_XY[1], 2.5, 0.1, time.time())

    rig.statuses, rig.eff = [], []
    seg1 = rig.spin_for(2.0, drive_acquire)
    st1 = list(rig.statuses)
    rig.statuses = []
    t_phase2[0] = time.time()
    seg2 = rig.spin_for(2.5, drive_low_conf)
    st2 = list(rig.statuses)
    st2_tail = _tail(st2)   # 相位切换后旧的高 confidence 目标仍在窗内 ~0.5 s，判据取**尾段**（已过该窗）
    tail = _tail(seg2)
    mean = _xy_mean(tail)
    res = {
        'phase1_tracking_seen': _source_seen(st1, 'tracking'),
        'phase2_tail_hold_seen': _source_seen(st2_tail, 'hold'),
        'phase2_tail_tracking_seen': _source_seen(st2_tail, 'tracking'),
        'phase2_xy_variation': round(_xy_var(tail), 4),
        'phase2_mean_xy': [None if mean[0] is None else round(mean[0], 3), None if mean[1] is None else round(mean[1], 3)],
        'n_samples_phase2': len(seg2),
    }
    res['pass'] = bool(
        res['phase1_tracking_seen'] and res['phase2_tail_hold_seen'] and
        not res['phase2_tail_tracking_seen'] and res['n_samples_phase2'] >= 5 and
        res['phase2_xy_variation'] < 0.25 and
        mean[0] is not None and math.hypot(mean[0] - HOLD_XY[0], mean[1] - HOLD_XY[1]) < 0.4)
    res['elapsed_s'] = round(time.time() - xt, 2)
    return res


def scenario_offset_lifetime(rig):
    """d：rl_offset 超幅 → 限幅；源停止（超寿命）→ 归零。"""
    xt = time.time()

    def drive_acquire():
        rig.pub_odom_now()
        rig.pub_active_now()
        rig.pub_ref_at(HOLD_XY[0], HOLD_XY[1], 2.5, time.time())
        rig.pub_target_at(TARGET_XY[0], TARGET_XY[1], 2.5, 0.9, time.time())

    def drive_offset_big():
        rig.pub_odom_now()
        rig.pub_active_now()
        rig.pub_ref_at(HOLD_XY[0], HOLD_XY[1], 2.5, time.time())
        rig.pub_target_at(TARGET_XY[0], TARGET_XY[1], 2.5, 0.9, time.time())
        rig.pub_offset_at(10.0, 10.0, 0.0, time.time())   # 水平幅值 ~14.1 >> rl_offset_max_m(2.0)

    def drive_offset_gone():
        rig.pub_odom_now()
        rig.pub_active_now()
        rig.pub_ref_at(HOLD_XY[0], HOLD_XY[1], 2.5, time.time())
        rig.pub_target_at(TARGET_XY[0], TARGET_XY[1], 2.5, 0.9, time.time())
        # 停发 rl_offset → 源停止、超寿命 → 应归零

    rig.statuses, rig.eff = [], []
    rig.spin_for(1.5, drive_acquire)
    rig.statuses = []
    seg_big = rig.spin_for(2.0, drive_offset_big)
    st_big = list(rig.statuses)
    rig.statuses = []
    seg_gone = rig.spin_for(2.0, drive_offset_gone)
    st_gone = list(rig.statuses)

    tail_big = _tail(seg_big)
    tail_gone = _tail(seg_gone)
    # 相位切换有 ~0.5 s 过渡窗（offset 源刚发 / 刚停尚未超寿命）→ 判据取**尾段**。
    st_big_tail = _tail(st_big)
    st_gone_tail = _tail(st_gone)
    mean_big = _xy_mean(tail_big)
    mean_gone = _xy_mean(tail_gone)
    disp_big = (math.hypot(mean_big[0] - HOLD_XY[0], mean_big[1] - HOLD_XY[1])
                if mean_big[0] is not None else None)
    disp_gone = (math.hypot(mean_gone[0] - HOLD_XY[0], mean_gone[1] - HOLD_XY[1])
                 if mean_gone[0] is not None else None)
    res = {
        'big_offset_applied': all(s.get('offset_applied') for s in st_big_tail) if st_big_tail else False,
        'big_offset_clamped': any(s.get('offset_clamped') for s in st_big_tail),
        'big_displacement_m': None if disp_big is None else round(disp_big, 3),
        'gone_offset_applied': any(s.get('offset_applied') for s in st_gone_tail),
        'gone_displacement_m': None if disp_gone is None else round(disp_gone, 3),
        'n_samples_big': len(seg_big), 'n_samples_gone': len(seg_gone),
    }
    res['pass'] = bool(
        res['big_offset_applied'] and res['big_offset_clamped'] and
        disp_big is not None and abs(disp_big - 2.0) < 0.4 and
        not res['gone_offset_applied'] and
        disp_gone is not None and disp_gone < 0.4 and
        res['n_samples_big'] >= 5 and res['n_samples_gone'] >= 5)
    res['elapsed_s'] = round(time.time() - xt, 2)
    return res


SCENARIOS = {
    'a_follower_death': scenario_follower_death,
    'b_old_ref_replay': scenario_old_ref_replay,
    'c_low_confidence': scenario_low_confidence,
    'd_offset_lifetime': scenario_offset_lifetime,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='r032-bench')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()

    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)
    cfg_dir = os.path.join(out_dir, 'config')
    log_dir = os.path.join(out_dir, 'r032', 'log')
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)
    params_file = os.path.join(cfg_dir, 'r032_bench_params.yaml')
    with open(params_file, 'w', encoding='utf-8') as f:
        f.write(BENCH_PARAMS)

    rclpy.init()
    rig = Rig()
    result = {
        'run_id': args.run_id, 'suite': 'R-032 trajectory_tracker logic bench',
        'bench_params': BENCH_PARAMS,
        'scope_note': ('逻辑台架：隔离安全层（safety.enabled=false），聚焦 tracker 的参考裁定'
                       '（源时间 / confidence / 已获模式 / offset 寿命与限幅）；非整链 / 非 GUI / 非 C3 验收。'),
        'scenarios': {},
    }
    try:
        for name, fn in SCENARIOS.items():
            log = os.path.join(log_dir, '%s.log' % name)
            proc = _spawn_tracker(params_file, log)
            res = {'pass': False}
            try:
                ok = _wait_node_up()
                tc_ok, tc_msg = _transition('configure') if ok else (False, 'node not up')
                ta_ok, ta_msg = _transition('activate') if tc_ok else (False, 'configure failed')
                rig.get_logger().info('scenario %s: configure=%s activate=%s (%s | %s)'
                                      % (name, tc_ok, ta_ok, tc_msg, ta_msg))
                # 让 tracker 稳定 ~1 s（等 takeover：active_controller 已发布）
                t0 = time.time()
                while time.time() - t0 < 1.0:
                    rig.pub_odom_now()
                    rig.pub_active_now()
                    rclpy.spin_once(rig, timeout_sec=0.0)
                    time.sleep(0.05)
                try:
                    res = fn(rig)
                except Exception as exc:  # noqa: BLE001
                    res = {'pass': False, 'error': repr(exc)}
                res['activate_ok'] = bool(tc_ok and ta_ok)
                res['pass'] = bool(res.get('pass') and res['activate_ok'])
            finally:
                # 无论何种路径都回收本 run 的 tracker 进程组（start_new_session → 整组回收）
                _kill_tracker(proc)
            result['scenarios'][name] = res
            print('scenario %-20s PASS=%s' % (name, res.get('pass')))
            time.sleep(0.8)
    finally:
        rig.destroy_node()
        rclpy.shutdown()

    result['overall_pass'] = bool(
        result['scenarios'] and all(s.get('pass') for s in result['scenarios'].values()))
    os.makedirs(os.path.join(out_dir, 'r032'), exist_ok=True)
    with open(os.path.join(out_dir, 'r032', 'bench.json'), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print('\noverall_pass=%s -> %s' % (
        result['overall_pass'], os.path.join(out_dir, 'r032', 'bench.json')))
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
