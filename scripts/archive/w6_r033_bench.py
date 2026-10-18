#!/usr/bin/env python3
"""w6_r033_bench —— R-033（UAV 联合安全屏障与显式不可行路径）**逻辑台架**（Phase A，实跑 trajectory_tracker）。

对每个场景：本进程按场景**专用参数**起**真实** `trajectory_tracker`（`safety.enabled=true` +
场景静态障碍 / 目标几何，configure/activate），并以**合成驱动**发布 `/agc/air/odom`（= 机体当前位置）、
`/agc/air/active_controller`、`/agc/air/tracking_ref`（= 期望参考）、`/agc/targets`；订阅
`/fmu/in/trajectory_setpoint`（重建安全层**修正后的有效参考**）与 `/agc/air/tracking_status`
（R-032/R-033 配套诊断，含 `shield_infeasible` / `shield_reason` / `shield_action`）。

只回收**本进程组的 tracker 子进程**（terminate → wait → kill）；**不** 用裸名模式批杀、**不**做递归删除。
证据落 `experiments/w6-task-eval/<run_id>/r033/bench.json`（ADR-0007）。

场景（与评审 R-033 / spec 场景一一对应）：
  a_joint_overlap   —— 两障碍交叠，期望参考深入重叠区 → **联合复检不可行**（非交换投影收敛失败）→ hold
  b_swept_through   —— 参考**端点**安全但 当前位置→参考 的**直线段穿障** → 扫掠判据检出 → hold
  c_target_closure  —— 目标快速闭合使目标间距约束与障碍约束冲突 → 不可行 → hold（不给不可达参考）
  d_truth_clearance —— 汇总 a/b/c 的**真值最小净空**与**不可行事件计数**（真值最小净空非负）

⚠ **范围标注**：本台架为**逻辑台架**（真实 tracker + 合成驱动，观测参考层），**非**整链 / 非 GUI / 非 C3；
二维静态圆盘屏障为**限定实验模型**（不含车 / 动态障碍状态 = `unsupported`）。
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist

from agc_msgs.msg import TargetTrackArray, TargetTrack
from px4_msgs.msg import TrajectorySetpoint

ROOT = '/home/chen/AGC'

# --- 与 air.yaml safety 一致的余量口径（eff 半径 = 硬半径 + 跟踪误差 + 反应 + 可达性） ---
OBS_MARGIN = 2.5      # 跟踪误差（LQR 滞后）
TGT_MARGIN = 1.5
REACTION_TIME = 0.2
MAX_SPEED = 2.0
REACH = 0.3
EXTRA = REACTION_TIME * MAX_SPEED + REACH    # 0.7（反应 + 可达性）

PARAMS_HEAD = """trajectory_tracker:
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
"""

TARGET_Z = 2.5
CURRENT_Z = 2.5


def eff_obs_radius(r_hard):
    return r_hard + OBS_MARGIN + EXTRA


def eff_tgt_radius(tgt_min_dist):
    return tgt_min_dist + TGT_MARGIN + EXTRA


def bench_params(obstacles, target_min_dist):
    """按场景障碍表生成 tracker 参数（safety.enabled=true）。obstacles: [(name,x,y,r_hard)]。"""
    names = '["' + '", "'.join(o[0] for o in obstacles) + '"]'
    xs = '[' + ', '.join('%.4f' % o[1] for o in obstacles) + ']'
    ys = '[' + ', '.join('%.4f' % o[2] for o in obstacles) + ']'
    rs = '[' + ', '.join('%.4f' % o[3] for o in obstacles) + ']'
    return PARAMS_HEAD + (
        "    safety:\n"
        "      enabled: true\n"
        "      obstacle_margin: %.3f\n"
        "      target_margin: %.3f\n"
        "      target_min_dist: %.3f\n"
        "      reaction_time_s: %.3f\n"
        "      max_speed_mps: %.3f\n"
        "      reachability_margin: %.3f\n"
        "      obstacle_names: %s\n"
        "      obstacle_x: %s\n"
        "      obstacle_y: %s\n"
        "      obstacle_safety_radius: %s\n"
        % (OBS_MARGIN, TGT_MARGIN, target_min_dist, REACTION_TIME, MAX_SPEED, REACH,
           names, xs, ys, rs))


# ---- 场景定义（spec 场景 ①~③） ----
# 每项：obstacles [(name,x,y,r_hard)]；current 机体当前位置；desired 期望参考；target 目标位（可 None）；
#       target_min_dist；expect_action 期望安全动作；expect_reason 期望不可行判据。
SCENARIOS = [
    {
        'name': 'a_joint_overlap',
        # 两障碍 (0,0)/(10,0)，eff=5.2 → 间距 10 < 10.4 交叠；desired (5,0) 深入两盘重叠区。
        'obstacles': [('pole_a', 0.0, 0.0, 2.0), ('pole_b', 10.0, 0.0, 2.0)],
        'current': (5.0, 8.0),
        'desired': (5.0, 0.0),
        'target': (200.0, 0.0),
        'target_min_dist': 8.0,
        'expect_action': 'hold',
        'expect_reason': 'infeasible_joint',
    },
    {
        'name': 'b_swept_through',
        # 单障碍 (0,0)，eff=5.2；current (0,-8) 与 desired (0,8) **端点均安全**，但连线穿障。
        'obstacles': [('pole_c', 0.0, 0.0, 2.0)],
        'current': (0.0, -8.0),
        'desired': (0.0, 8.0),
        'target': (200.0, 0.0),
        'target_min_dist': 8.0,
        'expect_action': 'hold',
        'expect_reason': 'infeasible_swept',
    },
    {
        'name': 'c_target_closure',
        # 障碍 (0,0) eff=5.2；目标 (10.4,0) eff=10.2 → 目标盘与障碍盘交叠；desired (5.2,0) 落入冲突区。
        'obstacles': [('pole_d', 0.0, 0.0, 2.0)],
        'current': (5.0, 20.0),
        'desired': (5.2, 0.0),
        'target': (10.4, 0.0),
        'target_min_dist': 8.0,
        'expect_action': 'hold',
        'expect_reason': 'infeasible_joint',
    },
]


def _to_stamp(sec_float):
    return rclpy.time.Time(nanoseconds=int(sec_float * 1e9)).to_msg()


class Rig(Node):
    """合成驱动 + 证据采集。"""

    def __init__(self):
        super().__init__('w6_r033_rig')
        self.statuses = []
        self.setpoints = []      # (ex, ey)

        q_tl = QoSProfile(depth=1)
        q_tl.reliability = ReliabilityPolicy.RELIABLE
        q_tl.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_odom = self.create_publisher(Odometry, '/agc/air/odom', qos_profile_sensor_data)
        self.pub_active = self.create_publisher(String, '/agc/air/active_controller', q_tl)
        self.pub_ref = self.create_publisher(MultiDOFJointTrajectory, '/agc/air/tracking_ref', 10)
        self.pub_target = self.create_publisher(TargetTrackArray, '/agc/targets', 10)

        self.create_subscription(String, '/agc/air/tracking_status', self._on_status, 10)
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                                 self._on_setpoint, 10)

    def _on_status(self, m):
        try:
            self.statuses.append(json.loads(m.data))
        except Exception:  # noqa: BLE001
            self.statuses.append({'source': None})

    def _on_setpoint(self, m):
        # enuToNed: ned=(enu.y, enu.x, -enu.z) → enu=(ned.y, ned.x, -ned.z)
        self.setpoints.append((m.position[1], m.position[0]))

    def pub_odom_at(self, x, y, z=CURRENT_Z):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'world'
        o.child_frame_id = 'drone/base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
        o.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(o)

    def pub_active_now(self):
        m = String()
        m.data = 'trajectory_tracker'
        self.pub_active.publish(m)

    def pub_ref_at(self, x, y, z=TARGET_Z, stamp_s=None):
        m = MultiDOFJointTrajectory()
        m.header.frame_id = 'world'
        m.header.stamp = _to_stamp(stamp_s if stamp_s is not None else time.time())
        p = MultiDOFJointTrajectoryPoint()
        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z = x, y, z
        tf.rotation.w = 1.0
        p.transforms.append(tf)
        tw = Twist()
        p.velocities.append(tw)
        p.accelerations.append(Twist())
        m.points.append(p)
        self.pub_ref.publish(m)

    def pub_target_at(self, x, y, z=TARGET_Z, conf=0.9, stamp_s=None):
        arr = TargetTrackArray()
        arr.header.frame_id = 'world'
        arr.header.stamp = _to_stamp(stamp_s if stamp_s is not None else time.time())
        tr = TargetTrack()
        tr.header.frame_id = 'world'
        tr.header.stamp = _to_stamp(stamp_s if stamp_s is not None else time.time())
        tr.target_id = 'intruder_uav'
        tr.pose.pose.position.x, tr.pose.pose.position.y, tr.pose.pose.position.z = x, y, z
        tr.confidence = float(conf)
        tr.source = 'bench'
        arr.tracks.append(tr)
        self.pub_target.publish(arr)

    def spin_for(self, dur_s, publish_fn):
        t0 = time.time()
        while time.time() - t0 < dur_s:
            publish_fn()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)


def _spawn_tracker(params_file, log_path):
    return subprocess.Popen(
        ['ros2', 'run', 'agc_air', 'trajectory_tracker',
         '--ros-args', '--params-file', params_file],
        stdout=open(log_path, 'w'), stderr=subprocess.STDOUT, start_new_session=True)


def _lifecycle_run(args, timeout=25.0):
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


def _tail(pts, frac=0.6):
    if len(pts) < 2:
        return pts
    k = max(2, int(len(pts) * frac))
    return pts[-k:]


def _xy_mean(pts):
    if not pts:
        return (None, None)
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def _min_truth_clearance(spec, samples):
    """真值最小净空：对**硬半径**（安全半径 / target_min_dist）计净空（**不**含余量），取全程最小。"""
    mn = None
    for (ex, ey) in samples:
        for (_n, ox, oy, orr) in spec['obstacles']:
            c = math.hypot(ex - ox, ey - oy) - orr
            mn = c if mn is None else min(mn, c)
        if spec.get('target'):
            tx, ty = spec['target'][0], spec['target'][1]
            c = math.hypot(ex - tx, ey - ty) - spec['target_min_dist']
            mn = c if mn is None else min(mn, c)
    return mn


def _endpoint_infeasible(spec):
    """期望参考端点是否侵入任一**修正后**约束盘（用于如实报告本场景的"触发"）。"""
    ex, ey = spec['desired']
    for (_n, ox, oy, orr) in spec['obstacles']:
        if math.hypot(ex - ox, ey - oy) < eff_obs_radius(orr) - 1e-9:
            return True
    if spec.get('target'):
        tx, ty = spec['target'][0], spec['target'][1]
        if math.hypot(ex - tx, ey - ty) < eff_tgt_radius(spec['target_min_dist']) - 1e-9:
            return True
    return False


def run_scenario(rig, spec, cfg_dir, log_dir):
    name = spec['name']
    params_file = os.path.join(cfg_dir, 'r033_bench_params_%s.yaml' % name)
    with open(params_file, 'w', encoding='utf-8') as f:
        f.write(bench_params(spec['obstacles'], spec['target_min_dist']))

    proc = _spawn_tracker(params_file, os.path.join(log_dir, '%s.log' % name))
    res = {'pass': False}
    try:
        ok = _wait_node_up()
        tc_ok, tc_msg = _transition('configure') if ok else (False, 'node not up')
        ta_ok, ta_msg = _transition('activate') if tc_ok else (False, 'configure failed')

        def drive():
            rig.pub_odom_at(spec['current'][0], spec['current'][1])
            rig.pub_active_now()
            rig.pub_ref_at(spec['desired'][0], spec['desired'][1])
            if spec.get('target'):
                rig.pub_target_at(spec['target'][0], spec['target'][1], conf=0.9)

        # 稳定 1 s（等 takeover：active_controller 已发布）
        t0 = time.time()
        while time.time() - t0 < 1.0:
            drive()
            rclpy.spin_once(rig, timeout_sec=0.0)
            time.sleep(0.05)

        rig.statuses = []
        rig.setpoints = []
        rig.spin_for(2.5, drive)

        tail = _tail(rig.setpoints)
        mean = _xy_mean(tail)
        actions = sorted({s.get('shield_action', '') for s in rig.statuses if s})
        reasons = sorted({s.get('shield_reason', '') for s in rig.statuses if s})
        infeasible_count = sum(1 for s in rig.statuses if s.get('shield_infeasible'))
        mclr = _min_truth_clearance(spec, tail)
        hold_disp = (math.hypot(mean[0] - spec['current'][0], mean[1] - spec['current'][1])
                     if mean[0] is not None else None)

        res = {
            'desired_endpoint_infeasible': _endpoint_infeasible(spec),
            'infeasible_seen': infeasible_count > 0,
            'infeasible_count': infeasible_count,
            'actions_seen': actions,
            'reasons_seen': reasons,
            'tail_mean_xy': [None if mean[0] is None else round(mean[0], 3),
                             None if mean[1] is None else round(mean[1], 3)],
            'expected_hold_xy': [spec['current'][0], spec['current'][1]],
            'hold_disp_m': None if hold_disp is None else round(hold_disp, 3),
            'min_truth_clearance_m': None if mclr is None else round(mclr, 3),
            'n_setpoint_samples': len(tail),
            'n_status': len(rig.statuses),
            'configure_ok': bool(tc_ok), 'activate_ok': bool(ta_ok),
        }
        res['pass'] = bool(
            tc_ok and ta_ok and
            res['n_setpoint_samples'] >= 5 and
            res['infeasible_seen'] and
            spec['expect_action'] in actions and
            spec['expect_reason'] in reasons and
            hold_disp is not None and hold_disp < 0.5 and
            mclr is not None and mclr >= -0.05)
        res['activate_msg'] = (tc_msg + ' | ' + ta_msg)[:200]
    finally:
        _kill_tracker(proc)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='r033-bench')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()

    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)
    cfg_dir = os.path.join(out_dir, 'config')
    log_dir = os.path.join(out_dir, 'r033', 'log')
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    rclpy.init()
    rig = Rig()
    result = {
        'run_id': args.run_id, 'suite': 'R-033 UAV joint safety barrier logic bench',
        'scope_note': (
            '逻辑台架：真实 trajectory_tracker（safety.enabled=true）+ 合成驱动，聚焦**联合安全屏障**'
            '（联合约束复检 / 扫掠复检 / 显式不可行 hold）。**二维静态圆盘**为限定实验模型'
            '（不含车 / 动态障碍状态 = unsupported）；**非整链** / 非 GUI / 非 C3 验收。'),
        'model_limitations': {
            'model': '2D static disk barrier',
            'dynamic_obstacle_state': 'unsupported',
            'own_vehicle_footprint_3d': 'unsupported',
            'net_payload_geometry': 'unsupported',
        },
        'scenarios': {},
    }
    try:
        for spec in SCENARIOS:
            res = run_scenario(rig, spec, cfg_dir, log_dir)
            result['scenarios'][spec['name']] = res
            print('scenario %-20s PASS=%s' % (spec['name'], res.get('pass')))
            time.sleep(0.8)
    finally:
        rig.destroy_node()
        rclpy.shutdown()

    # 场景 ④：真值最小净空汇总 + 不可行事件计数（跨 a/b/c）
    min_clr = None
    inf_total = 0
    for s in result['scenarios'].values():
        if s.get('min_truth_clearance_m') is not None:
            min_clr = (s['min_truth_clearance_m'] if min_clr is None
                       else min(min_clr, s['min_truth_clearance_m']))
        inf_total += int(s.get('infeasible_count', 0))
    result['scenarios']['d_truth_clearance'] = {
        'min_truth_clearance_m': None if min_clr is None else round(min_clr, 3),
        'infeasible_event_count': inf_total,
        'aggregated_over': ['a_joint_overlap', 'b_swept_through', 'c_target_closure'],
        'pass': bool(min_clr is not None and min_clr >= -0.05 and inf_total >= 3),
    }
    print('scenario %-20s PASS=%s' % ('d_truth_clearance',
                                      result['scenarios']['d_truth_clearance']['pass']))

    result['overall_pass'] = bool(
        result['scenarios'] and all(s.get('pass') for s in result['scenarios'].values()))
    os.makedirs(os.path.join(out_dir, 'r033'), exist_ok=True)
    with open(os.path.join(out_dir, 'r033', 'bench.json'), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print('\noverall_pass=%s -> %s' % (
        result['overall_pass'], os.path.join(out_dir, 'r033', 'bench.json')))
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
