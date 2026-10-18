#!/usr/bin/env python3
"""w5_r009_check —— W5 R-009 执行中继硬化 + GZ 侧末端失效停车验收。

背景（评审风险 R-009）：地面执行中继 `ground_drive_relay` 原以 ROS 时钟判接收年龄，而 GZ 的
DiffDrive 插件**没有**命令超时（已核验 libgz-sim8-diff-drive-system.so 参数清单无 timeout/deadman），
故中继自身死亡或时钟异常时末端可能持续执行最后命令。
契约真源：docs/ROS2迁移交接文档.md#execution-port（唯一执行口）、#params（参数单一注入）。

本脚本三个场景（`--scenario`）：
  relay    —— 不依赖 GZ、独立进程拉起中继，验证「中继自身」鲁棒化（S1–S6）。见下。
  evidence —— 整仿真内**动态取证**：不作断言，只记录事实——满速稳态后 SIGKILL 中继，
              随后 5 s 采样 /agc/ground/odom 真值速度与 /agc/gt/agc_car 真值位姿，
              记录「杀进程后真值是否仍运动、持续多久、累计位移」。产物 r009_evidence_dynamic.json。
              （须在**末端机制落地前**跑，量化危害；机制落地后复跑会因归零而观察不到持续运动。）
  sim      —— 整仿真内**集成验收** K1/K2/K3（末端机制落地后）：K1 杀中继→真值在标定窗口内停稳；
              K2 正常回归（命令生效、零命令停住、守门人不误触发）；K3（可选）重启中继→恢复。
              产物 r009.json。

`--scenario relay` 子测（PASS 判据见各 test 的 `pass` 字段）：
  S1 单调时钟超时（ROS 钟冻结）→ watchdog_s 量级内输出归零（旧实现永保 1.5）。
  S2 stamp 倒退拒收。 S3 错 frame_id 拒收。 S4 NaN / inf 拒收。 S5 限幅。 S6 正常透传。

`--scenario sim` 标定口径（上界来历，见 ADR-0021(c) 本车实测动力学）：
  本车 v_max = 1.5 m/s、锁轮制动 a ≈ 0.9 m/s²（= DiffDrive max_wheel_acceleration × 轮半径）。
  末端机制超时 T = cmd_deadman_timeout_s（与中继 watchdog_s 同量级，取值口径 = 契约 #params 单一来源）：
    * 失源盲行 d_blind = v_max · T = 1.5 × 0.3 = 0.45 m；
    * 制动距离 s_brake = v_max² / (2a) = 1.5² / (2 × 0.9) = 1.25 m；
    * 制动耗时 t_brake = v_max / a = 1.67 s。
  K1 判据上界（**取保守口径**，与 R-008 同法）：
    * 停稳窗口上界 K_STOP_WINDOW_S = T + t_brake + 余量 0.5 = 2.5 s；
    * 残余位移上界 K_RESIDUAL_BOUND_M = d_blind + s_brake + 余量 0.3 = 2.0 m。
  （上界取整加余量、不刻意放宽；实测应显著小于上界，实测值一并落 JSON。）

退出码：0 PASS / 1 断言 FAIL / 2 基础设施错误（供 shell 决定是否重试）。
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
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose

RAW_TOPIC = '/agc/ground/cmd_vel_raw'
CMD_TOPIC = '/agc/ground/cmd_vel'
OUT_TOPIC = '/model/agc_car/cmd_vel'
ODOM_TOPIC = '/agc/ground/odom'
GT_TOPIC = '/agc/gt/agc_car'
GZ_DIFFDRIVE_LIB = (
    '/home/chen/AGC/../../../opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8-diff-drive-system.so')
EVIDENCE_GREP = 'max_linear_velocity|max_angular_velocity|odom_topic|timeout|deadman'

# ---- 本车实测动力学 / 末端机制标定（口径见模块 docstring，来源 ADR-0021(c) + car.yaml）----
CAR_V_MAX = 1.5           # 车体线速上限（config/car.yaml model.max_linear_velocity）
A_BRAKE = 0.9             # 锁轮制动减速度（本车实测）
K_TIMEOUT_S = 0.3         # 末端机制超时（cad 与 car.yaml cmd_deadman_timeout_s / 中继 watchdog_s 一致）
K_V_STEADY = 1.3          # 满速稳态判据（真值速度 > 此值才注入故障；留链路损耗余量）
K_STOP_WINDOW_S = 2.5     # 杀中继 → 真值停稳窗口上界 = T + v_max/a + 0.5
K_RESIDUAL_BOUND_M = 2.0  # 杀中继后残余位移上界 = v_max·T + v_max²/(2a) + 0.3
K_SAMPLE_S = 5.0          # 杀后采样时长
K_ZERO_THRESH = 0.05      # 归零判据（真值速度 m/s）
EVIDENCE_SAMPLE_S = 5.0   # evidence 场景杀后采样时长

K2_V_CMD = 0.8            # K2 正常回归命令线速
K2_V_TOL = 0.2            # K2 稳态真值 ± 容差


class Probe(Node):
    """共享探针：发 raw 命令、订阅实际命令 / odom / gt，提供 set_pose 与 lifecycle 查询。"""

    def __init__(self):
        super().__init__('w5_r009_check')
        self.samples = []   # (t_monotonic, out_linear.x, out_angular.z)  —— 实际转发命令
        self.raw_pub = self.create_publisher(TwistStamped, RAW_TOPIC, 10)
        self.x = self.y = self.yaw = 0.0
        self.ovx = self.ovy = 0.0
        self.have_odom = False
        self.gx = self.gy = 0.0
        self.have_gt = False
        self.create_subscription(Twist, OUT_TOPIC, self._on_out, 50)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 50)
        self.create_subscription(PoseStamped, GT_TOPIC, self._on_gt, 50)
        self.pose_cli = self.create_client(SetEntityPose, '/world/agc_world/set_pose')
        self.state_cli = self.create_client(GetState, '/collision_monitor/get_state')

    def _on_out(self, m):
        self.samples.append((time.monotonic(), float(m.linear.x), float(m.angular.z)))

    def _on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.ovx = m.twist.twist.linear.x
        self.ovy = m.twist.twist.linear.y
        self.have_odom = True

    def _on_gt(self, m):
        self.gx = m.pose.position.x
        self.gy = m.pose.position.y
        self.have_gt = True

    def odom_speed(self):
        return math.hypot(self.ovx, self.ovy)

    def window(self, t_from):
        return [s for s in self.samples if s[0] >= t_from]

    def publish_cmd(self, v, w=0.0, frame='car/base_link'):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.raw_pub.publish(m)

    def spin_for(self, sec):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.01)
        return time.monotonic()

    def settle(self, sec=1.5):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            self.publish_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def set_pose(self, x, y, z, yaw, timeout=6.0):
        if not self.pose_cli.wait_for_service(timeout_sec=timeout):
            return False
        req = SetEntityPose.Request()
        req.entity.name = 'agc_car'
        req.entity.type = 2
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.z = math.sin(yaw * 0.5)
        req.pose.orientation.w = math.cos(yaw * 0.5)
        fut = self.pose_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() and fut.result().success)

    def wait_active(self, timeout=70.0):
        if not self.state_cli.wait_for_service(timeout_sec=15.0):
            return False, 'no /collision_monitor/get_state'
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            fut = self.state_cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.done() and fut.result() is not None and fut.result().current_state.id == 3:
                return True, 'active'
            time.sleep(1.0)
        return False, 'collision_monitor not active in %.0fs' % timeout


def make_cmd(v, w, stamp, frame='car/base_link'):
    m = TwistStamped()
    m.header.frame_id = frame
    sec = int(math.floor(stamp))
    m.header.stamp.sec = sec
    m.header.stamp.nanosec = int(round((stamp - sec) * 1e9))
    m.twist.linear.x = float(v)
    m.twist.angular.z = float(w)
    return m


def vmax(samples):
    return max((abs(s[1]) for s in samples), default=0.0)


def wmax(samples):
    return max((abs(s[2]) for s in samples), default=0.0)


def finite_all(samples):
    return all(math.isfinite(s[1]) and math.isfinite(s[2]) for s in samples)


def pgrep(pattern):
    try:
        out = subprocess.check_output(['pgrep', '-f', pattern], text=True)
    except subprocess.CalledProcessError:
        return []
    return [int(p) for p in out.split() if p.strip()]


def evidence_gz_diffdrive():
    """记录事实：GZ DiffDrive 参数清单中是否含 timeout/deadman（不下'车会持续运动'的结论）。"""
    ev = {'lib': GZ_DIFFDRIVE_LIB, 'grep_pattern': EVIDENCE_GREP}
    try:
        raw = subprocess.run(['strings', GZ_DIFFDRIVE_LIB], capture_output=True, text=True,
                             timeout=60).stdout
    except Exception as exc:  # noqa: BLE001
        ev['error'] = repr(exc)
        return ev
    lines = [ln for ln in raw.splitlines() if __import__('re').search(EVIDENCE_GREP, ln)]
    ev['matched_lines'] = lines
    ev['has_timeout_token'] = any('timeout' in ln for ln in lines)
    ev['has_deadman_token'] = any('deadman' in ln for ln in lines)
    return ev


def start_relay(exe, params_file, log_path, overrides):
    args = [exe, '--ros-args']
    if params_file:
        args += ['--params-file', params_file]
    for k, v in overrides.items():
        args += ['-p', '%s:=%s' % (k, v)]
    log = open(log_path, 'w')
    proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
    return proc, log


def metrics_dir(args):
    d = os.path.join(args.experiments_root, 'w5-ground-nav-safety', args.run_id, 'metrics')
    os.makedirs(d, exist_ok=True)
    return d


def relay_args(args):
    return dict(exe=args.relay_exe, params_file=args.params_file,
                overrides={'use_sim_time': 'true', 'in_topic': RAW_TOPIC, 'out_topic': OUT_TOPIC},
                watchdog_s=0.3)


# =====================================================================================
# 场景 relay（S1–S6，独立进程，不依赖 GZ）
# =====================================================================================
def run_relay(args):
    out_dir = metrics_dir(args)
    res = {
        'run_id': args.run_id, 'scenario': 'relay', 'tests': {},
        'relay': relay_args(args), 'evidence': evidence_gz_diffdrive(),
    }

    rclpy.init()
    obs = Probe()

    proc, log = start_relay(
        args.relay_exe, args.params_file, os.path.join(out_dir, 'relay.log'),
        {'use_sim_time': 'true', 'in_topic': RAW_TOPIC, 'out_topic': OUT_TOPIC})
    res['relay']['pid'] = proc.pid

    def publish_for(sec, msg):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            obs.raw_pub.publish(msg)
            rclpy.spin_once(obs, timeout_sec=0.01)
        return time.monotonic()

    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20.0 and not obs.samples:
            rclpy.spin_once(obs, timeout_sec=0.05)
        if not obs.samples:
            res['overall_pass'] = False
            res['error'] = 'relay 未在 20 s 内上线（输出话题无消息）'
            return dump(res, out_dir, 'r009_relay.json')

        watchdog_s = res['relay']['watchdog_s']

        # ---- S1 单调时钟超时（ROS 时钟冻结）----
        t_s1 = time.monotonic()
        for _ in range(6):
            publish_for(0.1, make_cmd(1.5, 0.0, 0.0))
        v_peak_s1 = vmax(obs.window(t_s1))
        t_end = time.monotonic()
        t_zero = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            rclpy.spin_once(obs, timeout_sec=0.01)
            for s in obs.samples:
                if s[0] >= t_end and abs(s[1]) < 0.05:
                    t_zero = s[0]
                    break
            if t_zero:
                break
        dur_zero = round(t_zero - t_end, 3) if t_zero else None
        res['tests']['S1_monotonic_timeout_frozen_clock'] = {
            'cmd': 'v=1.5 (stamp=0, ROS 钟冻结无 /clock)，随后停发',
            'v_peak_before_stop': round(v_peak_s1, 3),
            'zero_reached': bool(t_zero is not None),
            'stop_to_zero_s': dur_zero,
            'bound_s': round(watchdog_s + 0.5, 3),
            'pass': bool(v_peak_s1 >= 1.4 and t_zero is not None and dur_zero <= watchdog_s + 0.5),
        }

        # ---- S2 stamp 倒退拒收 ----
        t_s2 = time.monotonic()
        for st in (1.0, 1.1, 1.2, 1.3):
            publish_for(0.15, make_cmd(1.0, 0.0, st))
        publish_for(0.5, make_cmd(1.5, 0.0, 1.05))
        w2 = obs.window(t_s2)
        res['tests']['S2_stamp_regression_reject'] = {
            'cmd': 'stamp 1.0..1.3 v=1.0 -> stamp 1.05 v=1.5（倒退）',
            'v_max_observed': round(vmax(w2), 3),
            'v_1_0_seen': bool(vmax(w2) >= 0.9),
            'pass': bool(vmax(w2) <= 1.1 and vmax(w2) >= 0.9),
        }

        # ---- S3 错 frame_id 拒收 ----
        t_s3 = time.monotonic()
        publish_for(0.3, make_cmd(0.7, 0.0, 2.0))
        publish_for(0.5, make_cmd(1.5, 0.0, 2.1, 'map'))
        w3 = obs.window(t_s3)
        res['tests']['S3_wrong_frame_reject'] = {
            'cmd': "frame_id='map' v=1.5（先发 car/base_link v=0.7 建立基线）",
            'v_max_observed': round(vmax(w3), 3),
            'pass': bool(vmax(w3) <= 0.8 and vmax(w3) >= 0.6),
        }

        # ---- S4 NaN / inf 拒收 ----
        t_s4 = time.monotonic()
        publish_for(0.3, make_cmd(0.6, 0.0, 3.0))
        publish_for(0.3, make_cmd(float('nan'), 0.0, 3.1))
        publish_for(0.3, make_cmd(float('inf'), 0.0, 3.2))
        w4 = obs.window(t_s4)
        res['tests']['S4_nonfinite_reject'] = {
            'cmd': 'linear.x = nan / +inf',
            'finite_observed': bool(finite_all(w4)),
            'v_max_observed': round(vmax(w4), 3) if finite_all(w4) else None,
            'pass': bool(finite_all(w4) and vmax(w4) <= 0.7 and vmax(w4) >= 0.5),
        }

        # ---- S5 限幅 ----
        t_s5a = time.monotonic()
        publish_for(0.4, make_cmd(5.0, 0.0, 4.0))
        w5a = obs.window(t_s5a)
        t_s5b = time.monotonic()
        publish_for(0.4, make_cmd(0.0, 9.0, 4.1))
        w5b = obs.window(t_s5b)
        res['tests']['S5_clamp'] = {
            'cmd': 'v=5.0 -> 1.5 (v_max)；omega=9.0 -> 2.0 (omega_max)',
            'v_after_clamp': round(vmax(w5a), 3),
            'w_after_clamp': round(wmax(w5b), 3),
            'pass': bool(abs(vmax(w5a) - 1.5) <= 0.05 and abs(wmax(w5b) - 2.0) <= 0.05),
        }

        # ---- S6 正常透传 ----
        t_s6 = time.monotonic()
        publish_for(0.5, make_cmd(0.8, 0.5, 5.0))
        w6 = obs.window(t_s6)
        res['tests']['S6_passthrough'] = {
            'cmd': 'v=0.8 / omega=0.5',
            'v_max_observed': round(vmax(w6), 3),
            'w_max_observed': round(wmax(w6), 3),
            'pass': bool(abs(vmax(w6) - 0.8) <= 0.05 and abs(wmax(w6) - 0.5) <= 0.05),
        }
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=5)
        except Exception:  # noqa: BLE001
            proc.kill()
        log.close()
        obs.destroy_node()
        rclpy.shutdown()

    res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    return dump(res, out_dir, 'r009_relay.json')


# =====================================================================================
# 场景 evidence（动态取证：只记事实，不下结论）
# =====================================================================================
def run_evidence(args):
    out_dir = metrics_dir(args)
    res = {
        'run_id': args.run_id, 'scenario': 'evidence',
        'note': '只记录事实与数值，不下结论；须在末端机制落地前采集（机制落地后会归零而观察不到持续运动）。',
        'cmd_raw': 'linear.x=%.2f frame_id=car/base_link（脚本化命令源直发 %s）' % (CAR_V_MAX, RAW_TOPIC),
        'evidence_static': evidence_gz_diffdrive(),
    }

    rclpy.init()
    n = Probe()
    ok = False
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20.0 and not (n.have_odom and n.have_gt):
            rclpy.spin_once(n, timeout_sec=0.05)
        active, amsg = n.wait_active()
        res['collision_monitor_active'] = active
        res['collision_monitor_state_msg'] = amsg
        if not (active and n.have_odom and n.have_gt):
            res['overall_pass'] = False
            res['error'] = '链路未就绪：%s odom=%s gt=%s' % (amsg, n.have_odom, n.have_gt)
            return dump(res, out_dir, 'r009_evidence_dynamic.json')

        # 传送到开阔区
        if not n.set_pose(30.0, 30.0, 0.02, 0.0):
            res['overall_pass'] = False
            res['error'] = 'set_pose 失败'
            return dump(res, out_dir, 'r009_evidence_dynamic.json')
        n.settle(2.0)

        # 满速 + 稳态
        t0 = time.monotonic()
        while time.monotonic() - t0 < 6.0:
            n.publish_cmd(CAR_V_MAX, 0.0)
            rclpy.spin_once(n, timeout_sec=0.005)
            if n.odom_speed() > 1.4:
                break
        v_steady = n.odom_speed()
        res['v_steady_before_kill_mps'] = round(v_steady, 3)

        pids = pgrep('[g]round_drive_relay')
        res['relay_pids'] = pids
        if not pids:
            res['overall_pass'] = False
            res['error'] = 'ground_drive_relay 进程未找到'
            return dump(res, out_dir, 'r009_evidence_dynamic.json')

        g_at_kill = (n.gx, n.gy)
        t_kill = time.monotonic()
        for p in pids:
            os.kill(p, signal.SIGKILL)

        # 杀后 5 s 采样（不再发命令）
        timeline = []
        last_moving_t = t_kill
        t_end = t_kill + EVIDENCE_SAMPLE_S
        while time.monotonic() < t_end:
            rclpy.spin_once(n, timeout_sec=0.005)
            now = time.monotonic()
            spd = n.odom_speed()
            if spd > K_ZERO_THRESH:
                last_moving_t = now
            timeline.append({
                't_s': round(now - t_kill, 3),
                'odom_speed_mps': round(spd, 3),
                'gt_x': round(n.gx, 3), 'gt_y': round(n.gy, 3),
                'cmd_out_linear': round(n.samples[-1][1], 3) if n.samples else None,
            })

        motion_duration_s = round(last_moving_t - t_kill, 3)
        displacement_m = math.hypot(n.gx - g_at_kill[0], n.gy - g_at_kill[1])
        res['facts'] = {
            'kill_to_end_sample_s': EVIDENCE_SAMPLE_S,
            'v_steady_gt': round(v_steady, 3),
            'still_moving_after_kill': bool(motion_duration_s > 0.5),
            'motion_duration_s': motion_duration_s,
            'cumulative_displacement_m': round(displacement_m, 3),
            'speed_at_end_mps': round(n.odom_speed(), 3),
            'max_speed_after_kill_mps': round(max((s['odom_speed_mps'] for s in timeline), default=0.0), 3),
            'cmd_out_at_end': timeline[-1]['cmd_out_linear'] if timeline else None,
        }
        res['timeline'] = timeline
        ok = True
    finally:
        n.destroy_node()
        rclpy.shutdown()

    res['overall_pass'] = bool(ok)
    return dump(res, out_dir, 'r009_evidence_dynamic.json')


# =====================================================================================
# 场景 sim（集成验收 K1/K2/K3）
# =====================================================================================
def _drive_to_steady(n, v_cmd, v_target, timeout, dwell=1.0):
    """发 v_cmd 命令直到真值速度 > v_target，再持续 dwell 秒到达稳态；返回稳态速度。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        n.publish_cmd(v_cmd, 0.0)
        rclpy.spin_once(n, timeout_sec=0.005)
        if n.odom_speed() > v_target:
            break
    t1 = time.monotonic()
    while time.monotonic() - t1 < dwell:
        n.publish_cmd(v_cmd, 0.0)
        rclpy.spin_once(n, timeout_sec=0.005)
    return n.odom_speed()


def _stop_and_measure(n, t_kill, duration, stop_thresh, quiet_s):
    """采到真值速度持续 < stop_thresh（或超时）；返回 (t_first_zero, t_settled, timeline)。"""
    timeline = []
    t_first_zero = None
    last_move = t_kill
    t_end = t_kill + duration
    while time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.005)
        now = time.monotonic()
        spd = n.odom_speed()
        timeline.append({'t_s': round(now - t_kill, 3), 'odom_speed_mps': round(spd, 3),
                         'gt_x': round(n.gx, 3), 'gt_y': round(n.gy, 3)})
        if spd > stop_thresh:
            last_move = now
            t_first_zero = None
        elif t_first_zero is None:
            t_first_zero = now
        if t_first_zero is not None and now - last_move > quiet_s:
            break
    return t_first_zero, time.monotonic(), timeline


def run_sim(args):
    out_dir = metrics_dir(args)
    res = {
        'run_id': args.run_id, 'scenario': 'sim', 'tests': {},
        'calibration': {
            'v_max_mps': CAR_V_MAX, 'a_brake_mps2': A_BRAKE, 'timeout_s': K_TIMEOUT_S,
            'blind_dist_m': round(CAR_V_MAX * K_TIMEOUT_S, 3),
            'brake_dist_m': round(CAR_V_MAX ** 2 / (2 * A_BRAKE), 3),
            'stop_window_bound_s': K_STOP_WINDOW_S, 'residual_bound_m': K_RESIDUAL_BOUND_M,
            'note': '上界口径见模块 docstring（v_max·T + v_max²/(2a) + 余量）',
        },
        'evidence_static': evidence_gz_diffdrive(),
    }

    rclpy.init()
    n = Probe()
    relay_proc = None
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20.0 and not (n.have_odom and n.have_gt):
            rclpy.spin_once(n, timeout_sec=0.05)
        active, amsg = n.wait_active()
        res['collision_monitor_active'] = active
        res['collision_monitor_state_msg'] = amsg
        if not (active and n.have_odom and n.have_gt):
            res['error'] = 'infra: 链路未就绪：%s odom=%s gt=%s' % (amsg, n.have_odom, n.have_gt)
            return dump(res, out_dir, 'r009.json', infra=True)

        # ---- K2 正常回归：命令生效、零命令停住，守门人不误触发 ----
        n.set_pose(18.0, 18.0, 0.02, 0.0)
        n.settle(1.6)
        v_steady = _drive_to_steady(n, K2_V_CMD, K2_V_CMD - K2_V_TOL, 5.0)
        res['tests']['K2_normal_regression_drive'] = {
            'cmd': 'linear.x=%.2f' % K2_V_CMD,
            'v_steady_gt': round(v_steady, 3),
            'pass': bool(abs(v_steady - K2_V_CMD) <= K2_V_TOL),
        }
        n.settle(1.0)
        stopped_v = n.odom_speed()
        res['tests']['K2_normal_regression_stop'] = {
            'cmd': 'linear.x=0.0（连续发）',
            'v_after_stop_gt': round(stopped_v, 3),
            'pass': bool(stopped_v < K_ZERO_THRESH),
        }

        # ---- K1 杀中继 → 真值停车 ----
        n.set_pose(30.0, 30.0, 0.02, 0.0)
        n.settle(1.6)
        v_steady_k1 = _drive_to_steady(n, CAR_V_MAX, K_V_STEADY, 6.0)
        g_at_kill = (n.gx, n.gy)
        pids = pgrep('[g]round_drive_relay')
        k1 = {'relay_pids': pids, 'v_steady_before_kill_gt': round(v_steady_k1, 3)}
        if not pids:
            k1['pass'] = False
            k1['error'] = 'ground_drive_relay 进程未找到'
            res['tests']['K1_kill_relay_stop'] = k1
        else:
            t_kill = time.monotonic()
            for p in pids:
                os.kill(p, signal.SIGKILL)
            t_zero, _, timeline = _stop_and_measure(
                n, t_kill, K_SAMPLE_S, K_ZERO_THRESH, 0.5)
            residual = math.hypot(n.gx - g_at_kill[0], n.gy - g_at_kill[1])
            window_s = round((t_zero - t_kill), 3) if t_zero else None
            k1.update({
                'stop_window_s': window_s,
                'stop_window_bound_s': K_STOP_WINDOW_S,
                'residual_displacement_m': round(residual, 3),
                'residual_bound_m': K_RESIDUAL_BOUND_M,
                'max_speed_after_kill_mps': round(max((s['odom_speed_mps'] for s in timeline), default=0.0), 3),
                'timeline_head': timeline[:6],
                'pass': bool(t_zero is not None and window_s is not None
                             and window_s <= K_STOP_WINDOW_S and residual <= K_RESIDUAL_BOUND_M),
            })
            res['tests']['K1_kill_relay_stop'] = k1

        # ---- K3 重启中继 → 命令恢复 ----
        log_path = os.path.join(out_dir, 'relay_restart.log')
        relay_proc, logf = start_relay(args.relay_exe, args.params_file, log_path, {})
        time.sleep(2.0)
        v_resume = _drive_to_steady(n, K2_V_CMD, 0.5, 6.0)
        res['tests']['K3_relay_restart_recovery'] = {
            'cmd': '重启中继后 linear.x=%.2f' % K2_V_CMD,
            'relay_restart_pid': relay_proc.pid,
            'v_after_restart_gt': round(v_resume, 3),
            'pass': bool(v_resume > 0.5),
        }
        logf.close()
    finally:
        if relay_proc is not None:
            try:
                relay_proc.terminate()
                relay_proc.wait(timeout=5)
            except Exception:  # noqa: BLE001
                relay_proc.kill()
        n.destroy_node()
        rclpy.shutdown()

    res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    return dump(res, out_dir, 'r009.json')


def dump(res, out_dir, filename, infra=False):
    path = os.path.join(out_dir, filename)
    with open(path, 'w') as fh:
        json.dump(res, fh, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-34s %s  %s' % (name, 'PASS' if t.get('pass') else 'FAIL',
                                {k: v for k, v in t.items() if k != 'pass'}))
    if 'facts' in res:
        print('facts:', res['facts'])
    if 'evidence' in res:
        ev = res['evidence']
        print('evidence: has_timeout_token=%s has_deadman_token=%s' % (
            ev.get('has_timeout_token'), ev.get('has_deadman_token')))
    if 'evidence_static' in res:
        ev = res['evidence_static']
        print('evidence_static: has_timeout_token=%s has_deadman_token=%s' % (
            ev.get('has_timeout_token'), ev.get('has_deadman_token')))
    if 'error' in res:
        print('error:', res['error'])
        return 2 if infra else 1
    ok = bool(res.get('overall_pass'))
    print('\n%s -> %s' % ('PASS' if ok else 'FAIL', path))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='relay', choices=['relay', 'evidence', 'sim'])
    ap.add_argument('--run-id', default='20261015-w5-r009-0')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    ap.add_argument('--relay-exe',
                    default='/home/chen/AGC/install/agc_ground/lib/agc_ground/ground_drive_relay')
    ap.add_argument('--params-file',
                    default='/home/chen/AGC/install/agc_ground/share/agc_ground/config/ground.yaml')
    args = ap.parse_args()

    if args.scenario == 'relay':
        return run_relay(args)
    if args.scenario == 'evidence':
        return run_evidence(args)
    return run_sim(args)


if __name__ == '__main__':
    sys.exit(main())
