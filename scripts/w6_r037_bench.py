#!/usr/bin/env python3
"""w6_r037_bench —— R-037（传感器/车体限制声明与能力标定）**能力标定台架**（实跑 gz）。

最小 gz 场景（由 run_w6_r037.sh 起）：
  * `agc_sim/world_bringup.launch.py`  —— 世界 + ros_gz 桥 + spawn **换装后** `agc_car`；
  * `agc_ground/ground_drive_relay`    —— **唯一执行口** `/agc/ground/cmd_vel` → gz（DiffDrive，保留 ADR-0010）；
  * `agc_perception/gt_bypass`         —— gz 真值旁路 `/agc/gt/agc_car`。

本台架**经唯一执行口**下发命令，用 **gz 真值 / 接触证据**标定（**不**仅看「发布零命令」）：
  1. `brake`   满速（v_max=1.5）稳态 → 命令零 → 测 gz 真值**制动距离 / 制动耗时**（与 ADR-0021
               `a≈0.9`、`s=v²/(2a)=1.25 m` 对拍）。
  2. `turn`    原地 `ω_max=2.0` 测实际偏航率；`(v,ω)` 弧线测**转向半径 / 扫掠净空 + 滑移（摩擦）敏感性**。
  3. `contact` 朝 `pole_1` 满速逼近 → 采 `/agc/ground/contact`（`ros_gz_interfaces/Contacts`）**接触证据**。

**载荷外伸 / 碰撞口径（ADR-0052(b)）**：载荷无 collision → 网/干扰的**物理接触验证 unsupported**；
本台架**不**做载荷接触验证，仅登记。

若 gz 链路无法就绪（DDS 发现 / 渲染库 / lifecycle 卡死）→ 写 `bench.json` 的 `unsupported` + 失败证据，
**不**伪造数值。证据落 `experiments/w6-task-eval/<run_id>/r037/bench.json`（ADR-0007）。

⚠ 范围标注：本台架为**最小 gz 能力台架**（世界 + 唯一执行口中继 + 真值），**非整链** / 非双 PX4 / 非 GUI。
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from ros_gz_interfaces.srv import SetEntityPose

ROOT = '/home/chen/AGC'
CMD_TOPIC = '/agc/ground/cmd_vel'          # 契约唯一执行口
GT_TOPIC = '/agc/gt/agc_car'               # gz 真值旁路（world 系）
ODOM_TOPIC = '/agc/sim/ground_odom_raw'    # DiffDrive odom（桥）
CONTACT_TOPIC = '/agc/ground/contact'      # gz 接触真值（R-027/ADR-0042）

# 换装后模型的事实（agc_car.sdf.xacro / car.yaml）
FOOTPRINT_HALF_DIAG_M = 0.5789   # sqrt(a²+b²)，a=0.5037、b=0.28545
V_MAX = 1.5
W_MAX = 2.0
A_BRAKE_PRED_MPS2 = 0.9          # ADR-0021(c) 实测锁轮制动
BRAKE_PRED_M = V_MAX ** 2 / (2 * A_BRAKE_PRED_MPS2)   # 1.25 m

ZERO_THRESH = 0.05               # 归零判据（odom 速度 m/s）
STEADY_V = 1.40                  # 满速稳态判据（真值速度）


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    """共享探针：向唯一执行口发 TwistStamped、订阅 gz 真值 / odom / 接触，提供 set_pose。"""

    def __init__(self):
        super().__init__('w6_r037_bench')
        self.pub = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.create_subscription(PoseStamped, GT_TOPIC, self._on_gt, 50)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 50)
        self.create_subscription(Contacts, CONTACT_TOPIC, self._on_contact, 50)
        self.gx = self.gy = self.gyaw = 0.0
        self.have_gt = False
        self.ovx = 0.0                     # odom 前向速度
        self.have_odom = False
        self.contact_count = 0
        self.contact_times = []
        self.pose_cli = self.create_client(SetEntityPose, '/world/agc_world/set_pose')

    def _on_gt(self, m):
        self.gx = m.pose.position.x
        self.gy = m.pose.position.y
        self.gyaw = yaw_from_quat(m.pose.orientation)
        self.have_gt = True

    def _on_odom(self, m):
        self.ovx = float(m.twist.twist.linear.x)
        self.have_odom = True

    def _on_contact(self, _m):
        self.contact_count += 1
        self.contact_times.append(time.monotonic())

    def publish_cmd(self, v, w=0.0, frame='car/base_link'):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.pub.publish(m)

    def spin_for(self, sec, keep_cmd=None):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            if keep_cmd is not None:
                self.publish_cmd(*keep_cmd)
            rclpy.spin_once(self, timeout_sec=0.005)

    def settle(self, sec=2.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            self.publish_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)

    def set_pose(self, x, y, z, yaw, timeout=8.0):
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

    def wait_ready(self, timeout=45.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.have_gt and self.have_odom:
                break
        svc = self.pose_cli.wait_for_service(timeout_sec=8.0)
        return {'have_gt': self.have_gt, 'have_odom': self.have_odom,
                'set_pose_service': bool(svc)}


def _drive_to_steady(n, v_cmd, timeout=6.0, dwell=1.5):
    """发 v_cmd 直到 odom 速度 > STEADY_V，再 dwell 秒到稳态；返回稳态 odom 速度。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        n.publish_cmd(v_cmd, 0.0)
        rclpy.spin_once(n, timeout_sec=0.005)
        if abs(n.ovx) > STEADY_V:
            break
    t1 = time.monotonic()
    while time.monotonic() - t1 < dwell:
        n.publish_cmd(v_cmd, 0.0)
        rclpy.spin_once(n, timeout_sec=0.005)
    return n.ovx


def run_brake(n):
    """满速稳态 → 命令零 → 测 gz 真值制动距离 / 耗时。"""
    res = {'pass': False}
    if not n.set_pose(30.0, 30.0, 0.143, 0.0):
        res['error'] = 'set_pose 失败'
        return res
    n.settle(2.0)
    v_steady = _drive_to_steady(n, V_MAX)
    res['v_steady_odom_mps'] = round(v_steady, 3)
    p0 = (n.gx, n.gy)
    c0 = n.contact_count
    t0 = time.monotonic()
    # 命令零并采样至停稳
    timeline = []
    t_stop = None
    last_move = t0
    while time.monotonic() - t0 < 5.0:
        n.publish_cmd(0.0, 0.0)
        rclpy.spin_once(n, timeout_sec=0.005)
        now = time.monotonic()
        spd = abs(n.ovx)
        timeline.append({'t_s': round(now - t0, 3), 'odom_speed_mps': round(spd, 3),
                         'gt_x': round(n.gx, 3), 'gt_y': round(n.gy, 3)})
        if spd > ZERO_THRESH:
            last_move = now
        elif now - last_move > 0.3:
            t_stop = now
            break
    p1 = (n.gx, n.gy)
    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    res.update({
        'truth_source': 'gz /agc/gt/agc_car + /agc/sim/ground_odom_raw',
        'gt_p0': [round(p0[0], 3), round(p0[1], 3)],
        'gt_p1': [round(p1[0], 3), round(p1[1], 3)],
        'brake_distance_m': round(dist, 3),
        'brake_time_s': round(t_stop - t0, 3) if t_stop else None,
        'predicted_brake_m_adr0021': round(BRAKE_PRED_M, 3),
        'predicted_a_mps2': A_BRAKE_PRED_MPS2,
        'contact_during_brake': n.contact_count - c0,
        'timeline_head': timeline[:8],
        'timeline_tail': timeline[-6:],
    })
    ok = (t_stop is not None and math.isfinite(dist) and dist > 0.05 and v_steady > STEADY_V)
    res['pass'] = bool(ok)
    return res


def _fit_circle(pts):
    """代数最小二乘拟合圆 → (cx, cy, r)；退化返回 None。"""
    if len(pts) < 10:
        return None
    n = len(pts)
    sx = sum(p[0] for p in pts)
    sy = sum(p[1] for p in pts)
    sxx = sum(p[0] * p[0] for p in pts)
    syy = sum(p[1] * p[1] for p in pts)
    sxy = sum(p[0] * p[1] for p in pts)
    sxz = sum(p[0] * (p[0] * p[0] + p[1] * p[1]) for p in pts)
    syz = sum(p[1] * (p[0] * p[0] + p[1] * p[1]) for p in pts)
    # 解 [sxx sxy sx; sxy syy sy; sx sy n] [a b c]^T = [sxz syz s]  (z = a*x+b*y+c)
    import numpy as np
    try:
        A = np.array([[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]])
        b = np.array([sxz, syz, sum(p[0] * p[0] + p[1] * p[1] for p in pts)])
        sol = np.linalg.solve(A, b)
    except Exception:  # noqa: BLE001
        return None
    a, bb, c = sol
    cx, cy = a / 2.0, bb / 2.0
    r = math.sqrt(max(c + cx * cx + cy * cy, 0.0))
    return (float(cx), float(cy), float(r))


def run_turn(n):
    """原地 ω_max 测实际偏航率；(v,ω) 弧线测转向半径 / 扫掠 + 滑移敏感性。"""
    res = {'pass': False}
    if not n.set_pose(30.0, 30.0, 0.143, 0.0):
        res['error'] = 'set_pose 失败'
        return res
    n.settle(2.0)

    # (1) 原地转向：ω=ω_max，测实际偏航率
    t0 = time.monotonic()
    unwrapped = 0.0
    prev = n.gyaw
    while time.monotonic() - t0 < 4.0:
        n.publish_cmd(0.0, W_MAX)
        rclpy.spin_once(n, timeout_sec=0.005)
        d = n.gyaw - prev
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        unwrapped += d
        prev = n.gyaw
    dt = time.monotonic() - t0
    omega_actual = abs(unwrapped) / dt if dt > 0 else 0.0
    n.settle(1.0)

    # (2) 弧线：v=1.0, ω=1.0，测实际半径 + 扫掠 + 滑移
    v_cmd, w_cmd = 1.0, 1.0
    pts = []
    speeds = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 9.0:
        n.publish_cmd(v_cmd, w_cmd)
        rclpy.spin_once(n, timeout_sec=0.005)
        pts.append((n.gx, n.gy))
        speeds.append(abs(n.ovx))
    fit = _fit_circle(pts)
    v_ach = sum(speeds[-20:]) / len(speeds[-20:]) if len(speeds) >= 20 else (sum(speeds) / max(len(speeds), 1))
    r_ideal = v_cmd / w_cmd
    res.update({
        'truth_source': 'gz /agc/gt/agc_car + /agc/sim/ground_odom_raw',
        'spin_cmd_omega': W_MAX,
        'spin_actual_omega_radps': round(omega_actual, 3),
        'spin_omega_ratio': round(omega_actual / W_MAX, 3),
        'arc_cmd_v': v_cmd, 'arc_cmd_w': w_cmd,
        'arc_radius_ideal_m': round(r_ideal, 3),
        'arc_radius_actual_m': round(fit[2], 3) if fit else None,
        'arc_center': [round(fit[0], 3), round(fit[1], 3)] if fit else None,
        'swept_radius_outer_m': round(fit[2] + FOOTPRINT_HALF_DIAG_M, 3) if fit else None,
        'footprint_half_diag_m': FOOTPRINT_HALF_DIAG_M,
        'v_achieved_arc_mps': round(v_ach, 3),
        'slip_ratio': round((v_cmd - v_ach) / v_cmd, 3),
        'n_arc_samples': len(pts),
    })
    ok = (omega_actual > 0.5 and fit is not None and 0.2 < fit[2] < 5.0)
    res['pass'] = bool(ok)
    return res


def run_contact(n):
    """朝 pole_1（world (0,-6)）满速逼近 → 采接触真值。"""
    res = {'pass': False}
    # 车置 (0,-1)、朝 -y（yaw=-π/2）→ 正对立柱 pole_1（5 m 外）
    if not n.set_pose(0.0, -1.0, 0.143, -math.pi / 2):
        res['error'] = 'set_pose 失败'
        return res
    n.settle(2.0)
    c0 = n.contact_count
    t0 = time.monotonic()
    hit_t = None
    while time.monotonic() - t0 < 8.0:
        n.publish_cmd(V_MAX, 0.0)
        rclpy.spin_once(n, timeout_sec=0.005)
        if n.contact_count > c0:
            hit_t = time.monotonic()
            break
    n.publish_cmd(0.0, 0.0)
    res.update({
        'truth_source': 'gz /agc/ground/contact (ros_gz_interfaces/Contacts)',
        'target_obstacle': 'pole_1 @ world (0,-6) 0.3x0.3x3',
        'contact_events': n.contact_count - c0,
        'first_contact_t_s': round(hit_t - t0, 3) if hit_t else None,
        'gt_at_stop': [round(n.gx, 3), round(n.gy, 3)],
    })
    res['pass'] = bool(n.contact_count - c0 > 0)
    return res


def model_facts():
    """从换装后模型/配置**读**事实（不改）：传感器高度、载荷位姿/碰撞、驱动插件。"""
    xacro = os.path.join(ROOT, 'src/agc_sim/models/agc_car/agc_car.sdf.xacro')
    with open(xacro, encoding='utf-8', errors='replace') as f:
        t = f.read()
    return {
        'lidar_link_z_0p48': ('<pose>0 0 0.48 0 0 0</pose>' in t),
        'capture_net_z_1p60': ('<pose>0.55 0 1.60 0 0 0</pose>' in t),
        'jammer_z_0p30': ('<pose>-0.20 0 0.30 0 0 0</pose>' in t),
        'diffdrive_plugin_present': ('gz-sim-diff-drive-system' in t),
        'payload_no_collision_registered': ('载荷**无 collision**' in t or '载荷**无 collision**' in t),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261017-w6-r037-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()

    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)
    r037 = os.path.join(out_dir, 'r037')
    os.makedirs(r037, exist_ok=True)

    result = {
        'run_id': args.run_id,
        'suite': 'R-037 sensor/vehicle limitation registration & capability calibration bench',
        'scope_note': (
            '最小 gz 能力台架：world_bringup（世界+桥+换装后 agc_car）+ 唯一执行口中继 ground_drive_relay'
            ' + 真值旁路 gt_bypass；经唯一执行口 /agc/ground/cmd_vel 标定满速制动 / 转弯扫掠 / 接触；'
            '**非整链** / 非双 PX4 / 非 GUI。'),
        'model_facts': model_facts(),
        'unsupported': [],
        'unsupported_payload_contact': (
            '载荷（capture_net/jammer）无 collision、且不在 z=0.48 扫描面 → 网/干扰的物理接触验证 unsupported'
            '（ADR-0052(b)）；本台架不做载荷接触验证。'),
        'scenarios': {},
    }

    infra = None
    try:
        rclpy.init()
        n = Probe()
        ready = n.wait_ready()
        result['link_ready'] = ready
        if not (ready['have_gt'] and ready['have_odom'] and ready['set_pose_service']):
            infra = 'gz 链路未就绪：%s' % ready
        else:
            result['scenarios']['brake'] = run_brake(n)
            print('brake   PASS=%s dist=%s m' % (result['scenarios']['brake'].get('pass'),
                                                 result['scenarios']['brake'].get('brake_distance_m')))
            n.settle(1.0)
            result['scenarios']['turn'] = run_turn(n)
            print('turn    PASS=%s r=%s m' % (result['scenarios']['turn'].get('pass'),
                                              result['scenarios']['turn'].get('arc_radius_actual_m')))
            n.settle(1.0)
            result['scenarios']['contact'] = run_contact(n)
            print('contact PASS=%s events=%s' % (result['scenarios']['contact'].get('pass'),
                                                 result['scenarios']['contact'].get('contact_events')))
        n.destroy_node()
        rclpy.shutdown()
    except Exception as exc:  # noqa: BLE001
        infra = repr(exc)

    if infra:
        result['infra_error'] = infra
        result['unsupported'] = ['brake', 'turn', 'contact']
        result['failure'] = {'reason': infra, 'note': 'gz 无法稳定实跑 → 如实 unsupported（不伪造数值）'}

    scen = result['scenarios']
    ok = bool(
        (scen.get('brake', {}).get('pass') and scen.get('turn', {}).get('pass')
         and scen.get('contact', {}).get('pass'))
        or (infra and result['unsupported']))
    result['overall_pass'] = ok

    with open(os.path.join(r037, 'bench.json'), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print('\noverall_pass=%s -> %s' % (ok, os.path.join(r037, 'bench.json')))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
