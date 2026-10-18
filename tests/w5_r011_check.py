#!/usr/bin/env python3
"""w5_r011_check —— W5 R-011 动作方向闭环 + C6 逃逸能力标定验收。

在 `agc_task w5_safety_scenario.launch.py`（GUI，硬约束 C3）运行中执行：先杀掉 RL 命令源，改由本脚本
作「脚本化命令源」直接发 `/agc/ground/cmd_vel_raw`（与 R-008 同法），命令**取修复后的**
`interface.action_to_twist` 输出（车前净空置 0 m → `magnitude = max_offset` 的满偏左 / 右转），
经安全层（collision_monitor）+ 唯一执行中继（ground_drive_relay）到 GZ，测车**真实**横向逃逸能力。

子测：
  D 方向闭环：`left`/`left_fast` → 车世界 yaw **增加**（左转，正 `angular.z`）；`right`/`right_fast`
              → yaw **减小**（右转）。记录 odom yaw / 横向 y 的符号，证明 ROS 约定端到端成立。
  E C6 逃逸标定：各动作在**反应窗口 T**（s）内的横向位移（米）；`max(|lateral|) ≥ 配置 max_offset`
              才 PASS，否则记录可达值供下调 `max_offset`（契约 C6：参数须用真实逃逸能力）。

标定口径（先学后做，ADR-0021(b)）：反应窗口 `T = 2.0 s` 取 Nav2 `time_before_collision` 上界
（t_before ∈ [1.2, 2.0] s）。理论横向界（常 yaw 率圆弧）：lateral(T) ≈ (v/ω)·(1 − cos(ω·T))。
车端限幅：GZ DiffDrive `max_angular_velocity = 2.0`；RL 节点另限 `max_yaw_rate = 1.0`（本测 w=0.96 < 1.0 未截）。

结果落 experiments/w5-ground-nav-safety/<run_id>/metrics/r011.json。
"""

import argparse
import json
import math
import os
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose

from agc_rl import interface

RAW = '/agc/ground/cmd_vel_raw'
OUT = '/agc/ground/cmd_vel'
ODOM = '/agc/ground/odom'

# 反应窗口（s）：ADR-0021(b) Nav2 time_before_collision 上界（t_before ∈ [1.2, 2.0]）
T_WINDOW = 2.0
# 开阔区（远离 pole_1 (0,-6) / pole_2 (6,6)，正前方无遮挡 → 安全层不干预）
OPEN = (20.0, 25.0)
MIN_YAW_DELTA = 0.20      # 方向可判的最小 yaw 变化（rad）
MIN_LATERAL = 0.10        # 方向可判的最小横向位移（m）
REACH_TOL = 0.05          # 可达判据容差（m）

RUNS = [('left', 1), ('right', 2), ('left_fast', 3), ('right_fast', 4)]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def load_control(params_file):
    import yaml
    with open(params_file) as fh:
        d = yaml.safe_load(fh)
    return d['ground_policy_rl']['ros__parameters']['control']


class R011(Node):
    def __init__(self):
        super().__init__('w5_r011_check')
        self.cmd_pub = self.create_publisher(TwistStamped, RAW, 10)
        self.x = self.y = self.yaw = 0.0
        self.ovx = self.ovy = 0.0
        self.v_out = 0.0
        self.w_out = 0.0
        self.n_out = 0
        self.create_subscription(Odometry, ODOM, self._on_odom, 50)
        self.create_subscription(TwistStamped, OUT, self._on_out, 50)
        self.pose_cli = self.create_client(SetEntityPose, '/world/agc_world/set_pose')
        self.state_cli = self.create_client(GetState, '/collision_monitor/get_state')

    def _on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_from_quat(m.pose.pose.orientation)
        self.ovx = m.twist.twist.linear.x
        self.ovy = m.twist.twist.linear.y

    def _on_out(self, m):
        self.v_out = m.twist.linear.x
        self.w_out = m.twist.angular.z
        self.n_out += 1

    def odom_speed(self):
        return math.hypot(self.ovx, self.ovy)

    def publish_cmd(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'car/base_link'
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.cmd_pub.publish(m)

    def spin(self, seconds, v=0.0, w=0.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.publish_cmd(v, w)
            rclpy.spin_once(self, timeout_sec=0.005)

    def settle(self, seconds=1.5):
        self.spin(seconds, 0.0, 0.0)

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

    def set_pose(self, x, y, yaw, z=0.02, timeout=6.0):
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


def run_one(n, name, action, control):
    """一次满偏转向：沿 +x 达稳态速 → 持续给 action_to_twist 输出 T 秒 → 测横向 / yaw。"""
    v_cmd, w_cmd = interface.action_to_twist(action, 0.0, 0.0, control)  # 净空 0 → 满偏（magnitude=max_offset）
    omega_cmd = abs(w_cmd)
    radius_theory = v_cmd / omega_cmd if omega_cmd > 1e-9 else float('inf')
    lateral_theory = radius_theory * (1.0 - math.cos(omega_cmd * T_WINDOW)) if omega_cmd > 1e-9 else 0.0

    n.set_pose(OPEN[0], OPEN[1], 0.0)   # 正对 +x（yaw=0）
    n.settle(1.5)

    # 沿 +x 达稳态速：目标 = 该动作的线速 v_cmd
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0 and n.odom_speed() < 0.9 * v_cmd:
        n.spin(0.02, v_cmd, 0.0)
    v_steady = n.odom_speed()

    # 持续满偏转向 T 秒
    x0, y0, yaw0 = n.x, n.y, n.yaw
    n.spin(T_WINDOW, v_cmd, w_cmd)
    x1, y1, yaw1 = n.x, n.y, n.yaw

    lateral = y1 - y0                 # 世界 y 位移（yaw0≈0 时即车体系横向，+ = 左）
    forward = x1 - x0
    yaw_delta = wrap_pi(yaw1 - yaw0)
    omega_meas = yaw_delta / T_WINDOW
    radius_meas = (abs(v_steady) / abs(omega_meas)) if abs(omega_meas) > 1e-9 else None

    n.settle(1.5)
    return {
        'action': action,
        'cmd_v_mps': round(v_cmd, 4),
        'cmd_w_radps': round(w_cmd, 4),
        'omega_cmd_clamped': bool(omega_cmd > control['max_yaw_rate'] + 1e-9),
        'v_steady_mps': round(v_steady, 4),
        'yaw_delta_rad': round(yaw_delta, 4),
        'omega_meas_radps': round(omega_meas, 4),
        'turn_radius_meas_m': round(radius_meas, 4) if radius_meas else None,
        'turn_radius_theory_m': round(radius_theory, 4),
        'lateral_disp_m': round(lateral, 4),
        'lateral_reach_m': round(abs(lateral), 4),
        'lateral_theory_m': round(lateral_theory, 4),
        'forward_disp_m': round(forward, 4),
        'window_s': T_WINDOW,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261015-w5-r011-0')
    ap.add_argument('--mode', default='gui')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    ap.add_argument('--params-file',
                    default='/home/chen/AGC/install/agc_rl/share/agc_rl/config/rl.yaml')
    args = ap.parse_args()

    rclpy.init()
    n = R011()
    control = load_control(args.params_file)
    res = {'run_id': args.run_id, 'mode': args.mode, 'runs': {}, 'calibration': {}, 'runs_order': []}

    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0 and n.n_out == 0:
            rclpy.spin_once(n, timeout_sec=0.05)
        active, amsg = n.wait_active()
        res['collision_monitor_active'] = active
        res['collision_monitor_state_msg'] = amsg
        if not active:
            res['error'] = 'infra: %s' % amsg
            return dump(res, args, infra=True)

        for name, action in RUNS:
            res['runs'][name] = run_one(n, name, action, control)
            res['runs_order'].append(name)
            r = res['runs'][name]
            left = name.startswith('left')
            r['direction_ok'] = bool(
                (r['yaw_delta_rad'] > MIN_YAW_DELTA and r['lateral_disp_m'] > MIN_LATERAL) if left
                else (r['yaw_delta_rad'] < -MIN_YAW_DELTA and r['lateral_disp_m'] < -MIN_LATERAL))
            r['reach_ok'] = bool(r['lateral_reach_m'] >= control['max_offset'] - REACH_TOL)
            print('%-11s v=%.2f w=%+.2f | yawΔ=%+.3f lateral=%+.3f m (reach %.3f) dir=%s'
                  % (name, r['cmd_v_mps'], r['cmd_w_radps'], r['yaw_delta_rad'],
                     r['lateral_disp_m'], r['lateral_reach_m'], r['direction_ok']))

        reachable = max((r['lateral_reach_m'] for r in res['runs'].values()), default=0.0)
        res['calibration'] = {
            'reaction_window_T_s': T_WINDOW,
            'window_basis': 'Nav2 time_before_collision 上界（ADR-0021(b) t_before∈[1.2,2.0]）',
            'open_area': list(OPEN),
            'configured_min_offset_m': control['min_offset'],
            'configured_max_offset_m': control['max_offset'],
            'max_yaw_rate_cfg_radps': control['max_yaw_rate'],
            'car_max_angular_radps': 2.0,
            'car_max_linear_mps': 1.5,
            'reachable_max_lateral_m': round(reachable, 4),
            'config_max_offset_reachable': bool(reachable >= control['max_offset'] - REACH_TOL),
            'recommended_max_offset_m': (round(reachable, 3) if reachable < control['max_offset'] - REACH_TOL
                                         else control['max_offset']),
        }
        res['direction_pass'] = all(r['direction_ok'] for r in res['runs'].values())
        res['c6_pass'] = bool(res['calibration']['config_max_offset_reachable'])
        res['overall_pass'] = bool(res['direction_pass'] and res['c6_pass'])
        return dump(res, args)
    finally:
        n.destroy_node()
        rclpy.shutdown()


def dump(res, args, infra=False):
    out_dir = os.path.join(args.experiments_root, 'w5-ground-nav-safety', args.run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'r011.json')
    with open(path, 'w') as fh:
        json.dump(res, fh, indent=2, ensure_ascii=False)
    if 'error' in res:
        print('error:', res['error'])
        print('-> %s' % path)
        return 2 if infra else 1
    print('calibration:', json.dumps(res.get('calibration'), ensure_ascii=False))
    print('direction_pass=%s c6_pass=%s' % (res.get('direction_pass'), res.get('c6_pass')))
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


if __name__ == '__main__':
    import sys
    sys.exit(main())
