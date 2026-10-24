#!/usr/bin/env python3
"""w7b_p2_replan_bench —— P2 臂 B「重规划触发」**受控台架**（ADR-0055 / ADR-0056 审计）。

定位：**诊断台架**（非确证、不改 P1/P2 verdict）。起真实二进制：
  * `install/agc_ground/lib/agc_ground/ground_baseline`（Navfn + 自研 LQR + 内建 Costmap2DROS）；
  * C6 另起 `install/agc_rl/lib/agc_rl/agc_rl_high_level`（20 Hz 拦截点，低通 `target_tau_s=0.5`、
    `lead_s=1.0`、`max_lead_m=1.0`），复现臂 B 的 goal 流；
  * 台架自持 `static_transform_publisher` + 合成 `/agc/map`（latched）/ `/agc/ground/odom` /
    `/agc/task/state` / `/agc/ground/goal_info`（对齐 R-012 / R-041 基线台架做法）。

用例（逐原因验证既有规则，**不**以 A 臂 0 次重规划为验收目标）：
  C1 固定目标 + 固定地图 + 固定 goal → 期望 0 次「目标身份变」、0 次「地图变化」（仅周期刷新）
  C2 goal 单次阶跃 1.2 m → 期望恰好 1 次「目标身份变」（限流 kReplanGuardS=0.2）
  C3 goal 微扰 0.3 m @20 Hz → 期望 0 次「目标身份变」
  C4 地图真实增删一个硬障碍 → 期望恰好 1 次「地图变化」
  C5 地图自由/未知翻转（-1 ↔ 0）→ 期望 0 次「地图变化」（低于硬障碍签名）
  C6 目标 twist 微扰（模拟 air_fusion 噪声）经 high_level_node → 记录 goal 抖动与触发；逐条判伪/真实

产物：`experiments/w7-rl-b/<run_id>/metrics/p2_replan_bench.{json,md}` 与 `log/`、`config/`。
清理只回收**本进程组 / 本台架二进制**（`start_new_session` + `killpg`），**不**用裸名 `pkill -f`。
环境：`ROS_DOMAIN_ID=0`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`（见报告 / runner）。
"""

import argparse
import json
import math
import struct
import os
import re
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from agc_msgs.msg import AgcTaskState, TargetTrack, TargetTrackArray
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from rcl_interfaces.msg import Log
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

ROOT = '/home/chen/AGC'
GROUND_BIN = os.path.join(ROOT, 'install', 'agc_ground', 'lib', 'agc_ground')
RL_BIN = os.path.join(ROOT, 'install', 'agc_rl', 'lib', 'agc_rl')

GOAL_TOPIC = '/agc/ground/goal'
GOAL_HL_TOPIC = '/agc/ground/goal_hl'
GOAL_INFO_TOPIC = '/agc/ground/goal_info'
TARGET_TOPIC = '/agc/targets'
ODOM_TOPIC = '/agc/ground/odom'
MAP_TOPIC = '/agc/map'
PATH_TOPIC = '/agc/ground/path'
CMD_RAW_TOPIC = '/agc/ground/cmd_vel_raw'
STATE_TOPIC = '/agc/task/state'

TARGET_XY = (4.5, 3.0)
FIXED_GOAL = (4.5, 3.0)
GROUND_APPROACH = 3
REPLAN_DISP = 1.0
MAP_RES = 0.5
MAP_W = 160
MAP_H = 160
MAP_OX = -40.0
MAP_OY = -40.0

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

# /rosout 文本 → 触发分类 + 诊断量
TRIG_GOAL = '重规划触发：目标身份变'
TRIG_MAP = '重规划触发：地图变化'
TRIG_AGE = '重规划触发：路径超有效时限'


# ------------------------------------------------------------------ 进程工具

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


def _reap_stale(patterns):
    """只回收本台架真实二进制（完整安装路径 + 方括号防自匹配），不做递归删除。"""
    for pat in patterns:
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
    time.sleep(0.3)


REAP_PATTERNS = [
    'install/agc_ground/lib/agc_ground/ground_baselin[e]',
    'install/agc_rl/lib/agc_rl/agc_rl_high_leve[l]',
    'tf2_ros/static_transform_publishe[r]',
]


# ------------------------------------------------------------------ 台架节点

class Bench(Node):
    def __init__(self, name, goal_topic=GOAL_TOPIC):
        super().__init__('w7b_p2_bench_' + name)
        self.x = self.y = 0.0
        self.yaw = 0.0
        self.t_origin = None           # 目标发布起始时刻（C6 twist 相位）
        self.paths = []                # {'wall','n','end_x','end_y'}
        self.cmds = []
        self.logs = []                 # {'wall','text'}
        self.hl_goals = []             # {'wall','x','y'}
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.info_pub = self.create_publisher(String, GOAL_INFO_TOPIC, LATCHED)
        self.state_pub = self.create_publisher(AgcTaskState, STATE_TOPIC, 10)
        self.odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 20)
        self.map_pub = self.create_publisher(OccupancyGrid, MAP_TOPIC, LATCHED)
        self.target_pub = self.create_publisher(TargetTrackArray, TARGET_TOPIC, 20)
        # ADR-0092 C7/C8：合成动态层 marking（scan / pointcloud）；其他用例不发布。
        self.scan_pub = self.create_publisher(LaserScan, '/agc/ground/scan_filtered', 10)
        self.points_pub = self.create_publisher(PointCloud2, '/agc/sim/car_points', 10)
        # 反复 latched 发布 world→car/base_link 静态 TF，确定性覆盖 costmap 激活的 TF 发现窗口。
        self.tf_pub = self.create_publisher(TFMessage, '/tf_static', LATCHED)
        self.create_subscription(Path, PATH_TOPIC, self._on_path, 10)
        self.create_subscription(TwistStamped, CMD_RAW_TOPIC, self._on_cmd, 50)
        self.create_subscription(Log, '/rosout', self._on_log, 100)
        self.create_subscription(PoseStamped, GOAL_HL_TOPIC, self._on_hl_goal, 20)
        self.chg = self.create_client(ChangeState, '/ground_baseline/change_state')
        self.getst = self.create_client(GetState, '/ground_baseline/get_state')

    # ---- 订阅回调
    def _on_path(self, m):
        end = m.poses[-1].pose.position if m.poses else None
        self.paths.append({'wall': time.time(), 'n': len(m.poses),
                           'end_x': (end.x if end else float('nan')),
                           'end_y': (end.y if end else float('nan'))})

    def _on_cmd(self, m):
        self.cmds.append({'wall': time.time(), 'lin': m.twist.linear.x, 'ang': m.twist.angular.z})

    def _on_log(self, m):
        if m.name == 'ground_baseline':
            self.logs.append({'wall': time.time(), 'text': m.msg})

    def _on_hl_goal(self, m):
        self.hl_goals.append({'wall': time.time(),
                              'x': m.pose.position.x, 'y': m.pose.position.y})

    # ---- 合成输入
    def make_map(self, occupied_idx=None, unknown_idx=None):
        m = OccupancyGrid()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = MAP_RES
        m.info.width = MAP_W
        m.info.height = MAP_H
        m.info.origin.position.x = MAP_OX
        m.info.origin.position.y = MAP_OY
        m.info.origin.orientation.w = 1.0
        data = [0] * (MAP_W * MAP_H)
        for i in (occupied_idx or []):
            if 0 <= i < len(data):
                data[i] = 100
        for i in (unknown_idx or []):
            if 0 <= i < len(data):
                data[i] = -1
        m.data = data
        return m

    def pub_map(self, **kw):
        self.map_pub.publish(self.make_map(**kw))

    def pub_tf(self):
        t = TransformStamped()
        t.header.frame_id = 'world'
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = 'car/base_link'
        t.transform.rotation.w = 1.0
        msg = TFMessage()
        msg.transforms.append(t)
        self.tf_pub.publish(msg)

    def pub_odom(self):
        o = Odometry()
        o.header.frame_id = 'world'
        o.child_frame_id = 'car/base_link'
        o.header.stamp = self.get_clock().now().to_msg()
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        o.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        self.odom_pub.publish(o)

    def pub_state(self):
        s = AgcTaskState()
        s.header.frame_id = 'world'
        s.header.stamp = self.get_clock().now().to_msg()
        s.state = GROUND_APPROACH
        s.state_name = 'GROUND_APPROACH'
        self.state_pub.publish(s)

    def pub_dynamic_marking(self, x, y, z=2.0):
        """ADR-0092 C7/C8：在 (x,y,z) 发布一帧合成 scan + pointcloud（移动目标样式）。

        scan 用 car/base_link 坐标系，只在一个方位角给出距离；pointcloud 用 world 系给一个 3D 点。
        不改变任何话题 / 参数，仅用于验证 dynamic 层 marking 不应触发地图签名。
        """
        scan = LaserScan()
        scan.header.frame_id = 'car/base_link'
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / 360.0
        scan.range_min = 0.05
        scan.range_max = 20.0
        n = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1
        scan.ranges = [float('inf')] * n
        bearing = math.atan2(y - self.y, x - self.x)
        rng = math.hypot(x - self.x, y - self.y)
        idx = int(round((bearing - scan.angle_min) / scan.angle_increment))
        if 0 <= idx < n:
            scan.ranges[idx] = max(scan.range_min, min(scan.range_max, rng))
        self.scan_pub.publish(scan)

        pc = PointCloud2()
        pc.header.frame_id = 'world'
        pc.header.stamp = scan.header.stamp
        pc.height = 1
        pc.width = 1
        pc.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc.is_bigendian = False
        pc.point_step = 12
        pc.row_step = 12
        pc.is_dense = True
        pc.data = struct.pack('fff', float(x), float(y), float(z))
        self.points_pub.publish(pc)

    def pub_goal(self, x, y):
        g = PoseStamped()
        g.header.frame_id = 'world'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    def pub_info(self):
        m = String()
        # 有效性保持 true（本台架只审计重规划触发，不审计失效策略）。
        m.data = 'valid=1;reason=TARGET_FRESH;target_id=intruder_1;src_stamp=-1.000000;age=-1.000000'
        self.info_pub.publish(m)

    def pub_target(self, vx, vy, conf=0.9):
        msg = TargetTrackArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        tr = TargetTrack()
        tr.header.stamp = msg.header.stamp
        tr.header.frame_id = 'world'
        tr.target_id = 'intruder_1'
        tr.pose.pose.position.x = TARGET_XY[0]
        tr.pose.pose.position.y = TARGET_XY[1]
        tr.pose.pose.orientation.w = 1.0
        tr.twist.twist.linear.x = vx
        tr.twist.twist.linear.y = vy
        tr.confidence = conf
        msg.tracks.append(tr)
        self.target_pub.publish(msg)

    # ---- 驱动
    def pump(self, duration, hz=20.0, goal=None, map_kw=None, target=None):
        """发布基础流；target=(vx,vy) 时同时按 20 Hz 发布 /agc/targets。"""
        t0 = time.time()
        last = 0.0
        period = 1.0 / hz
        while True:
            el = time.time() - t0
            if el >= duration:
                break
            if time.time() - last >= period:
                last = time.time()
                self.pub_map(**(map_kw or {}))
                self.pub_odom(); self.pub_state(); self.pub_tf(); self.pub_info()
                if goal is not None:
                    self.pub_goal(*goal)
                if target is not None:
                    self.pub_target(target[0], target[1])
            rclpy.spin_once(self, timeout_sec=0.005)

    def pump_fn(self, duration, fn, hz=20.0):
        """基础流 + 每拍调用 fn(el)（fn 内自行发布 goal / map / target）。"""
        t0 = time.time()
        last = 0.0
        period = 1.0 / hz
        while True:
            el = time.time() - t0
            if el >= duration:
                break
            if time.time() - last >= period:
                last = time.time()
                self.pub_odom(); self.pub_state(); self.pub_tf(); self.pub_info()
                fn(el)
            rclpy.spin_once(self, timeout_sec=0.005)

    def spin_until(self, cond, timeout, hz=20.0, goal=None, map_kw=None, target=None):
        t0 = time.time()
        last = 0.0
        period = 1.0 / hz
        while True:
            if time.time() - t0 >= timeout:
                return False
            if time.time() - last >= period:
                last = time.time()
                self.pub_map(**(map_kw or {}))
                self.pub_odom(); self.pub_state(); self.pub_tf(); self.pub_info()
                if goal is not None:
                    self.pub_goal(*goal)
                if target is not None:
                    self.pub_target(target[0], target[1])
            rclpy.spin_once(self, timeout_sec=0.005)
            if cond():
                return True

    # ---- lifecycle
    def _wait_service(self, client, timeout):
        """边 spin 边等发现：rclpy 的图事件需 executor 处理，纯 wait_for_service 在本环境会假超时。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if client.service_is_ready():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return client.service_is_ready()

    def transition(self, tid, timeout=20.0):
        if not self._wait_service(self.chg, timeout):
            return False
        req = ChangeState.Request()
        req.transition.id = tid
        fut = self.chg.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.done() and fut.result() is not None and fut.result().success)

    def state_id(self):
        if not self._wait_service(self.getst, 5.0):
            return None
        fut = self.getst.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done() and fut.result() is not None:
            return int(fut.result().current_state.id)
        return None

    def ensure_active(self, attempts=3):
        detail = {'configure_ok': False, 'activate_ok': False, 'activations': []}
        for i in range(attempts):
            self.pump(1.5)
            st = self.state_id()
            detail['activations'].append({'attempt': i, 'pre_state': st})
            if st == 3:
                detail['configure_ok'] = detail['activate_ok'] = True
                return True, detail
            if st != 2:
                detail['configure_ok'] = bool(self.transition(1))
            ok = self.transition(3, timeout=40.0)
            detail['activations'][-1]['activate_success'] = bool(ok)
            if ok:
                detail['activate_ok'] = True
                return True, detail
            time.sleep(0.5)
        return False, detail


# ------------------------------------------------------------------ 参数 / 进程

def _write(path, text):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def _ground_params_text(goal_topic, replan_disp=REPLAN_DISP):
    return (
        'ground_baseline:\n  ros__parameters:\n'
        '    use_sim_time: false\n'
        '    base_frame_id: "car/base_link"\n'
        '    global_frame_id: "world"\n'
        '    odom_topic: "%s"\n'
        '    goal_topic: "%s"\n'
        '    goal_info_topic: "%s"\n'
        '    state_topic: "%s"\n'
        '    cmd_raw_topic: "%s"\n'
        '    path_topic: "%s"\n'
        '    rate_hz: 20.0\n'
        '    planner_plugin: "nav2_navfn_planner::NavfnPlanner"\n'
        '    planner_id: "GridBased"\n'
        '    replan_period_s: 2.0\n'
        '    replan_displacement_m: %.3f\n'
        '    max_path_age_s: 5.0\n'
        '    goal_reached_tol_m: 0.3\n'
        '    lookahead_m: 1.0\n'
        '    k_yaw: 1.5\n'
        '    reference_speed: 1.0\n'
        '    max_linear_velocity: 1.5\n'
        '    max_angular_velocity: 2.0\n'
        '    stop_on_invalid: True\n'
        '    lqr:\n'
        '      q_pos: 4.0\n      q_vel: 4.0\n      r: 1.0\n      dt: 0.05\n      acc_max: 4.0\n'
        'costmap:\n  ros__parameters:\n'
        '    use_sim_time: false\n'
        '    update_frequency: 5.0\n'
        '    publish_frequency: 2.0\n'
        '    global_frame: world\n'
        '    robot_base_frame: car/base_link\n'
        '    rolling_window: false\n'
        '    width: 40\n    height: 40\n    resolution: 0.25\n    robot_radius: 0.6\n'
        '    track_unknown_space: true\n'
        '    plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]\n'
        '    static_layer:\n'
        '      plugin: "nav2_costmap_2d::StaticLayer"\n'
        '      map_subscribe_transient_local: True\n'
        '      map_topic: "%s"\n'
        '    obstacle_layer:\n'
        '      plugin: "nav2_costmap_2d::ObstacleLayer"\n'
        '      enabled: True\n'
        '      observation_sources: scan\n'
        '      scan:\n'
        '        topic: /agc/ground/scan_filtered\n'
        '        max_obstacle_height: 2.5\n        clearing: True\n        marking: True\n'
        '        data_type: "LaserScan"\n'
        '        raytrace_max_range: 20.0\n        raytrace_min_range: 0.0\n'
        '        obstacle_max_range: 15.0\n        obstacle_min_range: 0.0\n'
        '    voxel_layer:\n'
        '      plugin: "nav2_costmap_2d::VoxelLayer"\n'
        '      enabled: True\n      publish_voxel_map: True\n      origin_z: 0.0\n'
        '      z_resolution: 0.25\n      z_voxels: 16\n      max_obstacle_height: 2.5\n'
        '      min_obstacle_height: 0.30\n      mark_threshold: 0\n'
        '      observation_sources: pointcloud\n'
        '      pointcloud:\n'
        '        topic: /agc/sim/car_points\n        data_type: "PointCloud2"\n'
        '        max_obstacle_height: 2.5\n        min_obstacle_height: 0.30\n'
        '        clearing: True\n        marking: True\n'
        '        raytrace_max_range: 20.0\n        raytrace_min_range: 0.0\n'
        '        obstacle_max_range: 15.0\n        obstacle_min_range: 0.0\n'
        '    inflation_layer:\n'
        '      plugin: "nav2_costmap_2d::InflationLayer"\n'
        '      cost_scaling_factor: 3.0\n      inflation_radius: 1.0\n'
        '    always_send_full_costmap: True\n'
        % (ODOM_TOPIC, goal_topic, GOAL_INFO_TOPIC, STATE_TOPIC, CMD_RAW_TOPIC, PATH_TOPIC,
           replan_disp, MAP_TOPIC)
    )


def _hl_params_text():
    return (
        'agc_rl_high_level:\n  ros__parameters:\n'
        '    use_sim_time: false\n'
        '    goal_topic: "%s"\n'
        '    odom_topic: "%s"\n'
        '    targets_topic: "%s"\n'
        '    state_topic: "%s"\n'
        '    rate_hz: 20.0\n'
        '    lead_s: 1.0\n'
        '    target_max_age_s: 0.5\n'
        '    confidence_min: 0.5\n'
        '    target_tau_s: 0.5\n'
        '    target_gap_reset_s: 1.0\n'
        '    max_lead_m: 1.0\n' % (GOAL_HL_TOPIC, ODOM_TOPIC, TARGET_TOPIC, STATE_TOPIC)
    )


def _start_ground(log_dir, tag, params):
    tag = _tagged(tag)
    log = open(os.path.join(log_dir, tag + '_ground.log'), 'w')
    proc = subprocess.Popen([os.path.join(GROUND_BIN, 'ground_baseline'), '--ros-args',
                             '--params-file', params],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return proc, log


def _start_high_level(log_dir, tag, params):
    tag = _tagged(tag)
    log = open(os.path.join(log_dir, tag + '_high_level.log'), 'w')
    proc = subprocess.Popen([os.path.join(RL_BIN, 'agc_rl_high_level'), '--ros-args',
                             '--params-file', params],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return proc, log


# ------------------------------------------------------------------ 触发统计

def parse_triggers(logs):
    """从 ground_baseline /rosout 文本提取触发次数与诊断分量。"""
    out = {'goal': 0, 'map': 0, 'age': 0, 'goal_logs': [], 'map_logs': [], 'age_logs': []}
    for l in logs:
        t = l['text']
        if TRIG_GOAL in t:
            out['goal'] += 1
            out['goal_logs'].append(_goal_fields(t))
        elif TRIG_MAP in t:
            out['map'] += 1
            out['map_logs'].append(_map_fields(t))
        elif TRIG_AGE in t:
            out['age'] += 1
            out['age_logs'].append(t)
    return out


def _f(pattern, text, cast=float, default=None):
    m = re.search(pattern, text)
    if not m:
        return default
    try:
        return cast(m.group(1))
    except (ValueError, IndexError):
        return default


def _goal_fields(t):
    return {
        'raw': t,
        'goal_delta_m': _f(r'goal_delta_m=([-\d.]+)', t),
        'threshold': _f(r'threshold=([-\d.]+)', t),
        'last_plan_x': _f(r'last_plan_goal=\(([-\d.]+)', t),
        'last_plan_y': _f(r'last_plan_goal=\([-\d.]+,([-\d.]+)\)', t),
        # `\b` 避免匹配到 `last_plan_goal=(...)` 中的子串（二者相邻）。
        'goal_x': _f(r'\bgoal=\(([-\d.]+)', t),
        'goal_y': _f(r'\bgoal=\([-\d.]+,([-\d.]+)\)', t),
    }


def _map_fields(t):
    layers = _f(r'layers=\[([^\]]*)\]', t, cast=str, default='')
    return {
        'raw': t,
        'map_sig': _f(r'map_sig=(0x[0-9a-f]+)', t, cast=str),
        'last_plan_map_sig': _f(r'last_plan_map_sig=(0x[0-9a-f]+)', t, cast=str),
        'sx': _f(r'sx=(\d+)', t, cast=int),
        'sy': _f(r'sy=(\d+)', t, cast=int),
        'origin_x': _f(r'origin=\(([-\d.]+)', t),
        'origin_y': _f(r'origin=\([-\d.]+,([-\d.]+)\)', t),
        'hard_cells': _f(r'hard_cells=(\d+)', t, cast=int),
        'layers': layers,
    }


def _window(logs, t0, t1):
    return [l for l in logs if t0 <= l['wall'] <= t1]


def _case_result(name, expectation, t0, t1, h, extra=None, note=''):
    logs = _window(h.logs, t0, t1)
    trg = parse_triggers(logs)
    res = {
        'case': name,
        'expectation': expectation,
        'window_s': round(t1 - t0, 3),
        'triggers': {'goal_identity': trg['goal'], 'map_change': trg['map'],
                     'path_age': trg['age'], 'total': trg['goal'] + trg['map'] + trg['age']},
        'goal_trigger_logs': trg['goal_logs'],
        'map_trigger_logs': trg['map_logs'],
        'age_trigger_logs': trg['age_logs'],
        'path_publishes_in_window': len(_window(h.paths, t0, t1)),
        'note': note,
    }
    if extra:
        res.update(extra)
    return res


# ------------------------------------------------------------------ 用例

TAG = ''   # 由 main 设置：预修复 / 修复后 复用同一台架但产物文件名可区分


def _tagged(name):
    return name + TAG


def _prepare(case, cfg_dir, log_dir, goal_topic=GOAL_TOPIC):
    params = os.path.join(cfg_dir, 'p2_replan_%s_ground%s.yaml' % (case, TAG))
    _write(params, _ground_params_text(goal_topic))
    return _start_ground(log_dir, case, params)


def sc_c1_fixed(log_dir, cfg_dir):
    # 先起台架节点、后起被测进程：本环境下可让 DDS 发现即时完成（避免冷启动 50 s 发现窗口）。
    h = Bench('c1')
    proc, log = _prepare('c1_fixed', cfg_dir, log_dir)
    res = {'case': 'c1_fixed'}
    try:
        ok, life = h.ensure_active()
        res['lifecycle'] = life
        got = h.spin_until(lambda: len(h.paths) > 0, 6.0, goal=FIXED_GOAL)
        res['first_path'] = bool(got)
        if not ok or not got:
            res['error'] = 'ground_baseline 未激活或无路径'
            return res
        h.pump(1.0, goal=FIXED_GOAL)          # 让首次规划绑定稳定
        t0 = time.time()
        h.pump(5.0, goal=FIXED_GOAL)
        t1 = time.time()
        res.update(_case_result(
            'c1_fixed', '0 次目标身份变 / 0 次地图变化（仅周期刷新）', t0, t1, h,
            extra={'goal_xy': list(FIXED_GOAL)},
            note='固定目标 (4.5,3.0) 零速度 + 固定地图 + 固定 goal；合成传感器无输入。'))
    finally:
        _kill(proc)
        h.destroy_node()
        log.close()
    return res


def sc_c2_goal_step(log_dir, cfg_dir):
    h = Bench('c2')
    proc, log = _prepare('c2_goal_step', cfg_dir, log_dir)
    res = {'case': 'c2_goal_step'}
    try:
        ok, life = h.ensure_active()
        res['lifecycle'] = life
        got = h.spin_until(lambda: len(h.paths) > 0, 6.0, goal=FIXED_GOAL)
        res['first_path'] = bool(got)
        if not ok or not got:
            res['error'] = 'ground_baseline 未激活或无路径'
            return res
        h.pump(1.0, goal=FIXED_GOAL)
        new_goal = (FIXED_GOAL[0] + 1.2, FIXED_GOAL[1])
        t0 = time.time()
        h.pump(2.5, goal=new_goal)
        t1 = time.time()
        res.update(_case_result(
            'c2_goal_step', '恰好 1 次目标身份变（位移 1.2 m > 1.0 m，受限流）', t0, t1, h,
            extra={'base_goal': list(FIXED_GOAL), 'step_goal': list(new_goal),
                   'step_m': 1.2, 'threshold_m': REPLAN_DISP},
            note='goal 单次阶跃 1.2 m；日志应带 goal_delta_m 实际位移。'))
    finally:
        _kill(proc)
        h.destroy_node()
        log.close()
    return res


def sc_c3_goal_jitter(log_dir, cfg_dir):
    h = Bench('c3')
    proc, log = _prepare('c3_goal_jitter', cfg_dir, log_dir)
    res = {'case': 'c3_goal_jitter'}
    try:
        ok, life = h.ensure_active()
        res['lifecycle'] = life
        got = h.spin_until(lambda: len(h.paths) > 0, 6.0, goal=FIXED_GOAL)
        res['first_path'] = bool(got)
        if not ok or not got:
            res['error'] = 'ground_baseline 未激活或无路径'
            return res
        h.pump(1.0, goal=FIXED_GOAL)
        t0 = time.time()

        def jitter(el):
            h.pub_map()
            h.pub_goal(FIXED_GOAL[0] + 0.3 * math.sin(2 * math.pi * 1.0 * el),
                       FIXED_GOAL[1] + 0.3 * math.cos(2 * math.pi * 1.0 * el))

        h.pump_fn(5.0, jitter)
        t1 = time.time()
        res.update(_case_result(
            'c3_goal_jitter', '0 次目标身份变（抖幅 0.3 m < 1.0 m）', t0, t1, h,
            extra={'jitter_amplitude_m': 0.3, 'threshold_m': REPLAN_DISP},
            note='goal 微扰 0.3 m @20 Hz（正弦）。'))
    finally:
        _kill(proc)
        h.destroy_node()
        log.close()
    return res


def sc_c4_map_add_obstacle(log_dir, cfg_dir):
    h = Bench('c4')
    proc, log = _prepare('c4_map_add_obstacle', cfg_dir, log_dir)
    res = {'case': 'c4_map_add_obstacle'}
    # 地图 160x160 @0.5，origin -40,-40；在车前约 6 m 处放 3x3 硬障碍（世界 ~ (6,0)）
    cx = int((6.0 - MAP_OX) / MAP_RES)
    cy = int((0.0 - MAP_OY) / MAP_RES)
    obstructed = [cy * MAP_W + cx + dx + dy * MAP_W for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
    try:
        ok, life = h.ensure_active()
        res['lifecycle'] = life
        got = h.spin_until(lambda: len(h.paths) > 0, 6.0, goal=FIXED_GOAL)
        res['first_path'] = bool(got)
        if not ok or not got:
            res['error'] = 'ground_baseline 未激活或无路径'
            return res
        # 基线稳定窗：确认固定地图本身不产生地图触发
        h.pump(1.5, goal=FIXED_GOAL)
        tb0 = time.time()
        h.pump(2.0, goal=FIXED_GOAL)
        tb1 = time.time()
        baseline = _case_result('c4_baseline', '基线固定地图：0 次地图变化', tb0, tb1, h)
        # 真实增障窗
        t0 = time.time()
        h.pump(2.5, goal=FIXED_GOAL, map_kw={'occupied_idx': obstructed})
        t1 = time.time()
        res['baseline_window'] = baseline
        res['obstacle_cells'] = len(obstructed)
        res.update(_case_result(
            'c4_map_add_obstacle', '恰好 1 次地图变化（真实增 3x3 硬障碍）', t0, t1, h,
            note='static 层地图在车前 6 m 增 3x3=9 格 occupied(100)。'))
    finally:
        _kill(proc)
        h.destroy_node()
        log.close()
    return res


def sc_c5_map_free_unknown(log_dir, cfg_dir):
    h = Bench('c5')
    proc, log = _prepare('c5_map_free_unknown', cfg_dir, log_dir)
    res = {'case': 'c5_map_free_unknown'}
    # 车前 6 m 的 3x3 区域在 free(0) / unknown(-1) 间翻转（低于硬障碍签名）
    cx = int((6.0 - MAP_OX) / MAP_RES)
    cy = int((0.0 - MAP_OY) / MAP_RES)
    patch = [cy * MAP_W + cx + dx + dy * MAP_W for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
    try:
        ok, life = h.ensure_active()
        res['lifecycle'] = life
        got = h.spin_until(lambda: len(h.paths) > 0, 6.0, goal=FIXED_GOAL)
        res['first_path'] = bool(got)
        if not ok or not got:
            res['error'] = 'ground_baseline 未激活或无路径'
            return res
        h.pump(1.5, goal=FIXED_GOAL)
        t0 = time.time()

        def flip(el):
            if int(el * 5) % 2 == 0:      # 5 Hz 翻转
                h.pub_map(unknown_idx=patch)
            else:
                h.pub_map()
            h.pub_goal(*FIXED_GOAL)

        h.pump_fn(4.0, flip)
        t1 = time.time()
        res.update(_case_result(
            'c5_map_free_unknown', '0 次地图变化（-1 未知翻转不构成硬障碍）', t0, t1, h,
            extra={'patch_cells': len(patch), 'flip_hz': 5.0},
            note='地图 3x3 区域 free(0)↔unknown(-1) 翻转；按契约 0=free/-1=unknown，不应进入硬障碍签名。'))
    finally:
        _kill(proc)
        h.destroy_node()
        log.close()
    return res


def sc_c6_target_twist(log_dir, cfg_dir):
    gp = os.path.join(cfg_dir, 'p2_replan_c6_ground%s.yaml' % TAG)
    hp = os.path.join(cfg_dir, 'p2_replan_c6_high_level%s.yaml' % TAG)
    _write(gp, _ground_params_text(GOAL_HL_TOPIC))
    _write(hp, _hl_params_text())
    h = Bench('c6')
    proc, log = _start_ground(log_dir, 'c6_target_twist', gp)
    hl_proc, hl_log = _start_high_level(log_dir, 'c6_target_twist', hp)
    res = {'case': 'c6_target_twist'}
    try:
        ok, life = h.ensure_active()
        res['lifecycle'] = life
        # 目标流须在等待首路径期间就发布，否则 high_level_node 无 goal → 低层永远无路径。
        got = h.spin_until(lambda: len(h.paths) > 0, 15.0, target=(0.0, 0.0))
        res['first_path'] = bool(got)
        if not ok or not got:
            res['error'] = 'ground_baseline 未激活或无路径（goal 来自 high-level）'
            return res
        # 目标 twist 微扰：v = 0.6 m/s 正弦，f=0.7 Hz；位置恒为 (4.5,3.0)。
        h.pump(1.0, target=(0.0, 0.0))
        t0 = time.time()

        def twist(el):
            vx = 0.6 * math.sin(2 * math.pi * 0.7 * el)
            vy = 0.6 * math.cos(2 * math.pi * 0.7 * el)
            h.pub_map()
            # goal 由真实 high_level_node 发布（本台架不直接发 goal）
            h.pub_target(vx, vy)

        h.pump_fn(6.0, twist)
        t1 = time.time()
        goals = [g for g in h.hl_goals if t0 + 1.0 <= g['wall'] <= t1]   # 跳过低通瞬态
        jitter_span = 0.0
        jitter_std = 0.0
        if goals:
            xs = [g['x'] for g in goals]
            ys = [g['y'] for g in goals]
            jitter_span = round(math.hypot(max(xs) - min(xs), max(ys) - min(ys)), 4)
            mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
            jitter_std = round(math.sqrt(sum((x - mx) ** 2 + (y - my) ** 2 for x, y in zip(xs, ys))
                                          / len(xs)), 4)
        trg = _case_result(
            'c6_target_twist', '目标静止 + twist 噪声：goal 抖动应 < 1.0 m → 0 次目标身份变',
            t0 + 1.0, t1, h,
            extra={'hl_goals_in_window': len(goals),
                   'hl_goal_jitter_span_m': jitter_span,
                   'hl_goal_jitter_rms_m': jitter_std,
                   'truth_target_xy': list(TARGET_XY),
                   'noise': {'twist_amplitude_mps': 0.6, 'freq_hz': 0.7}},
            note='C6 经 high_level_node（20 Hz / target_tau_s=0.5 / lead_s=1.0 / max_lead_m=1.0）；'
                 '目标真值静止，任何 >1.0 m 的 goal 位移触发都属伪触发。')
        trg['classification'] = [
            {'goal_delta_m': r.get('goal_delta_m'),
             'judgement': ('伪触发' if (r.get('goal_delta_m') or 0) > REPLAN_DISP else '真实目标移动')}
            for r in trg.get('goal_trigger_logs', [])]
        res.update(trg)
    finally:
        _kill(hl_proc); _kill(proc)
        h.destroy_node()
        hl_log.close()
        log.close()
    return res



def sc_c7_dynamic_marking(log_dir, cfg_dir):
    h = Bench('c7')
    proc, log = _prepare('c7_dynamic_marking', cfg_dir, log_dir)
    res = {'case': 'c7_dynamic_marking'}
    try:
        ok, life = h.ensure_active()
        res['lifecycle'] = life
        got = h.spin_until(lambda: len(h.paths) > 0, 6.0, goal=FIXED_GOAL)
        res['first_path'] = bool(got)
        if not ok or not got:
            res['error'] = 'ground_baseline 未激活或无路径'
            return res
        h.pump(1.5, goal=FIXED_GOAL)

        t0 = time.time()

        def move(el):
            x = TARGET_XY[0] + 0.6 * math.sin(2 * math.pi * 1.0 * el)
            y = TARGET_XY[1] + 0.6 * math.cos(2 * math.pi * 1.0 * el)
            h.pub_map()
            h.pub_goal(*FIXED_GOAL)
            h.pub_dynamic_marking(x, y, 2.0)

        h.pump_fn(4.0, move)
        t1 = time.time()
        res.update(_case_result(
            'c7_dynamic_marking', '0 次地图变化（动态层 marking 不参与静态签名）', t0, t1, h,
            extra={'dynamic_amplitude_m': 0.6, 'dynamic_hz': 1.0, 'marking_z_m': 2.0},
            note='合成移动目标样式 scan / pointcloud marking；fixed map / goal 不变。'))
    finally:
        _kill(proc)
        h.destroy_node()
        log.close()
    return res


def sc_c8_static_with_dynamic(log_dir, cfg_dir):
    h = Bench('c8')
    proc, log = _prepare('c8_static_with_dynamic', cfg_dir, log_dir)
    res = {'case': 'c8_static_with_dynamic'}
    cx = int((6.0 - MAP_OX) / MAP_RES)
    cy = int((0.0 - MAP_OY) / MAP_RES)
    obstructed = [cy * MAP_W + cx + dx + dy * MAP_W for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
    try:
        ok, life = h.ensure_active()
        res['lifecycle'] = life
        got = h.spin_until(lambda: len(h.paths) > 0, 6.0, goal=FIXED_GOAL)
        res['first_path'] = bool(got)
        if not ok or not got:
            res['error'] = 'ground_baseline 未激活或无路径'
            return res
        h.pump(1.5, goal=FIXED_GOAL)
        tb0 = time.time()
        h.pump(2.0, goal=FIXED_GOAL)
        tb1 = time.time()
        baseline = _case_result('c8_baseline', '基线固定地图 + 无动态输入：0 次地图变化',
                                tb0, tb1, h)
        t0 = time.time()

        def add_obstacle(el):
            x = TARGET_XY[0] + 0.6 * math.sin(2 * math.pi * 1.0 * el)
            y = TARGET_XY[1] + 0.6 * math.cos(2 * math.pi * 1.0 * el)
            h.pub_map(occupied_idx=obstructed)
            h.pub_goal(*FIXED_GOAL)
            h.pub_dynamic_marking(x, y, 2.0)

        h.pump_fn(2.5, add_obstacle)
        t1 = time.time()
        res['baseline_window'] = baseline
        res['obstacle_cells'] = len(obstructed)
        res.update(_case_result(
            'c8_static_with_dynamic', '恰好 1 次地图变化（真实静态增障仍触发）', t0, t1, h,
            extra={'dynamic_amplitude_m': 0.6, 'dynamic_hz': 1.0, 'obstacle_cells': len(obstructed)},
            note='static 层在车前 6 m 增 3x3 occupied；同时动态 scan/pointcloud 移动。'))
    finally:
        _kill(proc)
        h.destroy_node()
        log.close()
    return res


CASES = [
    ('c1_fixed', sc_c1_fixed),
    ('c2_goal_step', sc_c2_goal_step),
    ('c3_goal_jitter', sc_c3_goal_jitter),
    ('c4_map_add_obstacle', sc_c4_map_add_obstacle),
    ('c5_map_free_unknown', sc_c5_map_free_unknown),
    ('c6_target_twist', sc_c6_target_twist),
    ('c7_dynamic_marking', sc_c7_dynamic_marking),
    ('c8_static_with_dynamic', sc_c8_static_with_dynamic),
]


# ------------------------------------------------------------------ 报告

EXPECT = {
    'c1_fixed': {'goal_identity': 0, 'map_change': 0},
    'c2_goal_step': {'goal_identity': 1, 'map_change': 0},
    'c3_goal_jitter': {'goal_identity': 0, 'map_change': 0},
    'c4_map_add_obstacle': {'goal_identity': 0, 'map_change': 1},
    'c5_map_free_unknown': {'goal_identity': 0, 'map_change': 0},
    'c6_target_twist': {'goal_identity': 0, 'map_change': 0},
    'c7_dynamic_marking': {'goal_identity': 0, 'map_change': 0},
    'c8_static_with_dynamic': {'goal_identity': 0, 'map_change': 1},
}


def _verdict(case, r):
    exp = EXPECT.get(case, {})
    if 'error' in r:
        return '证据不足', '台架未激活 / 无路径'
    if case in ('c4_map_add_obstacle', 'c8_static_with_dynamic') and 'baseline_window' in r:
        base = r['baseline_window']['triggers']
        if base['map_change'] != 0:
            return '缺陷', '固定地图基线窗已出现地图触发（应为 0）'
    got = r.get('triggers', {})
    for k, v in exp.items():
        if got.get(k) != v:
            return '缺陷', '期望 %s=%d，实际=%s' % (k, v, got.get(k))
    return '符合预期', ''


def write_report(run_dir, result):
    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)
    js = os.path.join(run_dir, 'metrics', 'p2_replan_bench%s.json' % TAG)
    md_path = os.path.join(run_dir, 'metrics', 'p2_replan_bench%s.md' % TAG)
    with open(js, 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    md = ['# P2 重规划触发受控台架 · %s' % result['run_id'], '',
          '> **诊断台架**（非确证）：真实 `ground_baseline` 二进制 + 合成 map / odom / goal / target；',
          '> 不参与 P1 / P2 verdict。A 臂 0 次重规划**不是**本台架的验收目标。', '',
          '## 用例汇总', '',
          '| 用例 | 期望 | 实际（目标身份变 / 地图变化 / 路径超时） | 判定 |',
          '|---|---|---|---|']
    for c, r in result['cases'].items():
        exp = ', '.join('%s≤%d' % (k, v) if False else '%s=%d' % (k, v)
                        for k, v in EXPECT.get(c, {}).items())
        got = r.get('triggers', {})
        actual = '%s / %s / %s' % (got.get('goal_identity'), got.get('map_change'), got.get('path_age'))
        v, why = _verdict(c, r)
        md.append('| `%s` | %s | %s | %s%s |' % (
            c, exp, actual, v, ('（%s）' % why) if why else ''))
    md += ['', '## 逐用例诊断量', '']
    for c, r in result['cases'].items():
        md.append('### `%s`' % c)
        md.append('- 窗口 %.2f s；期望：%s' % (r.get('window_s', 0), r.get('expectation', '')))
        if r.get('hl_goal_jitter_span_m') is not None:
            md.append('- high-level goal 抖动：span=%.3f m / RMS=%.3f m（%d 条）' % (
                r['hl_goal_jitter_span_m'], r['hl_goal_jitter_rms_m'], r['hl_goals_in_window']))
        for lg in r.get('goal_trigger_logs', []):
            md.append('- 目标触发：goal_delta_m=%s threshold=%s last_plan=(%s,%s) goal=(%s,%s)' % (
                lg.get('goal_delta_m'), lg.get('threshold'), lg.get('last_plan_x'),
                lg.get('last_plan_y'), lg.get('goal_x'), lg.get('goal_y')))
        for lg in r.get('map_trigger_logs', []):
            md.append('- 地图触发：sig %s → %s；sx=%s sy=%s origin=(%s,%s) hard_cells=%s；layers=[%s]' % (
                lg.get('last_plan_map_sig'), lg.get('map_sig'), lg.get('sx'), lg.get('sy'),
                lg.get('origin_x'), lg.get('origin_y'), lg.get('hard_cells'), lg.get('layers')))
        if r.get('classification'):
            for x in r['classification']:
                md.append('- C6 分类：goal_delta=%s → **%s**' % (x['goal_delta_m'], x['judgement']))
        md.append('')
    with open(md_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(md) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261024-w7-rl-b-p2-replan-audit')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    ap.add_argument('--cases', default=','.join(c for c, _ in CASES))
    ap.add_argument('--tag', default='', help='产物文件名后缀（如 -postfix），复用同一 run 目录')
    args = ap.parse_args()

    global TAG
    TAG = args.tag
    run_dir = os.path.join(args.experiments_root, 'w7-rl-b', args.run_id)
    cfg_dir = os.path.join(run_dir, 'config')
    log_dir = os.path.join(run_dir, 'log')
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(os.path.join(run_dir, 'metrics'), exist_ok=True)

    selected = [c.strip() for c in args.cases.split(',') if c.strip()]
    result = {
        'run_id': args.run_id,
        'kind': 'diagnostic_bench（ADR-0055 / ADR-0056 审计；非确证，不改 verdict）',
        'scope_note': ('节点行为台架：真实 ground_baseline 二进制 + 合成 map/odom/goal/target；'
                       'C6 另起真实 agc_rl_high_level。**非整链** / 非 GUI；不以 A 臂 0 次重规划为验收目标。'),
        'env': {'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID'),
                'ROS_AUTOMATIC_DISCOVERY_RANGE': os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')},
        'cases': {},
    }

    for name, fn in CASES:
        if name not in selected:
            continue
        last = None
        for attempt in range(2):        # 仅吸收 TF/DDS 发现抖动；不掩盖产品错误
            rclpy.init()
            _reap_stale(REAP_PATTERNS)
            try:
                r = fn(log_dir, cfg_dir)
            except Exception as exc:  # noqa: BLE001
                r = {'case': name, 'error': repr(exc)}
            finally:
                try:
                    rclpy.shutdown()
                except Exception:  # noqa: BLE001
                    pass
            _reap_stale(REAP_PATTERNS)
            r['attempts'] = attempt
            last = r
            if 'error' not in r:
                break
            time.sleep(0.5)
        result['cases'][name] = last
        v, _ = _verdict(name, last)
        print('%-24s %-6s triggers=%s' % (name, v, last.get('triggers')))

    write_report(run_dir, result)
    print('\n-> %s' % os.path.join(run_dir, 'metrics', 'p2_replan_bench%s.{json,md}' % TAG))
    return 0


if __name__ == '__main__':
    sys.exit(main())
