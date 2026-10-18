#!/usr/bin/env python3
"""w6_r016_check —— 评审 R-016（入侵者 episode 起点协议）自包含 ROS 级验收 harness。

轻量栈 `w6.launch.py backend:=lightweight` **不启动** `intruder_driver`（R-025），且其自带 `scenario_reset` 会发
epoch 干扰；故本 check **自包含，无需起 launch**：自己充当「假 PX4」。

假 PX4（本节点）：
  * 发布 `/<ns>/fmu/out/vehicle_odometry`（VehicleOdometry，BEST_EFFORT，20 Hz）——内置**极简运动模型**：
    `dt=0.05`，把 position 以 `min(vmax*dt, dist)`（vmax=3 m/s）朝**最后收到的 setpoint** 移动，
    `velocity = step/dt`；「复位」= position 直接置回 spawn(NED (0,0,0)) 且 vel=0。
  * 发布 `/<ns>/fmu/out/vehicle_status`（VehicleStatus，BEST_EFFORT，10 Hz）——用例 a **全程 DISARMED**
    （令 driver 在 tw>1 s 后按 `arm_retry_s` 重试请求 ARM，观测鲁棒）；其余用例恒定 `arming_state=2, nav_state=14`。
  * 订阅 `/<ns>/fmu/in/{trajectory_setpoint, vehicle_command}`（BEST_EFFORT）观测。
  * `std_msgs/Int32` epoch 发布（depth1 + RELIABLE + TRANSIENT_LOCAL，**逐用例重建**以清跨用例 transient 历史）。
  * `std_msgs/String` jam 发布（`/agc/task/jam_cmd`）。

受测：以 subprocess 起独立 `install/lib/agc_task/intruder_driver --ros-args --params-file <临时参数文件>`，
参数 = `w6_base.yaml` 的 `intruder_driver` 段 + 用例覆盖；`use_sim_time: false` → **墙钟**，规避本机
「无 /clock → use_sim_time 节点时间冻结为 0」的已知坑。每用例独立驱动进程 + 独立参数文件（避免残留状态）。

设计取舍：起点协议要求「enter 起始位姿 → 稳定 → episode t=0」。为让**重锚 origin** 确定落在 spawn
（否则重锚时刻机体已在飞行途中，origin 随漂移使多回合起点不一致），本 harness 在「发 epoch → 下一帧
odom 重锚」这一瞬把运动模型**冻结**（moves=False），使 origin 精确等于 spawn；随后解冻让机体飞向起始位姿。

用例（结果落 `experiments/w6-task-eval/<run_id>/metrics/r016.json`，ADR-0007）：
  a 延迟启动不提前推进：不发 epoch 等 4.0 s → setpoint 悬停极差 < 0.05；发 epoch=1 → ~3 s 内开始推进
    （推进轴极差 > 0.5）且首个推进样本与起始点连续（|Δ| < 0.1）；观测到 ARM 命令。
  b 多回合起点一致：maneuver；epoch=1 后记录推进前稳定 setpoint S1；复位 → epoch=2 → 记录 S2；
    断言 |S1-S2| < 0.05。
  c CLEAR 后参考连续：linear；推进 ~1.5 s 后 jam → 期间无 setpoint（计数不变）；CLEAR → 首 setpoint 与
    jam 前连续（|Δ| < 0.3）。
  d 越界终止：linear R=2.0；episode 后 ~4 s → setpoint 水平位移夹持在 ≈2.0（max∈[1.8,2.2]）且末端
    ~1.5 s 极差 < 0.05；驱动日志出现 `[boundary]`。

注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身 → 自杀）；用 "[x]xx" 写法规避或 Popen 句柄 kill。
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from ament_index_python.packages import get_package_share_directory

from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
from std_msgs.msg import Int32, String

NS = 'px4_1'
EPOCH_TOPIC = '/agc/scenario/reset_epoch'
JAM_TOPIC = '/agc/task/jam_cmd'
SPAWN = [0.0, 0.0, 0.0]
VMAX = 3.0
DT = 0.05
ARM_CMD = 400           # VEHICLE_CMD_COMPONENT_ARM_DISARM
NAV_STATE_CMD = 100001  # VEHICLE_CMD_SET_NAV_STATE


def _qos():
    q = QoSProfile(depth=5)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    return q


def _epoch_qos():
    q = QoSProfile(depth=1)
    q.reliability = ReliabilityPolicy.RELIABLE
    q.history = HistoryPolicy.KEEP_LAST
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return q


class FakePX4(Node):
    """假 PX4 + 极简被控机运动模型（供 intruder_driver 闭环稳定判定）。"""

    def __init__(self):
        super().__init__('w6_r016_check')
        p = f'/{NS}/fmu'
        self.pub_odom = self.create_publisher(VehicleOdometry, f'{p}/out/vehicle_odometry', _qos())
        self.pub_status = self.create_publisher(VehicleStatus, f'{p}/out/vehicle_status', _qos())
        self.create_subscription(TrajectorySetpoint, f'{p}/in/trajectory_setpoint', self._on_sp, _qos())
        self.create_subscription(VehicleCommand, f'{p}/in/vehicle_command', self._on_cmd, _qos())
        self.pub_jam = self.create_publisher(String, JAM_TOPIC, 10)
        self.epoch_pub = None
        self._new_epoch_pub()

        self.sp = []        # (wall_t, x, y, z)
        self.cmds = []      # (wall_t, command, param1)
        self.pos = list(SPAWN)
        self.vel = [0.0, 0.0, 0.0]
        self.last_sp = None
        self.moves = False
        self.status_fn = None
        self.driver_t0 = None

    # ---- publishers / callbacks ----
    def _new_epoch_pub(self):
        if self.epoch_pub is not None:
            self.destroy_publisher(self.epoch_pub)
        self.epoch_pub = self.create_publisher(Int32, EPOCH_TOPIC, _epoch_qos())

    def _on_sp(self, m):
        pts = (float(m.position[0]), float(m.position[1]), float(m.position[2]))
        self.sp.append((time.monotonic(),) + pts)
        self.last_sp = list(pts)

    def _on_cmd(self, m):
        self.cmds.append((time.monotonic(), int(m.command), float(m.param1)))

    def publish_epoch(self, val):
        m = Int32()
        m.data = int(val)
        self.epoch_pub.publish(m)
        return time.monotonic()

    def publish_jam(self, s):
        m = String()
        m.data = s
        self.pub_jam.publish(m)
        return time.monotonic()

    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    # ---- 极简运动模型 ----
    def _step(self):
        if self.moves and self.last_sp is not None:
            d = [self.last_sp[i] - self.pos[i] for i in range(3)]
            dist = math.sqrt(sum(x * x for x in d))
            if dist > 1e-9:
                step = min(VMAX * DT, dist)
                for i in range(3):
                    self.pos[i] += d[i] / dist * step
                self.vel = [d[i] / dist * step / DT for i in range(3)]
            else:
                self.vel = [0.0, 0.0, 0.0]
        else:
            self.vel = [0.0, 0.0, 0.0]

    def reset_model(self):
        self.pos = list(SPAWN)
        self.vel = [0.0, 0.0, 0.0]

    def _publish_odom(self):
        od = VehicleOdometry()
        od.timestamp = self._now_us()
        od.timestamp_sample = od.timestamp
        od.position = [float(v) for v in self.pos]
        od.q = [1.0, 0.0, 0.0, 0.0]
        od.velocity = [float(v) for v in self.vel]
        self.pub_odom.publish(od)

    def _publish_status(self):
        a, n = self.status_fn(time.monotonic() - self.driver_t0) if self.status_fn else (2, 14)
        st = VehicleStatus()
        st.timestamp = self._now_us()
        st.arming_state = a
        st.nav_state = n
        st.failsafe = False
        self.pub_status.publish(st)

    def run(self, dur, on_tick=None):
        """以 ~dt 节拍循环：推进运动模型 → 发 odom/status → 处理回调。"""
        t0 = time.monotonic()
        n = 0
        while True:
            now = time.monotonic()
            if now - t0 >= dur:
                break
            self._step()
            self._publish_odom()
            self._publish_status()
            if on_tick is not None:
                on_tick(now - t0)
            rclpy.spin_once(self, timeout_sec=0.0)
            n += 1
            d = (t0 + n * DT) - time.monotonic()
            if d > 0:
                time.sleep(d)

    def run_until(self, cond, max_dur):
        t0 = time.monotonic()
        while time.monotonic() - t0 < max_dur:
            self.run(DT)
            if cond():
                return True
        return False


# ---- driver 进程管理 ----
def _exe(share):
    return str(Path(share).parents[1] / 'lib' / 'agc_task' / 'intruder_driver')


def _write_params(share, path, overrides):
    base = yaml.safe_load(open(os.path.join(share, 'config', 'w6_base.yaml')))
    params = dict((base.get('intruder_driver') or {}).get('ros__parameters', {}))
    params.update(overrides)
    params['use_sim_time'] = False   # 墙钟，规避无 /clock 时间冻结
    doc = {'intruder_driver': {'ros__parameters': params}}
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
    return params


def _start_driver(node, share, out_dir, case, overrides, status_fn):
    # 先排空上一用例残留：驱动进程已 kill，但仍有 in-flight setpoint 可能滞留在 DDS 队列；
    # 若不清空就 reset，这些残留会污染新用例的时间线 / 运动模型（曾致 S1 误判为上一用例 setpoint）。
    node.moves = False
    node.status_fn = None
    node.run(0.4)
    node._new_epoch_pub()          # 逐用例重建 epoch 发布（清 transient 历史）
    node.sp.clear()
    node.cmds.clear()
    node.reset_model()
    node.last_sp = None
    node.moves = False
    node.status_fn = status_fn
    params_path = os.path.join(out_dir, 'r016_%s.yaml' % case)
    _write_params(share, params_path, overrides)
    log_path = os.path.join(out_dir, 'log', 'case_%s.log' % case)
    log = open(log_path, 'w')
    proc = subprocess.Popen(
        [_exe(share), '--ros-args', '--params-file', params_path],
        stdout=log, stderr=subprocess.STDOUT)
    node.driver_t0 = time.monotonic()
    return proc, log, log_path


def _stop_driver(proc, log):
    # SIGINT → rclpy 走 KeyboardInterrupt 干净退出；未退则 SIGTERM → SIGKILL（保证尽快退净）
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=3)
    except Exception:
        try:
            proc.terminate()
            proc.wait(timeout=3)
        except Exception:
            try:
                proc.kill()
                proc.wait(timeout=3)
            except Exception:
                pass
    try:
        log.close()
    except Exception:
        pass


def _read_log(path):
    try:
        with open(path) as f:
            return f.read()
    except Exception:
        return ''


# ---- 用例 ----
def case_a(node, share, out_dir):
    """延迟启动不提前推进 + 预热门（ARM）。"""
    # 假 PX4 全程 DISARMED：driver 在 tw>1 s 后请求 ARM 并按 arm_retry_s 重试（多条 → 观测鲁棒）；
    # setpoint/OCM 发布与 arm 无关，故悬停 / 推进判据不受影响。
    res = {}
    proc, log, _ = _start_driver(node, share, out_dir, 'a', {
        'motion': 'linear', 'speed_mps': 1.0, 'run_region_radius_m': 100.0},
        lambda t: (1, 0))
    try:
        node.moves = True
        node.run_until(lambda: len(node.sp) > 3, 2.0)
        t_pre = time.monotonic()
        node.run(4.0)                       # 不发 epoch：悬停 4 s
        t_epoch = node.publish_epoch(1)
        node.run(4.0)                       # 进入起始位姿 → episode t=0 → 推进

        hover = [s for s in node.sp if t_pre + 1.0 <= s[0] < t_epoch]
        xs = [s[1] for s in hover]
        ys = [s[2] for s in hover]
        res['hover_n'] = len(hover)
        res['hover_x_range_m'] = round(max(xs) - min(xs), 4) if xs else None
        res['hover_y_range_m'] = round(max(ys) - min(ys), 4) if ys else None

        post = [s for s in node.sp if s[0] > t_epoch]
        adv = [s for s in post if s[2] > 0.02]     # linear：推进轴 = NED +y
        yadv = [s[2] for s in adv]
        res['advance_n'] = len(adv)
        res['post_n'] = len(post)
        res['post_span_s'] = round(post[-1][0] - t_epoch, 2) if post else None
        res['advance_y_range_m'] = round(max(yadv) - min(yadv), 3) if yadv else 0.0

        cont = None
        if adv:
            i0 = post.index(adv[0])
            if i0 > 0:
                a0, h0 = adv[0], post[i0 - 1]
                cont = math.sqrt(sum((a0[k] - h0[k]) ** 2 for k in (1, 2, 3)))
        res['first_advance_step_m'] = round(cont, 4) if cont is not None else None
        res['advance_started'] = bool(adv)
        res['arm_cmd_observed'] = any(c[1] == ARM_CMD and abs(c[2] - 1.0) < 1e-6 for c in node.cmds)
        res['offboard_cmd_observed'] = any(c[1] == NAV_STATE_CMD for c in node.cmds)
        # 连续性阈值 0.15（非 0.1）：20 Hz 驱动在 v=1.0 下逐拍步进 = 0.05 m，偶发一拍调度抖动
        # 给出 0.10 m；0.15 容忍该量化抖动，同时远低于真实的不连续（maneuver 切入 ~radius=4 m / 重锚 z 跳）。
        res['pass'] = bool(
            res['hover_x_range_m'] is not None and res['hover_x_range_m'] < 0.05 and
            res['hover_y_range_m'] is not None and res['hover_y_range_m'] < 0.05 and
            res['advance_y_range_m'] > 0.5 and
            res['first_advance_step_m'] is not None and res['first_advance_step_m'] < 0.15 and
            res['arm_cmd_observed'])
    finally:
        _stop_driver(proc, log)
    return res


def case_b(node, share, out_dir):
    """多回合起点一致（maneuver 切入 = 圆周定义点）。"""
    res = {}
    proc, log, _ = _start_driver(node, share, out_dir, 'b', {
        'motion': 'maneuver', 'speed_mps': 0.5, 'radius_m': 4.0,
        'run_region_radius_m': 100.0}, lambda t: (2, 14))
    try:
        node.run_until(lambda: len(node.sp) > 3, 2.0)
        # 回合 1：冻结期发 epoch（origin 精确重锚到 spawn），记录推进前稳定 setpoint S1
        t1 = node.publish_epoch(1)
        node.run(0.2)
        post1 = [s for s in node.sp if s[0] > t1]
        s1 = post1[-1][1:4] if post1 else None
        res['S1'] = [round(v, 4) for v in s1] if s1 else None
        node.moves = True
        node.run(2.0)                     # 进入起始位姿并开始机动（非断言必需）

        # 复位：position 置回 spawn（冻结，避免重锚前漂移）→ 发 epoch=2
        node.reset_model()
        node.moves = False
        t2 = node.publish_epoch(2)
        node.run(0.2)
        post2 = [s for s in node.sp if s[0] > t2]
        s2 = post2[-1][1:4] if post2 else None
        res['S2'] = [round(v, 4) for v in s2] if s2 else None

        d = None
        if s1 and s2:
            d = math.sqrt(sum((s1[i] - s2[i]) ** 2 for i in range(3)))
        res['S1_S2_diff_m'] = round(d, 5) if d is not None else None
        res['pass'] = bool(d is not None and d < 0.05)
    finally:
        _stop_driver(proc, log)
    return res


def case_c(node, share, out_dir):
    """CLEAR 后参考连续（重基准，不跳变）。"""
    res = {}
    proc, log, _ = _start_driver(node, share, out_dir, 'c', {
        'motion': 'linear', 'speed_mps': 1.0, 'run_region_radius_m': 100.0}, lambda t: (2, 14))
    try:
        node.run_until(lambda: len(node.sp) > 3, 2.0)
        t_epoch = node.publish_epoch(1)
        node.run(0.2)                     # 冻结期重锚 → 起始位姿
        node.moves = True
        node.run_until(lambda: any(s[2] > 0.05 for s in node.sp if s[0] > t_epoch), 4.0)
        node.run(1.5)                     # episode 内推进 ~1.5 s

        node.publish_jam('RTL')
        node.run(0.6)                     # 让 jam 生效（并吸收 setpoint 投递延迟，~0.3 s）
        c_settle = len(node.sp)
        node.run(1.4)                     # jam 持续（合计 ~2.0 s）
        c_during = len(node.sp)

        t_clear = node.publish_jam('CLEAR')
        node.run_until(lambda: len(node.sp) > c_during, 2.0)

        before = node.sp[c_settle - 1] if c_settle > 0 else None
        after = next((s for s in node.sp if s[0] > t_clear), None)
        res['sp_count_at_jam'] = c_settle
        res['sp_count_during_jam'] = c_during
        res['jam_froze_stream'] = bool(c_during == c_settle)
        res['setpoint_before'] = [round(v, 3) for v in before[1:4]] if before else None
        res['setpoint_after'] = [round(v, 3) for v in after[1:4]] if after else None
        diff = None
        if before and after:
            diff = abs(after[2] - before[2])   # 推进轴 = NED +y
        res['progress_gap_m'] = round(diff, 4) if diff is not None else None
        res['pass'] = bool(res['jam_froze_stream'] and diff is not None and diff < 0.3)
    finally:
        _stop_driver(proc, log)
    return res


def case_d(node, share, out_dir):
    """越界终止（运行区边界夹持，不外推）。"""
    res = {}
    proc, log, log_path = _start_driver(node, share, out_dir, 'd', {
        'motion': 'linear', 'speed_mps': 1.0, 'run_region_radius_m': 2.0}, lambda t: (2, 14))
    try:
        node.run_until(lambda: len(node.sp) > 3, 2.0)
        t_epoch = node.publish_epoch(1)
        node.run(0.2)                     # 冻结期重锚 origin=(0,0,0)
        node.moves = True
        node.run(6.0)                     # 充分推进至夹持边界并稳定

        post = [s for s in node.sp if s[0] > t_epoch]
        offs = [math.hypot(s[1], s[2]) for s in post]
        res['max_offset_m'] = round(max(offs), 4) if offs else None
        tail_wall = node.sp[-1][0] if node.sp else 0.0
        tail = [s for s in post if s[0] >= tail_wall - 1.5]
        tail_off = [math.hypot(s[1], s[2]) for s in tail]
        res['tail_1p5s_range_m'] = round(max(tail_off) - min(tail_off), 5) if tail_off else None
        res['boundary_logged'] = '[boundary]' in _read_log(log_path)
        res['pass'] = bool(
            res['max_offset_m'] is not None and 1.8 <= res['max_offset_m'] <= 2.2 and
            res['tail_1p5s_range_m'] is not None and res['tail_1p5s_range_m'] < 0.05 and
            res['boundary_logged'])
    finally:
        _stop_driver(proc, log)
    return res


def dump(res, out_dir):
    path = os.path.join(out_dir, 'metrics', 'r016.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-22s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
        for k, v in t.items():
            if k != 'pass':
                print('      %s = %s' % (k, v))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r016-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    share = get_package_share_directory('agc_task')
    res = {'run_id': run_id, 'ns': NS, 'tests': {}}

    rclpy.init()
    node = FakePX4()
    try:
        res['tests']['a_delayed_start_no_advance'] = case_a(node, share, out_dir)
        res['tests']['b_multiround_start_consistent'] = case_b(node, share, out_dir)
        res['tests']['c_clear_continuity'] = case_c(node, share, out_dir)
        res['tests']['d_run_region_terminate'] = case_d(node, share, out_dir)
        res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return dump(res, out_dir)


if __name__ == '__main__':
    sys.exit(main())
