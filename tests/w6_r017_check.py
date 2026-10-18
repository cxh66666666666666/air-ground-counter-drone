#!/usr/bin/env python3
"""w6_r017_check —— 评审 R-017（干扰单一授权与已校验事件）自包含 ROS 级验收 harness。

结构拷贝自 `tests/w6_r016_check.py`。轻量栈 `w6.launch.py backend:=lightweight` **不启动** `intruder_driver`，
且其自带 `scenario_reset` 会发 epoch 干扰；故本 check **自包含，无需起 launch**：自己充当「假 PX4」。

假 PX4（本节点）：
  * 发布 `/<ns>/fmu/out/vehicle_odometry`（VehicleOdometry，BEST_EFFORT，20 Hz）——内置**极简运动模型**
    （拷贝 r016：`dt=0.05`，把 position 以 `min(vmax*dt, dist)`（vmax=3 m/s）朝**最后收到的 setpoint**
    移动）。
  * 发布 `/<ns>/fmu/out/vehicle_status`（VehicleStatus，BEST_EFFORT；本 harness 恒定 `arming_state=2,
    nav_state=14`）。
  * 订 `/<ns>/fmu/in/{trajectory_setpoint, vehicle_command}`（BEST_EFFORT）观测。
  * `std_msgs/Int32` epoch 发布（depth1 + RELIABLE + TRANSIENT_LOCAL，**逐用例重建**以清 transient 历史）。
  * `std_msgs/Bool` 包络发布（`/agc/task/envelope_in`）——**可控**（`env_publishing_` 是否持续发 +
    `env_value_` 值），模拟 capture_adjudicator 的 20 Hz 势。
  * `std_msgs/String` **原始**命令发布（`/agc/task/jam_cmd`，仅供注入）。
  * `std_msgs/String` **已校验授权事件**观测（订 `/agc/task/jam_state`）。

受测：每用例以 subprocess 起**独立** `install/lib/agc_task/{jammer, intruder_driver}
--ros-args --params-file <临时参数文件>`；jammer 参数 = `w6_base.yaml` 的 `jammer` 段、driver 参数 =
`intruder_driver` 段 + 用例覆盖；`use_sim_time: false` → **墙钟**，规避本机
「无 /clock → use_sim_time 节点时间冻结为 0」的已知坑。每用例独立 fresh 参数文件（避免残留状态）。

用例（结果落 `experiments/w6-task-eval/<run_id>/metrics/r017.json`，ADR-0007）：
  a 未知命令不切断：包络 freshness=true/value=true；流已流动；发 `"BOGUS"` → 流继续、无 `JAM_*` 授权事件、
    jammer 未发 `SET_NAV_STATE`。
  b 包络外/过期不生效：b1 包络 false；b2 包络曾新鲜但停发 > `envelope_max_age_s`；两子测发 `"RTL"`
    均无 `JAM_*` 事件、流继续。
  c 包络内合法→授权事件→停发 Offboard（流可观测）+ CLEAR 连续：`"RTL"` → `JAM_RTL` + jammer
    `SET_NAV_STATE param1≈5` + `wall_t > t_jam+0.6` 的 setpoint 计数 == 0；`"CLEAR"` → 流恢复且首 setpoint
    与 jam 前连续（`|Δ| < 0.3 m`）。
  d grep：`intruder_driver.py` 内**无** `jam_cmd` 字符串；`w6_base.yaml` 的 `intruder_driver` 段**无**
    `jam_cmd_topic` 键。

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
from std_msgs.msg import Bool, Int32, String

NS = 'px4_1'
EPOCH_TOPIC = '/agc/scenario/reset_epoch'
JAM_CMD_TOPIC = '/agc/task/jam_cmd'
JAM_STATE_TOPIC = '/agc/task/jam_state'
ENV_TOPIC = '/agc/task/envelope_in'
SPAWN = [0.0, 0.0, 0.0]
VMAX = 3.0
DT = 0.05
NAV_STATE_CMD = 100001  # VEHICLE_CMD_SET_NAV_STATE
JAM_NAV_PARAM1 = (5.0, 18.0, 4.0)   # RTL / LAND / HOLD（jammer 注入的 nav_state）


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
        super().__init__('w6_r017_check')
        p = f'/{NS}/fmu'
        self.pub_odom = self.create_publisher(VehicleOdometry, f'{p}/out/vehicle_odometry', _qos())
        self.pub_status = self.create_publisher(VehicleStatus, f'{p}/out/vehicle_status', _qos())
        self.create_subscription(TrajectorySetpoint, f'{p}/in/trajectory_setpoint', self._on_sp, _qos())
        self.create_subscription(VehicleCommand, f'{p}/in/vehicle_command', self._on_cmd, _qos())
        # R-017：原始命令注入（harness → jammer）
        self.pub_jam_cmd = self.create_publisher(String, JAM_CMD_TOPIC, 10)
        # R-017：作用包络（harness → jammer），可控 publishing / value
        self.pub_env = self.create_publisher(Bool, ENV_TOPIC, 10)
        # R-017：已校验授权事件观测（jammer → 下游）
        self.create_subscription(String, JAM_STATE_TOPIC, self._on_jam_state, 10)
        self.epoch_pub = None
        self._new_epoch_pub()

        self.sp = []            # (wall_t, x, y, z)
        self.cmds = []          # (wall_t, command, param1)
        self.jam_states = []    # (wall_t, data)  已校验授权事件
        self.pos = list(SPAWN)
        self.vel = [0.0, 0.0, 0.0]
        self.last_sp = None
        self.moves = False
        self.status_fn = None
        self.driver_t0 = None
        # 可控包络
        self.env_publishing_ = False
        self.env_value_ = True

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

    def _on_jam_state(self, m):
        self.jam_states.append((time.monotonic(), m.data.strip().upper()))

    def publish_epoch(self, val):
        m = Int32()
        m.data = int(val)
        self.epoch_pub.publish(m)
        return time.monotonic()

    def publish_jam_cmd(self, s):
        m = String()
        m.data = s
        self.pub_jam_cmd.publish(m)
        return time.monotonic()

    def _publish_env(self):
        m = Bool()
        m.data = bool(self.env_value_)
        self.pub_env.publish(m)

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
        """以 ~dt 节拍循环：推进运动模型 → 发 odom/status（+包络若 publishing）→ 处理回调。"""
        t0 = time.monotonic()
        n = 0
        while True:
            now = time.monotonic()
            if now - t0 >= dur:
                break
            self._step()
            self._publish_odom()
            self._publish_status()
            if self.env_publishing_:
                self._publish_env()
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


# ---- 进程管理 ----
def _exe(share, name):
    return str(Path(share).parents[1] / 'lib' / 'agc_task' / name)


def _write_params(share, path, node_name, overrides):
    base = yaml.safe_load(open(os.path.join(share, 'config', 'w6_base.yaml')))
    params = dict((base.get(node_name) or {}).get('ros__parameters', {}))
    params.update(overrides)
    params['use_sim_time'] = False   # 墙钟，规避无 /clock 时间冻结
    doc = {node_name: {'ros__parameters': params}}
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
    return params


def _start_case(node, share, out_dir, case, driver_overrides):
    """排空上一用例残留 → 重建 epoch → 清缓冲 → 起 jammer + intruder_driver 两 subprocess。"""
    node.moves = False
    node.status_fn = None
    node.env_publishing_ = False
    node.run(0.4)
    node._new_epoch_pub()          # 逐用例重建 epoch 发布（清 transient 历史）
    node.sp.clear()
    node.cmds.clear()
    node.jam_states.clear()
    node.reset_model()
    node.last_sp = None
    node.moves = False
    node.status_fn = lambda t: (2, 14)
    node.driver_t0 = time.monotonic()

    jam_path = os.path.join(out_dir, 'r017_%s_jammer.yaml' % case)
    drv_path = os.path.join(out_dir, 'r017_%s_driver.yaml' % case)
    _write_params(share, jam_path, 'jammer', {})
    _write_params(share, drv_path, 'intruder_driver', driver_overrides)
    log_j = open(os.path.join(out_dir, 'log', 'case_%s_jammer.log' % case), 'w')
    log_d = open(os.path.join(out_dir, 'log', 'case_%s_driver.log' % case), 'w')
    proc_j = subprocess.Popen(
        [_exe(share, 'jammer'), '--ros-args', '--params-file', jam_path],
        stdout=log_j, stderr=subprocess.STDOUT)
    proc_d = subprocess.Popen(
        [_exe(share, 'intruder_driver'), '--ros-args', '--params-file', drv_path],
        stdout=log_d, stderr=subprocess.STDOUT)
    return proc_d, proc_j, (log_d, log_j)


def _stop_proc(proc, log):
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


def _stop_case(proc_d, proc_j, logs):
    _stop_proc(proc_d, logs[0])
    _stop_proc(proc_j, logs[1])


def _auth_events(node):
    """返回**已校验授权事件**（`JAM_*` 且非 `JAM_NONE`）的数据列表。"""
    return [e[1] for e in node.jam_states if e[1].startswith('JAM_') and e[1] != 'JAM_NONE']


def _jammer_nav_cmds(node):
    """返回 jammer 注入的 `SET_NAV_STATE`（param1 ∈ {RTL/LAND/HOLD}）——区别于 driver 的 OFFBOARD(14)。"""
    return [(c[0], c[2]) for c in node.cmds
            if c[1] == NAV_STATE_CMD and any(abs(c[2] - p) < 0.5 for p in JAM_NAV_PARAM1)]


# ---- 用例 ----
def case_a(node, share, out_dir):
    """未知命令不切断（包络新鲜合法，但命令非法 → 不产出授权事件、流继续）。"""
    res = {}
    proc_d, proc_j, logs = _start_case(
        node, share, out_dir, 'a',
        {'motion': 'linear', 'speed_mps': 1.0, 'run_region_radius_m': 100.0})
    try:
        node.env_publishing_ = True
        node.env_value_ = True
        node.moves = True
        node.run_until(lambda: len(node.sp) > 3, 2.0)
        node.run(1.2)
        n0 = len(node.sp)
        node.publish_jam_cmd('BOGUS')
        node.run(1.2)
        n1 = len(node.sp)
        auth = _auth_events(node)
        nav = _jammer_nav_cmds(node)
        res['sp_count_before'] = n0
        res['sp_count_after'] = n1
        res['sp_delta'] = n1 - n0
        res['authorized_events'] = auth
        res['jammer_nav_cmds'] = [p for _, p in nav]
        res['pass'] = bool(res['sp_delta'] > 10 and len(auth) == 0 and len(nav) == 0)
    finally:
        _stop_case(proc_d, proc_j, logs)
    return res


def case_b(node, share, out_dir):
    """包络外 / 过期不生效（两子测都要过）。"""
    res = {}
    ov = {'motion': 'linear', 'speed_mps': 1.0, 'run_region_radius_m': 100.0}

    # b1：包络 publishing=true/value=false（包络外）
    proc_d, proc_j, logs = _start_case(node, share, out_dir, 'b1', ov)
    try:
        node.env_publishing_ = True
        node.env_value_ = False
        node.moves = True
        node.run_until(lambda: len(node.sp) > 3, 2.0)
        node.run(0.6)
        n0 = len(node.sp)
        node.publish_jam_cmd('RTL')
        node.run(1.2)
        n1 = len(node.sp)
        auth = _auth_events(node)
        res['b1_out_of_envelope'] = {
            'sp_delta': n1 - n0, 'authorized_events': auth, 'pass': bool(n1 - n0 > 10 and len(auth) == 0)}
    finally:
        _stop_case(proc_d, proc_j, logs)

    # b2：包络曾新鲜 (0.6 s) → 停发 > envelope_max_age_s(0.5) → 过期
    proc_d, proc_j, logs = _start_case(node, share, out_dir, 'b2', ov)
    try:
        node.env_publishing_ = True
        node.env_value_ = True
        node.moves = True
        node.run_until(lambda: len(node.sp) > 3, 2.0)
        node.run(0.6)                    # 新鲜窗口
        node.env_publishing_ = False     # 停发包络
        node.run(1.2)                    # > 0.5 s + 裕度 → 过期
        n0 = len(node.sp)
        node.publish_jam_cmd('RTL')
        node.run(1.2)
        n1 = len(node.sp)
        auth = _auth_events(node)
        res['b2_envelope_stale'] = {
            'sp_delta': n1 - n0, 'authorized_events': auth, 'pass': bool(n1 - n0 > 10 and len(auth) == 0)}
    finally:
        _stop_case(proc_d, proc_j, logs)

    res['pass'] = bool(res['b1_out_of_envelope']['pass'] and res['b2_envelope_stale']['pass'])
    return res


def case_c(node, share, out_dir):
    """包络内合法 → 授权事件 → 停发 Offboard（流可观测）+ CLEAR 连续。"""
    res = {}
    proc_d, proc_j, logs = _start_case(
        node, share, out_dir, 'c',
        {'motion': 'linear', 'speed_mps': 1.0, 'run_region_radius_m': 100.0})
    try:
        node.env_publishing_ = True
        node.env_value_ = True
        node.run_until(lambda: len(node.sp) > 3, 2.0)
        t_epoch = node.publish_epoch(1)
        node.run(0.2)                          # 冻结期重锚 origin=(0,0,0)
        node.moves = True
        node.run_until(lambda: any(s[2] > 0.05 for s in node.sp if s[0] > t_epoch), 4.0)
        node.run(1.5)                          # t0_ 已置、轨迹推进

        t_jam = node.publish_jam_cmd('RTL')
        node.run(0.8)                          # 让授权事件 → driver 停发（吸收投递延迟）

        # 流已冻结：记 jam **前末个 setpoint**（= 冻结前最后发布的 setpoint，对齐 r016 口径）
        c_settle = len(node.sp)
        before = node.sp[c_settle - 1] if c_settle > 0 else None
        frozen = [s for s in node.sp if s[0] > t_jam + 0.6]

        auth = _auth_events(node)
        nav = [(t, p) for t, p in _jammer_nav_cmds(node) if t > t_jam]

        res['authorized_events'] = auth
        res['jammer_nav_cmds_after_jam'] = [[round(t - t_jam, 3), p] for t, p in nav]
        res['sp_after_jam_plus_0p6'] = len(frozen)
        res['jam_froze_stream'] = bool(len(frozen) == 0)
        res['setpoint_before'] = [round(v, 3) for v in before[1:4]] if before else None

        t_clear = node.publish_jam_cmd('CLEAR')
        node.run_until(lambda: len(node.sp) > c_settle, 2.0)
        after = next((s for s in node.sp if s[0] > t_clear), None)
        res['setpoint_after'] = [round(v, 3) for v in after[1:4]] if after else None
        gap = abs(after[2] - before[2]) if (after and before) else None
        res['progress_gap_m'] = round(gap, 4) if gap is not None else None

        res['pass'] = bool(
            'JAM_RTL' in auth and
            len(nav) > 0 and
            res['jam_froze_stream'] and
            gap is not None and gap < 0.3)
    finally:
        _stop_case(proc_d, proc_j, logs)
    return res


def case_d(node, share, out_dir, repo_root):
    """grep：driver 无 `jam_cmd` 字面；yaml `intruder_driver` 段无 `jam_cmd_topic` 键。"""
    res = {}
    drv_txt = open(os.path.join(repo_root, 'src/agc_task/scripts/intruder_driver.py')).read()
    cfg = yaml.safe_load(open(os.path.join(repo_root, 'src/agc_task/config/w6_base.yaml')))
    idrv = (cfg.get('intruder_driver') or {}).get('ros__parameters', {})
    res['driver_has_jam_cmd_literal'] = bool('jam_cmd' in drv_txt)
    res['intruder_driver_has_jam_cmd_topic_key'] = bool('jam_cmd_topic' in idrv)
    res['intruder_driver_keys'] = sorted(idrv.keys())
    res['pass'] = bool(
        not res['driver_has_jam_cmd_literal'] and
        not res['intruder_driver_has_jam_cmd_topic_key'])
    return res


def dump(res, out_dir):
    path = os.path.join(out_dir, 'metrics', 'r017.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-32s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
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
    ap.add_argument('--repo-root', default='/home/chen/AGC')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r017-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    share = get_package_share_directory('agc_task')
    res = {'run_id': run_id, 'ns': NS, 'tests': {}}

    rclpy.init()
    node = FakePX4()
    try:
        res['tests']['a_unknown_cmd_no_cut'] = case_a(node, share, out_dir)
        res['tests']['b_envelope_gate'] = case_b(node, share, out_dir)
        res['tests']['c_validated_auth_and_clear'] = case_c(node, share, out_dir)
        res['tests']['d_grep_no_raw_path'] = case_d(node, share, out_dir, args.repo_root)
        res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return dump(res, out_dir)


if __name__ == '__main__':
    sys.exit(main())
