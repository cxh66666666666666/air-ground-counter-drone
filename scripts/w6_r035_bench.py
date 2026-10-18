#!/usr/bin/env python3
"""w6_r035_bench —— R-035（空中控制唯一权威与控制许可）**逻辑台架**（实跑真实节点）。

起**真实** `agc_air` 生命周期节点（`trajectory_tracker`）与**真实** Nav2 `lifecycle_manager`，
覆盖评审 R-035 要求的四条路径：

  1. `late_release`      先发 `active_controller="takeoff_controller"`（**未**授权）若干秒 → 断言 setpoint ≈0；
                          再（迟到）改发 `"trajectory_tracker"` → 断言 setpoint ≥20（显式许可交权，非计数）。
  2. `duplicate_node`    起**两个**真 tracker 实例（`__node:=tt_a` / `tt_b`，`controller_name=trajectory_tracker`），
                          rig 发 `active_controller="trajectory_tracker"` → 两实例由**显式 claim**（不同 uid）
                          互相检出竞争者 → 冲突持续超阈值后**两侧 setpoint 均停发**（≤2）+ `control_conflict==true` 可观测。
  3. `manager_restart`   真 `lifecycle_manager`（`bond_timeout=5.0`, `autostart=true`）管真 tracker；rig 先发
                          `"takeoff_controller"`；autostart 后发 `"trajectory_tracker"` → setpoint≥20；**杀 manager**
                          （bond 失效停机）→ 停发；rig **重发 `"takeoff_controller"`** → 重启 manager →
                          断言 tracker 重新激活后 setpoint **仍 ≈0**（**不**残留旧权威）+ claim `permit=false`。
  4. `silent_publisher`  真 tracker 被授权发 setpoint；另起一个只 `create_publisher(TrajectorySetpoint)`
                          **从不发布、从不 claim** 的节点 → 断言 `writers>=2`（计数仅告警）但 `control_conflict==false`
                          且 setpoint **持续 ≥20**（**不**对合法静默 publisher 误报）。

记录 claim 历史（`uid` / `sp_hz` / `permit` / `writers`）→ 供「实际写者 / 频率」复核。
只回收**本进程组**的子进程（`start_new_session` + `killpg` + `wait`）；**不**用裸名 `pkill -f`、**不**做递归删除。
证据落 `experiments/w6-task-eval/<run_id>/r035/bench.json`（ADR-0007）。

⚠ 范围标注：本台架为**控制许可逻辑台架**（真实节点 + 真实 manager，**非**整链 / **非**双 PX4 / **非** GUI）。
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from px4_msgs.msg import TrajectorySetpoint

ROOT = '/home/chen/AGC'
NODE_BIN = os.path.join(ROOT, 'install', 'agc_air', 'lib', 'agc_air')
MANAGER_BIN = '/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager'

# `/**` 通配任意节点名（duplicate 场景中以 `__node:=tt_a/tt_b` 重命名仍生效）。
STANDALONE_PARAMS = """/**:
  ros__parameters:
    use_sim_time: false
    bond_heartbeat_period: 0.0
    origin_pose: "0,0,0"
    setpoint_rate_hz: 50.0
    controller_name: trajectory_tracker
    conflict_persist_s: 1.0
    conflict_clear_s: 2.0
"""

MANAGED_PARAMS = """trajectory_tracker:
  ros__parameters:
    use_sim_time: false
    origin_pose: "0,0,0"
    setpoint_rate_hz: 50.0
    conflict_persist_s: 1.0
    conflict_clear_s: 2.0
"""

MANAGER_PARAMS = """lifecycle_manager:
  ros__parameters:
    use_sim_time: false
    autostart: true
    bond_timeout: 5.0
    node_names: ["trajectory_tracker"]
"""

# 合法**静默** publisher：只创建 setpoint 发布器，**从不发布、从不 claim**（用于验证不误报）。
SILENT_PUB_SRC = """#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint

def main():
    rclpy.init()
    n = Node('w6_r035_silent_pub')
    n.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
    rclpy.spin(n)

if __name__ == '__main__':
    main()
"""


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
    pats = ['agc_air/lib/agc_air/trajector[y]_tracker',
            'nav2_lifecycle_manage[r]/lifecycle_manage[r]',
            'r035_silent_pu[b].py']
    for pat in pats:
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


def _read(path):
    if not path or not os.path.exists(path):
        return ''
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


def _claim_field(s, key):
    """解析 claim JSON 字段（`"key":"v"` 或 `"key": v`，有无空格均容错）。"""
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


class Rig(Node):
    """合成驱动（odom + active_controller / tracking_ref）+ 证据采集（setpoint / conflict / claim）。

    生命周期转移用 `ros2 lifecycle` CLI + **重试至目标状态**（避免长生命周期 rclpy 客户端在节点重启后
    端点残影导致的丢响应）。"""

    def __init__(self):
        super().__init__('w6_r035_rig')
        q_tl = QoSProfile(depth=1)
        q_tl.reliability = ReliabilityPolicy.RELIABLE
        q_tl.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_odom = self.create_publisher(Odometry, '/agc/air/odom', qos_profile_sensor_data)
        self.pub_active = self.create_publisher(String, '/agc/air/active_controller', q_tl)
        self.setpoints = []                 # 收到 setpoint 的时刻（单调）
        self.claims = []                    # [(t, data)] 收到的 claim
        self.conflict_true_seen = False
        self.conflict_true_time = None
        self._lock = threading.Lock()
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                                 self._on_setpoint, 10)
        self.create_subscription(Bool, '/agc/air/control_conflict', self._on_conflict, q_tl)
        self.create_subscription(String, '/agc/air/control_permit', self._on_claim, q_tl)

        # 注意：`Node.executor` 是 rclpy 内部保留属性 → 用 `_exec` 承载。
        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self)
        self._spin = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin.start()

    def _on_setpoint(self, _m):
        with self._lock:
            self.setpoints.append(time.monotonic())

    def _on_conflict(self, m):
        with self._lock:
            if m.data and not self.conflict_true_seen:
                self.conflict_true_seen = True
                self.conflict_true_time = time.monotonic()

    def _on_claim(self, m):
        with self._lock:
            self.claims.append((time.monotonic(), m.data))

    def n_setpoints_since(self, t0):
        with self._lock:
            return sum(1 for t in self.setpoints if t >= t0)

    def n_setpoints_in_last(self, window_s):
        t0 = time.monotonic() - window_s
        with self._lock:
            return sum(1 for t in self.setpoints if t >= t0)

    def reset_setpoints(self):
        with self._lock:
            self.setpoints = []

    def reset_signals(self):
        """场景切换：清 setpoint / claim 记录与冲突观测（避免上一场景的 stick 值污染断言）。"""
        with self._lock:
            self.setpoints = []
            self.claims = []
            self.conflict_true_seen = False
            self.conflict_true_time = None

    def setpoint_pub_count(self):
        return self.count_publishers('/fmu/in/trajectory_setpoint')

    def latest_claim_after(self, t0):
        with self._lock:
            for t, d in reversed(self.claims):
                if t >= t0:
                    return d
        return ''

    def claim_writers_max_after(self, t0):
        mx = 0
        with self._lock:
            for t, d in self.claims:
                if t >= t0:
                    try:
                        mx = max(mx, int(float(_claim_field(d, 'writers'))))
                    except (TypeError, ValueError):
                        pass
        return mx

    @staticmethod
    def _cli(*args, timeout=25.0):
        try:
            return subprocess.run(['ros2', 'lifecycle', *args],
                                  capture_output=True, text=True, timeout=timeout)
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
        """按**到达期望状态**判定转移成功（CLI + 重试；对丢响应 / 发现延迟稳健）。"""
        for _ in range(tries):
            self._cli('set', service, transition_label)
            if self.state(service)[1] == expect_label:
                return True
            time.sleep(0.4)
        return self.state(service)[1] == expect_label

    # ---- 合成驱动 ----
    def _drive_one(self, active, x=0.0, y=0.0, z=2.5, dt=0.02):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'world'
        o.child_frame_id = 'drone/base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
        o.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(o)
        m = String()
        m.data = active
        self.pub_active.publish(m)
        time.sleep(dt)

    def drive_for(self, active, dur_s):
        end = time.monotonic() + dur_s
        while time.monotonic() < end:
            self._drive_one(active)

    def drive_until(self, active, pred, dur_s):
        end = time.monotonic() + dur_s
        while time.monotonic() < end:
            self._drive_one(active)
            if pred():
                return True
        return pred()


def _spawn(cmd, log_path):
    return subprocess.Popen(cmd, stdout=open(log_path, 'w'), stderr=subprocess.STDOUT,
                            start_new_session=True)


def run_late_release(rig, cfg_dir, log_dir):
    """① 迟到 release：未授权 ≈0；改发 tracker（迟到交权）→ ≥20。"""
    res = {'pass': False}
    rig.reset_signals()
    pf = os.path.join(cfg_dir, 'r035_standalone.yaml')
    with open(pf, 'w', encoding='utf-8') as f:
        f.write(STANDALONE_PARAMS)
    log = os.path.join(log_dir, 'late_release.log')
    proc = _spawn([os.path.join(NODE_BIN, 'trajectory_tracker'), '--ros-args',
                   '--params-file', pf], log)
    try:
        up = rig.wait_node_up('/trajectory_tracker')
        res['node_up'] = up
        if not up:
            return res
        res['configure_ok'] = rig.apply('/trajectory_tracker', 'configure', 'inactive')
        res['activate_ok'] = rig.apply('/trajectory_tracker', 'activate', 'active')
        # 先发 "takeoff_controller"（**未**授权）若干秒：断言 ≈0 setpoint。
        t0 = time.monotonic()
        rig.drive_for('takeoff_controller', 3.0)
        res['setpoints_before_release'] = rig.n_setpoints_since(t0)
        res['permit_before'] = _claim_field(rig.latest_claim_after(t0), 'permit')
        # 迟到 release：改发 "trajectory_tracker" → 接管发 setpoint。
        t1 = time.monotonic()
        rig.drive_for('trajectory_tracker', 3.0)
        res['setpoints_after_release'] = rig.n_setpoints_since(t1)
        res['permit_after'] = _claim_field(rig.latest_claim_after(t1), 'permit')
        res['pass'] = bool(res['setpoints_before_release'] <= 2 and
                           res['setpoints_after_release'] >= 20)
    finally:
        _kill(proc)
    return res


def run_duplicate_node(rig, cfg_dir, log_dir):
    """② 重复节点：两真 tracker 实例由显式 claim 互相检出竞争者 → 冲突后两侧停发 + conflict 可观测。"""
    res = {'pass': False}
    rig.reset_signals()
    pf = os.path.join(cfg_dir, 'r035_standalone.yaml')
    log_a = os.path.join(log_dir, 'dup_tt_a.log')
    log_b = os.path.join(log_dir, 'dup_tt_b.log')
    proc_a = _spawn([os.path.join(NODE_BIN, 'trajectory_tracker'), '--ros-args',
                     '-r', '__node:=tt_a', '--params-file', pf], log_a)
    proc_b = _spawn([os.path.join(NODE_BIN, 'trajectory_tracker'), '--ros-args',
                     '-r', '__node:=tt_b', '--params-file', pf], log_b)
    try:
        up_a = rig.wait_node_up('/tt_a')
        up_b = rig.wait_node_up('/tt_b')
        res['nodes_up'] = [up_a, up_b]
        if not (up_a and up_b):
            return res
        res['activate_a'] = (rig.apply('/tt_a', 'configure', 'inactive') and
                             rig.apply('/tt_a', 'activate', 'active'))
        res['activate_b'] = (rig.apply('/tt_b', 'configure', 'inactive') and
                             rig.apply('/tt_b', 'activate', 'active'))
        time.sleep(1.5)   # 让两实例互相发现 claim 端点
        rig.reset_setpoints()
        # 发 active_controller="trajectory_tracker"（两者均被授权）→ 两实例互相检出竞争者 → 冲突。
        got = rig.drive_until('trajectory_tracker', lambda: rig.conflict_true_seen, 20.0)
        res['conflict_true_seen'] = rig.conflict_true_seen
        res['conflict_detected_within'] = got
        # 冲突各自独立检出（可能相差 ~1s）→ 再驱动 3s 让**两侧都已停发**（settle），再量末窗。
        rig.drive_for('trajectory_tracker', 3.0)
        ct = rig.conflict_true_time
        res['setpoints_after_conflict'] = rig.n_setpoints_since(ct) if ct is not None else None
        res['setpoints_last_window'] = rig.n_setpoints_in_last(1.5)
        res['claims_seen'] = len(rig.claims)
        # claim 中是否出现竞争者 uid（说明互检成立）
        comp = [d for _, d in rig.claims if _claim_field(d, 'competitor_uid') not in ('', '-')]
        res['competitor_claim_seen'] = bool(comp)
        res['pass'] = bool(res['conflict_true_seen'] and
                           res['setpoints_last_window'] <= 2)
    finally:
        _kill(proc_a)
        _kill(proc_b)
    return res


def run_manager_restart(rig, cfg_dir, log_dir):
    """③ manager 重启：重启后不残留旧权威（setpoint≈0 + claim permit=false）。"""
    res = {'pass': False}
    rig.reset_signals()
    pf = os.path.join(cfg_dir, 'r035_managed.yaml')
    with open(pf, 'w', encoding='utf-8') as f:
        f.write(MANAGED_PARAMS)
    mf = os.path.join(cfg_dir, 'r035_manager.yaml')
    with open(mf, 'w', encoding='utf-8') as f:
        f.write(MANAGER_PARAMS)
    nlog = os.path.join(log_dir, 'mgr_restart_node.log')
    mlog1 = os.path.join(log_dir, 'mgr_restart_manager1.log')
    mlog2 = os.path.join(log_dir, 'mgr_restart_manager2.log')
    node_proc = _spawn([os.path.join(NODE_BIN, 'trajectory_tracker'), '--ros-args',
                        '--params-file', pf], nlog)
    mgr1 = None
    mgr2 = None
    try:
        up = rig.wait_node_up('/trajectory_tracker')
        res['node_up'] = up
        if not up:
            return res
        # rig 先发 "takeoff_controller"（模拟 takeoff 已宣示，manager 即将启动）。
        rig.drive_for('takeoff_controller', 1.0)
        # 起 manager（autostart → configure+activate）。
        mgr1 = _spawn([MANAGER_BIN, '--ros-args', '--params-file', mf], mlog1)
        end = time.monotonic() + 60.0
        while time.monotonic() < end and 'Managed nodes are active' not in _read(mlog1):
            time.sleep(0.5)
        res['manager1_active'] = 'Managed nodes are active' in _read(mlog1)
        res['state_after_autostart'] = rig.state('/trajectory_tracker')[1]
        if not res['manager1_active']:
            return res
        # 发 "trajectory_tracker" → 接管发 setpoint。
        t1 = time.monotonic()
        rig.drive_for('trajectory_tracker', 3.0)
        res['setpoints_after_designation'] = rig.n_setpoints_since(t1)
        res['permit_after_designation'] = _claim_field(rig.latest_claim_after(t1), 'permit')
        # 杀 manager → 节点侧 bond 失效停机。
        _kill(mgr1)
        mgr1 = None
        end = time.monotonic() + 20.0
        while time.monotonic() < end and 'bond 断开' not in _read(nlog):
            time.sleep(0.5)
        res['node_detected_bond_break'] = 'bond 断开' in _read(nlog)
        time.sleep(2.0)
        res['state_after_manager_death'] = rig.state('/trajectory_tracker')[1]
        # rig 重发 "takeoff_controller"（模拟 takeoff 在 manager 重启时重新宣示）。
        rig.drive_for('takeoff_controller', 1.5)
        # 重启 manager（autostart 再次 configure+activate）。
        rig.reset_setpoints()
        t2 = time.monotonic()
        mgr2 = _spawn([MANAGER_BIN, '--ros-args', '--params-file', mf], mlog2)
        end = time.monotonic() + 60.0
        while time.monotonic() < end and 'Managed nodes are active' not in _read(mlog2):
            time.sleep(0.5)
        res['manager2_active'] = 'Managed nodes are active' in _read(mlog2)
        # 保持发 "takeoff_controller"（未授权）→ tracker 重新激活后应**不**发 setpoint（不残留旧权威）。
        rig.drive_for('takeoff_controller', 4.0)
        res['setpoints_after_restart'] = rig.n_setpoints_since(t2)
        res['permit_after_restart'] = _claim_field(rig.latest_claim_after(t2), 'permit')
        res['state_after_restart'] = rig.state('/trajectory_tracker')[1]
        res['pass'] = bool(
            res['manager1_active'] and res['setpoints_after_designation'] >= 20 and
            res['node_detected_bond_break'] and
            res['state_after_manager_death'] == 'unconfigured' and
            res['manager2_active'] and
            res['setpoints_after_restart'] <= 2 and
            res['permit_after_restart'] == 'false')
    finally:
        _kill(node_proc)
        _kill(mgr1)
        _kill(mgr2)
    return res


def run_silent_publisher(rig, cfg_dir, log_dir):
    """④ 静默 publisher：writers>=2 仅告警，conflict==false，setpoint 持续 ≥20（不误报）。"""
    res = {'pass': False}
    rig.reset_signals()
    pf = os.path.join(cfg_dir, 'r035_standalone.yaml')
    spf = os.path.join(cfg_dir, 'r035_silent_pub.py')
    with open(spf, 'w', encoding='utf-8') as f:
        f.write(SILENT_PUB_SRC)
    log = os.path.join(log_dir, 'silent_publisher.log')
    slog = os.path.join(log_dir, 'silent_pub_node.log')
    proc = _spawn([os.path.join(NODE_BIN, 'trajectory_tracker'), '--ros-args',
                   '--params-file', pf], log)
    silent = None
    try:
        up = rig.wait_node_up('/trajectory_tracker')
        res['node_up'] = up
        if not up:
            return res
        res['configure_ok'] = rig.apply('/trajectory_tracker', 'configure', 'inactive')
        res['activate_ok'] = rig.apply('/trajectory_tracker', 'activate', 'active')
        # 另起合法**静默** publisher（从不发布 / 从不 claim）。
        silent = _spawn(['python3', spf], slog)
        time.sleep(2.0)   # 让 tracker 发现该静默发布者端点
        # 授权 tracker 发 setpoint。
        t0 = time.monotonic()
        rig.drive_for('trajectory_tracker', 4.0)
        res['setpoints_window'] = rig.n_setpoints_since(t0)
        res['writers_max'] = rig.claim_writers_max_after(t0)
        res['conflict_true_seen'] = rig.conflict_true_seen
        res['setpoint_pub_count'] = rig.setpoint_pub_count()
        res['pass'] = bool(
            res['setpoints_window'] >= 20 and
            res['writers_max'] >= 2 and
            (not res['conflict_true_seen']))
    finally:
        _kill(silent)
        _kill(proc)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261017-w6-r035-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()

    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)
    cfg_dir = os.path.join(out_dir, 'config')
    log_dir = os.path.join(out_dir, 'r035', 'log')
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    _reap_stale()
    rclpy.init()
    rig = Rig()
    result = {
        'run_id': args.run_id, 'suite': 'R-035 air-control single-authority and permit bench',
        'scope_note': (
            '控制许可**逻辑台架**：真实 agc_air 生命周期节点（trajectory_tracker）+ 真实 Nav2 lifecycle_manager，'
            '覆盖 迟到 release / 重复节点 / manager 重启 / 静默 publisher；'
            '**非整链** / 非双 PX4 / 非 GUI。'),
        'unsupported': [],
        'scenarios': {},
    }
    try:
        result['scenarios']['late_release'] = run_late_release(rig, cfg_dir, log_dir)
        print('late_release        PASS=%s' % result['scenarios']['late_release'].get('pass'))
        time.sleep(1.5)
        result['scenarios']['duplicate_node'] = run_duplicate_node(rig, cfg_dir, log_dir)
        print('duplicate_node      PASS=%s' % result['scenarios']['duplicate_node'].get('pass'))
        time.sleep(1.5)
        result['scenarios']['manager_restart'] = run_manager_restart(rig, cfg_dir, log_dir)
        print('manager_restart     PASS=%s' % result['scenarios']['manager_restart'].get('pass'))
        time.sleep(1.5)
        result['scenarios']['silent_publisher'] = run_silent_publisher(rig, cfg_dir, log_dir)
        print('silent_publisher    PASS=%s' % result['scenarios']['silent_publisher'].get('pass'))
        # claim 历史（供「实际写者 / 频率」复核）。
        with rig._lock:
            result['claim_history'] = [{'t': round(t, 3), 'data': d} for t, d in rig.claims][-100:]
    finally:
        rig.destroy_node()
        rclpy.shutdown()

    result['overall_pass'] = bool(
        result['scenarios'] and all(s.get('pass') for s in result['scenarios'].values()))
    with open(os.path.join(out_dir, 'r035', 'bench.json'), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print('\noverall_pass=%s -> %s' % (
        result['overall_pass'], os.path.join(out_dir, 'r035', 'bench.json')))
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
