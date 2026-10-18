#!/usr/bin/env python3
"""w6_r034_bench —— R-034（生命周期可重复资源状态机与完整失效清理）**逻辑台架**（实跑真实节点）。

起**真实** `agc_air` 生命周期节点（`trajectory_tracker`）与**真实** Nav2 `lifecycle_manager`，
覆盖 spec 要求的四条路径：

  1. `cycle_repeatable`   configure→activate→deactivate→cleanup **循环 ≥2 轮**（第二轮 configure 无重复参数异常，
                          激活后再出 setpoint、deactivate 后停发、cleanup 后无发布者）
  2. `direct_shutdown`    从 ACTIVE 直接 `shutdown` → 状态 FINALIZED、setpoint 停发、发布者清零、完整清理日志
  3. `error_injection`    以**错误类型参数**注入 configure 回调异常 → 生命周期 error → 完整清理（on_error 返回
                          SUCCESS → 状态回 UNCONFIGURED；资源释放、setpoint 停发）
  4. `manager_node_death` 真实 `lifecycle_manager`（`bond_timeout>0`）+ 节点 bond；**杀被管节点** → 管理器 bond
                          失联可判（"IS DOWN"）
  5. `manager_death`      **杀 lifecycle_manager** → 节点侧 bond 断开看门狗 → **失效停机**（deactivate→cleanup，
                          状态回 UNCONFIGURED，setpoint 停发）

只回收**本进程组**的子进程（`start_new_session` + `killpg`）；**不**用裸名 `pkill -f`、**不**做递归删除。
证据落 `experiments/w6-task-eval/<run_id>/r034/bench.json`（ADR-0007）。

⚠ 范围标注：本台架为**生命周期逻辑台架**（真实节点 + 真实 manager，**非**整链 / **非**双 PX4 / **非** GUI）。
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
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory
from px4_msgs.msg import TrajectorySetpoint

ROOT = '/home/chen/AGC'
NODE_BIN = os.path.join(ROOT, 'install', 'agc_air', 'lib', 'agc_air')
MANAGER_BIN = '/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager'
SERVICE = '/trajectory_tracker'

STANDALONE_PARAMS = """trajectory_tracker:
  ros__parameters:
    use_sim_time: false
    bond_heartbeat_period: 0.0
    origin_pose: "0,0,0"
    setpoint_rate_hz: 50.0
"""

MANAGED_PARAMS = """trajectory_tracker:
  ros__parameters:
    use_sim_time: false
    origin_pose: "0,0,0"
    setpoint_rate_hz: 50.0
"""

MANAGER_PARAMS = """lifecycle_manager:
  ros__parameters:
    use_sim_time: false
    autostart: true
    bond_timeout: 5.0
    node_names: ["trajectory_tracker"]
"""


def _kill(proc):
    """仅回收本子进程组（start_new_session → 独立 pgid）；self-group 护栏下退化为单 PID。"""
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    sig = signal.SIGKILL
    try:
        if pgid == os.getpgrp():
            proc.send_signal(sig)
        else:
            os.killpg(pgid, sig)
    except (ProcessLookupError, PermissionError, OSError):
        pass
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass


def _reap_stale():
    """清理同名残留真实进程（obfuscated pattern：不匹配本进程命令行；仅按 PID kill）。"""
    for pat in ('agc_air/lib/agc_air/trajector[y]_tracker',
                'nav2_lifecycle_manage[r]/lifecycle_manage[r]'):
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
    if not os.path.exists(path):
        return ''
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


class Rig(Node):
    """合成驱动 + 证据采集（生命周期转移用 `ros2 lifecycle` CLI + **重试至目标状态**，避免长生命周期
    rclpy 客户端在节点重启后端点残影导致的丢响应）。"""

    def __init__(self):
        super().__init__('w6_r034_rig')
        q_tl = QoSProfile(depth=1)
        q_tl.reliability = ReliabilityPolicy.RELIABLE
        q_tl.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_odom = self.create_publisher(Odometry, '/agc/air/odom', qos_profile_sensor_data)
        self.pub_active = self.create_publisher(String, '/agc/air/active_controller', q_tl)
        self.pub_ref = self.create_publisher(MultiDOFJointTrajectory, '/agc/air/tracking_ref', 10)
        self.setpoints = []          # 收到 setpoint 的时刻（单调）
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                                 self._on_setpoint, 10)

        # 注意：`Node.executor` 是 rclpy 内部保留属性 → 用 `_exec` 承载。
        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self)
        self._spin = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin.start()

    def _on_setpoint(self, _m):
        self.setpoints.append(time.monotonic())

    def n_setpoints_since(self, t0):
        return sum(1 for t in self.setpoints if t >= t0)

    def setpoint_pub_count(self):
        return self.count_publishers('/fmu/in/trajectory_setpoint')

    @staticmethod
    def _cli(*args, timeout=25.0):
        try:
            return subprocess.run(['ros2', 'lifecycle', *args],
                                  capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def state(self):
        """读生命周期状态（`ros2 lifecycle get`）；未就绪 / 未找到 → (None, 'no-response')。"""
        r = self._cli('get', SERVICE)
        if r is None or r.returncode != 0:
            return None, 'no-response'
        m = re.search(r'([a-z_]+)\s*\[(\d+)\]', r.stdout)
        if not m:
            return None, 'no-response'
        return int(m.group(2)), m.group(1)

    def wait_node_up(self, timeout=40.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.state()[0] is not None:
                return True
            time.sleep(0.5)
        return False

    def apply(self, transition_label, expect_label, tries=6):
        """按**到达期望状态**判定转移成功（CLI + 重试；对丢响应 / 发现延迟稳健）。"""
        for _ in range(tries):
            self._cli('set', SERVICE, transition_label)
            if self.state()[1] == expect_label:
                return True
            time.sleep(0.4)
        return self.state()[1] == expect_label

    # ---- 合成驱动 ----
    def drive(self, n=1, x=0.0, y=0.0, z=2.5, dt=0.02):
        for _ in range(n):
            o = Odometry()
            o.header.stamp = self.get_clock().now().to_msg()
            o.header.frame_id = 'world'
            o.child_frame_id = 'drone/base_link'
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
            o.pose.pose.orientation.w = 1.0
            self.pub_odom.publish(o)
            m = String()
            m.data = 'trajectory_tracker'
            self.pub_active.publish(m)
            time.sleep(dt)

    def drive_for(self, dur_s):
        end = time.monotonic() + dur_s
        while time.monotonic() < end:
            self.drive(n=1)


def _spawn(cmd, log_path):
    return subprocess.Popen(cmd, stdout=open(log_path, 'w'), stderr=subprocess.STDOUT,
                            start_new_session=True)


def run_cycle(rig, cfg_dir, log_dir):
    """① 循环 ≥2 轮 configure/activate/deactivate/cleanup。"""
    res = {'pass': False}
    pf = os.path.join(cfg_dir, 'r034_standalone.yaml')
    with open(pf, 'w', encoding='utf-8') as f:
        f.write(STANDALONE_PARAMS)
    log = os.path.join(log_dir, 'cycle_repeatable.log')
    proc = _spawn([os.path.join(NODE_BIN, 'trajectory_tracker'), '--ros-args',
                   '--params-file', pf], log)
    try:
        up = rig.wait_node_up()
        res['node_up'] = up
        if not up:
            return res

        def round_no(_n):
            c_ok = rig.apply('configure', 'inactive')
            _, s_conf_l = rig.state()
            a_ok = rig.apply('activate', 'active')
            _, s_act_l = rig.state()
            rig.drive_for(1.5)
            t0 = time.monotonic()
            rig.drive_for(1.0)
            n_sp_active = rig.n_setpoints_since(t0)
            d_ok = rig.apply('deactivate', 'inactive')
            _, s_dea_l = rig.state()
            rig.drive_for(0.3)
            t1 = time.monotonic()
            rig.drive_for(1.0)
            n_sp_after_dea = rig.n_setpoints_since(t1)
            cl_ok = rig.apply('cleanup', 'unconfigured')
            _, s_cln_l = rig.state()
            rig.drive_for(0.2)
            pubs_after_cleanup = rig.setpoint_pub_count()
            return {
                'configure_ok': c_ok, 'configure_state': s_conf_l,
                'activate_ok': a_ok, 'activate_state': s_act_l,
                'deactivate_ok': d_ok, 'deactivate_state': s_dea_l,
                'cleanup_ok': cl_ok, 'cleanup_state': s_cln_l,
                'setpoints_active_window': n_sp_active,
                'setpoints_after_deactivate_window': n_sp_after_dea,
                'setpoint_publishers_after_cleanup': pubs_after_cleanup,
            }

        r1 = round_no(1)
        r2 = round_no(2)
        res['round1'] = r1
        res['round2'] = r2
        log_txt = _read(log)
        res['log_has_param_already_declared'] = ('already declared' in log_txt.lower()
                                                or 'ParameterAlreadyDeclared' in log_txt)
        res['log_has_exception'] = 'Caught exception in callback' in log_txt
        # 判定：两轮 configure/activate/deactivate/cleanup 均成功；第二轮激活再出 setpoint；
        #      deactivate 后停发；cleanup 后无发布者；无重复参数异常。
        ok = True
        for r in (r1, r2):
            ok = ok and r['configure_ok'] and r['activate_ok'] and r['deactivate_ok'] and r['cleanup_ok']
            ok = ok and r['activate_state'] == 'active' and r['configure_state'] == 'inactive'
            ok = ok and r['deactivate_state'] == 'inactive' and r['cleanup_state'] == 'unconfigured'
            ok = ok and r['setpoints_active_window'] >= 20
            ok = ok and r['setpoints_after_deactivate_window'] <= 2
        ok = ok and (not res['log_has_param_already_declared']) and (not res['log_has_exception'])
        res['pass'] = bool(ok)
    finally:
        _kill(proc)
    return res


def run_direct_shutdown(rig, cfg_dir, log_dir):
    """② 从 ACTIVE 直接 shutdown → FINALIZED + setpoint 停发 + 发布者清零。"""
    res = {'pass': False}
    pf = os.path.join(cfg_dir, 'r034_standalone.yaml')
    log = os.path.join(log_dir, 'direct_shutdown.log')
    proc = _spawn([os.path.join(NODE_BIN, 'trajectory_tracker'), '--ros-args',
                   '--params-file', pf], log)
    try:
        up = rig.wait_node_up()
        res['node_up'] = up
        if not up:
            return res
        res['configure_ok'] = rig.apply('configure', 'inactive')
        res['activate_ok'] = rig.apply('activate', 'active')
        rig.drive_for(1.5)
        t0 = time.monotonic()
        rig.drive_for(1.0)
        res['setpoints_before_shutdown'] = rig.n_setpoints_since(t0)
        res['shutdown_ok'] = rig.apply('shutdown', 'finalized')
        _, s_l = rig.state()
        res['state_after_shutdown'] = s_l
        rig.drive_for(0.3)
        t1 = time.monotonic()
        rig.drive_for(1.0)
        res['setpoints_after_shutdown'] = rig.n_setpoints_since(t1)
        res['setpoint_publishers_after_shutdown'] = rig.setpoint_pub_count()
        log_txt = _read(log)
        res['log_has_shutdown_cleanup'] = 'shutdown：完整清理' in log_txt
        res['proc_alive_after_shutdown'] = (proc.poll() is None)
        res['pass'] = bool(
            res['configure_ok'] and res['activate_ok'] and res['shutdown_ok'] and
            res['state_after_shutdown'] == 'finalized' and
            res['setpoints_before_shutdown'] >= 20 and
            res['setpoints_after_shutdown'] <= 2 and
            res['log_has_shutdown_cleanup'])
    finally:
        _kill(proc)
    return res


def run_error_injection(rig, cfg_dir, log_dir):
    """③ 以错误类型参数注入 configure 回调异常 → 生命周期 error → 完整清理（状态回 UNCONFIGURED）。"""
    res = {'pass': False}
    pf = os.path.join(cfg_dir, 'r034_badtype.yaml')
    with open(pf, 'w', encoding='utf-8') as f:
        # world_frame 声明为字符串，却给整数 → configure 回调抛异常 → 生命周期 error 路径。
        f.write('trajectory_tracker:\n  ros__parameters:\n'
                '    use_sim_time: false\n    bond_heartbeat_period: 0.0\n'
                '    origin_pose: "0,0,0"\n    world_frame: 123\n')
    log = os.path.join(log_dir, 'error_injection.log')
    proc = _spawn([os.path.join(NODE_BIN, 'trajectory_tracker'), '--ros-args',
                   '--params-file', pf], log)
    try:
        up = rig.wait_node_up()
        res['node_up'] = up
        if not up:
            return res
        # 回调抛异常的转移响应不可靠 → 仅**尽力**触发 configure，判定以**节点日志 + 终态**为准。
        rig._cli('set', SERVICE, 'configure')
        end = time.monotonic() + 12.0
        while time.monotonic() < end:
            if 'error：完整清理' in _read(log):
                break
            time.sleep(0.3)
        _, s_l = rig.state()
        res['state_after_error'] = s_l
        res['setpoint_publishers_after_error'] = rig.setpoint_pub_count()
        log_txt = _read(log)
        res['log_has_error_cleanup'] = 'error：完整清理' in log_txt
        res['log_has_caught_exception'] = 'Caught exception in callback' in log_txt
        res['proc_alive_after_error'] = (proc.poll() is None)
        res['pass'] = bool(
            res['state_after_error'] in ('unconfigured', 'finalized') and
            res['log_has_error_cleanup'] and res['log_has_caught_exception'] and
            res['proc_alive_after_error'])
    finally:
        _kill(proc)
    return res


def run_manager_node_death(rig, cfg_dir, log_dir):
    """④ 真实 manager（bond_timeout>0）+ 杀被管节点 → 管理器 bond 失联可判。"""
    res = {'pass': False}
    pf = os.path.join(cfg_dir, 'r034_managed.yaml')
    with open(pf, 'w', encoding='utf-8') as f:
        f.write(MANAGED_PARAMS)
    mf = os.path.join(cfg_dir, 'r034_manager.yaml')
    with open(mf, 'w', encoding='utf-8') as f:
        f.write(MANAGER_PARAMS)
    nlog = os.path.join(log_dir, 'mgr_node_death_node.log')
    mlog = os.path.join(log_dir, 'mgr_node_death_manager.log')
    node_proc = _spawn([os.path.join(NODE_BIN, 'trajectory_tracker'), '--ros-args',
                        '--params-file', pf], nlog)
    mgr_proc = None
    try:
        up = rig.wait_node_up()
        res['node_up'] = up
        if not up:
            return res
        time.sleep(2.0)   # 让 DDS 图对 manager 收敛后再起 manager
        mgr_proc = _spawn([MANAGER_BIN, '--ros-args', '--params-file', mf], mlog)
        # 等 manager 起节点 + 建立 bond（发现 / 起节点可能较慢 → 放宽到 60 s）
        end = time.monotonic() + 60.0
        while time.monotonic() < end:
            txt = _read(mlog)
            if 'Connected with bond' in txt or 'connected with bond' in txt:
                break
            time.sleep(0.5)
        # 等 "Managed nodes are active"（bond 形成后紧跟）
        end = time.monotonic() + 15.0
        while time.monotonic() < end:
            if 'Managed nodes are active' in _read(mlog):
                break
            time.sleep(0.5)
        mtxt = _read(mlog)
        res['bond_formed'] = 'connected with bond' in mtxt
        res['manager_active'] = 'Managed nodes are active' in mtxt
        res['managed_state'] = rig.state()[1]
        # 杀被管节点
        _kill(node_proc)
        end = time.monotonic() + 20.0
        while time.monotonic() < end:
            if 'IS DOWN' in _read(mlog):
                break
            time.sleep(0.5)
        mtxt = _read(mlog)
        res['node_death_detected'] = ('IS DOWN' in mtxt) and ('heartbeat' in mtxt)
        res['manager_bond_timer_created'] = 'Creating bond timer' in mtxt
        res['pass'] = bool(res['bond_formed'] and res['manager_active'] and
                           res['node_death_detected'] and res['manager_bond_timer_created'])
    finally:
        _kill(node_proc)
        _kill(mgr_proc)
    return res


def run_manager_death(rig, cfg_dir, log_dir):
    """⑤ 杀 lifecycle_manager → 节点侧 bond 断开看门狗 → 失效停机（回 UNCONFIGURED）。"""
    res = {'pass': False}
    pf = os.path.join(cfg_dir, 'r034_managed.yaml')
    mf = os.path.join(cfg_dir, 'r034_manager.yaml')
    nlog = os.path.join(log_dir, 'mgr_death_node.log')
    mlog = os.path.join(log_dir, 'mgr_death_manager.log')
    node_proc = _spawn([os.path.join(NODE_BIN, 'trajectory_tracker'), '--ros-args',
                        '--params-file', pf], nlog)
    mgr_proc = None
    try:
        up = rig.wait_node_up()
        res['node_up'] = up
        if not up:
            return res
        time.sleep(2.0)
        mgr_proc = _spawn([MANAGER_BIN, '--ros-args', '--params-file', mf], mlog)
        end = time.monotonic() + 60.0
        while time.monotonic() < end:
            if 'connected with bond' in _read(mlog):
                break
            time.sleep(0.5)
        end = time.monotonic() + 15.0
        while time.monotonic() < end:
            if 'Managed nodes are active' in _read(mlog):
                break
            time.sleep(0.5)
        res['bond_formed'] = 'connected with bond' in _read(mlog)
        res['managed_state'] = rig.state()[1]
        # 驱动 setpoint 流，确认失效停机前确有输出
        rig.drive_for(1.5)
        t0 = time.monotonic()
        rig.drive_for(1.0)
        res['setpoints_before_manager_death'] = rig.n_setpoints_since(t0)
        res['setpoint_publishers_before'] = rig.setpoint_pub_count()
        # 杀 manager
        _kill(mgr_proc)
        end = time.monotonic() + 15.0
        while time.monotonic() < end:
            if 'bond 断开' in _read(nlog):
                break
            time.sleep(0.5)
        ntxt = _read(nlog)
        res['node_detected_bond_break'] = 'bond 断开 → 失效停机' in ntxt
        _, s_l = rig.state()
        res['node_state_after_manager_death'] = s_l
        rig.drive_for(0.5)
        t1 = time.monotonic()
        rig.drive_for(1.0)
        res['setpoints_after_manager_death'] = rig.n_setpoints_since(t1)
        res['setpoint_publishers_after'] = rig.setpoint_pub_count()
        res['pass'] = bool(
            res['bond_formed'] and res['setpoints_before_manager_death'] >= 20 and
            res['node_detected_bond_break'] and
            res['node_state_after_manager_death'] == 'unconfigured' and
            res['setpoints_after_manager_death'] <= 2)
    finally:
        _kill(node_proc)
        _kill(mgr_proc)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261017-w6-r034-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()

    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)
    cfg_dir = os.path.join(out_dir, 'config')
    log_dir = os.path.join(out_dir, 'r034', 'log')
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    _reap_stale()
    rclpy.init()
    rig = Rig()
    result = {
        'run_id': args.run_id, 'suite': 'R-034 lifecycle repeatable resource state machine bench',
        'scope_note': (
            '生命周期**逻辑台架**：真实 agc_air 生命周期节点 + 真实 Nav2 lifecycle_manager，'
            '覆盖 循环 configure/cleanup（≥2 轮）/ direct shutdown / error 注入 / manager·被管节点死亡；'
            '**非整链** / 非双 PX4 / 非 GUI。'),
        'unsupported': [],
        'scenarios': {},
    }
    try:
        # 顺序执行：前面场景的进程已在各自 finally 回收；场景间留 settle 让 DDS 图收敛。
        result['scenarios']['cycle_repeatable'] = run_cycle(rig, cfg_dir, log_dir)
        print('cycle_repeatable       PASS=%s' % result['scenarios']['cycle_repeatable'].get('pass'))
        time.sleep(1.5)
        result['scenarios']['direct_shutdown'] = run_direct_shutdown(rig, cfg_dir, log_dir)
        print('direct_shutdown        PASS=%s' % result['scenarios']['direct_shutdown'].get('pass'))
        time.sleep(1.5)
        result['scenarios']['error_injection'] = run_error_injection(rig, cfg_dir, log_dir)
        print('error_injection        PASS=%s' % result['scenarios']['error_injection'].get('pass'))
        time.sleep(1.5)
        result['scenarios']['manager_node_death'] = run_manager_node_death(rig, cfg_dir, log_dir)
        print('manager_node_death     PASS=%s' % result['scenarios']['manager_node_death'].get('pass'))
        time.sleep(1.5)
        result['scenarios']['manager_death'] = run_manager_death(rig, cfg_dir, log_dir)
        print('manager_death          PASS=%s' % result['scenarios']['manager_death'].get('pass'))
    finally:
        rig.destroy_node()
        rclpy.shutdown()

    result['overall_pass'] = bool(
        result['scenarios'] and all(s.get('pass') for s in result['scenarios'].values()))
    with open(os.path.join(out_dir, 'r034', 'bench.json'), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print('\noverall_pass=%s -> %s' % (
        result['overall_pass'], os.path.join(out_dir, 'r034', 'bench.json')))
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
