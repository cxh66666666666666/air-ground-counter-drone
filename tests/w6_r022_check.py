#!/usr/bin/env python3
"""w6_r022_check —— 评审 R-022（FSM 命令幂等与终态锁存）自包含 ROS 级验收 harness。

结构拷贝自 `scripts/w6_r021_check.py`（自包含：不起 launch，自己充当「假源 + 假 reset 服务」桩环境，
以 subprocess 直接起 `install/agc_task/lib/agc_task/task_manager` + 临时参数文件，`use_sim_time: False`，
墙钟，规避无 `/clock` 时间冻结）；Popen 句柄 kill，**禁止** pkill -f "<名字>"（会匹配 shell 自身 → 自杀）。

假源（本 harness 节点）：
  * 发布 `/agc/ground/odom`、`/agc/targets`、`/agc/ground/odom_valid`（`Bool True`）、
    `/agc/ground/cmd_vel`（`TwistStamped`）、`/agc/scenario/reset_epoch`（`std_msgs/Int32`，transient_local）。
  * 提供假服务 `/agc/scenario/reset`（`Trigger`，成功并 epoch++；case d 可**延迟应答**模拟慢应答）。
  * 订阅 `/agc/task/state`、`/agc/task/episode`。

用例（结果落 `experiments/w6-task-eval/<run_id>/metrics/r022.json`，ADR-0007）：
  a active 重入 start → 拒绝（success=false）、状态不变（非 INFRA_FAILURE）、未再调用 reset。
  b 终态后 start = 新 episode（success=true、态回 AIR_SEARCH、`/agc/task/episode` 值递增）。
  c 终态后迟到 abort → 不覆写（仍 MISSION_FAILED）。
  d reset 慢应答 + 超时 → 无副作用（success=false、message 含 indeterminate、state 仍 IDLE）。
  e 重复 abort 幂等（首次 ABORTED，二次被拒且状态不变）。
  f `grep` 证 `capture_hit` 值仍恰为 `CAPTURE_NET`、`src/agc_msgs` 未改。

注：每个 task_manager 用例起**独立进程**，避免状态机跨用例污染。
"""

import argparse
import json
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory

from agc_msgs.msg import AgcTaskState, TargetTrack, TargetTrackArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger

ROOT = '/home/chen/AGC'
ODOM_TOPIC = '/agc/ground/odom'
TARGET_TOPIC = '/agc/targets'
ODOM_VALID_TOPIC = '/agc/ground/odom_valid'
CMD_TOPIC = '/agc/ground/cmd_vel'
STATE_TOPIC = '/agc/task/state'
EPOCH_TOPIC = '/agc/scenario/reset_epoch'
EPISODE_TOPIC = '/agc/task/episode'
RESET_SERVICE = '/agc/scenario/reset'
START_SERVICE = '/agc/task/start'
ABORT_SERVICE = '/agc/task/abort'
DT = 0.05


def _tl_qos():
    """对齐 scenario_reset / episode 发布端：depth=1 + RELIABLE + TRANSIENT_LOCAL（R-015 / R-022）。"""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class R22Harness(Node):
    def __init__(self):
        super().__init__('w6_r022_check')
        self.pub_odom = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.pub_targets = self.create_publisher(TargetTrackArray, TARGET_TOPIC, 10)
        self.pub_ov = self.create_publisher(Bool, ODOM_VALID_TOPIC, 10)
        self.pub_cmd = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.pub_epoch = self.create_publisher(Int32, EPOCH_TOPIC, _tl_qos())
        self.create_subscription(AgcTaskState, STATE_TOPIC, self._on_state, 10)
        self.create_subscription(Int32, EPISODE_TOPIC, self._on_episode, _tl_qos())
        self.srv_reset = self.create_service(Trigger, RESET_SERVICE, self._on_reset)
        self.create_timer(DT, self._pub_tick)
        self.start_cli = self.create_client(Trigger, START_SERVICE)
        self.abort_cli = self.create_client(Trigger, ABORT_SERVICE)

        # 假源开关（默认全关，逐用例开启）
        self.on_odom = self.on_targets = self.on_ov = self.on_cmd = self.on_epoch = False
        self.car = (0.0, 0.0, 0.0)
        self.tgt = (0.5, 0.0, 0.0)
        self.tgt_conf = 0.9
        self.tgt_id = 'intruder_uav'
        self.epoch = 0
        self.reset_calls = 0
        self.reset_delay_s = 0.0
        self.reset_buffers()

    # ---- callbacks ----
    def _on_state(self, m):
        self.state = int(m.state)
        self.state_name = m.state_name
        self.state_reason = m.reason_code

    def _on_episode(self, m):
        self.episodes.append(int(m.data))

    def _on_reset(self, req, resp):
        self.reset_calls += 1
        if self.reset_delay_s > 0.0:
            time.sleep(self.reset_delay_s)   # 模拟慢应答（> reset_call_timeout_s）
        self.epoch += 1
        self.pub_epoch.publish(Int32(data=self.epoch))
        resp.success = True
        resp.message = 'reset ok epoch=%d' % self.epoch
        return resp

    # ---- 发布 ----
    def _pub_tick(self):
        stamp = self.get_clock().now().to_msg()
        if self.on_odom:
            od = Odometry()
            od.header.frame_id = 'world'
            od.header.stamp = stamp
            od.child_frame_id = 'agc_car'
            od.pose.pose.position.x = self.car[0]
            od.pose.pose.position.y = self.car[1]
            od.pose.pose.position.z = self.car[2]
            od.pose.pose.orientation.w = 1.0
            self.pub_odom.publish(od)
        if self.on_targets:
            t = TargetTrack()
            t.header.frame_id = 'world'
            t.header.stamp = stamp
            t.target_id = self.tgt_id
            t.pose.pose.position.x = self.tgt[0]
            t.pose.pose.position.y = self.tgt[1]
            t.pose.pose.position.z = self.tgt[2]
            t.pose.pose.orientation.w = 1.0
            t.confidence = self.tgt_conf
            t.source = 'gt_adapter'
            arr = TargetTrackArray()
            arr.header.frame_id = 'world'
            arr.header.stamp = stamp
            arr.tracks = [t]
            self.pub_targets.publish(arr)
        if self.on_ov:
            self.pub_ov.publish(Bool(data=True))
        if self.on_cmd:
            c = TwistStamped()
            c.header.frame_id = 'world'
            c.header.stamp = stamp
            c.twist.linear.x = 0.1
            self.pub_cmd.publish(c)
        if self.on_epoch:
            self.pub_epoch.publish(Int32(data=self.epoch))

    # ---- 驱动 ----
    def run(self, dur):
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_until(self, cond, max_dur):
        t0 = time.monotonic()
        while time.monotonic() - t0 < max_dur:
            rclpy.spin_once(self, timeout_sec=0.02)
            if cond():
                return True
        return False

    def call(self, cli, timeout=30.0):
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def reset_buffers(self):
        self.state = AgcTaskState.IDLE
        self.state_name = ''
        self.state_reason = ''
        self.episodes = []

    def all_off(self):
        self.on_odom = self.on_targets = self.on_ov = self.on_cmd = self.on_epoch = False
        self.reset_delay_s = 0.0

    def gate_sources_on(self):
        self.on_odom = self.on_ov = self.on_cmd = self.on_epoch = True
        self.on_targets = False


# ---- 进程管理 ----
def _exe(share, name):
    return str(Path(share).parents[1] / 'lib' / 'agc_task' / name)


def _base_params(share, node_name):
    base = yaml.safe_load(open(os.path.join(share, 'config', 'w6_base.yaml')))
    return dict((base.get(node_name) or {}).get('ros__parameters', {}))


def _write_params(share, path, node_name, overrides):
    params = _base_params(share, node_name)
    params.update(overrides)
    params['use_sim_time'] = False
    doc = {node_name: {'ros__parameters': params}}
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
    return params


def _start_tm(share, out_dir, prefix, overrides):
    path = os.path.join(out_dir, 'r022_%s_task_manager.yaml' % prefix)
    _write_params(share, path, 'task_manager', overrides)
    log = open(os.path.join(out_dir, 'log', 'case_%s_task_manager.log' % prefix), 'w')
    proc = subprocess.Popen(
        [_exe(share, 'task_manager'), '--ros-args', '--params-file', path],
        stdout=log, stderr=subprocess.STDOUT)
    return proc, log, path


def _stop_proc(proc, log):
    if proc is None:
        return
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


def _wait_services(node, with_abort=False):
    if not node.start_cli.wait_for_service(timeout_sec=20.0):
        return False
    if with_abort and not node.abort_cli.wait_for_service(timeout_sec=20.0):
        return False
    return True


# ---- 用例 ----
def case_a(node, share, out_dir):
    """a. active 重入 start 拒绝、状态不变（非 INFRA_FAILURE）、未再调用 reset。"""
    res = {}
    overrides = {'air_present': False, 't_max_s': 120.0}
    node.reset_buffers()
    node.gate_sources_on()
    node.epoch = 0
    node.reset_calls = 0
    proc, log, _ = _start_tm(share, out_dir, 'a', overrides)
    try:
        if not _wait_services(node):
            res['error'] = '/agc/task/start service never appeared'
            res['pass'] = False
            return res
        node.run(1.3)                              # 学 epoch + 收初态
        r1 = node.call(node.start_cli)
        res['start1_success'] = bool(r1 is not None and r1.success)
        reached = node.run_until(lambda: node.state_name == 'AIR_SEARCH', 10.0)
        calls_after_start1 = node.reset_calls
        r2 = node.call(node.start_cli)             # active 重入
        node.run(0.5)
        calls_after_start2 = node.reset_calls
        res['rejected'] = bool(r2 is not None and not r2.success)
        res['reject_message'] = None if r2 is None else r2.message
        res['state_after_reentry'] = node.state_name
        res['state_unchanged'] = bool(node.state_name == 'AIR_SEARCH')
        res['not_infra_failure'] = bool(node.state_name != 'INFRA_FAILURE')
        res['reset_calls_after_start1'] = calls_after_start1
        res['reset_calls_after_start2'] = calls_after_start2
        res['no_reset_on_reentry'] = bool(calls_after_start2 == calls_after_start1)
        res['pass'] = bool(
            res['start1_success'] and reached and res['rejected'] and res['state_unchanged'] and
            res['not_infra_failure'] and res['no_reset_on_reentry'])
    finally:
        _stop_proc(proc, log)
        node.all_off()
    return res


def case_b(node, share, out_dir):
    """b. 终态后 start = 新 episode（episode 值递增）。"""
    res = {}
    overrides = {'air_present': False, 't_max_s': 2.0, 'fsm_hz': 20.0}
    node.reset_buffers()
    node.gate_sources_on()
    node.epoch = 0
    node.reset_calls = 0
    proc, log, _ = _start_tm(share, out_dir, 'b', overrides)
    try:
        if not _wait_services(node):
            res['error'] = '/agc/task/start service never appeared'
            res['pass'] = False
            return res
        node.run(1.3)
        r1 = node.call(node.start_cli)
        res['start1_success'] = bool(r1 is not None and r1.success)
        node.run(0.6)                              # 收 episode(1)
        ep1 = node.episodes[-1] if node.episodes else None
        failed = node.run_until(lambda: node.state_name == 'MISSION_FAILED', 10.0)
        res['reached_mission_failed'] = bool(failed)
        res['terminal_state'] = node.state_name
        r2 = node.call(node.start_cli)             # 终态后 start
        res['start2_success'] = bool(r2 is not None and r2.success)
        back = node.run_until(lambda: node.state_name == 'AIR_SEARCH', 10.0)
        node.run(0.3)
        res['state_after_new_start'] = node.state_name
        res['restarted_active'] = bool(back)
        ep2 = node.episodes[-1] if node.episodes else None
        res['episode_first'] = ep1
        res['episode_second'] = ep2
        res['episodes'] = list(node.episodes)
        res['episode_incremented'] = bool(ep1 is not None and ep2 is not None and ep2 > ep1)
        res['pass'] = bool(
            res['start1_success'] and failed and res['start2_success'] and res['restarted_active'] and
            res['episode_incremented'])
    finally:
        _stop_proc(proc, log)
        node.all_off()
    return res


def case_c(node, share, out_dir):
    """c. 终态后迟到 abort 不覆写已锁存终态。"""
    res = {}
    overrides = {'air_present': False, 't_max_s': 2.0, 'fsm_hz': 20.0}
    node.reset_buffers()
    node.gate_sources_on()
    node.epoch = 0
    node.reset_calls = 0
    proc, log, _ = _start_tm(share, out_dir, 'c', overrides)
    try:
        if not _wait_services(node, with_abort=True):
            res['error'] = 'start/abort service never appeared'
            res['pass'] = False
            return res
        node.run(1.3)
        r1 = node.call(node.start_cli)
        res['start1_success'] = bool(r1 is not None and r1.success)
        failed = node.run_until(lambda: node.state_name == 'MISSION_FAILED', 10.0)
        res['reached_mission_failed'] = bool(failed)
        terminal = node.state_name
        res['terminal_state'] = terminal
        r_abort = node.call(node.abort_cli)        # 迟到的 abort
        node.run(0.5)
        res['abort_success'] = None if r_abort is None else bool(r_abort.success)
        res['abort_rejected'] = bool(r_abort is not None and not r_abort.success)
        res['state_after_late_abort'] = node.state_name
        res['not_overwritten'] = bool(node.state_name == terminal)
        res['pass'] = bool(
            res['start1_success'] and failed and res['abort_rejected'] and res['not_overwritten'] and
            terminal == 'MISSION_FAILED')
    finally:
        _stop_proc(proc, log)
        node.all_off()
    return res


def case_d(node, share, out_dir):
    """d. reset 慢应答 + 超时 → 无副作用（success=false、含 indeterminate、state 仍 IDLE）。"""
    res = {}
    overrides = {'air_present': False, 'reset_call_timeout_s': 1.5}
    node.reset_buffers()
    node.gate_sources_on()
    node.epoch = 0
    node.reset_calls = 0
    node.reset_delay_s = 3.0                       # 慢应答 > reset_call_timeout_s
    proc, log, _ = _start_tm(share, out_dir, 'd', overrides)
    try:
        if not _wait_services(node):
            res['error'] = '/agc/task/start service never appeared'
            res['pass'] = False
            return res
        node.run(1.3)                              # 先收 IDLE 初态
        state_before = node.state_name
        r = node.call(node.start_cli, timeout=30.0)
        node.run(0.6)
        res['state_before_start'] = state_before
        res['start_success'] = None if r is None else bool(r.success)
        res['start_message'] = None if r is None else r.message
        res['message_indeterminate'] = bool(r is not None and 'indeterminate' in r.message)
        res['state_after_start'] = node.state_name
        res['state_unchanged_idle'] = bool(node.state_name == 'IDLE')
        res['not_infra_failure'] = bool(node.state_name != 'INFRA_FAILURE')
        res['not_active'] = bool(node.state_name not in
                                 ('AIR_SEARCH', 'TARGET_ACQUIRED', 'GROUND_APPROACH'))
        res['pass'] = bool(
            r is not None and not r.success and res['message_indeterminate'] and
            res['state_unchanged_idle'] and res['not_infra_failure'] and res['not_active'])
    finally:
        _stop_proc(proc, log)
        node.all_off()
    return res


def case_e(node, share, out_dir):
    """e. 重复 abort 幂等（首次 ABORTED，二次被拒且状态不变）。"""
    res = {}
    overrides = {'air_present': False, 't_max_s': 120.0}
    node.reset_buffers()
    node.gate_sources_on()
    node.epoch = 0
    node.reset_calls = 0
    proc, log, _ = _start_tm(share, out_dir, 'e', overrides)
    try:
        if not _wait_services(node, with_abort=True):
            res['error'] = 'start/abort service never appeared'
            res['pass'] = False
            return res
        node.run(1.3)
        r1 = node.call(node.start_cli)
        res['start_success'] = bool(r1 is not None and r1.success)
        node.run_until(lambda: node.state_name == 'AIR_SEARCH', 10.0)
        a1 = node.call(node.abort_cli)
        aborted = node.run_until(lambda: node.state_name == 'ABORTED', 10.0)
        res['abort1_success'] = None if a1 is None else bool(a1.success)
        res['aborted'] = bool(aborted)
        a2 = node.call(node.abort_cli)             # 重复 abort
        node.run(0.5)
        res['abort2_success'] = None if a2 is None else bool(a2.success)
        res['abort2_rejected'] = bool(a2 is not None and not a2.success)
        res['state_after_second_abort'] = node.state_name
        res['idempotent'] = bool(node.state_name == 'ABORTED')
        res['pass'] = bool(
            res['start_success'] and aborted and res['abort1_success'] and
            res['abort2_rejected'] and res['idempotent'])
    finally:
        _stop_proc(proc, log)
        node.all_off()
    return res


def case_f(_node, _share, _out_dir):
    """f. grep 证 capture_hit 值未变 + agc_msgs 未改。"""
    res = {}
    src = Path(ROOT) / 'src' / 'agc_task' / 'src' / 'capture_adjudicator.cpp'
    text = src.read_text(encoding='utf-8', errors='replace')
    res['hit_literal_present'] = ('hit.data = "CAPTURE_NET";' in text)

    def _git(args):
        try:
            out = subprocess.run(['git', '-C', ROOT] + args, capture_output=True, text=True,
                                 timeout=20.0)
            return out.returncode, (out.stdout or '').strip()
        except Exception as exc:  # noqa: BLE001
            return -1, repr(exc)

    rc1, d1 = _git(['diff', '--name-only', '--', 'src/agc_msgs'])
    rc2, d2 = _git(['diff', '--cached', '--name-only', '--', 'src/agc_msgs'])
    rc3, d3 = _git(['status', '--porcelain', '--', 'src/agc_msgs'])
    res['agc_msgs_diff'] = d1
    res['agc_msgs_cached_diff'] = d2
    res['agc_msgs_status'] = d3
    res['agc_msgs_unchanged'] = (rc1 == 0 and rc2 == 0 and rc3 == 0 and not d1 and not d2 and not d3)
    res['pass'] = bool(res['hit_literal_present'] and res['agc_msgs_unchanged'])
    return res


def dump(res, out_dir):
    path = os.path.join(out_dir, 'metrics', 'r022.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-34s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
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
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r022-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    share = get_package_share_directory('agc_task')
    res = {'run_id': run_id, 'tests': {}}

    rclpy.init()
    node = R22Harness()
    try:
        res['tests']['a_active_reentry_rejected'] = case_a(node, share, out_dir)
        res['tests']['b_terminal_then_start_new_episode'] = case_b(node, share, out_dir)
        res['tests']['c_late_abort_no_overwrite'] = case_c(node, share, out_dir)
        res['tests']['d_slow_reset_timeout_no_side_effect'] = case_d(node, share, out_dir)
        res['tests']['e_repeated_abort_idempotent'] = case_e(node, share, out_dir)
        res['tests']['f_frozen_value_and_abi'] = case_f(node, share, out_dir)
        res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    except Exception as exc:   # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return dump(res, out_dir)


if __name__ == '__main__':
    sys.exit(main())
