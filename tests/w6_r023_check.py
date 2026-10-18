#!/usr/bin/env python3
"""w6_r023_check —— 评审 R-023（连续跟踪 dwell 逐拍判据中断）自包含 ROS 级验收 harness。

结构拷贝自 `tests/w6_r022_check.py`（自包含：不起 launch，自己充当「假源 + 假 reset 服务」桩环境，
以 subprocess 直接起 `install/agc_task/lib/agc_task/task_manager` + 临时参数文件，`use_sim_time: False`，
墙钟，规避无 `/clock` 时间冻结）；Popen 句柄 kill，**禁止** pkill -f "<名字>"（会匹配 shell 自身 → 自杀）。

假源（本 harness 节点）：
  * 发布 `/agc/ground/odom`（车 0,0）、`/agc/targets`（`TargetTrackArray`，**支持空数组模式**）、
    `/agc/ground/odom_valid`（`Bool True`）、`/agc/ground/cmd_vel`（`TwistStamped`）、
    `/agc/scenario/reset_epoch`（`std_msgs/Int32`，transient_local）。
  * 提供假服务 `/agc/scenario/reset`（`Trigger`，成功且 epoch++）。
  * 订阅 `/agc/task/state`（`AgcTaskState`）、`/agc/task/dwell_event`（`String`，depth 10，收集到列表）。

用例（结果落 `experiments/w6-task-eval/<run_id>/metrics/r023.json`，ADR-0007）：
  a 抵近→空数组 > t_dwell→重获 → 不判 TRACKING_SUCCESS；再续 ≥ t_dwell → 成功（旧逻辑会跨失联段直接成功）。
  b 换 target_id → dwell 清零（出现过 TARGET_ID_CHANGED 中断）；不早成功，续时后成功。
  c 车 odom 消失 / 超龄 → 中断（CAR_ODOM_STALE / CAR_ODOM_MISSING）；不早成功，续时后成功。
  d 冻结 / epoch 变化 → 中断（EPOCH_CHANGED）；不早成功，续时后成功。
  e 分段有效时长记录：活跃段 ~1.2s 后置空数组 → DWELL_INTERRUPT 的 seg∈(0.6,1.5) 且 total>0。
  f `grep` 证 `capture_adjudicator.cpp` 含 `hit.data = "CAPTURE_NET";` + 工作树 `src/agc_msgs` 未改。

注：每个 task_manager 用例起**独立进程**，避免状态机跨用例污染。
"""

import argparse
import json
import os
import re
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
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger

ROOT = '/home/chen/AGC'
ODOM_TOPIC = '/agc/ground/odom'
TARGET_TOPIC = '/agc/targets'
ODOM_VALID_TOPIC = '/agc/ground/odom_valid'
CMD_TOPIC = '/agc/ground/cmd_vel'
STATE_TOPIC = '/agc/task/state'
EPOCH_TOPIC = '/agc/scenario/reset_epoch'
DWELL_TOPIC = '/agc/task/dwell_event'
RESET_SERVICE = '/agc/scenario/reset'
START_SERVICE = '/agc/task/start'
DT = 0.05

_T_DWELL = 2.0


def _tl_qos():
    """对齐 scenario_reset / epoch 发布端：depth=1 + RELIABLE + TRANSIENT_LOCAL（R-015 / R-022）。"""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _parse_event(s):
    """解析 dwell 事件串：返回 (reason, seg, total)（缺项为 None）。"""
    r = re.search(r'reason=(\S+)', s)
    seg = re.search(r'seg=([-+]?[\d.]+)', s)
    total = re.search(r'total=([-+]?[\d.]+)', s)
    return (r.group(1) if r else None,
            float(seg.group(1)) if seg else None,
            float(total.group(1)) if total else None)


class R23Harness(Node):
    def __init__(self):
        super().__init__('w6_r023_check')
        self.pub_odom = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.pub_targets = self.create_publisher(TargetTrackArray, TARGET_TOPIC, 10)
        self.pub_ov = self.create_publisher(Bool, ODOM_VALID_TOPIC, 10)
        self.pub_cmd = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.pub_epoch = self.create_publisher(Int32, EPOCH_TOPIC, _tl_qos())
        self.create_subscription(AgcTaskState, STATE_TOPIC, self._on_state, 10)
        self.create_subscription(String, DWELL_TOPIC, self._on_dwell, 10)
        self.srv_reset = self.create_service(Trigger, RESET_SERVICE, self._on_reset)
        self.create_timer(DT, self._pub_tick)
        self.start_cli = self.create_client(Trigger, START_SERVICE)

        # 假源开关（默认全关，逐用例开启）
        self.on_odom = self.on_targets = self.on_ov = self.on_cmd = self.on_epoch = False
        self.targets_empty = False
        self.car = (0.0, 0.0, 0.0)
        self.tgt = (1.0, 0.0, 0.0)
        self.tgt_conf = 0.9
        self.tgt_id = 'intruder_uav'
        self.epoch = 0
        self.reset_calls = 0
        self.reset_buffers()

    # ---- callbacks ----
    def _on_state(self, m):
        self.state = int(m.state)
        self.state_name = m.state_name
        self.state_reason = m.reason_code

    def _on_dwell(self, m):
        self.dwell_events.append(m.data)

    def _on_reset(self, req, resp):
        self.reset_calls += 1
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
            arr = TargetTrackArray()
            arr.header.frame_id = 'world'
            arr.header.stamp = stamp
            if self.targets_empty:
                arr.tracks = []                             # 空数组模式（R-023 关键注入）
            else:
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
        self.dwell_events = []

    def all_off(self):
        self.on_odom = self.on_targets = self.on_ov = self.on_cmd = self.on_epoch = False
        self.targets_empty = False

    def all_sources_on(self):
        self.on_odom = self.on_targets = self.on_ov = self.on_cmd = self.on_epoch = True

    def interrupts(self):
        """返回所有 DWELL_INTERRUPT 事件的 (reason, seg, total) 列表。"""
        out = []
        for s in self.dwell_events:
            if 'DWELL_INTERRUPT' in s:
                out.append(_parse_event(s))
        return out


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
    path = os.path.join(out_dir, 'r023_%s_task_manager.yaml' % prefix)
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


def _wait_services(node):
    return node.start_cli.wait_for_service(timeout_sec=20.0)


def _std_overrides():
    """R-023 用例统一参数覆盖（语义判据不削弱）。"""
    return {'air_present': False, 't_dwell_s': _T_DWELL, 'fsm_hz': 20.0,
            'd_track_m': 3.0, 'car_odom_max_age_s': 0.5, 't_max_s': 120.0}


def _fresh_sources(node):
    """全源开启（odom / ov / cmd / epoch / targets），车 0,0，目标 (1.0,0,0) 近距新鲜。"""
    node.reset_buffers()
    node.all_sources_on()
    node.targets_empty = False
    node.epoch = 0
    node.reset_calls = 0
    node.tgt_id = 'intruder_uav'
    node.tgt = (1.0, 0.0, 0.0)
    node.tgt_conf = 0.9


def _warm_to_ground(node):
    """助跑：start → 等到 GROUND_APPROACH。"""
    node.run(1.3)
    r1 = node.call(node.start_cli)
    reached = node.run_until(lambda: node.state_name == 'GROUND_APPROACH', 15.0)
    return bool(r1 is not None and r1.success), bool(reached)


# ---- 用例 ----
def case_a(node, share, out_dir):
    """a. 抵近→空数组 > t_dwell→重获：不早成功，续时后成功。"""
    res = {}
    _fresh_sources(node)
    proc, log, _ = _start_tm(share, out_dir, 'a', _std_overrides())
    try:
        if not _wait_services(node):
            res['error'] = '/agc/task/start service never appeared'
            res['pass'] = False
            return res
        res['start_success'], res['reached_ground_approach'] = _warm_to_ground(node)
        node.run(1.0)                                   # dwell 累计 ~1.0s
        node.targets_empty = True                       # 空数组：应中断清零（旧逻辑保留 dwell_start_）
        node.run(2.5)                                   # > t_dwell
        res['state_during_gap'] = node.state_name
        node.targets_empty = False                      # 恢复目标
        node.run(1.0)
        res['state_after_restore'] = node.state_name
        res['no_early_success'] = bool(node.state_name != 'TRACKING_SUCCESS')
        node.run(1.5)                                   # 自恢复起 ≥ t_dwell
        res['state_final'] = node.state_name
        res['eventual_success'] = bool(node.state_name == 'TRACKING_SUCCESS')
        res['dwell_events'] = list(node.dwell_events)
        res['pass'] = bool(res['start_success'] and res['reached_ground_approach'] and
                           res['no_early_success'] and res['eventual_success'])
    finally:
        _stop_proc(proc, log)
        node.all_off()
    return res


def case_b(node, share, out_dir):
    """b. 换 target_id → dwell 清零（TARGET_ID_CHANGED）；不早成功，续时后成功。"""
    res = {}
    _fresh_sources(node)
    proc, log, _ = _start_tm(share, out_dir, 'b', _std_overrides())
    try:
        if not _wait_services(node):
            res['error'] = '/agc/task/start service never appeared'
            res['pass'] = False
            return res
        res['start_success'], res['reached_ground_approach'] = _warm_to_ground(node)
        node.run(1.0)                                   # dwell 累计 ~1.0s（intruder_uav）
        node.tgt_id = 'other_uav'                       # 换目标（仍近距新鲜）
        node.run(1.0)
        res['state_after_id_change'] = node.state_name
        res['no_early_success'] = bool(node.state_name != 'TRACKING_SUCCESS')
        node.run(1.5)
        res['state_final'] = node.state_name
        res['eventual_success'] = bool(node.state_name == 'TRACKING_SUCCESS')
        res['interrupts'] = node.interrupts()
        res['dwell_events'] = list(node.dwell_events)
        res['has_id_changed'] = any(r == 'TARGET_ID_CHANGED' for r, _s, _t in res['interrupts'])
        res['pass'] = bool(res['start_success'] and res['reached_ground_approach'] and
                           res['no_early_success'] and res['eventual_success'] and
                           res['has_id_changed'])
    finally:
        _stop_proc(proc, log)
        node.all_off()
    return res


def case_c(node, share, out_dir):
    """c. 车 odom 消失 / 超龄 → 中断；不早成功，续时后成功。"""
    res = {}
    _fresh_sources(node)
    proc, log, _ = _start_tm(share, out_dir, 'c', _std_overrides())
    try:
        if not _wait_services(node):
            res['error'] = '/agc/task/start service never appeared'
            res['pass'] = False
            return res
        res['start_success'], res['reached_ground_approach'] = _warm_to_ground(node)
        node.run(1.0)                                   # dwell 累计 ~1.0s
        node.on_odom = False                            # 停发车 odom
        node.run(2.5)                                   # > car_odom_max_age_s（0.5）
        res['state_during_gap'] = node.state_name
        res['not_success_during_gap'] = bool(node.state_name != 'TRACKING_SUCCESS')
        node.on_odom = True                             # 恢复车 odom
        node.run(1.0)
        res['state_after_restore'] = node.state_name
        res['no_early_success'] = bool(node.state_name != 'TRACKING_SUCCESS')
        node.run(1.5)
        res['state_final'] = node.state_name
        res['eventual_success'] = bool(node.state_name == 'TRACKING_SUCCESS')
        res['interrupts'] = node.interrupts()
        res['dwell_events'] = list(node.dwell_events)
        res['has_car_interrupt'] = any(
            r in ('CAR_ODOM_STALE', 'CAR_ODOM_MISSING') for r, _s, _t in res['interrupts'])
        res['pass'] = bool(res['start_success'] and res['reached_ground_approach'] and
                           res['not_success_during_gap'] and res['no_early_success'] and
                           res['eventual_success'] and res['has_car_interrupt'])
    finally:
        _stop_proc(proc, log)
        node.all_off()
    return res


def case_d(node, share, out_dir):
    """d. epoch 变化 → 中断（EPOCH_CHANGED）；不早成功，续时后成功。"""
    res = {}
    _fresh_sources(node)
    proc, log, _ = _start_tm(share, out_dir, 'd', _std_overrides())
    try:
        if not _wait_services(node):
            res['error'] = '/agc/task/start service never appeared'
            res['pass'] = False
            return res
        res['start_success'], res['reached_ground_approach'] = _warm_to_ground(node)
        node.run(1.0)                                   # dwell 累计 ~1.0s
        node.epoch += 7                                 # 改 epoch（_pub_tick 发布到 reset_epoch）
        node.run(1.0)
        res['state_after_epoch_change'] = node.state_name
        res['no_early_success'] = bool(node.state_name != 'TRACKING_SUCCESS')
        node.run(1.5)
        res['state_final'] = node.state_name
        res['eventual_success'] = bool(node.state_name == 'TRACKING_SUCCESS')
        res['interrupts'] = node.interrupts()
        res['dwell_events'] = list(node.dwell_events)
        res['has_epoch_changed'] = any(r == 'EPOCH_CHANGED' for r, _s, _t in res['interrupts'])
        res['pass'] = bool(res['start_success'] and res['reached_ground_approach'] and
                           res['no_early_success'] and res['eventual_success'] and
                           res['has_epoch_changed'])
    finally:
        _stop_proc(proc, log)
        node.all_off()
    return res


def case_e(node, share, out_dir):
    """e. 分段有效时长记录：活跃段 ~1.2s 后置空数组 → seg∈(0.6,1.5) 且 total>0。"""
    res = {}
    _fresh_sources(node)
    proc, log, _ = _start_tm(share, out_dir, 'e', _std_overrides())
    try:
        if not _wait_services(node):
            res['error'] = '/agc/task/start service never appeared'
            res['pass'] = False
            return res
        res['start_success'], res['reached_ground_approach'] = _warm_to_ground(node)
        node.run(1.2)                                   # 有效段 ~1.2s
        node.targets_empty = True                       # 触发中断
        node.run(0.6)
        res['dwell_events'] = list(node.dwell_events)
        res['interrupts'] = node.interrupts()
        good = [(r, s, t) for (r, s, t) in res['interrupts']
                if s is not None and 0.6 < s < 1.5 and t is not None and t > 0.0]
        res['segment_recorded'] = len(good) > 0
        res['matched_interrupts'] = good
        res['pass'] = bool(res['start_success'] and res['reached_ground_approach'] and
                           res['segment_recorded'])
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
    path = os.path.join(out_dir, 'metrics', 'r023.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-38s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
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
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r023-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    share = get_package_share_directory('agc_task')
    res = {'run_id': run_id, 'tests': {}}

    rclpy.init()
    node = R23Harness()
    try:
        res['tests']['a_empty_array_gap_no_early_success'] = case_a(node, share, out_dir)
        res['tests']['b_target_id_change_resets_dwell'] = case_b(node, share, out_dir)
        res['tests']['c_car_odom_stale_interrupts'] = case_c(node, share, out_dir)
        res['tests']['d_epoch_change_interrupts'] = case_d(node, share, out_dir)
        res['tests']['e_segment_valid_duration_recorded'] = case_e(node, share, out_dir)
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
