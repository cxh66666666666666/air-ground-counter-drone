#!/usr/bin/env python3
"""w6_r021_check —— 评审 R-021（捕获跨回合污染）自包含 ROS 级验收 harness。

结构拷贝自 `tests/w6_r020_check.py`（自包含：不起 launch，自己充当「假源」桩环境，以
subprocess 直接起 `install/agc_task/lib/agc_task/{capture_adjudicator, task_manager}` + 临时参数
文件，`use_sim_time: False`，墙钟，规避无 `/clock` 时间冻结）；task_manager 驱动结构参考
`tests/w6_r013_check.py` / `w6_r015_check.py`（假 reset 服务 + 假门禁数据）。

假源（本 harness 节点）：
  * 发布 `/agc/ground/odom`、`/agc/targets`（`confidence` / `target_id` 可控）、
    `/agc/ground/odom_valid`、`/agc/ground/cmd_vel`（task_manager 门禁数据）、
    `/agc/task/state`（任务窗口，仅 adjudicator 用例；task_manager 用例由被测节点发布）、
    `/agc/scenario/reset_epoch`（`std_msgs/Int32`，transient_local）、
    `/agc/task/capture_event`（配套事件，仅 task_manager 用例直接注入）、
    `/agc/task/jam_confirm`。
  * 提供假服务 `/agc/scenario/reset`（`Trigger`，成功并 epoch++）。
  * 订阅 `/agc/task/capture_hit`、`/agc/task/capture_event`、`/agc/task/state`。

用例（结果落 `experiments/w6-task-eval/<run_id>/metrics/r021.json`，ADR-0007）：
  a 任务未开始（无 GROUND_APPROACH）入包络 → **无** `capture_hit` / `capture_event`。
  b 源停 / 输入超龄 / `confidence<min` → **不产出**。
  c 连续两回合（epoch 变化）各独立命中（第二回合 `fired_` 已复位，重新产出）。
  d 旧 jam 在 start 前到达被忽略；旧 capture（ep 不匹配）在新 episode 拒收（不结算）。
  e `target_id` 切换 → 身份不匹配不产出命中。
  f 同拍 capture + jam → 按**事件时刻**定序（先到者胜，非固定 capture 优先）。
  g `grep` 证 `capture_hit` 值仍恰为 `CAPTURE_NET`、`src/agc_msgs` 未改。

注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身 → 自杀）；用 "[x]xx" 写法规避或 Popen 句柄 kill。
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
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger

ROOT = '/home/chen/AGC'
ODOM_TOPIC = '/agc/ground/odom'
TARGET_TOPIC = '/agc/targets'
ODOM_VALID_TOPIC = '/agc/ground/odom_valid'
CMD_TOPIC = '/agc/ground/cmd_vel'
STATE_TOPIC = '/agc/task/state'
EPOCH_TOPIC = '/agc/scenario/reset_epoch'
CAPTURE_HIT_TOPIC = '/agc/task/capture_hit'
CAPTURE_EVENT_TOPIC = '/agc/task/capture_event'
JAM_CONFIRM_TOPIC = '/agc/task/jam_confirm'
RESET_SERVICE = '/agc/scenario/reset'
DT = 0.05


def _kv(s, key):
    """从空格分隔的标注串里取 `key=value` 的 value（无则 None）。"""
    for tok in s.split():
        if tok.startswith(key + '='):
            return tok.split('=', 1)[1]
    return None


def _epoch_qos():
    """对齐 scenario_reset 发布端：depth=1 + RELIABLE + TRANSIENT_LOCAL（R-016 / R-021）。"""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class R21Harness(Node):
    def __init__(self):
        super().__init__('w6_r021_check')
        self.pub_odom = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.pub_targets = self.create_publisher(TargetTrackArray, TARGET_TOPIC, 10)
        self.pub_ov = self.create_publisher(Bool, ODOM_VALID_TOPIC, 10)
        self.pub_cmd = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.pub_state = self.create_publisher(AgcTaskState, STATE_TOPIC, 10)
        self.pub_epoch = self.create_publisher(Int32, EPOCH_TOPIC, _epoch_qos())
        self.pub_capture_event = self.create_publisher(
            String, CAPTURE_EVENT_TOPIC, _epoch_qos())
        self.pub_jam_confirm = self.create_publisher(String, JAM_CONFIRM_TOPIC, 10)
        self.create_subscription(String, CAPTURE_HIT_TOPIC, self._on_hit, 10)
        self.create_subscription(String, CAPTURE_EVENT_TOPIC, self._on_event, 10)
        self.create_subscription(AgcTaskState, STATE_TOPIC, self._on_state, 10)
        self.srv_reset = self.create_service(Trigger, RESET_SERVICE, self._on_reset)
        self.create_timer(DT, self._pub_tick)

        # 假源开关（默认全关，逐用例开启）
        self.on_odom = self.on_targets = self.on_ov = self.on_cmd = False
        self.on_state = self.on_epoch = False
        self.state_val = AgcTaskState.IDLE
        self.car = (0.0, 0.0, 0.0)
        self.tgt = (0.0, 0.0, 0.0)
        self.tgt_conf = 0.9
        self.tgt_id = 'intruder_uav'
        self.epoch = 0
        self.reset_buffers()

    # ---- callbacks ----
    def _on_hit(self, m):
        self.hits.append(m.data.strip())

    def _on_event(self, m):
        self.events.append(m.data.strip())

    def _on_state(self, m):
        self.state = int(m.state)
        self.state_name = m.state_name
        self.state_reason = m.reason_code

    def _on_reset(self, req, resp):
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
        if self.on_state:
            st = AgcTaskState()
            st.header.stamp = stamp
            st.header.frame_id = 'world'
            st.state = self.state_val
            st.state_name = 'INJECTED'
            self.pub_state.publish(st)
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

    def reset_buffers(self):
        self.hits = []
        self.events = []
        self.state = AgcTaskState.IDLE
        self.state_name = ''
        self.state_reason = ''

    def all_off(self):
        self.on_odom = self.on_targets = self.on_ov = self.on_cmd = False
        self.on_state = self.on_epoch = False

    def set_car(self, x, y, z):
        self.car = (float(x), float(y), float(z))

    def set_target(self, x, y, z):
        self.tgt = (float(x), float(y), float(z))


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


def _start_node(share, out_dir, exe_name, node_name, prefix, overrides):
    path = os.path.join(out_dir, 'r021_%s_%s.yaml' % (prefix, node_name))
    _write_params(share, path, node_name, overrides)
    log = open(os.path.join(out_dir, 'log', 'case_%s_%s.log' % (prefix, node_name)), 'w')
    proc = subprocess.Popen(
        [_exe(share, exe_name), '--ros-args', '--params-file', path],
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


# ---- 用例 ----
def case_a(node, share, out_dir):
    """任务未开始（无 GROUND_APPROACH）入包络 → 无 capture_hit / capture_event。"""
    res = {}
    node.reset_buffers()
    node.set_car(0.0, 0.0, 0.0)
    node.set_target(1.0, 0.0, 0.0)
    node.tgt_conf, node.tgt_id = 0.9, 'intruder_uav'
    node.on_odom = node.on_targets = node.on_epoch = True
    node.on_state = True
    node.state_val = AgcTaskState.IDLE   # 任务未开始
    node.epoch = 1
    proc, log, _ = _start_node(share, out_dir, 'capture_adjudicator', 'capture_adjudicator', 'a', {})
    try:
        node.run(3.0)
    finally:
        _stop_proc(proc, log)
        node.all_off()
    res['hits'] = list(node.hits)
    res['events'] = list(node.events)
    res['no_capture_hit'] = not node.hits
    res['no_capture_event'] = not node.events
    res['pass'] = bool(res['no_capture_hit'] and res['no_capture_event'])
    return res


def case_b(node, share, out_dir):
    """源停 / 输入超龄 / confidence<min → 不产出。"""
    res = {}
    # b1：窗口开启但 confidence < min
    node.reset_buffers()
    node.set_car(0.0, 0.0, 0.0)
    node.set_target(1.0, 0.0, 0.0)
    node.tgt_conf, node.tgt_id = 0.1, 'intruder_uav'   # < confidence_min=0.5
    node.on_odom = node.on_targets = node.on_epoch = node.on_state = True
    node.state_val = AgcTaskState.GROUND_APPROACH
    node.epoch = 1
    proc, log, _ = _start_node(share, out_dir, 'capture_adjudicator', 'capture_adjudicator', 'b1', {})
    try:
        node.run(2.0)
    finally:
        _stop_proc(proc, log)
        node.all_off()
    b1_hits, b1_events = list(node.hits), list(node.events)

    # b2：源停 → 超龄后开窗口仍不产出
    node.reset_buffers()
    node.tgt_conf, node.tgt_id = 0.9, 'intruder_uav'
    node.on_odom = node.on_targets = node.on_epoch = True
    node.on_state = False
    node.state_val = AgcTaskState.IDLE          # 先关窗口
    node.epoch = 1
    proc, log, _ = _start_node(share, out_dir, 'capture_adjudicator', 'capture_adjudicator', 'b2', {})
    try:
        node.run(1.0)                            # 新鲜输入已缓存
        node.on_odom = node.on_targets = False   # **源停**
        node.run(1.0)                            # 年龄 > car/target_max_age_s(0.5)
        node.on_state = True
        node.state_val = AgcTaskState.GROUND_APPROACH   # 超龄后才开窗口
        node.run(1.5)
    finally:
        _stop_proc(proc, log)
        node.all_off()
    b2_hits, b2_events = list(node.hits), list(node.events)

    res['b1_low_conf_hits'] = b1_hits
    res['b1_low_conf_events'] = b1_events
    res['b1_low_conf_no_output'] = not b1_hits and not b1_events
    res['b2_source_stop_hits'] = b2_hits
    res['b2_source_stop_events'] = b2_events
    res['b2_source_stop_no_output'] = not b2_hits and not b2_events
    res['pass'] = bool(res['b1_low_conf_no_output'] and res['b2_source_stop_no_output'])
    return res


def case_c(node, share, out_dir):
    """连续两回合（epoch 变化）各独立命中（第二回合 fired_ 已复位，重新产出）。"""
    res = {}
    node.reset_buffers()
    node.set_car(0.0, 0.0, 0.0)
    node.set_target(1.0, 0.0, 0.0)
    node.tgt_conf, node.tgt_id = 0.9, 'intruder_uav'
    node.on_odom = node.on_targets = node.on_epoch = node.on_state = True
    node.state_val = AgcTaskState.IDLE   # 先关窗口，令发现期不命中
    node.epoch = 1
    proc, log, _ = _start_node(share, out_dir, 'capture_adjudicator', 'capture_adjudicator', 'c', {})
    try:
        node.run(3.0)                        # 发现完成（窗口关，不命中）
        node.state_val = AgcTaskState.GROUND_APPROACH   # 回合 1：开窗口 → 命中（ep=1）
        run1 = node.run_until(lambda: len(node.events) >= 1, 6.0)
        node.epoch = 2                       # 新 episode（epoch 变化 → fired_ 复位）
        run2 = node.run_until(lambda: len(node.events) >= 2, 6.0)
        node.run(0.3)
    finally:
        _stop_proc(proc, log)
        node.all_off()
    events = list(node.events)
    eps = [_kv(e, 'ep') for e in events]
    res['run1_hit'] = bool(run1)
    res['run2_hit'] = bool(run2)
    res['events'] = events
    res['hits'] = list(node.hits)
    res['event_epochs'] = eps
    res['two_round_epochs'] = sorted(set(eps))
    res['all_events_capture_net'] = bool(events) and all(e.startswith('CAPTURE_NET') for e in events)
    res['hit_value_exactly_CAPTURE_NET'] = all(d == 'CAPTURE_NET' for d in node.hits) and bool(node.hits)
    res['pass'] = bool(
        run1 and run2 and len(events) >= 2 and set(eps) >= {'1', '2'} and
        res['all_events_capture_net'] and res['hit_value_exactly_CAPTURE_NET'])
    return res


def case_e(node, share, out_dir):
    """target_id 切换 → 身份不匹配不产出命中。"""
    res = {}
    node.reset_buffers()
    node.set_car(0.0, 0.0, 0.0)
    node.set_target(1.0, 0.0, 0.0)
    node.tgt_conf, node.tgt_id = 0.9, 'other_uav'   # 与 required_target_id 不匹配
    node.on_odom = node.on_targets = node.on_epoch = node.on_state = True
    node.state_val = AgcTaskState.GROUND_APPROACH
    node.epoch = 1
    proc, log, _ = _start_node(share, out_dir, 'capture_adjudicator', 'capture_adjudicator', 'e',
                               {'required_target_id': 'intruder_uav'})
    try:
        node.run(2.0)
        mismatch_hits, mismatch_events = list(node.hits), list(node.events)
        node.tgt_id = 'intruder_uav'          # 身份匹配 → 允许产出
        matched = node.run_until(lambda: bool(node.hits), 4.0)
    finally:
        _stop_proc(proc, log)
        node.all_off()
    res['mismatch_hits'] = mismatch_hits
    res['mismatch_events'] = mismatch_events
    res['no_output_on_mismatch'] = not mismatch_hits and not mismatch_events
    res['matched_hit'] = bool(matched)
    res['matched_hits'] = list(node.hits)
    res['pass'] = bool(res['no_output_on_mismatch'] and res['matched_hit'])
    return res


def case_task_manager(node, share, out_dir):
    """d（旧 jam / 旧 capture 拒收）+ f（同拍 capture + jam 按事件时刻定序）。"""
    res = {'d': {}, 'f': {}}
    overrides = {'air_present': False, 'fsm_hz': 2.0}
    node.reset_buffers()
    node.on_odom = node.on_targets = node.on_ov = node.on_cmd = node.on_epoch = True
    node.on_state = False                      # 任务态由被测 task_manager 发布
    node.set_car(0.0, 0.0, 0.0)
    node.set_target(0.5, 0.0, 0.0)
    node.tgt_conf, node.tgt_id = 0.9, 'intruder_uav'
    node.epoch = 0
    proc, log, _ = _start_node(share, out_dir, 'task_manager', 'task_manager', 'tm', overrides)
    reached = None
    try:
        node.start_cli = node.create_client(Trigger, '/agc/task/start')
        if not node.start_cli.wait_for_service(timeout_sec=20.0):
            res['error'] = '/agc/task/start service never appeared'
            return res
        # 旧 jam 在 start 前到达（应被忽略：episode_epoch_ < 0）
        for _ in range(6):
            node.pub_jam_confirm.publish(String(data='JAM_RTL'))
            node.run(0.1)
        node.run(0.5)

        def start():
            fut = node.start_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=25.0)
            return fut.result() if fut.done() else None

        # --- episode 1 ---
        r1 = start()
        res['d']['start1_ok'] = bool(r1 is not None and r1.success)
        reached = node.run_until(lambda: node.state_name == 'GROUND_APPROACH', 10.0)
        node.run(1.0)                          # 两个 tick（fsm_hz=2）
        res['d']['state_after_prestart_jam'] = node.state_name
        res['d']['old_jam_ignored'] = bool(node.state_name == 'GROUND_APPROACH')
        # 旧 capture（ep 不匹配）→ 拒收
        node.pub_capture_event.publish(String(data='CAPTURE_NET ep=999 t=0.0 target=intruder_uav'))
        node.run(1.0)
        res['d']['state_after_ep_mismatch'] = node.state_name
        res['d']['ep_mismatch_rejected'] = bool(node.state_name == 'GROUND_APPROACH')
        # 本 episode capture（ep 匹配）→ 结算
        node.pub_capture_event.publish(
            String(data='CAPTURE_NET ep=%d t=0.0 target=intruder_uav' % node.epoch))
        settled = node.run_until(lambda: node.state_name == 'INTERCEPT_CONFIRMED', 5.0)
        res['d']['ep_match_settled'] = bool(settled)
        res['d']['ep_match_reason'] = node.state_reason
        res['d']['pass'] = bool(
            res['d']['start1_ok'] and reached and res['d']['old_jam_ignored'] and
            res['d']['ep_mismatch_rejected'] and settled and node.state_reason == 'CAPTURE_NET')

        # --- episode 2：f1 jam 先到 → jam 胜 ---
        r2 = start()
        res['f']['start2_ok'] = bool(r2 is not None and r2.success)
        node.run_until(lambda: node.state_name == 'GROUND_APPROACH', 10.0)
        node.pub_jam_confirm.publish(String(data='JAM_HOLD'))
        node.run(0.05)
        node.pub_capture_event.publish(
            String(data='CAPTURE_NET ep=%d t=0.0 target=intruder_uav' % node.epoch))
        f1_ok = node.run_until(lambda: node.state_name == 'INTERCEPT_CONFIRMED', 5.0)
        res['f']['jam_first_reason'] = node.state_reason
        res['f']['jam_first_wins'] = bool(f1_ok and node.state_reason == 'JAM_HOLD')

        # --- episode 3：f2 capture 先到 → capture 胜 ---
        r3 = start()
        res['f']['start3_ok'] = bool(r3 is not None and r3.success)
        node.run_until(lambda: node.state_name == 'GROUND_APPROACH', 10.0)
        node.pub_capture_event.publish(
            String(data='CAPTURE_NET ep=%d t=0.0 target=intruder_uav' % node.epoch))
        node.run(0.05)
        node.pub_jam_confirm.publish(String(data='JAM_RTL'))
        f2_ok = node.run_until(lambda: node.state_name == 'INTERCEPT_CONFIRMED', 5.0)
        res['f']['capture_first_reason'] = node.state_reason
        res['f']['capture_first_wins'] = bool(f2_ok and node.state_reason == 'CAPTURE_NET')
        res['f']['pass'] = bool(
            res['f']['start2_ok'] and res['f']['jam_first_wins'] and
            res['f']['start3_ok'] and res['f']['capture_first_wins'])
    finally:
        _stop_proc(proc, log)
        node.all_off()
    return res


def case_g(_node, _share, _out_dir):
    """grep 证 capture_hit 值未变 + agc_msgs 未改。"""
    res = {}
    src = Path(ROOT) / 'src' / 'agc_task' / 'src' / 'capture_adjudicator.cpp'
    text = src.read_text(encoding='utf-8', errors='replace')
    res['hit_literal_present'] = ('hit.data = "CAPTURE_NET";' in text)
    # agc_msgs 未改（git 工作树 + 暂存均空）
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
    res['agc_msgs_unchanged'] = (rc1 == 0 and rc2 == 0 and rc3 == 0 and
                                 not d1 and not d2 and not d3)
    res['pass'] = bool(res['hit_literal_present'] and res['agc_msgs_unchanged'])
    return res


def dump(res, out_dir):
    path = os.path.join(out_dir, 'metrics', 'r021.json')
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
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r021-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    share = get_package_share_directory('agc_task')
    res = {'run_id': run_id, 'tests': {}}

    rclpy.init()
    node = R21Harness()
    try:
        res['tests']['a_no_window_no_output'] = case_a(node, share, out_dir)
        res['tests']['b_stale_or_lowconf_no_output'] = case_b(node, share, out_dir)
        res['tests']['c_two_rounds_independent'] = case_c(node, share, out_dir)
        res['tests']['e_target_id_switch'] = case_e(node, share, out_dir)
        tm = case_task_manager(node, share, out_dir)
        res['tests']['d_old_event_reject'] = dict(tm.get('d', {}))
        res['tests']['f_earliest_event_wins'] = dict(tm.get('f', {}))
        if 'error' in tm:
            res['tests']['d_old_event_reject']['pass'] = False
            res['tests']['f_earliest_event_wins']['pass'] = False
            res['error'] = tm['error']
        res['tests']['g_frozen_value_and_abi'] = case_g(node, share, out_dir)
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
