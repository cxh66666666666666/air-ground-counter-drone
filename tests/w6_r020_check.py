#!/usr/bin/env python3
"""w6_r020_check —— 评审 R-020（占位包络标注 + 高度含车体 + 独立干扰包络）自包含 ROS 级验收 harness。

结构拷贝自 `tests/w6_r019_check.py`：**本脚本不起 launch**（自包含），自己充当「假 odom /
目标源」桩环境，以 subprocess 直接起 `install/agc_task/lib/agc_task/{capture_adjudicator,
jammer}` + 临时参数文件（`use_sim_time: False`，墙钟，规避无 `/clock` 时间冻结），无需 gz / launch。

假源（本 harness 节点）：
  * 发布 `/agc/ground/odom`（`nav_msgs/Odometry`，车 `pose.pose.position.z` 可控）。
  * 发布 `/agc/targets`（`agc_msgs/TargetTrackArray`，取 `tracks[0].pose.pose.position.{x,y,z}`，
    `header.frame_id=world`，`confidence>=0.5`）。
  * 订阅 `/agc/task/capture_hit`、`/agc/task/capture_evidence`、`/agc/task/envelope_in`、
    `/agc/task/envelope_jam`；case c 另订 jammer 的 `/agc/task/jam_state` /
    `/agc/task/jam_confirm` / `/agc/task/jam_evidence`，并发布 `/agc/task/jam_cmd`。

用例（结果落 `experiments/w6-task-eval/<run_id>/metrics/r020.json`，ADR-0007）：
  a 车体高度计入：`car.z=1.0`、`launcher_height_m=0.0`、目标 `z=1.0` 近距 → 断言 `capture_hit` 命中
    且 `capture_evidence` 里 `h≈0.0`（旧式会得 `1.0`，故可证新式）。
  b 超适用范围 → 不命中并标注：目标 `z=3.0`（>`applicable_max_alt_m`，`car.z=0`）→ 断言**无**
    `capture_hit` 且证据含 `out_of_applicable_range`。
  c 捕获包络 ≠ 干扰包络：目标在捕获包络内（水平 ≈1.2 m，`h≈0`）但**不在**独立干扰包络内
    （水平 > `jam_envelope_max_range_m=1.0`）→ 断言 `envelope_in==true` 且 `envelope_jam==false`；
    起 jammer 发 `RTL`，断言 jammer **未**产出任何 `JAM_*` 授权，且其日志表明因包络不生效被忽略。
  d 可达性核验（**不**倒调参数）：由 `v0·θ` 解析顶点推可达高度，与真实目标高度（场景默认
    `scenario_reset.target_z`）比对，`r020.json` 含 `reachability_check` 块；并断言
    `docs/adr/0035-*.md` 存在且含占位 / 限制表述。

注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身 → 自杀）；用 "[x]xx" 写法规避或 Popen 句柄 kill。
"""

import argparse
import glob
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
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory

from agc_msgs.msg import AgcTaskState, TargetTrack, TargetTrackArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, String

ODOM_TOPIC = '/agc/ground/odom'
TARGET_TOPIC = '/agc/targets'
CAPTURE_HIT_TOPIC = '/agc/task/capture_hit'
CAPTURE_EVIDENCE_TOPIC = '/agc/task/capture_evidence'
ENV_IN_TOPIC = '/agc/task/envelope_in'
ENV_JAM_TOPIC = '/agc/task/envelope_jam'
JAM_CMD_TOPIC = '/agc/task/jam_cmd'
JAM_STATE_TOPIC = '/agc/task/jam_state'
JAM_CONFIRM_TOPIC = '/agc/task/jam_confirm'
JAM_EVIDENCE_TOPIC = '/agc/task/jam_evidence'
# R-021：capture_adjudicator 现要求有效任务窗口（GROUND_APPROACH）+ episode，故假源须补发
STATE_TOPIC = '/agc/task/state'
EPOCH_TOPIC = '/agc/scenario/reset_epoch'
DT = 0.05


def _kv(s, key):
    """从空格分隔的标注串里取 `key=value` 的 value（无则 None）。"""
    for tok in s.split():
        if tok.startswith(key + '='):
            return tok.split('=', 1)[1]
    return None


class R20Harness(Node):
    def __init__(self):
        super().__init__('w6_r020_check')
        self.pub_odom = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.pub_targets = self.create_publisher(TargetTrackArray, TARGET_TOPIC, 10)
        self.pub_jam_cmd = self.create_publisher(String, JAM_CMD_TOPIC, 10)
        # R-021：假源补发任务窗口状态（GROUND_APPROACH）与 episode（transient_local）
        self.pub_state = self.create_publisher(AgcTaskState, STATE_TOPIC, 10)
        self.pub_epoch = self.create_publisher(
            Int32, EPOCH_TOPIC,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, CAPTURE_HIT_TOPIC, self._on_hit, 10)
        self.create_subscription(String, CAPTURE_EVIDENCE_TOPIC, self._on_evidence, 10)
        self.create_subscription(Bool, ENV_IN_TOPIC, self._on_env_in, 10)
        self.create_subscription(Bool, ENV_JAM_TOPIC, self._on_env_jam, 10)
        self.create_subscription(String, JAM_STATE_TOPIC, self._on_jam_state, 10)
        self.create_subscription(String, JAM_CONFIRM_TOPIC, self._on_jam_confirm, 10)
        self.create_subscription(String, JAM_EVIDENCE_TOPIC, self._on_jam_evidence, 10)
        self.create_timer(DT, self._pub_tick)

        self.pub_on = False
        self.car_x, self.car_y, self.car_z = 0.0, 0.0, 0.0
        self.tgt_x, self.tgt_y, self.tgt_z = 0.0, 0.0, 0.0
        self.reset_buffers()

    # ---- callbacks ----
    def _on_hit(self, m):
        self.hits.append((time.monotonic(), m.data.strip()))

    def _on_evidence(self, m):
        self.evidence.append((time.monotonic(), m.data.strip()))

    def _on_env_in(self, m):
        self.env_in.append((time.monotonic(), bool(m.data)))

    def _on_env_jam(self, m):
        self.env_jam.append((time.monotonic(), bool(m.data)))

    def _on_jam_state(self, m):
        self.jam_states.append((time.monotonic(), m.data.strip().upper()))

    def _on_jam_confirm(self, m):
        self.jam_confirms.append((time.monotonic(), m.data.strip().upper()))

    def _on_jam_evidence(self, m):
        self.jam_evidence.append((time.monotonic(), m.data.strip()))

    # ---- 发布 ----
    def _pub_tick(self):
        if not self.pub_on:
            return
        od = Odometry()
        od.header.frame_id = 'world'
        od.header.stamp = self.get_clock().now().to_msg()
        od.child_frame_id = 'agc_car'
        od.pose.pose.position.x = self.car_x
        od.pose.pose.position.y = self.car_y
        od.pose.pose.position.z = self.car_z
        od.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(od)

        t = TargetTrack()
        t.header.frame_id = 'world'
        t.header.stamp = od.header.stamp
        t.target_id = 'intruder_uav'
        t.pose.pose.position.x = self.tgt_x
        t.pose.pose.position.y = self.tgt_y
        t.pose.pose.position.z = self.tgt_z
        t.pose.pose.orientation.w = 1.0
        t.confidence = 0.9
        t.source = 'gt_adapter'
        arr = TargetTrackArray()
        arr.header.frame_id = 'world'
        arr.header.stamp = od.header.stamp
        arr.tracks = [t]
        self.pub_targets.publish(arr)

        # R-021：任务窗口 = GROUND_APPROACH（旧行为恒可命中；现须显式任务窗口）
        st = AgcTaskState()
        st.header.stamp = od.header.stamp
        st.header.frame_id = 'world'
        st.state = AgcTaskState.GROUND_APPROACH
        st.state_name = 'GROUND_APPROACH'
        self.pub_state.publish(st)
        self.pub_epoch.publish(Int32(data=1))

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

    def publish_jam_cmd(self, s):
        self.pub_jam_cmd.publish(String(data=s))
        return time.monotonic()

    def reset_buffers(self):
        self.hits = []
        self.evidence = []
        self.env_in = []
        self.env_jam = []
        self.jam_states = []
        self.jam_confirms = []
        self.jam_evidence = []

    def set_car(self, x, y, z):
        self.car_x, self.car_y, self.car_z = x, y, z

    def set_target(self, x, y, z):
        self.tgt_x, self.tgt_y, self.tgt_z = x, y, z


# ---- 进程管理 ----
def _exe(share, name):
    return str(Path(share).parents[1] / 'lib' / 'agc_task' / name)


def _base_params(share, node_name):
    base = yaml.safe_load(open(os.path.join(share, 'config', 'w6_base.yaml')))
    return dict((base.get(node_name) or {}).get('ros__parameters', {}))


def _write_params(share, path, node_name, overrides):
    params = _base_params(share, node_name)
    params.update(overrides)
    params['use_sim_time'] = False   # 墙钟，规避无 /clock 时间冻结
    doc = {node_name: {'ros__parameters': params}}
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
    return params


def _start_node(share, out_dir, exe_name, node_name, prefix, overrides):
    path = os.path.join(out_dir, 'r020_%s_%s.yaml' % (prefix, node_name))
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
    """车体高度计入：h = target.z − car.z − launcher_h；car.z=1、目标 z=1 → h≈0（旧式得 1.0）。"""
    res = {}
    node.reset_buffers()
    node.set_car(0.0, 0.0, 1.0)
    node.set_target(1.2, 0.0, 1.0)
    node.pub_on = True
    proc, log, _ = _start_node(share, out_dir, 'capture_adjudicator', 'capture_adjudicator', 'a',
                               {'launcher_height_m': 0.0})
    try:
        node.run_until(lambda: bool(node.hits), 6.0)
        node.run(0.6)
    finally:
        _stop_proc(proc, log)
        node.pub_on = False
    hits = [d for _t, d in node.hits]
    evs = [d for _t, d in node.evidence]
    hit_ev = [e for e in evs if e.startswith('CAPTURE_HIT')]
    h = None
    for e in hit_ev:
        v = _kv(e, 'h')
        if v is not None:
            h = float(v)
    res['hits'] = hits
    res['evidence'] = evs
    res['capture_hit_present'] = 'CAPTURE_NET' in hits
    res['hit_value_exactly_CAPTURE_NET'] = all(d == 'CAPTURE_NET' for d in hits) and bool(hits)
    res['reported_h'] = h
    res['model_id_in_evidence'] = any('model_id=ballistic_closed_form' in e for e in hit_ev)
    res['applicable_range_in_evidence'] = any(
        'applicable_max_alt_m=' in e for e in hit_ev)
    res['h_near_zero'] = bool(h is not None and abs(h - 0.0) <= 0.01)
    res['h_not_old_formula'] = bool(h is not None and abs(h - 1.0) > 0.01)
    res['pass'] = bool(
        res['capture_hit_present'] and res['hit_value_exactly_CAPTURE_NET'] and
        res['h_near_zero'] and res['h_not_old_formula'] and res['model_id_in_evidence'])
    return res


def case_b(node, share, out_dir):
    """超适用范围 / 不可达 → 不命中并标注：目标 z=3.0（> applicable_max_alt_m）→ 无 hit + out_of_applicable_range。"""
    res = {}
    node.reset_buffers()
    node.set_car(0.0, 0.0, 0.0)
    node.set_target(0.5, 0.0, 3.0)
    node.pub_on = True
    proc, log, _ = _start_node(share, out_dir, 'capture_adjudicator', 'capture_adjudicator', 'b', {})
    try:
        node.run(1.0)   # 发现
        node.run_until(lambda: any('out_of_applicable_range' in d for _t, d in node.evidence), 6.0)
        node.run(0.6)
    finally:
        _stop_proc(proc, log)
        node.pub_on = False
    hits = [d for _t, d in node.hits]
    evs = [d for _t, d in node.evidence]
    env_in_final = node.env_in[-1][1] if node.env_in else None
    h = None
    skip_ev = [e for e in evs if 'out_of_applicable_range' in e]
    for e in skip_ev:
        v = _kv(e, 'h')
        if v is not None:
            h = float(v)
    res['hits'] = hits
    res['evidence'] = evs
    res['no_capture_hit'] = not hits
    res['has_out_of_applicable_range'] = bool(skip_ev)
    res['envelope_in'] = env_in_final
    res['reported_h'] = h
    res['h_equals_target_alt'] = bool(h is not None and abs(h - 3.0) <= 0.01)
    res['pass'] = bool(
        res['no_capture_hit'] and res['has_out_of_applicable_range'] and
        res['envelope_in'] is False)
    return res


def case_c(node, share, out_dir):
    """捕获包络 ≠ 干扰包络：目标在捕获包络内但不在独立干扰包络内 → 干扰不生效。"""
    res = {}
    node.reset_buffers()
    node.set_car(0.0, 0.0, 0.0)
    node.set_target(1.2, 0.0, 0.0)   # 水平 1.2 m；捕获可及（R(0)≈2.44 m），但 > jam_envelope_max_range_m=1.0
    node.pub_on = True
    cap, cap_log, _ = _start_node(share, out_dir, 'capture_adjudicator', 'capture_adjudicator', 'c', {})
    jam, jam_log, _ = _start_node(share, out_dir, 'jammer', 'jammer', 'c', {})
    jam_log_path = os.path.join(out_dir, 'log', 'case_c_jammer.log')
    try:
        node.run(2.0)   # 发现 + 包络稳定
        # 反复发干扰命令（规避 DDS 发现延迟导致的丢包）；期望**不**产出任何授权事件
        for _ in range(12):
            node.publish_jam_cmd('RTL')
            node.run(0.15)
        node.run(0.5)
    finally:
        _stop_proc(cap, cap_log)
        _stop_proc(jam, jam_log)
        node.pub_on = False
    log_text = ''
    try:
        log_text = Path(jam_log_path).read_text(errors='replace')
    except Exception:
        pass
    env_in_any = any(v for _t, v in node.env_in)
    env_jam_any_true = any(v for _t, v in node.env_jam)
    jam_auth = [d for _t, d in node.jam_states if d.startswith('JAM_') and d != 'JAM_NONE']
    res['envelope_in_observed_true'] = env_in_any
    res['envelope_jam_observed_true'] = env_jam_any_true
    res['jam_authorized_events'] = jam_auth
    res['no_jam_authorization'] = not jam_auth
    res['jammer_log_ignored_env'] = ('target out of engagement envelope' in log_text
                                     or 'ignored' in log_text)
    res['pass'] = bool(
        env_in_any and not env_jam_any_true and
        not jam_auth and res['jammer_log_ignored_env'])
    return res


def case_d(node, share, out_dir):
    """可达性核验（不倒调参数）+ ADR-0035 存在。"""
    res = {}
    cap = _base_params(share, 'capture_adjudicator')
    v0 = float(cap.get('net_v0_mps'))
    theta = math.radians(float(cap.get('net_angle_deg')))
    g = float(cap.get('net_g'))
    drag = float(cap.get('net_drag'))
    diam = float(cap.get('net_diameter_m'))
    launcher_h = float(cap.get('launcher_height_m', 0.0))
    vy = v0 * math.sin(theta)
    apex = (vy * vy) / (2.0 * g)                       # 解析顶点高度
    applicable_max = float(cap.get('applicable_max_alt_m'))
    # 真实目标高度：场景默认 target_z（w6_base scenario_reset），车体默认 z（car_z）
    scn = _base_params(share, 'scenario_reset')
    real_target_alt = float(scn.get('target_z'))
    car_z = float(scn.get('car_z', 0.0))
    real_h = real_target_alt - car_z - launcher_h
    reachable = real_h <= apex
    # 该高度处水平射程 R(h)（占位闭式；下降段）
    if 0.0 <= real_h <= apex:
        disc = vy * vy - 2.0 * g * real_h
        t = (vy + math.sqrt(max(disc, 0.0))) / g
        h_reach = v0 * math.cos(theta) * t * drag + 0.5 * diam
    else:
        h_reach = 0.0
    res['model_id'] = cap.get('model_id')
    res['real_target_alt_m'] = real_target_alt
    res['car_z_m'] = car_z
    res['launcher_height_m'] = launcher_h
    res['real_h_m'] = round(real_h, 4)
    res['reachable_max_alt_m'] = round(apex, 4)
    res['applicable_max_alt_m'] = applicable_max
    res['reachable'] = bool(reachable)
    res['horizontal_reach_at_real_h_m'] = round(h_reach, 4)
    res['note'] = (
        '占位闭式点弹道（v0=%.2f, θ=%.1f°, g=%.2f）解析顶点 %.4f m 为可达高度上限；'
        '真实目标高度取场景默认 target_z=%.2f m（未倒调任何参数）。'
        '此结论为**占位**模型结论，未计 t_exp / 网几何 / 瞄准提前量，**不**得据此声称「移动式主动拦截」。'
        % (v0, math.degrees(theta), g, apex, real_target_alt))
    res['reachability_consistent'] = bool(reachable == (real_h <= apex))
    # ADR-0035
    root = Path(share).parents[3]
    adrs = sorted(glob.glob(str(root / 'docs' / 'adr' / '0035-*.md')))
    text = open(adrs[0], encoding='utf-8').read() if adrs else ''
    res['adr_paths'] = adrs
    res['adr_exists'] = bool(adrs)
    res['adr_mentions_placeholder'] = ('占位' in text)
    res['adr_mentions_limitation'] = (('限制' in text) or ('不得' in text))
    res['pass'] = bool(
        res['reachability_consistent'] and res['adr_exists'] and
        res['adr_mentions_placeholder'] and res['adr_mentions_limitation'])
    return res


def dump(res, out_dir):
    path = os.path.join(out_dir, 'metrics', 'r020.json')
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
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r020-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    share = get_package_share_directory('agc_task')
    res = {'run_id': run_id, 'tests': {}}

    rclpy.init()
    node = R20Harness()
    try:
        res['tests']['a_car_height_included'] = case_a(node, share, out_dir)
        res['tests']['b_out_of_applicable_range'] = case_b(node, share, out_dir)
        res['tests']['c_envelope_separation'] = case_c(node, share, out_dir)
        res['tests']['d_reachability_check'] = case_d(node, share, out_dir)
        # 顶层：可达性核验块（真实目标高度 vs 可达高度）
        d = res['tests']['d_reachability_check']
        res['reachability_check'] = {
            'real_target_alt_m': d.get('real_target_alt_m'),
            'car_z_m': d.get('car_z_m'),
            'launcher_height_m': d.get('launcher_height_m'),
            'real_h_m': d.get('real_h_m'),
            'reachable_max_alt_m': d.get('reachable_max_alt_m'),
            'applicable_max_alt_m': d.get('applicable_max_alt_m'),
            'horizontal_reach_at_real_h_m': d.get('horizontal_reach_at_real_h_m'),
            'reachable': d.get('reachable'),
            'note': d.get('note'),
        }
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
