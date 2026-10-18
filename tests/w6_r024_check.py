#!/usr/bin/env python3
"""w6_r024_check —— 评审 R-024（三类时间口径与 RTF 记录）自包含 ROS 级验收 harness。

结构拷贝自 `tests/w6_r023_check.py`（自包含：不起 launch，自己充当「假源 + 假 reset 服务」桩环境，
以 subprocess 直接起 `install/agc_task/lib/agc_task/task_manager` + 临时参数文件，Popen 句柄 kill，
**禁止** pkill -f "<名字>"）。

R-024 关键新增：本 harness **自己发布 `/clock`**（`rosgraph_msgs/Clock`）驱动 `task_manager` 的仿真
时钟（用例 a 传 `use_sim_time: True`），从而演示「暂停仿真（停 `/clock` / RTF→0）→ `mission_sim_budget`
（C4 仿真秒）不触发，而 `episode_wall_watchdog`（墙钟）触发并判 infra」。

用例（结果落 `experiments/w6-task-eval/<run_id>/metrics/r024.json`，ADR-0007）：
  a 暂停仿真 → mission 超时（`t_max_s`，仿真秒）**不**触发；wall watchdog 分类 infra（**核心**）；
    恢复 `/clock` 后 C4 才触发 `MISSION_FAILED(TIMEOUT)`（对照：C4 走 ROS 时钟）。
  b RTF 与起止事件：加速（rtf≈2.0）/ 减速（rtf≈0.5）两 run 按仿真时间可比；无仿真时钟 → rtf=None。
  c 三类口径分别标注、互不冒充；源码检查 `run_logger.cpp` 进程时长口径 + `task_manager.cpp` C4 判据串。
  d train/eval 单一预算来源：`classify_episode_outcome` 定义恰 1 处、eval adapter 复用；无第三套口径。
  e 冻结值与 ABI 回归：`capture_hit` 字面量不变 + 工作树 `src/agc_msgs` 未改。

注：每个 task_manager 用例起**独立进程**，避免状态机跨用例污染（本 harness 仅用例 a 起进程）。
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

# 复用 episode_runner 的唯一预算语义（R-024：single source）。
sys.path.insert(0, '/home/chen/AGC/scripts')
from episode_runner import TIME_BUDGETS, build_time_evidence, classify_episode_outcome  # noqa: E402

from agc_msgs.msg import AgcTaskState, TargetTrack, TargetTrackArray
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger

ROOT = '/home/chen/AGC'
SELF = Path(__file__).resolve()
ODOM_TOPIC = '/agc/ground/odom'
TARGET_TOPIC = '/agc/targets'
ODOM_VALID_TOPIC = '/agc/ground/odom_valid'
CMD_TOPIC = '/agc/ground/cmd_vel'
STATE_TOPIC = '/agc/task/state'
EPOCH_TOPIC = '/agc/scenario/reset_epoch'
CLOCK_TOPIC = '/clock'
RESET_SERVICE = '/agc/scenario/reset'
START_SERVICE = '/agc/task/start'
DT = 0.05


def _tl_qos():
    """对齐 scenario_reset / epoch 发布端：depth=1 + RELIABLE + TRANSIENT_LOCAL（R-015 / R-022）。"""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class R24Harness(Node):
    def __init__(self):
        super().__init__('w6_r024_check')
        self.pub_odom = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.pub_targets = self.create_publisher(TargetTrackArray, TARGET_TOPIC, 10)
        self.pub_ov = self.create_publisher(Bool, ODOM_VALID_TOPIC, 10)
        self.pub_cmd = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.pub_epoch = self.create_publisher(Int32, EPOCH_TOPIC, _tl_qos())
        self.clock_pub = self.create_publisher(Clock, CLOCK_TOPIC, 10)
        self.create_subscription(AgcTaskState, STATE_TOPIC, self._on_state, 10)
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

        # 仿真时钟（R-024）：harness 自己发 /clock 驱动 task_manager 的 use_sim_time 时钟。
        self.sim_mode = False       # True = 假源时间戳 / /clock 用自增的仿真时间
        self.publish_clock = False  # True = 每个 tick 推进 sim_ns 并发布 /clock
        self.sim_ns = 0
        self.reset_buffers()

    # ---- callbacks ----
    def _on_state(self, m):
        self.state = int(m.state)
        self.state_name = m.state_name
        self.state_reason = m.reason_code

    def _on_reset(self, req, resp):
        self.reset_calls += 1
        self.epoch += 1
        self.pub_epoch.publish(Int32(data=self.epoch))
        resp.success = True
        resp.message = 'reset ok epoch=%d' % self.epoch
        return resp

    # ---- 仿真时钟 ----
    def _sim_stamp(self):
        """把 sim_ns 转成 builtin_interfaces/Time（header / clock 共用）。"""
        m = TimeMsg()
        m.sec = int(self.sim_ns // 1000000000)
        m.nanosec = int(self.sim_ns % 1000000000)
        return m

    # ---- 发布 ----
    def _pub_tick(self):
        # 仿真模式下：模仿真实仿真，每个墙钟 tick 推进仿真时间并发布 /clock（冻结时停发 → RTF→0）。
        if self.sim_mode and self.publish_clock:
            self.sim_ns += int(DT * 1e9)
            self.clock_pub.publish(Clock(clock=self._sim_stamp()))
        stamp = self._sim_stamp() if self.sim_mode else self.get_clock().now().to_msg()
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
                arr.tracks = []
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

    def all_off(self):
        self.on_odom = self.on_targets = self.on_ov = self.on_cmd = self.on_epoch = False
        self.targets_empty = False


# ---- 进程管理 ----
def _exe(share, name):
    return str(Path(share).parents[1] / 'lib' / 'agc_task' / name)


def _base_params(share, node_name):
    base = yaml.safe_load(open(os.path.join(share, 'config', 'w6_base.yaml')))
    return dict((base.get(node_name) or {}).get('ros__parameters', {}))


def _write_params(share, path, node_name, overrides):
    params = _base_params(share, node_name)
    params.update(overrides)
    # 用例可传 use_sim_time=True；缺省保持 False（避免无 /clock 时挂起）。
    params.setdefault('use_sim_time', False)
    doc = {node_name: {'ros__parameters': params}}
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
    return params


def _start_tm(share, out_dir, prefix, overrides):
    path = os.path.join(out_dir, 'r024_%s_task_manager.yaml' % prefix)
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


# ---- 用例 ----
def case_a(node, share, out_dir):
    """a. 暂停仿真 → mission 超时不触发、wall watchdog 触发并判 infra（核心）。"""
    res = {}
    # 全源开启（odom / ov / cmd / epoch），**关目标源**（保持 AIR_SEARCH）。
    node.reset_buffers()
    node.on_odom = node.on_ov = node.on_cmd = node.on_epoch = True
    node.on_targets = False
    node.targets_empty = False
    node.epoch = 0
    node.reset_calls = 0
    # 仿真模式：task_manager use_sim_time=True，本 harness 发 /clock。
    node.sim_mode = True
    node.publish_clock = True
    node.sim_ns = 0
    overrides = {'air_present': False, 't_dwell_s': 2.0, 'fsm_hz': 20.0, 'd_track_m': 3.0,
                 't_max_s': 3.0, 'car_odom_max_age_s': 0.5, 'use_sim_time': True}
    proc, log, _ = _start_tm(share, out_dir, 'a', overrides)
    try:
        if not _wait_services(node):
            res['error'] = '/agc/task/start service never appeared'
            res['pass'] = False
            return res
        node.run(2.5)                                   # 推进仿真时钟（1Hz 状态定时器更新 prev_clock_）
        r1 = node.call(node.start_cli)
        res['start_success'] = bool(r1 is not None and r1.success)
        res['start_message'] = r1.message if r1 is not None else ''
        res['sim_ns_at_start'] = node.sim_ns
        node.run(0.5)
        res['state_after_start'] = node.state_name
        res['air_search_ok'] = bool(node.state_name == 'AIR_SEARCH')
        # 冻结仿真：停发 /clock（RTF→0），墙钟继续推进 5s（远超 t_max_s=3.0 仿真秒）
        node.publish_clock = False
        node.run(5.0)
        res['state_during_freeze'] = node.state_name
        res['mission_budget_not_triggered'] = bool(node.state_name != 'MISSION_FAILED')
        # wall watchdog 分类（墙钟 5s > watchdog 3s，terminal 未达终态）
        cls = classify_episode_outcome('ACTIVE', '', None, None, 5.0, 3.0)
        res['wall_watchdog_cls'] = cls
        res['wall_watchdog_classified_infra'] = bool(
            cls['outcome_class'] == 'wall_watchdog_timeout' and
            cls['classification'] == 'infra' and
            cls['budget_kind'] == 'episode_wall_watchdog' and
            cls['counted_as_success'] is False)
        # 恢复仿真：C4 走 ROS 时钟，恢复后才触发超时
        node.publish_clock = True
        reached = node.run_until(lambda: node.state_name == 'MISSION_FAILED', 8.0)
        res['state_after_resume'] = node.state_name
        res['reason_after_resume'] = node.state_reason
        res['mission_timeout_after_resume'] = bool(
            reached and node.state_name == 'MISSION_FAILED' and node.state_reason == 'TIMEOUT')
        res['pass'] = bool(res['start_success'] and res['air_search_ok'] and
                           res['mission_budget_not_triggered'] and
                           res['wall_watchdog_classified_infra'] and
                           res['mission_timeout_after_resume'])
    finally:
        _stop_proc(proc, log)
        node.all_off()
        node.sim_mode = False
        node.publish_clock = False
    return res


def case_b(_node, _share, _out_dir):
    """b. RTF 与起止事件可比（加速 / 减速两 run 按仿真时间可比）。"""
    res = {}
    ev1 = build_time_evidence(100.0, 110.0, 0.0, 20.0)   # 墙钟 10s / 仿真 20s → rtf 2.0（加速）
    ev2 = build_time_evidence(200.0, 240.0, 0.0, 20.0)   # 墙钟 40s / 仿真 20s → rtf 0.5（减速）
    res['ev1_accel'] = ev1
    res['ev2_decel'] = ev2
    res['sim_elapsed_comparable'] = bool(ev1['sim_elapsed_s'] == 20.0 == ev2['sim_elapsed_s'])
    res['wall_elapsed_differs'] = bool(ev1['wall_elapsed_s'] != ev2['wall_elapsed_s'])
    res['rtf_ev1_accel'] = bool(abs(ev1['rtf'] - 2.0) < 1e-9)
    res['rtf_ev2_decel'] = bool(abs(ev2['rtf'] - 0.5) < 1e-9)
    res['rtf_none_when_no_sim'] = bool(build_time_evidence(0.0, 5.0, None, None)['rtf'] is None)
    res['keys_have_start_end'] = bool(all(
        k in ev1 for k in ('wall_start_s', 'wall_end_s', 'sim_start_s', 'sim_end_s')))
    res['pass'] = bool(res['sim_elapsed_comparable'] and res['wall_elapsed_differs'] and
                       res['rtf_ev1_accel'] and res['rtf_ev2_decel'] and
                       res['rtf_none_when_no_sim'] and res['keys_have_start_end'])
    return res


def case_c(_node, _share, _out_dir):
    """c. 三类口径分别标注、互不冒充 + 源码检查。"""
    res = {}
    res['time_budgets_keys'] = sorted(TIME_BUDGETS.keys())
    clocks = [v['clock'] for v in TIME_BUDGETS.values()]
    owners = [v['owner'] for v in TIME_BUDGETS.values()]
    res['clocks'] = clocks
    res['owners'] = owners
    res['three_keys'] = bool(len(TIME_BUDGETS) == 3)
    res['clocks_distinct'] = bool(len(set(clocks)) == 3)
    res['owners_distinct'] = bool(len(set(owners)) == 3)
    mf = classify_episode_outcome('MISSION_FAILED', 'TIMEOUT', None, None, 1.0, 3.0)
    wd = classify_episode_outcome('ACTIVE', '', None, None, 5.0, 3.0)
    res['mission_failed_budget'] = mf['budget_kind']
    res['active_timeout_budget'] = wd['budget_kind']
    res['no_impersonation'] = bool(mf['budget_kind'] == 'mission_sim_budget' and
                                   wd['budget_kind'] == 'episode_wall_watchdog')
    rl = (Path(ROOT) / 'src' / 'agc_task' / 'src' / 'run_logger.cpp').read_text(
        encoding='utf-8', errors='replace')
    tm = (Path(ROOT) / 'src' / 'agc_task' / 'src' / 'task_manager.cpp').read_text(
        encoding='utf-8', errors='replace')
    res['run_logger_process_duration_kind'] = ('process_wall_incl_warmup_shutdown' in rl)
    # run_logger 以转义引号输出 JSON 键（源文本为 \"rtf\"），故按键名 rtf 判其存在。
    res['run_logger_rtf_key'] = ('rtf' in rl)
    res['task_manager_c4_ros_clock'] = ('(t - start_time_).seconds() > t_max_' in tm)
    res['pass'] = bool(res['three_keys'] and res['clocks_distinct'] and res['owners_distinct'] and
                       res['no_impersonation'] and res['run_logger_process_duration_kind'] and
                       res['run_logger_rtf_key'] and res['task_manager_c4_ros_clock'])
    return res


def case_d(_node, _share, _out_dir):
    """d. train / eval 同一预算语义（单一来源，不产生第三套口径）。"""
    res = {}
    needle_def = 'def classify_episode_outcome'
    defs = []
    for root in (os.path.join(ROOT, 'scripts'), os.path.join(ROOT, 'src')):
        for p in Path(root).rglob('*.py'):
            if p.resolve() == SELF:                     # 跳过本 harness（仅引用该名字做检查）
                continue
            try:
                txt = p.read_text(encoding='utf-8', errors='replace')
            except Exception:
                continue
            if needle_def in txt:
                defs.append(str(p))
    res['classify_def_files'] = sorted(defs)
    res['single_source'] = bool(len(defs) == 1 and defs[0].endswith('/episode_runner.py'))
    er = (Path(ROOT) / 'scripts' / 'episode_runner.py').read_text(encoding='utf-8', errors='replace')
    res['eval_adapter_calls'] = bool('classify_episode_outcome(' in er)
    bk_files = []
    for p in Path(os.path.join(ROOT, 'scripts')).rglob('*.py'):
        if p.resolve() == SELF:
            continue
        try:
            txt = p.read_text(encoding='utf-8', errors='replace')
        except Exception:
            continue
        if 'budget_kind' in txt:                        # 口径取值定义只应来自单一来源
            bk_files.append(p.name)
    res['budget_kind_files'] = sorted(bk_files)
    res['no_third_budget_scheme'] = bool(sorted(bk_files) == ['episode_runner.py'])
    res['scan_excluded_self'] = SELF.name
    res['pass'] = bool(res['single_source'] and res['eval_adapter_calls'] and
                       res['no_third_budget_scheme'])
    return res


def case_e(_node, _share, _out_dir):
    """e. 冻结值（capture_hit）与 ABI 回归（agc_msgs 未改）。"""
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
    path = os.path.join(out_dir, 'metrics', 'r024.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-52s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
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
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r024-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    share = get_package_share_directory('agc_task')
    res = {'run_id': run_id, 'tests': {}}

    rclpy.init()
    node = R24Harness()
    try:
        res['tests']['a_paused_sim_no_mission_timeout_wall_watchdog_infra'] = \
            case_a(node, share, out_dir)
        res['tests']['b_rtf_and_start_end_events_comparable'] = case_b(node, share, out_dir)
        res['tests']['c_three_budgets_distinct_not_impersonating'] = case_c(node, share, out_dir)
        res['tests']['d_train_eval_single_budget_source'] = case_d(node, share, out_dir)
        res['tests']['e_frozen_value_and_abi'] = case_e(node, share, out_dir)
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
