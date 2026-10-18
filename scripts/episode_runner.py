#!/usr/bin/env python3
"""episode_runner —— W6 评估 runner（R12 / C9）。

对每个 `(scene, seed)`：起 `w6.launch.py backend:=lightweight scene:=<scene> seed:=<seed>` → 调
`/agc/task/start` → 追踪 `/agc/task/state` 至终态或超时 → SIGINT 拆除（令 run_logger 落
`summary.json`）。汇总每回合 `metrics/summary.json` 为 campaign 报告（含 C9 两指标），
落 `experiments/<campaign>/<run_id>/`（ADR-0007）。

`task_manager` 只做单 episode；批量由本 runner 驱动（spec R12）。

R-024：区分三类时间口径（互不冒充）—— mission_sim_budget（task_manager 的仿真时钟
超时）/ episode_wall_watchdog（本 runner 墙钟看门狗，超时判 infra、不算成功）/
process_wall_duration（run_logger 进程时长）；并以 RTF（仿真增量 / 墙钟增量）与起止事件
记录 run 证据，供不同 RTF 的成功率 / 耗时可比。

用法：
  python3 scripts/episode_runner.py --scenes s01_static_none_nominal_track,s08_static_none_nominal_capture --seeds 0
  python3 scripts/episode_runner.py --all --seeds 0,1
"""

import argparse
import glob
import json
import os
import random
import signal
import subprocess
import sys
import time
from datetime import datetime

import yaml
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped

from agc_msgs.msg import AgcTaskState
from std_srvs.srv import Trigger

TERMINALS = {
    AgcTaskState.TRACKING_SUCCESS, AgcTaskState.INTERCEPT_CONFIRMED,
    AgcTaskState.MISSION_FAILED, AgcTaskState.ABORTED, AgcTaskState.INFRA_FAILURE,
}
TERMINAL_NAMES = {
    AgcTaskState.TRACKING_SUCCESS: 'TRACKING_SUCCESS',
    AgcTaskState.INTERCEPT_CONFIRMED: 'INTERCEPT_CONFIRMED',
    AgcTaskState.MISSION_FAILED: 'MISSION_FAILED',
    AgcTaskState.ABORTED: 'ABORTED',
    AgcTaskState.INFRA_FAILURE: 'INFRA_FAILURE',
}

ROOT = '/home/chen/AGC'

SUCCESS_TERMINALS = {'TRACKING_SUCCESS', 'INTERCEPT_CONFIRMED'}

# R-031：就绪门槛用到的真值旁路话题（与 scenario_reset 默认一致）与新鲜窗口。
GT_CAR_TOPIC = '/agc/gt/agc_car'
GT_TARGET_TOPIC = '/agc/gt/intruder_uav'
GT_FRESH_S = 1.0

# R-031：受限进程组回收的时间预算（SIGINT 优雅 → SIGKILL 强杀，各步 wait 确认退出）。
REAP_SIGINT_S = 8.0
REAP_SIGKILL_S = 4.0


def _num_str(v):
    """R-031：数值 / not_applicable（None 或字符串）统一格式化，避免 %.3f 对 None 抛异常。"""
    return ('%.3f' % v) if isinstance(v, (int, float)) else 'n/a'

# R-024：三类时间预算口径标签（互不冒充）。clock 字段标注时钟来源；owner 标注归属节点/进程。
TIME_BUDGETS = {
    'mission_sim_budget': {
        'clock': 'ros_sim', 'owner': 'task_manager', 'param': 't_max_s',
        'note': 'C4 任务超时（仿真时钟 now-start_time）；由 task_manager 拥有，非本 runner 墙钟',
    },
    'episode_wall_watchdog': {
        'clock': 'wall_monotonic', 'owner': 'episode_runner', 'param': 'wall_watchdog_s',
        'note': '外部看门狗：runner 墙钟等待 episode 结束的上限；超时判 infra、不算成功',
    },
    'process_wall_duration': {
        'clock': 'wall_steady', 'owner': 'run_logger', 'param': 'duration_s',
        'note': '进程时长（墙钟，含预热 / 关闭）；非 episode 预算',
    },
}


# R-025：本 runner 固定轻量后端（无 PX4 / 空中链路）。
BACKEND = 'lightweight'
LAUNCH = 'w6.launch.py'

# R-025：Runbook #status 保留为进度事实；本报告只如实标注 backend/mode，**不**改契约 C3 文字，
# **不**把「期望终态匹配」自动等价为整链 / C3 GUI 验收。轻量 PASS ≠ 整链 / GUI 验收。
FULL_CHAIN_NOTE = ('本报告 backend=lightweight（w6.launch.py 轻量栈：m0_skeleton+capture_adjudicator+jammer，无 PX4/空中链路）；'
                   'pass 仅绑定本报告的明确命题，**不代表**整链（w6.launch.py backend:=full）/ C3 GUI 验收。')


def build_proposition(backend, mode, scene, expected_terminal, expected_reason):
    """R-025：由 expected_terminal/expected_reason/backend 生成**明确验收命题**（pass 与命题强绑定）。"""
    if backend == BACKEND:
        head = '轻量栈（m0_skeleton+capture_adjudicator+jammer，%s）' % mode
        tail = '（非整链 / 非 C3 GUI 验收）'
    else:
        head = '整链 w6.launch.py backend:=full（双 PX4，%s）' % mode
        tail = ''
    term = expected_terminal or '?'
    if expected_reason:
        term = '%s(%s)' % (term, expected_reason)
    return '%s在 %s 达 %s%s' % (head, scene, term, tail)


def build_coverage(mode, episodes):
    """R-025：coverage 汇总块——逐轴给 value + evidence（出处）。未覆盖轴如实标注。"""
    started = [e for e in episodes if e.get('started')]
    motions = sorted({e.get('target_motion') for e in episodes if e.get('target_motion')})
    terminals = sorted({e.get('terminal_state') for e in episodes if e.get('terminal_state')})
    return {
        'backend': {'value': BACKEND,
                    'evidence': 'episode_runner 起 w6.launch.py backend:=lightweight'
                                '（m0_skeleton+capture_adjudicator+jammer；无 PX4/空中链路）'},
        'gui': {'value': mode, 'evidence': '--mode（headless / gui）'},
        'task_start': {'value': '%d/%d episodes started' % (len(started), len(episodes)),
                       'evidence': '每 episode /agc/task/start Trigger 响应（started + start_message）'},
        'own_takeover': {'value': 'not_applicable',
                         'evidence': '轻量栈不含空中链路：无 takeoff_controller / '
                                     '/agc/air/release_control（本轴 N/A，未覆盖）'},
        'target_motion': {'value': motions,
                          'evidence': 'scenarios.yaml 场景登记轴 axes.target_motion'},
        'real_terminal': {'value': terminals,
                          'evidence': '每 episode terminal_state（/agc/task/state 观测）'},
    }


def episode_outcome_match(terminal_state, reason_code, expected_terminal, expected_reason):
    return (terminal_state == expected_terminal) and (not expected_reason or reason_code == expected_reason)


def classify_episode_outcome(terminal_state, reason_code, expected_terminal, expected_reason,
                             wall_elapsed_s, wall_watchdog_s):
    """R-024：episode 结束判定的**唯一来源**（eval adapter；train adapter 必须复用同一语义）。
    区分三类口径：mission_sim_budget（仿真时钟终态）/ episode_wall_watchdog（墙钟超时，判 infra、
    **不算成功**）/ 进程时长（logger，仅记录、不参与判定）。
    """
    match = episode_outcome_match(terminal_state, reason_code, expected_terminal, expected_reason)
    if terminal_state == 'INFRA_FAILURE':
        return {'outcome_class': 'infra_failure', 'classification': 'infra', 'budget_kind': None,
                'counted_as_success': False, 'expected_match': bool(match),
                'reason': reason_code or 'infra_failure'}
    if terminal_state in SUCCESS_TERMINALS:
        return {'outcome_class': 'mission_terminal', 'classification': 'mission',
                'budget_kind': 'mission_sim_budget', 'counted_as_success': bool(match),
                'expected_match': bool(match), 'reason': reason_code}
    if terminal_state in ('MISSION_FAILED', 'ABORTED'):
        return {'outcome_class': 'mission_terminal', 'classification': 'mission',
                'budget_kind': 'mission_sim_budget', 'counted_as_success': False,
                'expected_match': bool(match), 'reason': reason_code}
    # 未达终态：等待被墙钟看门狗停止（EPISODE_WALL_WATCHDOG 超时）
    return {'outcome_class': 'wall_watchdog_timeout', 'classification': 'infra',
            'budget_kind': 'episode_wall_watchdog', 'counted_as_success': False,
            'expected_match': False, 'reason': 'episode_wall_watchdog exhausted'}


def classify_acceptance_layers(result):
    """R-031：三层验收判定——**三层全必过**方可 `pass`；`expected_match` 仅属命题层。

    ① 启动 / 运行完整性：`started` + 子进程退出确认 + `summary` 存在 + 同 run 关联（summary + state）。
    ② 终态命题：`expected_terminal` / `expected_reason`（= 原 `expected_match`）。
    ③ 指标有效性：C9 两指标为**有效数值**（`not_applicable`/缺失 → 无效），**不**以 0 冒充。
    """
    integrity = bool(
        result.get('started')
        and result.get('proc_exit_confirmed')
        and result.get('summary_present')
        and result.get('same_run_verified')
        and result.get('state_run_verified'))
    proposition = bool(result.get('expected_match'))
    metrics_ok = (isinstance(result.get('shield_infeasible_rate'), (int, float))
                  and isinstance(result.get('reset_failure_rate'), (int, float)))
    return {
        'layer_integrity': integrity,
        'layer_proposition': proposition,
        'layer_metrics': metrics_ok,
        'pass': bool(integrity and proposition and metrics_ok),
    }


def summarize_metrics(episodes):
    """R-031：C9 指标汇总——**不**剔除缺 summary 的回合；如实报缺失计数 / 比例；空集合 → not_applicable。

    均值**仅基于有效样本**；缺失 / `not_applicable` 回合**不**以 0 冒充有效（沿用 R-027 not_applicable 语义）。
    """
    def _one(key):
        vals, missing = [], 0
        for e in episodes:
            v = e.get(key)
            if isinstance(v, (int, float)):
                vals.append(float(v))
            else:
                missing += 1
        return {
            'value': (sum(vals) / len(vals)) if vals else 'not_applicable',
            'valid_count': len(vals),
            'missing_count': missing,
            'missing_ratio': (missing / len(episodes)) if episodes else 0.0,
        }
    return {
        'shield_infeasible_rate': _one('shield_infeasible_rate'),
        'reset_failure_rate': _one('reset_failure_rate'),
    }


def summarize_layers(episodes):
    """R-031：三层验收汇总——逐层通过计数 + 完整性失败场景 / 缺 summary 场景（不含 unsupported）。"""
    def _fail(scene_of):
        return [e['scene'] for e in episodes
                if not scene_of(e) and not e.get('unsupported')]
    return {
        'layer_integrity_count': sum(1 for e in episodes if e.get('layer_integrity')),
        'layer_proposition_count': sum(1 for e in episodes if e.get('layer_proposition')),
        'layer_metrics_count': sum(1 for e in episodes if e.get('layer_metrics')),
        'integrity_failures': _fail(lambda e: e.get('layer_integrity')),
        'summary_missing_episodes': _fail(lambda e: e.get('summary_present')),
    }


def compute_rtf(sim_start_s, sim_end_s, wall_start_s, wall_end_s):
    """RTF = 仿真时间增量 / 墙钟增量；仿真时钟缺席或墙钟增量 <= 0 → None。"""
    if sim_start_s is None or sim_end_s is None:
        return None
    wall = wall_end_s - wall_start_s
    if wall <= 0.0:
        return None
    return (sim_end_s - sim_start_s) / wall


def build_time_evidence(wall_start_s, wall_end_s, sim_start_s, sim_end_s):
    """R-024：同一 run 的起止事件 + RTF 证据（不同 RTF 的成功率 / 耗时可比）。"""
    sim_elapsed = None if (sim_start_s is None or sim_end_s is None) else (sim_end_s - sim_start_s)
    return {
        'wall_start_s': wall_start_s, 'wall_end_s': wall_end_s,
        'wall_elapsed_s': wall_end_s - wall_start_s,
        'sim_start_s': sim_start_s, 'sim_end_s': sim_end_s, 'sim_elapsed_s': sim_elapsed,
        'rtf': compute_rtf(sim_start_s, sim_end_s, wall_start_s, wall_end_s),
    }


def _registry():
    with open(os.path.join(ROOT, 'src/agc_task/config/scenarios.yaml')) as f:
        return yaml.safe_load(f)['scenes']


def _carriers():
    with open(os.path.join(ROOT, 'src/agc_task/config/scenario_carriers.yaml')) as f:
        return yaml.safe_load(f)['axes']


def axis_coverage(scene, registry, carriers):
    """R-026：逐轴覆盖账本——unsupported（无承载者）/ not_carried（轻量栈缺承载）/ covered。

    **如实标注**，绝不以登记命名冒充已执行；轻量栈的运动轴（无 PX4/intruder_driver）与
    感知轴（无空中链路）标 not_carried。
    """
    sc = registry.get(scene, {})
    axes = sc.get('axes') or {}
    support_status = sc.get('support_status', 'supported')
    ledger = {}
    for ax, val in axes.items():
        ent = (carriers.get(ax) or {}).get(val)
        if ent is None or not ent.get('supported', False):
            ledger[ax] = {'value': val, 'status': 'unsupported',
                          'carrier': (ent or {}).get('carrier'),
                          'reason': (ent or {}).get('reason', '无承载者登记')}
        elif ent.get('requires_air_link') and BACKEND == 'lightweight':
            ledger[ax] = {'value': val, 'status': 'not_carried',
                          'carrier': ent.get('carrier'),
                          'reason': '轻量栈无 PX4 / 空中链路（承载者不生效）'}
        else:
            ledger[ax] = {'value': val, 'status': 'covered',
                          'carrier': ent.get('carrier'), 'reason': ''}
    return support_status, ledger


def skipped_result(scene, seed, registry, carriers):
    """R-026：unsupported 场景**拒绝运行**并如实记账（不启动 launch、不产出可用评价）。"""
    sc = registry.get(scene, {})
    support_status, ledger = axis_coverage(scene, registry, carriers)
    return {
        'scene': scene, 'seed': seed,
        'scenario': (sc.get('params') or {}).get('run_logger', {}).get('scenario', scene),
        'expected_terminal': sc.get('expected_terminal'),
        'expected_reason': sc.get('expected_reason'),
        'started': False, 'executed': False, 'unsupported': True,
        'terminal_state': None, 'reason_code': '',
        'support_status': support_status, 'axis_coverage': ledger,
        'outcome_class': 'unsupported', 'classification': 'unsupported',
        'budget_kind': None, 'counted_as_success': False, 'expected_match': False,
        'backend': BACKEND, 'launch': LAUNCH,
        'target_motion': (sc.get('axes') or {}).get('target_motion'),
        # R-031：unsupported 回合不产出可用评价——三层全 false（未运行，无完整性 / 命题 / 指标）
        'layer_integrity': False, 'layer_proposition': False, 'layer_metrics': False,
        'summary_present': False, 'proc_exit_confirmed': False, 'state_run_verified': False,
        'pass': False,
        'proposition': 'unsupported（%s）：无承载者，launch 拒绝运行' % support_status,
        'classification_reason': '声明轴无承载者 → 拒绝运行',
    }


def build_axis_ledger(episodes):
    """R-026：campaign 级覆盖账本——逐轴汇总 status（covered / not_carried / unsupported）+ 场景数。"""
    ledger = {}
    for e in episodes:
        for ax, cell in (e.get('axis_coverage') or {}).items():
            row = ledger.setdefault(ax, {'statuses': {}, 'values': []})
            st = cell.get('status')
            row['statuses'][st] = row['statuses'].get(st, 0) + 1
            if cell.get('value') not in row['values']:
                row['values'].append(cell.get('value'))
    return ledger


class Monitor(Node):
    def __init__(self):
        super().__init__('w6_episode_monitor')
        self.state = None
        self.state_name = ''
        self.reason = ''
        self.sim = None
        self.sim_first = None
        self.terminal_wall = None   # R-031：首个终态观测的墙钟时刻（用于同 episode 关联）
        self.gt_car_rx = None       # R-031：车/目标真值旁路最近接收墙钟（就绪门槛用）
        self.gt_target_rx = None
        self.create_subscription(AgcTaskState, '/agc/task/state', self._on, 20)
        self.create_subscription(Clock, '/clock', self._on_clock, 1)
        self.create_subscription(PoseStamped, GT_CAR_TOPIC, self._on_gt_car, 10)
        self.create_subscription(PoseStamped, GT_TARGET_TOPIC, self._on_gt_target, 10)
        self.cli = self.create_client(Trigger, '/agc/task/start')

    def _on(self, m):
        self.state = m.state
        self.state_name = m.state_name
        self.reason = m.reason_code
        if m.state in TERMINALS and self.terminal_wall is None:
            self.terminal_wall = time.monotonic()

    def _on_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9
        if self.sim_first is None:
            self.sim_first = self.sim

    def _on_gt_car(self, m):
        self.gt_car_rx = time.monotonic()

    def _on_gt_target(self, m):
        self.gt_target_rx = time.monotonic()

    def gt_fresh(self):
        """R-031：车 / 目标真值旁路是否**均在新鲜窗口内**（scenario_reset 复位前置：gt 新鲜）。"""
        t = time.monotonic()
        car_ok = self.gt_car_rx is not None and (t - self.gt_car_rx) < GT_FRESH_S
        tgt_ok = self.gt_target_rx is not None and (t - self.gt_target_rx) < GT_FRESH_S
        return bool(car_ok and tgt_ok)

    def sim_span(self):
        return (self.sim_first, self.sim)

    def start(self, timeout=20.0):
        if not self.cli.wait_for_service(timeout_sec=timeout):
            return False, 'service unavailable'
        fut = self.cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return (fut.result().success, fut.result().message) if fut.done() else (False, 'timeout')

    def wait_terminal(self, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state in TERMINALS:
                return True
        return False


def wait_ready(mon, timeout_s):
    """R-029/R-031：就绪门槛 = `/agc/task/start` 服务就绪 **且** 车/目标真值旁路**新鲜**。

    仅看服务存在不足以复位成功（scenario_reset 复位前置要求 gt 新鲜，年龄 ≤ 0.5 s）；故就绪须等
    **真值数据在流动**，**不**以固定 `sleep` 代就绪（避免 start 早于 gt/桥就绪 → gt_stale → 假 infra）。
    """
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        rclpy.spin_once(mon, timeout_sec=0.1)
        if mon.cli.service_is_ready() and mon.gt_fresh():
            return True
    return mon.cli.service_is_ready() and mon.gt_fresh()


def _group_alive(pgid):
    """R-031：进程组是否仍存活（`killpg(pgid, 0)` 探测；无成员 → ProcessLookupError）。"""
    if pgid is None:
        return False
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _reap_process_group(proc, pgid):
    """R-031：受限进程组回收——SIGINT（优雅）→ wait → SIGKILL（清理游离子进程）→ 轮询确认整组退出。

    仅回收**本 run 的进程组**（Popen `start_new_session=True`；`pgid` 在 Popen 后即取，防 leader 先死
    无法按 pid 反查组）。**不**匹配裸名、**不**杀其它会话。`ros2 launch` 可能先退出而让子进程（如 gz sim）
    **忽略 SIGINT 存活**，故**无论 launch 是否已退**都补发组 SIGKILL，并轮询至整组回收（避免跨回合残留
    相互干扰）。返回 `(exit_confirmed, returncode)`。
    """
    def _sig(sig):
        try:
            os.killpg(pgid, sig)
        except (ProcessLookupError, TypeError):
            pass

    _sig(signal.SIGINT)
    try:
        proc.wait(timeout=REAP_SIGINT_S)
    except subprocess.TimeoutExpired:
        pass
    _sig(signal.SIGKILL)   # 清理可能忽略 SIGINT 的游离子进程（launch 可能已退出）
    try:
        proc.wait(timeout=REAP_SIGKILL_S)
    except subprocess.TimeoutExpired:
        pass
    deadline = time.monotonic() + REAP_SIGKILL_S
    while _group_alive(pgid) and time.monotonic() < deadline:
        time.sleep(0.2)
    return (not _group_alive(pgid)), proc.returncode

def run_episode(scene, seed, registry, mode, wall_watchdog_s, out_dir, carriers, ready_timeout_s):
    scenario = registry[scene]['params'].get('run_logger', {}).get('scenario', scene)
    world = os.path.join(ROOT, 'install/agc_sim/share/agc_sim/worlds/agc_world.sdf')
    gz = ('-r -v 3 ' if mode == 'gui' else '-s -r -v 3 ') + world
    log = os.path.join(out_dir, 'log', 'ep_%s_%d.log' % (scene, seed))
    os.makedirs(os.path.dirname(log), exist_ok=True)
    # 先建 Monitor、再记墙钟起点、再 Popen：从 episode 一开始就记录 /clock
    mon = Monitor()
    wall_start_s = time.monotonic()
    proc = subprocess.Popen(
        ['ros2', 'launch', 'agc_task', 'w6.launch.py',
         'backend:=lightweight', 'scene:=%s' % scene, 'seed:=%d' % seed, 'gz_args:=%s' % gz],
        stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    try:
        pgid = os.getpgid(proc.pid)   # R-031：Popen 后即取组 id（防 leader 先死无法反查组）
    except ProcessLookupError:
        pgid = None

    result = {'scene': scene, 'seed': seed, 'scenario': scenario,
              'expected_terminal': registry[scene].get('expected_terminal'),
              'expected_reason': registry[scene].get('expected_reason')}
    try:
        ready = wait_ready(mon, ready_timeout_s)
        result['ready'] = ready
        # 有界稳定窗口（收集 /clock 起点），非固定 18 s 就绪依赖
        t_settle = time.monotonic() + 2.0
        while time.monotonic() < t_settle:
            rclpy.spin_once(mon, timeout_sec=0.1)
        t_start_wall = time.monotonic()   # R-031：start 调用时刻（用于 state 同 episode 关联）
        ok, msg = mon.start()
        result['started'] = bool(ok)
        result['start_message'] = msg
        remaining = max(wall_watchdog_s - (time.monotonic() - wall_start_s), 1.0)
        reached = mon.wait_terminal(remaining)
        wall_end_s = time.monotonic()
        result['terminal_state'] = mon.state_name if reached else 'ACTIVE'
        result['reason_code'] = mon.reason
        result['terminal_reached'] = reached
    finally:
        if mon is not None:
            mon.destroy_node()
        # R-031：受限进程组回收（SIGINT 优雅 → wait → SIGKILL → wait），各步确认退出。
        # 仅回收**本 run 的进程组**（start_new_session），不匹配裸名 / 不杀其它会话 / 不用 rm -rf。
        result['proc_exit_confirmed'], result['proc_returncode'] = _reap_process_group(proc, pgid)

    # R-029/R-031：以唯一 run_id / 创建时间校验同一 run 关联（suffix 容忍，非固定路径猜测）
    pat = os.path.join(ROOT, 'experiments', 'w6-task-eval',
                       '*-%s-%d*/metrics/summary.json' % (scenario, seed))
    cands = [p for p in glob.glob(pat) if os.path.getmtime(p) >= wall_start_s - 1.0]
    result['summary_present'] = False
    if cands:
        pat = max(cands, key=os.path.getmtime)
        with open(pat) as f:
            sm = json.load(f)
        for k in ('run_id', 'episode_id', 'shield_infeasible_rate', 'reset_failure_rate',
                  'reset_retries', 'reset_failed', 'duration_s', 'odom_valid_ratio',
                  'cmd_vel_rate_hz', 'git_commit'):
            result[k] = sm.get(k)
        result['summary_path'] = pat
        result['summary_present'] = True
        # 同一 run 关联校验：run_id 含 `-<scenario>-<seed>-` 且创建时间落在本 episode 窗口内
        result['same_run_verified'] = bool(
            result.get('run_id') and ('-%s-%d-' % (scenario, seed)) in result['run_id'])
    else:
        result['summary_missing'] = pat
        result['same_run_verified'] = False

    # R-031：`/agc/task/state` 同 episode 关联——终态须在**本次 start 之后**观测（越窗终态=旧回合污染）。
    if result.get('terminal_reached'):
        result['state_run_verified'] = bool(
            mon.terminal_wall is not None and mon.terminal_wall >= t_start_wall)
    else:
        result['state_run_verified'] = True   # 未达终态：无越窗终态可判

    # R-024：合并时间证据（起止事件 + RTF）与看门狗预算
    result.update(build_time_evidence(wall_start_s, wall_end_s, *mon.sim_span()))
    result['wall_watchdog_s'] = wall_watchdog_s
    # R-024：episode 结束判定唯一来源
    exp_t, exp_r = result['expected_terminal'], result['expected_reason']
    result.update(classify_episode_outcome(
        result['terminal_state'], result['reason_code'], exp_t, exp_r,
        result['wall_elapsed_s'], wall_watchdog_s))
    # R-025：backend/launch 标注 + 明确验收命题
    result['backend'] = BACKEND
    result['launch'] = LAUNCH
    result['gui'] = mode
    result['target_motion'] = (registry[scene].get('axes') or {}).get('target_motion')
    # R-026：逐轴覆盖账本（如实标注 not_carried / unsupported / covered）
    result['support_status'], result['axis_coverage'] = axis_coverage(
        scene, registry, carriers)
    result['proposition'] = build_proposition(BACKEND, mode, scene, exp_t, exp_r)
    # R-031：三层验收判定（完整性 + 命题 + 指标有效性全必过）
    result.update(classify_acceptance_layers(result))
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenes', default=None, help='逗号分隔场景 id；缺省配合 --all')
    ap.add_argument('--all', action='store_true', help='跑登记表全部 12 场景')
    ap.add_argument('--seeds', default='0', help='逗号分隔 seed')
    ap.add_argument('--mode', default='headless', choices=['headless', 'gui'])
    ap.add_argument('--timeout', type=float, default=150.0, help='单回合等待终态上限（秒）')
    ap.add_argument('--wall-watchdog-s', type=float, default=None,
                    help='episode 墙钟看门狗预算（秒）；缺省回落到 --timeout')
    ap.add_argument('--ready-timeout-s', type=float, default=60.0,
                    help='R-029：等待 /agc/task/start 服务就绪的上限（秒）')
    ap.add_argument('--campaign', default='w6-task-eval')
    ap.add_argument('--run-id', default=None)
    args = ap.parse_args()

    wall_watchdog_s = args.wall_watchdog_s if args.wall_watchdog_s is not None else args.timeout

    registry = _registry()
    carriers = _carriers()
    if args.all:
        scenes = list(registry.keys())
    elif args.scenes:
        scenes = [s.strip() for s in args.scenes.split(',') if s.strip()]
    else:
        ap.error('需 --scenes 或 --all')
    seeds = [int(s) for s in args.seeds.split(',')]

    run_id = args.run_id or ('%s-%s-%s-%04d' % (
        datetime.now().strftime('%Y%m%d'), args.campaign,
        datetime.now().strftime('%H%M%S'), random.randint(0, 9999)))
    out_dir = os.path.join(ROOT, 'experiments', args.campaign, run_id)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)

    rclpy.init()
    episodes = []
    for scene in scenes:
        for seed in seeds:
            # R-026：unsupported 场景**拒绝运行**并如实记账（不启动 launch）
            support_status = registry.get(scene, {}).get('support_status', 'supported')
            if str(support_status).startswith('unsupported'):
                print('=== episode: %s seed=%d [SKIP unsupported=%s] ===' % (
                    scene, seed, support_status))
                episodes.append(skipped_result(scene, seed, registry, carriers))
                continue
            print('=== episode: %s seed=%d ===' % (scene, seed))
            r = run_episode(scene, seed, registry, args.mode, wall_watchdog_s, out_dir, carriers,
                            ready_timeout_s=args.ready_timeout_s)
            print('  terminal=%s reason=%s expected_match=%s pass=%s integrity=%s outcome_class=%s shield=%s reset=%s'
                  % (r.get('terminal_state'), r.get('reason_code'), r.get('expected_match'),
                     r.get('pass'), r.get('layer_integrity'), r.get('outcome_class'),
                     _num_str(r.get('shield_infeasible_rate')),
                     _num_str(r.get('reset_failure_rate'))))
            episodes.append(r)
    rclpy.shutdown()

    # R-031：指标汇总（均值仅基于有效样本；缺失 / not_applicable 如实标注）+ 三层验收汇总。
    metric_completeness = summarize_metrics(episodes)
    layer_summary = summarize_layers(episodes)
    rtfs = [e['rtf'] for e in episodes if e.get('rtf') is not None]
    report = {
        'run_id': run_id, 'campaign': args.campaign, 'mode': args.mode,
        # R-025：后端 / launch 标注（本 runner 恒为轻量后端；不冒充整链 / GUI 验收）
        'backend': BACKEND, 'launch': LAUNCH,
        'git_commit': next((e.get('git_commit') for e in episodes if e.get('git_commit')), 'unknown'),
        'n_episodes': len(episodes),
        # R-031：C9 两指标（均值仅基于**有效**样本；缺失 / not_applicable 如实标注，不以 0 冒充）
        'shield_infeasible_rate': metric_completeness['shield_infeasible_rate']['value'],
        'reset_failure_rate': metric_completeness['reset_failure_rate']['value'],
        'metric_completeness': metric_completeness,
        'expected_match_count': sum(1 for e in episodes if e.get('expected_match')),
        'pass_count': sum(1 for e in episodes if e.get('pass')),
        # R-031：三层验收汇总（任一未过均如实列出，便于定位假绿来源）
        **layer_summary,
        # R-026：覆盖账本（逐轴 status 汇总）+ unsupported / not_carried 如实计数
        'axis_coverage_ledger': build_axis_ledger(episodes),
        'unsupported_episodes': [e['scene'] for e in episodes if e.get('unsupported')],
        'not_carried_axes': sorted({ax for e in episodes
                                    for ax, c in (e.get('axis_coverage') or {}).items()
                                    if c.get('status') == 'not_carried'}),
        'time_budgets': TIME_BUDGETS,
        # R-025：coverage 汇总（逐轴 value + evidence；未覆盖轴如实标注）
        'coverage': build_coverage(args.mode, episodes),
        'full_chain_note': FULL_CHAIN_NOTE,
        'rtf_mean': (sum(rtfs) / len(rtfs)) if rtfs else None,
        'wall_watchdog_s': wall_watchdog_s,
        'episodes': episodes,
        # S-07（ADR-0063）：campaign 报告**引用**每个 episode run 的不可变产物（run_id + config/）。
        'run_references': [
            {'scene': e['scene'], 'seed': e['seed'], 'run_id': e.get('run_id'),
             'run_dir': os.path.join('experiments', 'w6-task-eval', str(e.get('run_id'))),
             'scene_manifest': 'config/scene_manifest.json',
             'params_applied': 'config/params_applied.yaml'}
            for e in episodes if e.get('run_id')
        ],
        # C9 口径标注（R7/8.4）：与旧义（RL 安全集为空）**不可直接对照**
        'metric_notes': {
            'shield_infeasible_rate':
                '重定义：安全层触发(STOP/SLOWDOWN)且触发时不可行(前向扫描≤contact_dist 或 TTC<ttc_min)'
                ' 的事件占比；与旧义(RL 安全集为空)不可直接对照。',
            'reset_failure_rate': '复位重试耗尽(>3)或复位服务失败的 episode 占比（本 runner 汇总）。',
        },
    }
    with open(os.path.join(out_dir, 'metrics', 'report.json'), 'w') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    md = ['# W6 campaign 报告 · %s' % run_id,
          '', '- git: `%s`' % report['git_commit'], '- 回合数: %d' % report['n_episodes'],
          '- backend: `%s` · launch: `%s` · mode: `%s`' % (
              report['backend'], report['launch'], report['mode']),
          '- 期望终态命中: %d/%d' % (report['expected_match_count'], report['n_episodes']),
          '',
          '> **%s**' % report['full_chain_note'],
          '', '## 覆盖矩阵（本报告可见轴；逐轴 value + evidence）']
    for axis, info in report['coverage'].items():
        md.append('- `%s` = %s — 证据: %s' % (axis, info.get('value'), info.get('evidence')))
    md += ['', '## 轴覆盖账本（R-026：unsupported / not_carried 如实标注）',
           '| axis | values | statuses |', '|---|---|---|']
    for axis, info in report['axis_coverage_ledger'].items():
        md.append('| %s | %s | %s |' % (
            axis, ','.join(str(v) for v in info.get('values', [])),
            ', '.join('%s=%d' % (k, v) for k, v in (info.get('statuses') or {}).items())))
    md += ['', '- `unsupported` 回合: %s（launch 拒绝运行）' % (report['unsupported_episodes'] or '无'),
           '- `not_carried` 轴: %s（轻量栈缺承载，未执行）' % (report['not_carried_axes'] or '无')]

    def _fmt_metric(summary):
        v = summary['value']
        vs = ('%.4f' % v) if isinstance(v, (int, float)) else '**not_applicable**'
        return '%s（valid=%d / missing=%d，缺失率=%.2f）' % (
            vs, summary['valid_count'], summary['missing_count'], summary['missing_ratio'])

    md += ['', '## R-031 三层验收（完整性 + 终态命题 + 指标有效性；三层全必过）',
           '- 完整性通过: %d/%d · 命题通过: %d/%d · 指标有效: %d/%d' % (
               report['layer_integrity_count'], report['n_episodes'],
               report['layer_proposition_count'], report['n_episodes'],
               report['layer_metrics_count'], report['n_episodes']),
           '- 完整性未过（缺 summary / 未退出 / 无同 run 关联）: %s' % (
               report['integrity_failures'] or '无'),
           '- 缺 summary 回合: %s（缺失计入分母，**不**剔除）' % (
               report['summary_missing_episodes'] or '无'),
           '', '## C9 指标（口径标注；均值仅基于有效样本）',
           '- `shield_infeasible_rate`: %s — %s' % (
               _fmt_metric(report['metric_completeness']['shield_infeasible_rate']),
               report['metric_notes']['shield_infeasible_rate']),
           '- `reset_failure_rate`: %s — %s' % (
               _fmt_metric(report['metric_completeness']['reset_failure_rate']),
               report['metric_notes']['reset_failure_rate']),
           '', '## 回合明细（pass=三层全必过；见「命题」列）',
           '| scene | seed | terminal | reason | integrity | proposition | metrics | pass | outcome_class | 命题 |',
           '|---|---|---|---|---|---|---|---|---|---|']
    for e in episodes:
        md.append('| %s | %d | %s | %s | %s | %s | %s | %s | %s | %s |' % (
            e['scene'], e['seed'], e.get('terminal_state'), e.get('reason_code'),
            e.get('layer_integrity'), e.get('layer_proposition'), e.get('layer_metrics'),
            e.get('pass'), e.get('outcome_class', 'n/a'), e.get('proposition', 'n/a')))
    with open(os.path.join(out_dir, 'metrics', 'report.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')

    print('\nreport → %s' % os.path.join(out_dir, 'metrics', 'report.json'))
    print('report → %s' % os.path.join(out_dir, 'metrics', 'report.md'))
    # R-031：退出码以**三层全必过**的 pass_count 为准（不再只看期望终态字符串匹配）。
    return 0 if (report['n_episodes'] > 0 and report['pass_count'] == report['n_episodes']) else 1


if __name__ == '__main__':
    sys.exit(main())
