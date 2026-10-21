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
from geometry_msgs.msg import PoseStamped, TwistStamped

from agc_msgs.msg import AgcTaskState
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

# W7-a（R1）：episode 结束判定 / 预算口径的**单一来源** = `agc_rl.termination`
# （训练 adapter 与评估 adapter 共用同一判定，spec R1 判据：摘要与 adapter end/reason 一致）。
from agc_rl.termination import (  # noqa: E402
    TERMINALS,
    TIME_BUDGETS,
    classify_episode_outcome,
    classify_acceptance_layers,
    summarize_metrics,
    build_time_evidence,
)

ROOT = '/home/chen/AGC'

# R-031：就绪门槛用到的真值旁路话题（与 scenario_reset 默认一致）与新鲜窗口。
GT_CAR_TOPIC = '/agc/gt/agc_car'
GT_TARGET_TOPIC = '/agc/gt/intruder_uav'
GT_FRESH_S = 1.0

# R-013 / W7-b 取证：地面里程计有效位——task_manager 的 start 门**要求** `odom_valid==true`；
# full 链下它由 ground_odom_adapter 晚到（实测曾被 task_manager 以 `odom_invalid` 拒 start）。
ODOM_VALID_TOPIC = '/agc/ground/odom_valid'

# W7-a（R5）：命令链回环门槛——`/agc/ground/cmd_vel`（唯一执行口输出）新鲜即命令链已活。
CMD_TOPIC = '/agc/ground/cmd_vel'
CMD_FRESH_S = 1.0

# R-031：受限进程组回收的时间预算（SIGINT 优雅 → SIGKILL 强杀，各步 wait 确认退出）。
REAP_SIGINT_S = 8.0
REAP_SIGKILL_S = 4.0


def _num_str(v):
    """R-031：数值 / not_applicable（None 或字符串）统一格式化，避免 %.3f 对 None 抛异常。"""
    return ('%.3f' % v) if isinstance(v, (int, float)) else 'n/a'

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


def _registry():
    with open(os.path.join(ROOT, 'src/agc_task/config/scenarios.yaml')) as f:
        return yaml.safe_load(f)['scenes']


def _carriers():
    with open(os.path.join(ROOT, 'src/agc_task/config/scenario_carriers.yaml')) as f:
        return yaml.safe_load(f)['axes']


def axis_coverage(scene, registry, carriers, backend=BACKEND):
    """R-026：逐轴覆盖账本——unsupported（无承载者）/ not_carried（轻量栈缺承载）/ covered。

    **如实标注**，绝不以登记命名冒充已执行；轻量栈的运动轴（无 PX4/intruder_driver）与
    感知轴（无空中链路）标 not_carried。
    ⚠ 修正（2026-10-20 自查）：`backend` 原为模块常量（永远按轻量栈判）→ 整链（`full`）episode 的
    空中依赖轴被误标 `not_carried`。改为**按实际 backend 判定**（缺省保持既有轻量栈行为）。
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
        elif ent.get('requires_air_link') and backend == 'lightweight':
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
        self.cmd_rx = None          # W7-a：/agc/ground/cmd_vel 回环最近接收墙钟
        self.odom_valid = None      # W7-b：/agc/ground/odom_valid 最近值（None=未收到）
        self.odom_valid_rx = None
        self.sim_at_start = None    # W7-b（ADR-0074）：start 调用时刻的仿真钟（固定观测时长基准）
        self.create_subscription(AgcTaskState, '/agc/task/state', self._on, 20)
        self.create_subscription(Clock, '/clock', self._on_clock, 1)
        self.create_subscription(PoseStamped, GT_CAR_TOPIC, self._on_gt_car, 10)
        self.create_subscription(PoseStamped, GT_TARGET_TOPIC, self._on_gt_target, 10)
        self.create_subscription(TwistStamped, CMD_TOPIC, self._on_cmd, 20)
        self.create_subscription(Bool, ODOM_VALID_TOPIC, self._on_odom_valid, 10)
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

    def _on_cmd(self, m):
        self.cmd_rx = time.monotonic()

    def _on_odom_valid(self, m):
        self.odom_valid = bool(m.data)
        self.odom_valid_rx = time.monotonic()

    def odom_valid_ok(self):
        """W7-b：`/agc/ground/odom_valid` 已收到且为 true（task_manager 的 start 门要求项）。"""
        return bool(self.odom_valid) and self.odom_valid_rx is not None and \
            (time.monotonic() - self.odom_valid_rx) < GT_FRESH_S

    def gt_fresh(self):
        """R-031：车 / 目标真值旁路是否**均在新鲜窗口内**（scenario_reset 复位前置：gt 新鲜）。"""
        t = time.monotonic()
        car_ok = self.gt_car_rx is not None and (t - self.gt_car_rx) < GT_FRESH_S
        tgt_ok = self.gt_target_rx is not None and (t - self.gt_target_rx) < GT_FRESH_S
        return bool(car_ok and tgt_ok)

    def cmd_fresh(self):
        """W7-a（R5）：`/agc/ground/cmd_vel` 回环是否在新鲜窗口内（命令链已活）。"""
        return self.cmd_rx is not None and (time.monotonic() - self.cmd_rx) < CMD_FRESH_S

    def sim_span(self):
        return (self.sim_first, self.sim)

    def start(self, timeout=20.0):
        if not self.cli.wait_for_service(timeout_sec=timeout):
            return False, 'service unavailable'
        fut = self.cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        self.sim_at_start = self.sim   # ADR-0074：固定观测时长基准（start 时刻的仿真钟）
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
        if mon.cli.service_is_ready() and mon.gt_fresh() and mon.cmd_fresh() and mon.odom_valid_ok():
            return True
    return mon.cli.service_is_ready() and mon.gt_fresh() and mon.cmd_fresh() and mon.odom_valid_ok()


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


# ADR-0075（2026-10-20）：**运行前清场**——防"上次 run 的残留 / 逃逸孤儿"污染本 run。
#
# 实测链条：上次 run 被**外部终止**（网络中断 / toolhost 只杀到 leader）时，launch 的子进程
# （`ros_gz_bridge` 的 `parameter_bridge`、`tf2_ros` 的 `static_transform_publisher`，
# 均为 `/opt/ros/jazzy/lib/...`，**不**含 `install/agc_` 字样）会逃逸成孤儿（被收割者收养），
# 继续持有 Fast DDS 参与者与 `/dev/shm` 段 → 后续 run 的 uXRCE agent 侧 participant 创建从
# **0.67 s 恶化到 1.5–2.7 s**，超过 PX4 客户端硬编码 1 s 预算 → `create entities failed:
# participant: 255` → 无 `/fmu/out/*` writer → 整链 infra（实测一条命令连试 4 次全失败；
# 清场后同一命令 **0.67 s** 成功）。
#
# 纪律（对齐 R-031）：只匹配**本项目**特征（`install/agc_*` / agent / PX4 / `agc_world`·`agc_car`
# 的桥与 tf / `agc_world.sdf` 的 gz），**不**按裸名匹配、**不**碰非 AGC 进程；在 Popen **之前**调用
# → 本 run 进程尚未创建，故命中者必为上次 run 的残留。
#
# 匹配口径（2026-10-20 诱饵测试修正）：以 **`argv[0]`（被调起的程序路径）** 为准，**不**扫整条命令行
# 子串——否则任何"命令行里提到这些字符串"的进程（如调试用的父 shell `bash -c '... agc_decoy ...'`、
# 或 `ps | grep MicroXRCEAgent` 的管道）都会被误杀（诱饵测试实测命中父 shell）。守卫（`agc_world` /
# `agc_car` / `agc_world.sdf`）仍取自整条命令行（它们是**参数**，且足够特异）。
_SHELL_EXES = ('/bin/bash', '/bin/sh', '/bin/dash', '/usr/bin/bash', '/usr/bin/sh')
STALE_PROCESS_RULES = (
    # (argv[0] 必须包含, 附加守卫：整条命令行需同时含其一；空元组 = 无守卫)
    ('/install/agc_', ()),
    ('MicroXRCEAgent', ()),
    ('px4_sitl_default/bin/px4', ()),
    ('ros_gz_bridge/parameter_bridge', ('agc_world',)),
    ('tf2_ros/static_transform_publisher', ('agc_car', 'car/')),
    ('gz sim', ('agc_world.sdf',)),   # gz sim 的 argv[0] 是 ruby，故本规则按整条命令行匹配
)


def sweep_stale_processes(log_path=None):
    """ADR-0075：运行前清场。返回被 SIGKILL 的 `[(pid, cmdline), ...]`（并追加到 run 日志作审计）。"""
    me = os.getpid()
    killed = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) == me:
            continue
        try:
            with open('/proc/%s/cmdline' % entry, 'rb') as f:
                raw = f.read().decode('utf-8', 'replace').split('\x00')
        except (OSError, IOError):
            continue
        argv0 = raw[0].strip() if raw else ''
        cmd = ' '.join(x for x in raw if x).strip()
        if not argv0:
            continue
        try:
            exe = os.readlink('/proc/%s/exe' % entry)
        except OSError:
            exe = ''
        # 规则：argv[0] 命中（`gz sim` 例外——其 argv[0] 为 ruby，改按命令行匹配且**排除 shell**）
        hit = False
        for pat, guard in STALE_PROCESS_RULES:
            if guard and pat == 'gz sim':
                # ruby 脚本 `gz sim ...`：命令行需含 `gz sim` 与守卫，且本体不是 shell
                if 'gz sim' in cmd and any(g in cmd for g in guard) and \
                        exe not in _SHELL_EXES and argv0 not in _SHELL_EXES:
                    hit = True
                    break
                continue
            if pat in argv0 or pat in exe:
                if not guard or any(g in cmd for g in guard):
                    hit = True
                    break
        if not hit:
            continue
        try:
            os.kill(int(entry), signal.SIGKILL)
        except OSError:
            continue
        killed.append((int(entry), cmd))
    if killed and log_path:
        with open(log_path, 'a') as f:
            f.write('=== ADR-0075 运行前清场：清理残留进程 %d 个 ===\n' % len(killed))
            for pid, cmd in killed:
                f.write('  - pid %d: %s\n' % (pid, cmd))
    return killed


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

def run_episode(scene, seed, registry, mode, wall_watchdog_s, out_dir, carriers, ready_timeout_s,
                command_source='rl', shield='true', backend='lightweight', target_source='',
                observe_yaw_mode='', air_tracking_ref='', log_tag='', obs_horizon_s=0.0,
                start_timeout_s=20.0):
    scenario = registry[scene]['params'].get('run_logger', {}).get('scenario', scene)
    world = os.path.join(ROOT, 'install/agc_sim/share/agc_sim/worlds/agc_world.sdf')
    gz = ('-r -v 3 ' if mode == 'gui' else '-s -r -v 3 ') + world
    # W7-b 臂配对：同一 (scene, seed) 会被**多个臂**各跑一次 → 日志名须带臂标识（否则后跑臂覆盖先跑臂，
    # 丢失同 run 证据）；缺省空 = 既有单臂调用不变。
    log = os.path.join(out_dir, 'log', 'ep_%s%s_%d.log' % (
        (log_tag + '_') if log_tag else '', scene, seed))
    os.makedirs(os.path.dirname(log), exist_ok=True)
    # ADR-0075：**运行前清场**（本 run 尚未创建任何进程 → 命中者必为上次 run 的残留 / 逃逸孤儿；
    # 实测污染会使 agent 侧 participant 创建 > PX4 的 1 s 预算 → 全链 infra）。
    stale_swept = sweep_stale_processes(log)
    # 先建 Monitor、再记墙钟起点、再 Popen：从 episode 一开始就记录 /clock
    mon = Monitor()
    wall_start_s = time.monotonic()
    # R-031 修正：`wall_start_s` 为 **monotonic**（用于时长），与 `os.path.getmtime`（epoch）**不可比**；
    # 另记 epoch 起点专供「本 run 的 summary 归属」过滤，避免静默挂到同场景同 seed 的**旧** summary。
    wall_start_unix_s = time.time()
    launch_args = ['ros2', 'launch', 'agc_task', 'w6.launch.py',
                   'backend:=%s' % backend, 'scene:=%s' % scene, 'seed:=%d' % seed,
                   'gz_args:=%s' % gz, 'command_source:=%s' % command_source,
                   'shield:=%s' % shield]
    # W7-b（P1）：观测来源覆盖（仅显式给出时透传，保持既有 W6 调用不变）
    if target_source:
        launch_args.append('target_source:=%s' % target_source)
    # W7-b（P1·甲）：空中机观测偏航模式覆盖（同上：仅显式给出时透传）
    if observe_yaw_mode:
        launch_args.append('observe_yaw_mode:=%s' % observe_yaw_mode)
    # W7-b（P1 公平性修正）：空中机是否使用目标跟踪参考（''=不覆盖；off=固定航迹臂）
    if air_tracking_ref:
        launch_args.append('air_tracking_ref:=%s' % air_tracking_ref)
    # 整链 / 轻量栈都经 world_bringup 追加 `GZ_SIM_SYSTEM_PLUGIN_PATH`（该 launch 以
    # `EnvironmentVariable` 读取，**未导出即抛异常**）；此处按 archive 脚本先例显式导出，
    # 使 run 不依赖调用方 shell 环境（本机 gz 插件路径前提）。
    launch_env = dict(os.environ)
    plugin_dir = os.path.join(ROOT, 'install', 'agc_sim', 'lib')
    launch_env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = plugin_dir + (
        ':' + launch_env['GZ_SIM_SYSTEM_PLUGIN_PATH']
        if launch_env.get('GZ_SIM_SYSTEM_PLUGIN_PATH') else '')
    proc = subprocess.Popen(
        launch_args, env=launch_env,
        stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    try:
        pgid = os.getpgid(proc.pid)   # R-031：Popen 后即取组 id（防 leader 先死无法反查组）
    except ProcessLookupError:
        pgid = None

    result = {'scene': scene, 'seed': seed, 'scenario': scenario,
              'command_source': command_source, 'shield': shield,
              'backend': backend, 'target_source': target_source or None,
              'observe_yaw_mode': observe_yaw_mode or None,
              'expected_terminal': registry[scene].get('expected_terminal'),
              'expected_reason': registry[scene].get('expected_reason'),
              # ADR-0075：运行前清场证据（本次清掉了哪些残留 / 孤儿；空 = 环境本就干净）
              'stale_swept_pids': [p for p, _ in stale_swept],
              'stale_swept_cmds': [c for _, c in stale_swept]}
    try:
        ready = wait_ready(mon, ready_timeout_s)
        result['ready'] = ready
        # 有界稳定窗口（收集 /clock 起点），非固定 18 s 就绪依赖
        t_settle = time.monotonic() + 2.0
        while time.monotonic() < t_settle:
            rclpy.spin_once(mon, timeout_sec=0.1)
        t_start_wall = time.monotonic()   # R-031：start 调用时刻（用于 state 同 episode 关联）
        ok, msg = mon.start(timeout=start_timeout_s)
        result['started'] = bool(ok)
        result['start_message'] = msg
        remaining = max(wall_watchdog_s - (time.monotonic() - wall_start_s), 1.0)
        reached = mon.wait_terminal(remaining)
        # ADR-0074：终态后**不立即拆除**——继续采到**固定观测时长**（自 start 起、按仿真钟），使
        # `visibility_metrics`（window_fixed_v1）的分母不随"早成功"截断（消除饱和 / 选择效应）。
        # 上限仍受墙钟看门狗约束（仿真不推进时如实早停）。
        if obs_horizon_s > 0 and mon.sim_at_start is not None:
            t_lim = time.monotonic() + max(
                wall_watchdog_s - (time.monotonic() - wall_start_s), 1.0)
            while time.monotonic() < t_lim:
                rclpy.spin_once(mon, timeout_sec=0.2)
                if mon.sim is None or (mon.sim - mon.sim_at_start) >= obs_horizon_s:
                    break
            result['obs_horizon_s'] = obs_horizon_s
            result['obs_horizon_reached'] = bool(
                mon.sim is not None and (mon.sim - mon.sim_at_start) >= obs_horizon_s)
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
    cands = [p for p in glob.glob(pat) if os.path.getmtime(p) >= wall_start_unix_s - 1.0]
    result['summary_present'] = False
    if cands:
        pat = max(cands, key=os.path.getmtime)
        with open(pat) as f:
            sm = json.load(f)
        for k in ('run_id', 'episode_id', 'shield_infeasible_rate', 'reset_failure_rate',
                  'reset_retries', 'reset_failed', 'duration_s', 'odom_valid_ratio',
                  'cmd_vel_rate_hz', 'git_commit', 'command_attribution'):
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
    #   ⚠ 修正（2026-10-20 自查）：原实现硬写常量 `BACKEND`（= 'lightweight'）→ 经本函数跑的
    #   **full 链** episode 其 `backend` 与验收命题被标成"轻量栈"（证据标注错误；不影响判定门，
    #   但会把 full 证据误标为非整链）。改用**调用参数** `backend`。
    result['backend'] = backend
    result['launch'] = LAUNCH
    result['gui'] = mode
    result['target_motion'] = (registry[scene].get('axes') or {}).get('target_motion')
    # R-026：逐轴覆盖账本（如实标注 not_carried / unsupported / covered）
    result['support_status'], result['axis_coverage'] = axis_coverage(
        scene, registry, carriers, backend)
    result['proposition'] = build_proposition(backend, mode, scene, exp_t, exp_r)
    # R-031：三层验收判定（完整性 + 命题 + 指标有效性全必过）
    result.update(classify_acceptance_layers(result))
    return result


# W7-b（2026-10-20，ADR-0075）—— **infra 重试协议**（机器噪声，非实验处理）：
#   本机 uXRCE-DDS agent 侧创建 participant 的时延**间歇**达 1.7–2.7 s（良好时 0.67–0.73 s），
#   超过 PX4 客户端**硬编码**的 1000 ms 预算（`uxrce_dds_client.cpp` 的
#   `uxr_run_session_until_all_status(&session, 1000, ...)`）→ PX4 报 `create entities failed:
#   participant: 255` 并 `return`（**不**重试）→ 无 `rt/fmu/out/*` writer → 无 `/agc/air/odom`
#   → 不起飞 → `air_hover_timeout`（INFRA_FAILURE）。
#   判据**唯一来源** = `agc_rl.termination.classify_episode_outcome` 的 `classification`：
#   * `classification == 'infra'`（含 `INFRA_FAILURE` 与未达终态的墙钟截断）→ **重试**该 (arm, seed)；
#   * `classification == 'mission'`（含 `MISSION_FAILED(TIMEOUT)`，契约 C4 超时属任务终态）→ **一律不重试**
#     （不得以重试掩盖 mission 结果）。
#   证据纪律：每次尝试**各留产物**——首试沿用原 `log_tag`，第 k 次重试追加 `.r<k>` 后缀（日志不覆盖）；
#   逐次 run_id / 终态 / 原因记入 `infra_attempts`（审计）；上报结果为**最后一次尝试**。
def run_episode_with_infra_retry(*args, infra_retries=0, **kwargs):
    """`run_episode` 的 infra 重试包装（协议见上）。`infra_retries=0` = 关闭（既有行为不变）。"""
    base_tag = kwargs.get('log_tag', '')
    attempts = []
    for k in range(int(infra_retries) + 1):
        kwargs['log_tag'] = base_tag if k == 0 else '%s.r%d' % (base_tag or 'ep', k)
        r = run_episode(*args, **kwargs)
        attempts.append({
            'attempt': k, 'log_tag': kwargs['log_tag'],
            'terminal_state': r.get('terminal_state'), 'reason_code': r.get('reason_code'),
            'classification': r.get('classification'), 'run_id': r.get('run_id'),
            'summary_path': r.get('summary_path'), 'wall_elapsed_s': r.get('wall_elapsed_s'),
        })
        if r.get('classification') != 'infra' or k == int(infra_retries):
            break
    r['infra_attempts'] = attempts
    r['infra_retries_used'] = len(attempts) - 1
    r['infra_retry_protocol'] = 'ADR-0075（infra 重试上限 %d）' % int(infra_retries)
    return r


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
    # W7-a（R5）：命令源单一选择——`rl`（RL 主控）默认 / `baseline`（Navfn + 自研 LQR 离线对照基线）。
    # 由 w6.launch.py 透传至 ground_bringup；缺省 `rl` → 既有 W6 证据行为不变。
    # W7-b（P2 臂 B）：`rl_high` = 高层（拦截点）+ 成熟低层（Navfn + 自研 LQR）。
    ap.add_argument('--command-source', default='rl', choices=['rl', 'baseline', 'rl_high'])
    # W7-b（T1/E1，ADR-0070(d)）：安全层开关——`true`（默认，起 collision_monitor）/
    # `false`（无 shield 对照臂：恒等透传，纯仿真，**不**用于部署）。
    ap.add_argument('--shield', default='true', choices=['true', 'false'])
    # W7-b（P1 / T6）：backend 与观测来源覆盖（缺省不变：lightweight / backend 能力表值）。
    ap.add_argument('--backend', default='lightweight', choices=['lightweight', 'full'])
    ap.add_argument('--target-source', default='', choices=['', 'air_fusion', 'gt_adapter'])
    # W7-b（P1·甲）：空中机观测偏航模式覆盖（''=air.yaml 默认 velocity）
    ap.add_argument('--observe-yaw-mode', default='', choices=['', 'velocity', 'target'])
    # ADR-0074：终态后继续采到**固定观测时长**（自 start 起、仿真钟秒；0=旧行为，终态即拆）
    ap.add_argument('--obs-horizon-s', type=float, default=0.0)
    # 放宽（2026-10-20）：/agc/task/start 等待上限（s）——须 ≥ task_manager 的 air_hover_timeout_s
    # （PX4 慢热时 start 事务内含悬停等待；缺省 60 > 45）。
    ap.add_argument('--start-timeout-s', type=float, default=60.0)
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
            print('=== episode: %s seed=%d [command_source=%s] ===' % (
                scene, seed, args.command_source))
            r = run_episode(scene, seed, registry, args.mode, wall_watchdog_s, out_dir, carriers,
                            ready_timeout_s=args.ready_timeout_s,
                            command_source=args.command_source, shield=args.shield,
                            backend=args.backend, target_source=args.target_source,
                            observe_yaw_mode=args.observe_yaw_mode,
                            obs_horizon_s=args.obs_horizon_s,
                            start_timeout_s=args.start_timeout_s)
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
        # W7-a（R5）：命令源（rl | baseline）——baseline = Navfn + 自研 LQR 离线对照基线。
        'command_source': args.command_source,
        # W7-b（T1/E1）：安全层开关（true=有 shield / false=无 shield 对照臂）
        'shield': args.shield,
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
