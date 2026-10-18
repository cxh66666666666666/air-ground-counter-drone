#!/usr/bin/env python3
"""w6_r019_check —— 评审 R-019（failsafe 因子分离 + 运行时参数锁定）自包含 ROS 级验收 harness。

结构拷贝自 `tests/w6_r018_check.py`：**本脚本不起 launch**（自包含），自己充当「假 PX4」与
「task_manager 桩环境」，以 subprocess 直接起 `install/agc_task/lib/agc_task/jammer`
+ 临时参数文件，无需 gz / launch。

假 PX4（本 harness 节点）：
  * 发布 `/px4_1/fmu/out/vehicle_status`（VehicleStatus，BEST_EFFORT，20 Hz；nav_state 由用例
    可控行为 `px4_behavior` 决定：`none` 不发 / `offboard` 恒 14 / `reach_hold` 收到命令后达命令模式
    并保持 / `lost_failsafe` 收到 `JAM_LOST` 授权后经 `com_of_loss_t` 进入锁定映射模式并保持）。
  * 订 `/px4_1/fmu/in/vehicle_command`（BEST_EFFORT）观测 jammer 注入（`SET_NAV_STATE` / 无命令）。
  * 发布 `/agc/task/envelope_in`（Bool，可控）+ 订 `/agc/task/jam_state`（授权）/`jam_confirm`
    （确认）/`jam_evidence`（证据）。

用例（结果落 `experiments/w6-task-eval/<run_id>/metrics/r019.json`，ADR-0007）：
  a `LOST` **仅断 Offboard**：授权 `JAM_LOST`；jammer **未**发任何 `SET_NAV_STATE`（command==100001）；
    证据含 `factor=offboard_only`。子检：假 PX4 模拟「收到授权后停 offboard，经 `com_of_loss_t` 后
    把 `nav_state` 置为映射值（POSCTL=2）并保持」→ jammer 随后确认 `JAM_LOST`。
  b `RTL` **强制模式**：jammer 发 `SET_NAV_STATE` 且 `param1==5`、授权 `JAM_RTL`、证据含
    `factor=forced_mode`；记录命令到达相对授权事件的时刻（与 a 的「无命令」形成可区分记录）。
  c 运行时参数记录且等于声明值：读 `config/px4_failsafe_lock.yaml` 与**安装后**的 `px4-rc.params`
    （先调用 `scripts/lock_px4_failsafe_params.sh`），断言三参数存在且值 == 声明值；并断言 jammer 的
    `lost_expected_nav_state` == `com_obl_rc_act` 的映射值（跨源一致性）。
  d 未实现因子③如实限制：`r019.json` 顶层写 `factor_3_data_link_loss`；
    断言 `docs/adr/0034-*.md` 存在且含该限制表述（含 `NAV_DLL_ACT` 与 `未实现`/`限制`）。

注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身 → 自杀）；用 "[x]xx" 写法规避或 Popen 句柄 kill。
"""

import argparse
import glob
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
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from ament_index_python.packages import get_package_share_directory

from px4_msgs.msg import VehicleCommand, VehicleStatus
from std_msgs.msg import Bool, String

NS = 'px4_1'
STATUS_TOPIC = f'/{NS}/fmu/out/vehicle_status'
JAM_CMD_TOPIC = '/agc/task/jam_cmd'
JAM_STATE_TOPIC = '/agc/task/jam_state'
JAM_CONFIRM_TOPIC = '/agc/task/jam_confirm'
JAM_EVIDENCE_TOPIC = '/agc/task/jam_evidence'
ENV_TOPIC = '/agc/task/envelope_in'
NAV_STATE_CMD = 100001   # VEHICLE_CMD_SET_NAV_STATE
POSCTL = 2
RTL = 5
JAM_NAV_PARAM1 = (5.0, 18.0, 4.0)   # RTL / LAND / HOLD
POSITIVE = ('JAM_RTL', 'JAM_LAND', 'JAM_HOLD', 'JAM_LOST')
DT = 0.05

# COM_OBL_RC_ACT（PX4 v1.15.4 enum）→ 触发的 nav_state 映射（用于跨源一致性核验）。
# 0 Position→POSCTL(2) / 1 Altitude→ALTCTL(1) / 2 Manual→MANUAL(0) / 3 Return→RTL(5) /
# 4 Land→LAND(18) / 5 Hold→LOITER(4)。
COM_OBL_RC_ACT_NAVSTATE = {0: 2, 1: 1, 2: 0, 3: 5, 4: 18, 5: 4}

LOCK_YAML_REL = 'src/agc_task/config/px4_failsafe_lock.yaml'
PX4_RC_PARAMS_REL = ('toolchain/PX4-Autopilot/build/px4_sitl_default/'
                     'etc/init.d-posix/px4-rc.params')
LOCK_SCRIPT_REL = 'scripts/lock_px4_failsafe_params.sh'


def _qos():
    q = QoSProfile(depth=5)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    return q


class R19Harness(Node):
    """假 PX4（单节点，timer 20 Hz 持续发布可控输入）。"""

    def __init__(self):
        super().__init__('w6_r019_check')
        p = f'/{NS}/fmu'
        self.pub_status = self.create_publisher(VehicleStatus, STATUS_TOPIC, _qos())
        self.create_subscription(VehicleCommand, f'{p}/in/vehicle_command', self._on_px4_cmd, _qos())
        self.pub_env = self.create_publisher(Bool, ENV_TOPIC, 10)
        self.pub_jam_cmd = self.create_publisher(String, JAM_CMD_TOPIC, 10)

        self.create_subscription(String, JAM_STATE_TOPIC, self._on_jam_state, 10)
        self.create_subscription(String, JAM_CONFIRM_TOPIC, self._on_jam_confirm, 10)
        self.create_subscription(String, JAM_EVIDENCE_TOPIC, self._on_jam_evidence, 10)

        self.create_timer(DT, self._pub_tick)

        # 可控开关
        self.px4_on_ = False
        self.env_on_ = False
        self.env_value_ = True
        # PX4 行为
        self.px4_behavior = 'offboard'      # none|offboard|reach_hold|lost_failsafe
        self.px4_sysid = 2
        self.cmd_t = None
        self.cmd_nav = 14
        # R-019：offboard-loss 模拟（lost_failsafe）
        self.com_of_loss_t = 1.0
        self.lost_expected_nav_state = 2
        self.lost_t = None
        # 观测缓冲
        self.jam_states = []      # (t, data)
        self.jam_confirms = []    # (t, data)
        self.jam_evidence = []    # (t, data)
        self.px4_cmds = []        # (t, command, param1, target_system)

    # ---- callbacks ----
    def _on_px4_cmd(self, m):
        self.px4_cmds.append((time.monotonic(), int(m.command), float(m.param1),
                              int(m.target_system)))
        if int(m.command) != NAV_STATE_CMD:
            return
        if not any(abs(float(m.param1) - v) < 0.5 for v in JAM_NAV_PARAM1):
            return
        self.cmd_t = time.monotonic()
        self.cmd_nav = int(round(float(m.param1)))

    def _on_jam_state(self, m):
        d = m.data.strip().upper()
        self.jam_states.append((time.monotonic(), d))
        # R-019 子检：模拟 offboard-loss —— 收到 JAM_LOST 授权后停 offboard
        if (self.px4_behavior == 'lost_failsafe' and d == 'JAM_LOST'
                and self.lost_t is None):
            self.lost_t = time.monotonic()

    def _on_jam_confirm(self, m):
        self.jam_confirms.append((time.monotonic(), m.data.strip().upper()))

    def _on_jam_evidence(self, m):
        self.jam_evidence.append((time.monotonic(), m.data.strip()))

    # ---- 发布 ----
    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _px4_status(self):
        if self.px4_behavior == 'none':
            return None
        if self.px4_behavior == 'lost_failsafe':
            if self.lost_t is None or (time.monotonic() - self.lost_t) < self.com_of_loss_t:
                return (2, 14)      # 失去 offboard 前 / 超时前
            return (2, self.lost_expected_nav_state)   # 经 COM_OF_LOSS_T 后进入锁定映射模式并保持
        if self.px4_behavior == 'reach_hold' and self.cmd_t is not None:
            return (2, self.cmd_nav)
        return (2, 14)

    def _pub_tick(self):
        if self.px4_on_:
            st = self._px4_status()
            if st is not None:
                a, n = st
                msg = VehicleStatus()
                msg.timestamp = self._now_us()
                msg.arming_state = a
                msg.nav_state = n
                msg.system_id = self.px4_sysid
                msg.failsafe = False
                self.pub_status.publish(msg)
        if self.env_on_:
            self.pub_env.publish(Bool(data=bool(self.env_value_)))

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

    # ---- 缓冲清理 ----
    def reset_buffers(self):
        self.jam_states.clear()
        self.jam_confirms.clear()
        self.jam_evidence.clear()
        self.px4_cmds.clear()
        self.cmd_t = None
        self.cmd_nav = 14
        self.lost_t = None


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


def _base_params(share, node_name):
    base = yaml.safe_load(open(os.path.join(share, 'config', 'w6_base.yaml')))
    return dict((base.get(node_name) or {}).get('ros__parameters', {}))


def _start_node(share, out_dir, exe_name, node_name, prefix, overrides):
    path = os.path.join(out_dir, 'r019_%s_%s.yaml' % (prefix, node_name))
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


def _auth(node, since=0.0):
    return [d for t, d in node.jam_states
            if t >= since and d.startswith('JAM_') and d != 'JAM_NONE']


def _confirms(node, since=0.0):
    return [d for t, d in node.jam_confirms if t >= since and d in POSITIVE]


def _evidence(node, since=0.0):
    return [d for t, d in node.jam_evidence if t >= since]


def _nav_cmds(node, since=0.0):
    return [(t, p1, sysid) for (t, cmd, p1, sysid) in node.px4_cmds
            if t >= since and cmd == NAV_STATE_CMD]


def _inject_until_auth(node, cmd, timeout=6.0):
    """反复发干扰命令直至观测到授权事件（JAM_*）——规避 DDS 发现延迟导致的丢包。

    返回首个授权事件的到达墙钟时刻；超时返回 None。
    """
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        node.publish_jam_cmd(cmd)
        node.run(0.15)
        if node.jam_states:
            return node.jam_states[0][0]
    return None


# ---- 用例 ----
def case_a(node, share, out_dir):
    """`LOST` 仅断 Offboard：不发 SET_NAV_STATE；证据 factor=offboard_only；断流后确认 JAM_LOST。"""
    res = {}
    node.reset_buffers()
    node.px4_on_ = True
    node.px4_behavior = 'lost_failsafe'
    node.com_of_loss_t = 1.0
    node.lost_expected_nav_state = 2
    node.env_on_ = True
    node.env_value_ = True
    # 确认窗口放宽至 5.0 s：LOST 需先历经 COM_OF_LOSS_T(=1.0 s) 的 offboard-loss 才进入映射模式，
    # 再持续 jam_sustain_s(=1.0 s)；默认 3.0 s 窗口对本「断流→失联→持续」链路偏紧。
    proc, log, _ = _start_node(share, out_dir, 'jammer', 'jammer', 'a',
                               {'jam_confirm_window_s': 5.0})
    try:
        node.run(1.5)                       # 发现 + 包络新鲜
        t = _inject_until_auth(node, 'LOST')
        # 断流后进入锁定参数定义的失联行为（COM_OF_LOSS_T + 持续）→ 确认
        node.run_until(lambda: _confirms(node, t if t else 0.0), 8.0)
        node.run(0.6)   # 证据与确认跨不同话题到达次序无保证：确认后短驻留让 JAM_CONFIRMED 证据落定
        auth = _auth(node, t if t else 0.0)
        cf = _confirms(node, t if t else 0.0)
        ev = _evidence(node, t if t else 0.0)
        navs = _nav_cmds(node)   # 全用例缓冲（命令先于授权事件发布，故不以授权时刻过滤）
        res['authorized_events'] = auth
        res['confirm_events'] = cf
        res['evidence'] = ev
        res['nav_state_cmds'] = [{'param1': p, 'target_system': s} for _t, p, s in navs]
        res['pass'] = bool(
            t is not None and 'JAM_LOST' in auth and
            not navs and                                   # ① 未发 SET_NAV_STATE
            any('factor=offboard_only' in e for e in ev) and  # ③ 证据含 factor 标签
            'JAM_LOST' in cf and                           # 子检：断流后确认
            any(e.startswith('JAM_CONFIRMED') for e in ev))
    finally:
        _stop_proc(proc, log)
        node.px4_on_ = False
    return res


def case_b(node, share, out_dir):
    """`RTL` 强制模式：发 SET_NAV_STATE(param1==5)、授权 JAM_RTL、证据 factor=forced_mode；记录时序。"""
    res = {}
    node.reset_buffers()
    node.px4_on_ = True
    node.px4_behavior = 'reach_hold'
    node.env_on_ = True
    node.env_value_ = True
    proc, log, _ = _start_node(share, out_dir, 'jammer', 'jammer', 'b', {})
    try:
        node.run(1.5)
        t = _inject_until_auth(node, 'RTL')
        node.run_until(lambda: _confirms(node, t if t else 0.0), 8.0)
        node.run(0.6)   # 证据与确认跨不同话题到达次序无保证：确认后短驻留让证据落定
        auth = _auth(node, t if t else 0.0)
        cf = _confirms(node, t if t else 0.0)
        ev = _evidence(node, t if t else 0.0)
        navs = _nav_cmds(node)   # 全用例缓冲（命令先于授权事件发布，故不以授权时刻过滤）
        res['authorized_events'] = auth
        res['confirm_events'] = cf
        res['evidence'] = ev
        res['nav_state_cmds'] = [{'param1': p, 'target_system': s} for _t, p, s in navs]
        # 命令到达相对授权事件的时刻（与 a 的「无命令」形成可区分记录）
        if navs and t is not None:
            res['first_nav_cmd_delta_vs_auth_s'] = round(navs[0][0] - t, 4)
        else:
            res['first_nav_cmd_delta_vs_auth_s'] = None
        res['pass'] = bool(
            t is not None and 'JAM_RTL' in auth and
            any(abs(p - 5.0) < 0.5 for _t, p, _s in navs) and   # 强制模式发 SET_NAV_STATE=5
            any('factor=forced_mode' in e for e in ev) and     # 证据含 factor 标签
            'JAM_RTL' in cf)
    finally:
        _stop_proc(proc, log)
        node.px4_on_ = False
    return res


def _parse_px4_params(path):
    out = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line.startswith('param set '):
                continue
            parts = line.split()
            if len(parts) >= 4:
                name = parts[2]
                raw = parts[3]
                try:
                    val = float(raw)
                    if val.is_integer():
                        val = int(val)
                except ValueError:
                    val = raw
                out[name] = val
    return out


def case_c(node, share, out_dir):
    """运行时参数记录且等于声明值 + 跨源一致性（lost_expected_nav_state == map(com_obl_rc_act)）。"""
    res = {}
    root = Path(share).parents[3]
    lock_yaml = root / LOCK_YAML_REL
    px4_rc = root / PX4_RC_PARAMS_REL
    lock_sh = root / LOCK_SCRIPT_REL
    declared = yaml.safe_load(open(lock_yaml)) or {}
    # 先调用锁定脚本注入 SITL 启动参数文件（幂等）
    run = subprocess.run(['bash', str(lock_sh)], capture_output=True, text=True)
    res['lock_script_stdout'] = run.stdout.strip().splitlines()
    res['lock_script_rc'] = run.returncode
    injected = _parse_px4_params(str(px4_rc)) if px4_rc.exists() else {}
    res['declared_params'] = declared
    res['injected_params'] = injected
    res['px4_rc_params_path'] = str(px4_rc)

    want = {
        'COM_OF_LOSS_T': declared.get('com_of_loss_t'),
        'COM_OBL_RC_ACT': declared.get('com_obl_rc_act'),
        'NAV_DLL_ACT': declared.get('nav_dll_act'),
    }
    params_match = all(
        name in injected and abs(float(injected[name]) - float(val)) < 1e-9
        for name, val in want.items())
    res['params_match_declared'] = params_match

    # 跨源一致性：jammer 的 lost_expected_nav_state == com_obl_rc_act 的映射
    jam = _base_params(share, 'jammer')
    lost_expected = int(jam.get('lost_expected_nav_state', -1))
    act = int(declared.get('com_obl_rc_act', -1))
    mapped = COM_OBL_RC_ACT_NAVSTATE.get(act)
    res['jammer_lost_expected_nav_state'] = lost_expected
    res['com_obl_rc_act'] = act
    res['com_obl_rc_act_mapped_nav_state'] = mapped
    res['lost_expected_matches_mapping'] = bool(
        mapped is not None and lost_expected == mapped)

    res['runtime_px4_params'] = {
        'COM_OF_LOSS_T': injected.get('COM_OF_LOSS_T'),
        'COM_OBL_RC_ACT': injected.get('COM_OBL_RC_ACT'),
        'NAV_DLL_ACT': injected.get('NAV_DLL_ACT'),
    }
    res['runtime_px4_params_source'] = (
        'lock 文件 %s；由 scripts/lock_px4_failsafe_params.sh 生成 SITL 启动参数文件 %s '
        '（rcS 以 `. px4-rc.params` source；每次 SITL 启动确定化）' % (
            LOCK_YAML_REL, PX4_RC_PARAMS_REL))
    res['pass'] = bool(params_match and res['lost_expected_matches_mapping'])
    return res


def case_d(node, share, out_dir):
    """未实现因子③如实限制：ADR-0034 存在且含限制表述。"""
    res = {}
    root = Path(share).parents[3]
    adrs = sorted(glob.glob(str(root / 'docs' / 'adr' / '0034-*.md')))
    res['adr_paths'] = adrs
    text = ''
    if adrs:
        text = open(adrs[0], encoding='utf-8').read()
    has_dll = 'NAV_DLL_ACT' in text
    has_limit = ('未实现' in text) or ('限制' in text)
    res['adr_exists'] = bool(adrs)
    res['adr_mentions_nav_dll_act'] = has_dll
    res['adr_mentions_limitation'] = has_limit
    res['pass'] = bool(adrs and has_dll and has_limit)
    return res


def dump(res, out_dir):
    path = os.path.join(out_dir, 'metrics', 'r019.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-32s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
        for k, v in t.items():
            if k != 'pass':
                print('      %s = %s' % (k, v))
    print('factor_3_data_link_loss = %s' % res.get('factor_3_data_link_loss'))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r019-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    share = get_package_share_directory('agc_task')
    res = {'run_id': run_id, 'ns': NS, 'tests': {},
           'factor_3_data_link_loss': {
               'implemented': False,
               'note': '本轮未隔离 native data-link-loss（NAV_DLL_ACT）：`LOST` 仅停发 Offboard 流，'
                       '触发 offboard-loss（COM_OF_LOSS_T/COM_OBL_RC_ACT），与 data-link-loss 为不同机制。'}}

    rclpy.init()
    node = R19Harness()
    try:
        res['tests']['a_lost_no_forced_mode'] = case_a(node, share, out_dir)
        res['tests']['b_rtl_forced_mode'] = case_b(node, share, out_dir)
        res['tests']['c_runtime_params'] = case_c(node, share, out_dir)
        res['tests']['d_factor3_limitation'] = case_d(node, share, out_dir)
        # 顶层：运行时参数记录 + 来源 + 因子可区分记录
        c = res['tests']['c_runtime_params']
        res['runtime_px4_params'] = c.get('runtime_px4_params')
        res['runtime_px4_params_source'] = c.get('runtime_px4_params_source')
        a = res['tests']['a_lost_no_forced_mode']
        b = res['tests']['b_rtl_forced_mode']
        res['factor_distinguishability'] = {
            'a_lost_nav_state_cmd_count': len(a.get('nav_state_cmds') or []),
            'b_rtl_nav_state_cmd_count': len(b.get('nav_state_cmds') or []),
            'b_first_nav_cmd_delta_vs_auth_s': b.get('first_nav_cmd_delta_vs_auth_s'),
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
