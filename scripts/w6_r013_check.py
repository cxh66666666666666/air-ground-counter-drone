#!/usr/bin/env python3
"""w6_r013_check —— 评审 R-013 起始异步事务与新鲜门禁验收。

在轻量 W6 栈（`agc_task w6.launch.py backend:=lightweight`，无 PX4）运行中执行，校验 `/agc/task/start`：
  a. 正常启动通过：复位后有**复位后**新鲜样本（odom_valid / 地面 odom / cmd_vel 回环），
     应答含 `[evid]` 证据摘要（reset_done epoch / 各门到达时刻 / 时钟采样）。
  b. 门禁数据缺席被拒：SIGSTOP `ground_odom_adapter`（复位后无新 odom_valid）→
     拒绝且 reason ∈ {odom_valid_stale, ground_odom_stale}；随后 SIGCONT 复原。
  d. 时钟冻结被拒：SIGSTOP 主桥（`parameter_bridge` config_file=bridge.yaml，停发 /clock）
     → 仿真时钟冻结（now() 不推进）→ 拒绝且 reason=clock_not_progressing；随后 SIGCONT 复原。
     （冻结方式：停发 /clock 桥接进程；本节点 use_sim_time=true 时 now() 停在被冻结值。）
  c. 世界名配置校验被拒：停掉当前 task_manager，改起一个 `world_name=wrong_world` 的
     task_manager（复用同一栈的 scenario_reset）→ 拒绝且 reason=single_world_mismatch。

子测顺序见 main()（c 会停掉主 task_manager，故置于最后）。结果落
`experiments/w6-task-eval/<run_id>/metrics/r013.json`（ADR-0007）。

注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
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

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from agc_msgs.msg import AgcTaskState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

ODOM_VALID = '/agc/ground/odom_valid'
CMD = '/agc/ground/cmd_vel'
ODOM = '/agc/ground/odom'
STATE = '/agc/task/state'
START = '/agc/task/start'
ABORT = '/agc/task/abort'

ADAPTER_PAT = '[g]round_odom_adapter'   # ground_odom_adapter 进程
# 主 parameter_bridge：/clock 的唯一发布者（node ros_gz_bridge）。其 config_file 经临时 launch
# 参数文件传入、不出现在命令行；而 set_pose_bridge 在可执行名后带位置参数
# `/world/.../set_pose@...`。故用 "--ros-args" 紧跟可执行名区分主桥（不误伤 set_pose_bridge）。
BRIDGE_PAT = '[p]arameter_bridge --ros-args'
TM_PAT = '[t]ask_manager'               # task_manager 进程（不含 _svc_client 节点名）


def pgrep(pattern):
    try:
        out = subprocess.check_output(['pgrep', '-f', pattern], text=True)
    except subprocess.CalledProcessError:
        return []
    return [int(p) for p in out.split() if p.strip()]


def sig(pid, signal_):
    try:
        os.kill(pid, signal_)
        return True
    except Exception:
        return False


def freeze(pattern):
    pids = pgrep(pattern)
    return pids, [p for p in pids if sig(p, signal.SIGSTOP)]


def thaw(pids):
    return [p for p in pids if sig(p, signal.SIGCONT)]


def cli_service_call(name, timeout=20.0):
    """经 ros2 CLI 调服务（规避同名节点重启后的客户端端点缓存问题）。"""
    try:
        out = subprocess.run(
            ['ros2', 'service', 'call', name, 'std_srvs/srv/Trigger'],
            capture_output=True, text=True, timeout=timeout)
        return out.returncode == 0, (out.stdout or '') + (out.stderr or '')
    except subprocess.TimeoutExpired:
        return False, 'timeout'


def parse_reason(text):
    m = re.search(r'start rejected:\s*([A-Za-z0-9_]+)', text)
    return m.group(1) if m else None


class R13Checker(Node):
    def __init__(self):
        super().__init__('w6_r013_check')
        self.ov = None
        self.have_cmd = False
        self.have_odom = False
        self.state = 0
        self.state_name = ''
        self.create_subscription(Bool, ODOM_VALID, self._on_ov, 20)
        self.create_subscription(TwistStamped, CMD, self._on_cmd, 20)
        self.create_subscription(Odometry, ODOM, self._on_odom, 20)
        self.create_subscription(AgcTaskState, STATE, self._on_state, 20)
        self.start_cli = self.create_client(Trigger, START)
        self.abort_cli = self.create_client(Trigger, ABORT)

    def _on_ov(self, m):
        self.ov = bool(m.data)

    def _on_cmd(self, _msg):
        self.have_cmd = True

    def _on_odom(self, _msg):
        self.have_odom = True

    def _on_state(self, m):
        self.state = m.state
        self.state_name = m.state_name

    def spin(self, sec):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_until(self, cond, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if cond():
                return True, time.monotonic() - t0
        return False, time.monotonic() - t0

    def call(self, cli, timeout=20.0):
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return None
        return fut.result()

    def start(self, timeout=20.0):
        res = self.call(self.start_cli, timeout)
        if res is None:
            return {'ok': None, 'message': 'call timeout'}
        return {'ok': bool(res.success), 'message': res.message}

    def abort(self):
        self.call(self.abort_cli, timeout=10.0)
        self.spin(0.5)


def spawn_wrong_world_tm(out_dir):
    """起一个 world_name=wrong_world 的独立 task_manager（复用栈内 scenario_reset）。"""
    share = get_package_share_directory('agc_task')
    import yaml
    base = yaml.safe_load(open(os.path.join(share, 'config', 'w6_base.yaml')))
    params = dict(base['task_manager']['ros__parameters'])
    params['world_name'] = 'wrong_world'
    doc = {'task_manager': {'ros__parameters': params}}
    path = os.path.join(out_dir, 'r013_wrong_world.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
    log = open(os.path.join(out_dir, 'wrong_world_tm.log'), 'w')
    proc = subprocess.Popen(
        ['ros2', 'run', 'agc_task', 'task_manager',
         '--ros-args', '--params-file', path, '-p', 'use_sim_time:=true'],
        stdout=log, stderr=subprocess.STDOUT)
    return proc, path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r013-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)

    rclpy.init()
    n = R13Checker()
    res = {'run_id': run_id, 'tests': {}, 'frozen': {}}
    resumed = []
    spawned = None
    try:
        # 等门控服务出现（task_manager 起即建服务）
        if not n.start_cli.wait_for_service(timeout_sec=30.0):
            res['overall_pass'] = False
            res['error'] = '/agc/task/start service never appeared'
            return dump(res, out_dir)
        # 等门禁数据就绪（odom_valid / cmd / odom 至少到达一次）
        n.spin_until(lambda: n.ov is not None and n.have_cmd and n.have_odom, 30.0)

        # ---------------- a. 正常启动通过（复位后新鲜数据） ----------------
        a = dict(n.start())
        a['state_name_after'] = n.state_name
        if a.get('ok'):
            n.spin_until(lambda: n.state_name == 'AIR_SEARCH', 3.0)
            a['state_name_after'] = n.state_name
        a['has_evid'] = '[evid]' in (a.get('message') or '')
        m = re.search(r'reset_done=([0-9.]+)', a.get('message') or '')
        a['reset_done'] = float(m.group(1)) if m else None
        a['pass'] = bool(a.get('ok') and 'AIR_SEARCH' in (a.get('message') or '')
                         and a['has_evid'] and a['reset_done'] is not None
                         and a['reset_done'] > 0.0 and n.state_name == 'AIR_SEARCH')
        res['tests']['a_normal_pass'] = a
        n.abort()

        # ---------------- b. 复位后新鲜数据缺席 → 拒绝 ----------------
        pids, frozen = freeze(ADAPTER_PAT)
        res['frozen']['adapter_pids'] = pids
        time.sleep(0.5)
        b = dict(n.start())
        b['reason'] = parse_reason(b.get('message') or '')
        b['pass'] = bool(b.get('ok') is False
                         and b['reason'] in ('odom_valid_stale', 'ground_odom_stale'))
        res['tests']['b_stale_reject'] = b
        resumed += thaw(frozen)
        n.abort()

        # ---------------- d. 时钟冻结 → 拒绝 ----------------
        bpids, bfrozen = freeze(BRIDGE_PAT)
        res['frozen']['bridge_pids'] = bpids
        n.spin(2.0)   # 令 1 Hz 计时器采样到冻结后的时钟
        d = dict(n.start())
        d['reason'] = parse_reason(d.get('message') or '')
        d['pass'] = bool(d.get('ok') is False and d['reason'] == 'clock_not_progressing')
        res['tests']['d_clock_frozen'] = d
        resumed += thaw(bfrozen)
        n.spin(2.0)   # 等 /clock 恢复推进

        # ---------------- c. 世界名配置校验 → 拒绝 ----------------
        tm_pids = pgrep(TM_PAT)
        res['frozen']['task_manager_pids'] = tm_pids
        for p in tm_pids:
            sig(p, signal.SIGTERM)
        time.sleep(2.0)
        for p in tm_pids:
            sig(p, signal.SIGKILL)
        spawned, _ = spawn_wrong_world_tm(out_dir)
        time.sleep(6.0)
        ok_cli, txt = cli_service_call(START, timeout=25.0)
        reason = parse_reason(txt)
        c = {'ok_cli': ok_cli, 'reason': reason, 'success_false': 'success=False' in txt,
             'raw': txt.strip()[-400:]}
        c['pass'] = bool(c['success_false'] and reason == 'single_world_mismatch')
        res['tests']['c_wrong_world'] = c

        res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
        return dump(res, out_dir)
    finally:
        if spawned is not None:
            try:
                spawned.terminate()
                spawned.wait(timeout=5)
            except Exception:
                pass
        thaw(resumed)
        n.destroy_node()
        rclpy.shutdown()


def dump(res, out_dir):
    path = os.path.join(out_dir, 'metrics', 'r013.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-22s %s  %s' % (name, 'PASS' if t.get('pass') else 'FAIL', t))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


if __name__ == '__main__':
    sys.exit(main())
