#!/usr/bin/env python3
"""w6_r014_check —— 评审 R-014 C7 出生净空统一候选校验验收。

在轻量 W6 栈（`agc_task w6.launch.py backend:=lightweight`，无 PX4）运行中执行：先等 `/agc/scenario/reset`
服务出现，再**杀掉 launch 起的 `scenario_reset`**（服务名固定 `/agc/scenario/reset`，不能并存两个
server），随后对每个用例按参数重启一个 `scenario_reset` 节点并调服务：

  a. 车压立柱（目标远）          → 拒，reason 以 `car_obstacle_clearance` 开头
  b. 己方 UAV 近邻（与车重合）   → 拒，reason = `car_own_uav_clearance`
  c. 越界（车 x=45）             → 拒，reason = `boundary_out_car`
  d. 极限合法（车—目标恰 6.0 m） → 收
  e. 随机全非法（每轮候选都压柱）→ 拒，reason 以 `car_obstacle_clearance` 开头
  f. 随机可合法（重采样找到合法）→ 收

服务调用走 `ros2 service call` CLI（规避同名节点重启后的客户端端点缓存）。结果落
`experiments/w6-task-eval/<run_id>/metrics/r014.json`（ADR-0007）。

注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory

SERVICE = '/agc/scenario/reset'

# 场景实体几何事实（决策依据见 ADR-0029）：立柱 pole_1(0,-6) / pole_2(6,6)，0.3×0.3；
# 默认 spawn car(-4,-3) / target(6.5,2.5)。
CASES = [
    {
        'name': 'a_car_on_pole',
        'overrides': {'car_x': 0.0, 'car_y': -6.0, 'target_x': 20.0, 'target_y': 20.0,
                      'spawn_randomize': False},
        'expect_success': False, 'reason_prefix': 'car_obstacle_clearance',
    },
    {
        'name': 'b_own_uav_overlap',
        'overrides': {'own_uav_present': True, 'own_uav_x': -4.0, 'own_uav_y': -3.0},
        'expect_success': False, 'reason_exact': 'car_own_uav_clearance',
    },
    {
        'name': 'c_out_of_bounds',
        'overrides': {'car_x': 45.0},
        'expect_success': False, 'reason_exact': 'boundary_out_car',
    },
    {
        'name': 'd_limit_legal',
        'overrides': {'car_x': -4.0, 'car_y': -3.0, 'target_x': 2.0, 'target_y': -3.0},
        'expect_success': True,
    },
    {
        'name': 'e_random_all_illegal',
        'overrides': {'car_x': 0.0, 'car_y': -6.0, 'spawn_randomize': True,
                      'spawn_search_radius_m': 0.1, 'spawn_seed': 0,
                      'target_x': 20.0, 'target_y': 20.0},
        'expect_success': False, 'reason_prefix': 'car_obstacle_clearance',
    },
    {
        'name': 'f_random_legal',
        'overrides': {'car_x': -4.0, 'car_y': -3.0, 'target_x': 6.5, 'target_y': 2.5,
                      'spawn_randomize': True, 'spawn_search_radius_m': 3.0, 'spawn_seed': 0},
        'expect_success': True,
    },
]


def run_cli(args, timeout=20.0):
    try:
        out = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return out.returncode, (out.stdout or '') + (out.stderr or '')
    except subprocess.TimeoutExpired:
        return -1, 'timeout'


def service_present(name):
    # 用 --no-daemon 直连发现：ros2 daemon 的图缓存会残留已死节点的服务名，导致误判
    # "服务仍在" / "新节点已就绪"。
    rc, txt = run_cli(['ros2', 'service', 'list', '--no-daemon'], timeout=12.0)
    if rc != 0:
        return False
    return any(line.strip() == name for line in txt.splitlines())


def wait_service(name, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if service_present(name):
            return True
        time.sleep(1.0)
    return False


def wait_service_gone(name, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if not service_present(name):
            return True
        time.sleep(0.5)
    return False


def parse_reply(txt):
    m = re.search(r'success=(True|False)', txt)
    success = (m.group(1) == 'True') if m else None
    m2 = re.search(r"message='([^']*)'", txt)
    message = m2.group(1) if m2 else ''
    reason = None
    if message.startswith('reset rejected:'):
        reason = message[len('reset rejected:'):].strip()
    return success, message, reason


def write_params(share, path, overrides):
    base = yaml.safe_load(open(os.path.join(share, 'config', 'w6_base.yaml')))
    params = dict(base['scenario_reset']['ros__parameters'])
    params.update(overrides)
    params['use_sim_time'] = True
    doc = {'scenario_reset': {'ros__parameters': params}}
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
    return params


def run_case(share, out_dir, case, idx):
    params_path = os.path.join(out_dir, 'r014_case_%s.yaml' % case['name'])
    write_params(share, params_path, case['overrides'])
    log_path = os.path.join(out_dir, 'log', 'case_%s.log' % case['name'])
    log = open(log_path, 'w')
    # 直接执行 install 里的可执行文件（**不用** `ros2 run`：那是 python 包装、把节点作为子进程，
    # terminate 只杀包装 → 节点残留 → DDS 上累积同名 server，CLI 调用会随机命中旧参数）。
    exe = str(Path(share).parents[1] / 'lib' / 'agc_task' / 'scenario_reset')
    proc = subprocess.Popen(
        [exe, '--ros-args', '--params-file', params_path],
        stdout=log, stderr=subprocess.STDOUT)
    rec = {'name': case['name'], 'overrides': case['overrides']}
    try:
        if not wait_service(SERVICE, 25.0):
            rec['pass'] = False
            rec['error'] = 'service never appeared'
            return rec
        time.sleep(1.5)   # 等节点就绪 + 若 config 非法可见启动日志
        rc, txt = run_cli(['ros2', 'service', 'call', SERVICE, 'std_srvs/srv/Trigger'],
                          timeout=25.0)
        success, message, reason = parse_reply(txt)
        rec.update({'rc': rc, 'success': success, 'message': message, 'reason': reason,
                    'raw': txt.strip()[-400:]})
        if case['expect_success']:
            rec['pass'] = bool(success is True)
        elif 'reason_exact' in case:
            rec['pass'] = bool(success is False and reason == case['reason_exact'])
        else:
            rec['pass'] = bool(success is False and (reason or '').startswith(
                case['reason_prefix']))
        return rec
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=5)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        log.close()
        # 确保本用例节点已从 DDS 注销（服务消失）再进入下一用例，避免同名 server 叠加
        wait_service_gone(SERVICE, 15.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()
    run_id = args.run_id or (datetime.now().strftime('%Y%m%d') + '-w6-r014-0')
    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', run_id)
    os.makedirs(os.path.join(out_dir, 'metrics'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'log'), exist_ok=True)

    share = get_package_share_directory('agc_task')
    res = {'run_id': run_id, 'service': SERVICE, 'tests': {}}

    # 1. 等 launch 起的 /agc/scenario/reset 服务出现（确认栈已就绪）
    if not wait_service(SERVICE, 30.0):
        res['overall_pass'] = False
        res['error'] = '/agc/scenario/reset service never appeared (stack not ready)'
        return dump(res, out_dir)

    # 2. 杀掉 launch 起的 scenario_reset（服务名固定，不能并存两个 server）
    run_cli(['pkill', '-TERM', '-f', '[s]cenario_reset'])
    if not wait_service_gone(SERVICE, 15.0):
        res['overall_pass'] = False
        res['error'] = 'launch scenario_reset did not deregister'
        return dump(res, out_dir)

    # 3. 逐用例
    for idx, case in enumerate(CASES):
        res['tests'][case['name']] = run_case(share, out_dir, case, idx)
        time.sleep(1.0)

    res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    return dump(res, out_dir)


def dump(res, out_dir):
    path = os.path.join(out_dir, 'metrics', 'r014.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-24s %s  success=%s reason=%s' % (
            name, 'PASS' if t.get('pass') else 'FAIL', t.get('success'), t.get('reason')))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    return 0 if res.get('overall_pass') else 1


if __name__ == '__main__':
    sys.exit(main())
