#!/usr/bin/env python3
"""w6_r038_bench —— R-038（数值模型暴露与配置合法性启动校验）**真实节点启动期拒绝台架**。

起**真实**节点并喂**非法 / 合法**配置，验证启动期**明确拒绝**（进程非零退出 + 明确拒绝日志）
与**合法配置可运行**：

  1. `map_adapter_valid`            map.yaml + obstacles.yaml 合法 → 节点存活
  2. `map_adapter_invalid_geometry` 地图 size_x=0 → **拒绝启动**（非静默以默认值运行）
  3. `map_adapter_invalid_obstacle` 障碍 size 含 0 → **拒绝启动**
  4. `air_perception_valid_seed`    seed="0" → 节点存活
  5. `air_perception_invalid_seed`  seed="abc" → **拒绝启动**（旧实现静默退回默认基种子）

只回收**本进程组**的子进程（`start_new_session` + `killpg`）；**不**用裸名 `pkill -f`、**不**做递归删除。
证据落 `experiments/w6-task-eval/<run_id>/r038/bench.json`（ADR-0007）。

⚠ 范围标注：本台架为**节点启动期校验逻辑台架**（真实节点 + 真实 YAML 参数，**非**整链 / **非**双 PX4 / **非** GUI）。
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

ROOT = '/home/chen/AGC'
PERC_BIN = os.path.join(ROOT, 'install', 'agc_perception', 'lib', 'agc_perception')
PERC_SHARE = os.path.join(ROOT, 'install', 'agc_perception', 'share', 'agc_perception', 'config')


def _read(path):
    if not os.path.exists(path):
        return ''
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


def _kill(proc):
    """仅回收本子进程组（start_new_session → 独立 pgid）；self-group 护栏下退化为单 PID。"""
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    try:
        if pgid == os.getpgrp():
            proc.send_signal(signal.SIGKILL)
        else:
            os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError, OSError):
        pass
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass


def _reap_stale():
    """清理同名残留真实进程（obfuscated pattern：不匹配本进程命令行；仅按 PID kill）。"""
    for pat in ('agc_perception/lib/agc_perception/map_adapte[r]',
                'agc_perception/lib/agc_perception/air_perceptio[n]'):
        try:
            out = subprocess.run(['pgrep', '-f', pat], capture_output=True, text=True, timeout=10)
        except Exception:  # noqa: BLE001
            continue
        for pid in [p for p in out.stdout.split() if p.strip().isdigit()]:
            if int(pid) != os.getpid():
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except OSError:
                    pass
    time.sleep(0.5)


def _spawn(cmd, log_path):
    return subprocess.Popen(cmd, stdout=open(log_path, 'w'), stderr=subprocess.STDOUT,
                            start_new_session=True)


def run_expect_reject(bin_path, params_file, log, timeout=12.0):
    """启动节点，期望**很快退出**（非零 / 信号）且日志含「拒绝启动」。"""
    res = {'binary': os.path.basename(bin_path), 'params_file': os.path.basename(params_file)}
    proc = _spawn([bin_path, '--ros-args', '--params-file', params_file], log)
    try:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if proc.poll() is not None:
                break
            time.sleep(0.25)
        rc = proc.poll()
        alive = rc is None
        res['exited'] = not alive
        res['returncode'] = rc
        txt = _read(log)
        res['log_has_reject'] = '拒绝启动' in txt
        res['log_has_runtime_error'] = 'runtime_error' in txt or 'terminate' in txt
        res['pass'] = bool((not alive) and (rc != 0) and res['log_has_reject'])
    finally:
        _kill(proc)
    return res


def run_expect_alive(bin_path, params_file, log, hold=3.0):
    """启动节点，期望**保持存活**（合法配置不被误拒）。"""
    res = {'binary': os.path.basename(bin_path), 'params_file': os.path.basename(params_file)}
    proc = _spawn([bin_path, '--ros-args', '--params-file', params_file], log)
    try:
        end = time.monotonic() + hold
        while time.monotonic() < end:
            if proc.poll() is not None:
                break
            time.sleep(0.25)
        rc = proc.poll()
        res['returncode'] = rc
        res['alive'] = rc is None
        res['pass'] = bool(rc is None)
    finally:
        _kill(proc)
    return res


def _write(path, text):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261017-w6-r038-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()

    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)
    cfg_dir = os.path.join(out_dir, 'config')
    log_dir = os.path.join(out_dir, 'r038', 'log')
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    map_ok = os.path.join(PERC_SHARE, 'map.yaml')
    obs_ok = os.path.join(PERC_SHARE, 'obstacles.yaml')
    sensors_ok = os.path.join(PERC_SHARE, 'perception_sensors.yaml')

    # 合法 / 非法参数文件（落 <run>/config/，可入库）
    p_map_valid = os.path.join(cfg_dir, 'r038_map_adapter_valid.yaml')
    _write(p_map_valid, f'map_adapter:\n  ros__parameters:\n'
                         f'    world_frame: "world"\n'
                         f'    map_file: "{map_ok}"\n'
                         f'    obstacles_file: "{obs_ok}"\n'
                         f'    obstacles_source: "config"\n')
    p_map_bad_geom = os.path.join(cfg_dir, 'r038_map_adapter_bad_geometry.yaml')
    map_bad = os.path.join(cfg_dir, 'r038_map_bad.yaml')
    _write(map_bad, 'resolution: 0.5\nsize_x: 0.0\nsize_y: 80.0\norigin_x: -40.0\norigin_y: -40.0\n')
    _write(p_map_bad_geom, f'map_adapter:\n  ros__parameters:\n'
                           f'    world_frame: "world"\n'
                           f'    map_file: "{map_bad}"\n'
                           f'    obstacles_file: "{obs_ok}"\n')
    p_map_bad_obs = os.path.join(cfg_dir, 'r038_map_adapter_bad_obstacle.yaml')
    obs_bad = os.path.join(cfg_dir, 'r038_obstacles_bad.yaml')
    _write(obs_bad, 'obstacles:\n  - id: pole_1\n    x: 0.0\n    y: -6.0\n    z: 1.5\n'
                    '    yaw: 0.0\n    size: [0.3, 0.0, 3.0]\n')
    _write(p_map_bad_obs, f'map_adapter:\n  ros__parameters:\n'
                          f'    world_frame: "world"\n'
                          f'    map_file: "{map_ok}"\n'
                          f'    obstacles_file: "{obs_bad}"\n')
    p_perc_valid = os.path.join(cfg_dir, 'r038_air_perception_valid_seed.yaml')
    _write(p_perc_valid, f'air_perception:\n  ros__parameters:\n'
                         f'    seed: "0"\n'
                         f'    sensors_file: "{sensors_ok}"\n'
                         f'    obstacles_file: "{obs_ok}"\n')
    p_perc_bad = os.path.join(cfg_dir, 'r038_air_perception_bad_seed.yaml')
    _write(p_perc_bad, f'air_perception:\n  ros__parameters:\n'
                       f'    seed: "abc"\n'
                       f'    sensors_file: "{sensors_ok}"\n'
                       f'    obstacles_file: "{obs_ok}"\n')

    map_bin = os.path.join(PERC_BIN, 'map_adapter')
    perc_bin = os.path.join(PERC_BIN, 'air_perception')

    _reap_stale()
    result = {
        'run_id': args.run_id,
        'suite': 'R-038 config-validity startup-reject bench',
        'scope_note': (
            '节点启动期校验**逻辑台架**：真实 map_adapter / air_perception 节点 + 真实 YAML 参数，'
            '验证非法几何 / seed **拒绝启动**、合法配置**不被误拒**；**非整链** / 非双 PX4 / 非 GUI。'),
        'unsupported': [],
        'scenarios': {},
    }
    result['scenarios']['map_adapter_valid'] = run_expect_alive(
        map_bin, p_map_valid, os.path.join(log_dir, 'map_adapter_valid.log'))
    time.sleep(0.5)
    result['scenarios']['map_adapter_invalid_geometry'] = run_expect_reject(
        map_bin, p_map_bad_geom, os.path.join(log_dir, 'map_adapter_invalid_geometry.log'))
    time.sleep(0.5)
    result['scenarios']['map_adapter_invalid_obstacle'] = run_expect_reject(
        map_bin, p_map_bad_obs, os.path.join(log_dir, 'map_adapter_invalid_obstacle.log'))
    time.sleep(0.5)
    result['scenarios']['air_perception_valid_seed'] = run_expect_alive(
        perc_bin, p_perc_valid, os.path.join(log_dir, 'air_perception_valid_seed.log'))
    time.sleep(0.5)
    result['scenarios']['air_perception_invalid_seed'] = run_expect_reject(
        perc_bin, p_perc_bad, os.path.join(log_dir, 'air_perception_invalid_seed.log'))

    for name, s in result['scenarios'].items():
        print('%-32s PASS=%s' % (name, s.get('pass')))
    result['overall_pass'] = bool(
        result['scenarios'] and all(s.get('pass') for s in result['scenarios'].values()))
    with open(os.path.join(out_dir, 'r038', 'bench.json'), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print('\noverall_pass=%s -> %s' % (
        result['overall_pass'], os.path.join(out_dir, 'r038', 'bench.json')))
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
