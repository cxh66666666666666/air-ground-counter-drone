#!/usr/bin/env python3
"""w7_episode_runner —— W7-a **单 worker 可重放 runner**（T4 · spec R4）。

对每个 attempt：按 `agc_rl.seeding` 派生种子 → 起 `w6.launch.py backend:=lightweight
command_source:=train scene:=<scene> seed:=<spawn_seed> spawn_randomize:=true`（外部适配层作命令源）
→ `GroundAgcEnv.reset/step` 闭环（规则策略驱动）→ SIGINT 拆除进程组 → 记录回合。

可重放（R4 判据「连续两回合可复现」）：同 `(scene, episode)` 的多次 attempt 面对**共同外生随机过程**
（spawn / sensor / channel / target / physics 不随 attempt 变，R3）。runner 比对 attempt 之间的
**外生随机流 / spawn / 噪声序列 / 确定性策略动作序列**是否一致，并**记录**实时异步调度导致的
**物理轨迹 / 终态分叉边界**（R3：不承诺 bitwise 物理一致）。

产物（[ADR-0007](../../docs/adr/0007-experiment-artifacts-convention.md) 唯一落点，`run_id` 不可覆盖）：
`experiments/w7-rl/<run_id>/{config/*.json, episodes/ep_a*.json, metrics/replay.json, log/*.log}`。

用法：
  python3 scripts/w7_episode_runner.py --scene s01_static_none_nominal_track \
      --master-seed 7 --attempts 2 --obs-noise-sigma 0.02
"""

import argparse
import datetime
import json
import math
import os
import random
import subprocess
import sys
import time

ROOT = '/home/chen/AGC'
CAMPAIGN = 'w7-rl'
sys.path.insert(0, os.path.join(ROOT, 'scripts'))   # 复用 episode_runner 的进程组回收

from agc_rl import env as env_mod            # noqa: E402
from agc_rl import seeding                  # noqa: E402
from agc_rl import termination              # noqa: E402
from agc_rl.policy import load_policy       # noqa: E402
from episode_runner import _reap_process_group   # noqa: E402

WORLD = os.path.join(ROOT, 'install/agc_sim/share/agc_sim/worlds/agc_world.sdf')
LAUNCH = 'w6.launch.py'
SPAWN_TOL_M = 0.05


def launch_stack(scene, spawn_seed, mode, log_path, spawn_randomize):
    gz = ('-r -v 3 ' if mode == 'gui' else '-s -r -v 3 ') + WORLD
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    proc = subprocess.Popen(
        ['ros2', 'launch', 'agc_task', LAUNCH,
         'backend:=lightweight', 'command_source:=train', 'scene:=%s' % scene,
         'seed:=%d' % spawn_seed,
         'spawn_randomize:=%s' % ('true' if spawn_randomize else 'false'), 'gz_args:=%s' % gz],
        stdout=open(log_path, 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        pgid = None
    return proc, pgid


def run_attempt(scene, master_seed, episode_index, attempt_index, args, out_dir, w7cfg):
    plan = seeding.make_seed_plan(master_seed, scene, episode_index, attempt_index)
    log = os.path.join(out_dir, 'log', 'ep_a%d.log' % attempt_index)
    proc, pgid = launch_stack(scene, plan.spawn_seed, args.mode, log, w7cfg['spawn_randomize'])
    wall_start = time.monotonic()

    ep = {'scene': scene, 'master_seed': master_seed, 'episode_index': episode_index,
          'attempt_index': attempt_index, 'seed_plan': plan.to_dict(),
          'noise_seq': None, 'spawn': None, 'reset': None, 'actions': [],
          'poses': [], 'terminal': None, 'outcome': None, 'steps': 0}

    env = None
    try:
        # 参数取 `config/w7.yaml` + `config/rl.yaml`（**单一来源**，spec R6；env 内部读取）。
        env = env_mod.GroundAgcEnv(scene, seed_plan=plan)
        policy = load_policy('')   # 确定性规则策略（go_to_goal；avoid_obstacles=False，ADR-0004）
        ep['noise_seq'] = env.noise_sequence(20)

        obs, info = env.reset(seed=master_seed, options={
            'episode_index': episode_index, 'attempt_index': attempt_index,
            'service_wait_s': args.service_wait_s})
        ep['reset'] = info
        ep['spawn'] = info.get('spawn')
        ep['reset_obs_head'] = obs[:6]

        t0 = time.monotonic()
        terminated = truncated = False
        while not (terminated or truncated):
            action = int(policy.act(obs, 12))
            ep['actions'].append(action)
            ep['poses'].append(list(env.robot_pose()))
            obs, _reward, terminated, truncated, info = env.step(action)
            ep['steps'] += 1
            if time.monotonic() - t0 > float(env.cfg['wall_watchdog_s']):
                truncated = True
        ep['terminal'] = info.get('task_state')
        ep['outcome'] = info.get('outcome')
        ep['sim_end_s'] = env._node.sim_s
    finally:
        if env is not None:
            env.close()
        ep['proc_exit_confirmed'], ep['proc_returncode'] = _reap_process_group(proc, pgid)
    ep['wall_elapsed_s'] = time.monotonic() - wall_start
    # R-024：RTF = 仿真增量 / 墙钟增量（供不同 RTF 的分叉归因；非可复现量）
    r0 = (ep['reset'] or {}).get('sim_s')
    ep['rtf'] = (None if (r0 is None or ep.get('sim_end_s') is None or ep['wall_elapsed_s'] <= 0)
                 else (ep['sim_end_s'] - r0) / ep['wall_elapsed_s'])
    return ep


def _pose_dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def compare_replay(ep_a, ep_b):
    """比对两次同 `(scene, episode)` attempt 的可复现性 + 记录轨迹分叉边界。"""
    s_exo = all(ep_a['seed_plan']['streams'][s] == ep_b['seed_plan']['streams'][s]
                for s in seeding.EXOGENOUS_STREAMS)
    spawn_delta = (None if (ep_a['spawn'] is None or ep_b['spawn'] is None)
                   else _pose_dist(ep_a['spawn'], ep_b['spawn']))
    noise_equal = ep_a['noise_seq'] == ep_b['noise_seq']
    npref = min(len(ep_a['actions']), len(ep_b['actions']))
    prefix_equal = ep_a['actions'][:npref] == ep_b['actions'][:npref]
    div_step = next((i for i in range(npref) if ep_a['actions'][i] != ep_b['actions'][i]), None)
    n = min(len(ep_a['poses']), len(ep_b['poses']))
    traj_delta = max((_pose_dist(ep_a['poses'][i], ep_b['poses'][i]) for i in range(n)), default=None)

    checks = {
        'exogenous_streams_identical': bool(s_exo),
        'spawn_within_tolerance': (spawn_delta is not None and spawn_delta <= SPAWN_TOL_M),
        'noise_sequence_identical': bool(noise_equal),
        'deterministic_policy_prefix_identical': bool(prefix_equal and npref > 0),
    }
    boundary = {
        'trajectory_divergence_m': traj_delta,
        'action_divergence_step': div_step,
        'terminal_a': ep_a['terminal'], 'terminal_b': ep_b['terminal'],
        'steps_a': len(ep_a['actions']), 'steps_b': len(ep_b['actions']),
        'rtf_a': ep_a.get('rtf'), 'rtf_b': ep_b.get('rtf'),
        'note': '实时异步调度下**物理轨迹 / 终态**不承诺 bitwise（R3 边界）；'
                '上列可复现量为 seed 级不变量。',
    }
    return {'checks': checks, 'spawn_delta_m': spawn_delta, 'boundary': boundary}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', default='s01_static_none_nominal_track')
    ap.add_argument('--master-seed', type=int, default=7)
    ap.add_argument('--episode-index', type=int, default=0)
    ap.add_argument('--attempts', type=int, default=2)
    ap.add_argument('--mode', default='headless', choices=['headless', 'gui'])
    ap.add_argument('--service-wait-s', type=float, default=60.0,
                    help='等待 /agc/task/start 就绪的上限（秒；runner 稳健性，非 W7 平台参数）')
    # 注：obs_version / control_dt_s / max_steps / wall_watchdog_s / obs_noise_sigma / spawn_randomize
    # 属 W7 平台参数，**不**设 CLI —— 单一来源 `agc_rl/config/w7.yaml`（spec R6）。
    ap.add_argument('--campaign', default=CAMPAIGN)
    ap.add_argument('--run-id', default=None)
    args = ap.parse_args()

    run_id = args.run_id or ('%s-%s-%s-%04d' % (
        datetime.datetime.now().strftime('%Y%m%d'), args.campaign,
        datetime.datetime.now().strftime('%H%M%S'), random.randint(0, 9999)))
    out_dir = os.path.join(ROOT, 'experiments', args.campaign, run_id)
    if os.path.exists(out_dir):
        print('拒绝写入已有 run 目录（run_id 唯一不可覆盖）：%s' % out_dir)
        return 2
    for sub in ('config', 'episodes', 'metrics', 'log'):
        os.makedirs(os.path.join(out_dir, sub), exist_ok=False)

    # W7 平台参数：单一来源 `agc_rl/config/w7.yaml`（env 侧另读 `rl.yaml`）；缺失回落内置默认。
    _rl_p, _w7_p = env_mod._load_share_yaml()
    w7cfg = dict(env_mod._W7_DEFAULTS)
    if isinstance(_w7_p, dict):
        w7cfg.update({k: v for k, v in _w7_p.items() if k in w7cfg})

    git_commit = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip()
    with open(os.path.join(out_dir, 'config', 'run_identity.json'), 'w', encoding='utf-8') as f:
        json.dump({'campaign': args.campaign, 'run_id': run_id, 'git_commit': git_commit,
                   'runner': 'scripts/w7_episode_runner.py', 'backend': 'lightweight',
                   'launch': LAUNCH, 'command_source': 'train', 'scene': args.scene,
                   'master_seed': args.master_seed, 'episode_index': args.episode_index,
                   'attempts': args.attempts, 'w7_config': w7cfg,
                   'config_source': 'agc_rl/config/{rl,w7}.yaml',
                   'time_budgets': termination.TIME_BUDGETS}, f, indent=2, ensure_ascii=False)

    episodes = []
    for attempt in range(args.attempts):
        print('=== attempt %d (scene=%s master_seed=%d episode=%d) ===' % (
            attempt, args.scene, args.master_seed, args.episode_index))
        ep = run_attempt(args.scene, args.master_seed, args.episode_index, attempt, args,
                         out_dir, w7cfg)
        print('  spawn=%s steps=%d terminal=%s outcome=%s' % (
            ep['spawn'], ep['steps'], ep['terminal'],
            (ep['outcome'] or {}).get('outcome_class')))
        with open(os.path.join(out_dir, 'episodes', 'ep_a%d.json' % attempt), 'w',
                  encoding='utf-8') as f:
            json.dump(ep, f, indent=2, ensure_ascii=False)
        episodes.append(ep)

    # R4：连续两回合可复现判定（≥2 attempt 时）
    replay = None
    if len(episodes) >= 2:
        replay = compare_replay(episodes[0], episodes[1])
        replay['overall_pass'] = all(replay['checks'].values())
        replay['a0'] = {'spawn': episodes[0]['spawn'], 'noise_head': episodes[0]['noise_seq'][:5],
                        'terminal': episodes[0]['terminal']}
        replay['a1'] = {'spawn': episodes[1]['spawn'], 'noise_head': episodes[1]['noise_seq'][:5],
                        'terminal': episodes[1]['terminal']}
    report = {
        'run_id': run_id, 'campaign': args.campaign, 'git_commit': git_commit,
        'scene': args.scene, 'master_seed': args.master_seed,
        'episode_index': args.episode_index, 'n_attempts': len(episodes),
        'command_source': 'train', 'backend': 'lightweight', 'launch': LAUNCH,
        'obs_version': int(w7cfg['obs_version']), 'w7_config': w7cfg,
        'replay': replay,
        'episodes': [{'attempt': e['attempt_index'], 'spawn': e['spawn'],
                      'steps': e['steps'], 'terminal': e['terminal'],
                      'outcome_class': (e['outcome'] or {}).get('outcome_class'),
                      'counted_as_success': (e['outcome'] or {}).get('counted_as_success'),
                      'rtf': e.get('rtf'), 'wall_elapsed_s': e.get('wall_elapsed_s')}
                     for e in episodes],
        'reproducibility_note': '可复现量 = seed 计划 / spawn / 外生噪声序列 / 动作序列；'
                                '物理轨迹分叉为已知边界（ADR-0069(b)）。',
    }
    with open(os.path.join(out_dir, 'metrics', 'replay.json'), 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    if replay:
        for k, v in replay['checks'].items():
            print('%-34s %s' % (k, 'PASS' if v else 'FAIL'))
        b = replay['boundary']
        print('boundary: spawn_delta_m=%.3g trajectory_divergence_m=%s action_divergence_step=%s '
              'terminal(a/b)=%s/%s' % (replay['spawn_delta_m'], b['trajectory_divergence_m'],
                                        b['action_divergence_step'], b['terminal_a'], b['terminal_b']))
    print('\nreport -> %s' % os.path.join(out_dir, 'metrics', 'replay.json'))
    return 0 if (replay is None or replay['overall_pass']) else 1


if __name__ == '__main__':
    sys.exit(main())
