#!/usr/bin/env python3
"""w7b_build_offline_dataset —— W7-b T5（E5 / F-16）离线数据集 / 回放库构建。

从**既有**可复现 run 产物（默认 W7-a 的 `experiments/w7-rl/*/episodes/ep_a*.json`，含 spawn /
动作序列 / 位姿轨迹 / 种子计划 / 终态）聚合为**离线数据集索引**，条目带身份与版本，**可重建**。

产物（ADR-0007 唯一落点；`run_id` 不可覆盖）：
  `experiments/w7-rl-b/<run_id>/{config/dataset_manifest.json, metrics/dataset.json}`
`dataset_manifest.json` 记：来源 campaign、条目（source run_id / attempt / scene / master_seed /
steps / spawn / terminal / outcome / 文件 sha256）、字段版本、可重建命令。

**范围声明**：本数据集为**离线回放 / 离线学习**用（F-16）；**不**新增任何命令者，**不**替代在线执行链。

用法：
  python3 scripts/w7b_build_offline_dataset.py --sources w7-rl --campaign w7-rl-b
"""

import argparse
import glob
import hashlib
import json
import os
import random
import sys
from datetime import datetime

ROOT = '/home/chen/AGC'
DATASET_VERSION = 'offline_traj_v1'


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sources', default='w7-rl',
                    help='来源 campaign（逗号分隔；默认 w7-rl = W7-a 可复现 run）')
    ap.add_argument('--campaign', default='w7-rl-b')
    ap.add_argument('--run-id', default=None)
    args = ap.parse_args()

    run_id = args.run_id or ('%s-%s-%s-%04d' % (
        datetime.now().strftime('%Y%m%d'), args.campaign,
        datetime.now().strftime('%H%M%S'), random.randint(0, 9999)))
    out_dir = os.path.join(ROOT, 'experiments', args.campaign, run_id)
    if os.path.exists(out_dir):
        print('拒绝写入已有 run 目录（run_id 不可覆盖）：%s' % out_dir)
        return 2
    for sub in ('config', 'metrics'):
        os.makedirs(os.path.join(out_dir, sub), exist_ok=True)

    sources = [s.strip() for s in args.sources.split(',') if s.strip()]
    entries = []
    src_files = []
    for camp in sources:
        pattern = os.path.join(ROOT, 'experiments', camp, '*', 'episodes', 'ep_a*.json')
        for path in sorted(glob.glob(pattern)):
            try:
                with open(path) as f:
                    ep = json.load(f)
            except (OSError, json.JSONDecodeError):
                continue
            entries.append({
                'source_campaign': camp,
                'source_file': os.path.relpath(path, ROOT),
                'sha256': sha256_file(path),
                'scene': ep.get('scene'), 'master_seed': ep.get('master_seed'),
                'attempt_index': ep.get('attempt_index'),
                'steps': ep.get('steps'), 'spawn': ep.get('spawn'),
                'terminal': ep.get('terminal'),
                'outcome_class': (ep.get('outcome') or {}).get('outcome_class'),
                'seed_plan': ep.get('seed_plan'),
                'n_actions': len(ep.get('actions') or []),
                'n_poses': len(ep.get('poses') or []),
                'noise_seq_head': (ep.get('noise_seq') or [])[:5],
            })
            src_files.append(os.path.relpath(path, ROOT))

    git_commit = os.popen('git -C "%s" rev-parse HEAD 2>/dev/null' % ROOT).read().strip() or 'unknown'
    manifest = {
        'dataset_version': DATASET_VERSION,
        'campaign': args.campaign, 'run_id': run_id,
        'sources': sources, 'n_entries': len(entries),
        'git_commit': git_commit,
        'fields': ['source_file', 'sha256', 'scene', 'master_seed', 'attempt_index',
                   'steps', 'spawn', 'terminal', 'outcome_class', 'seed_plan',
                   'n_actions', 'n_poses', 'noise_seq_head'],
        'rebuild_command': 'python3 scripts/w7b_build_offline_dataset.py --sources %s' % ','.join(sources),
        'entries': entries,
        'scope_note': '离线回放 / 离线学习数据集（F-16）；**不**新增命令者，**不**替代在线执行链。',
    }
    with open(os.path.join(out_dir, 'config', 'dataset_manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)

    dataset = {
        'run_id': run_id, 'dataset_version': DATASET_VERSION, 'n_entries': len(entries),
        'sources': sources, 'source_files': src_files,
        'scenes': sorted({e['scene'] for e in entries if e['scene']}),
        'terminals': sorted({e['terminal'] for e in entries if e['terminal']}),
        'total_steps': sum(int(e['steps'] or 0) for e in entries),
        'manifest': 'config/dataset_manifest.json',
        'rebuildable': True,
        'proposition': 'E5 / F-16：离线数据集可由既有可复现 run 产物**重建**，条目带 run/文件身份与字段版本。',
    }
    with open(os.path.join(out_dir, 'metrics', 'dataset.json'), 'w') as f:
        json.dump(dataset, f, indent=2, ensure_ascii=False)

    print('entries=%d scenes=%s terminals=%s total_steps=%d' % (
        len(entries), dataset['scenes'], dataset['terminals'], dataset['total_steps']))
    print('-> %s' % os.path.join(out_dir, 'config', 'dataset_manifest.json'))
    return 0 if entries else 1


if __name__ == '__main__':
    sys.exit(main())
