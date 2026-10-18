#!/usr/bin/env python3
"""evidence_archive —— S-08 证据不可变归档与入库 manifest。

为每个 experiment run 生成三件产物（**只新增，不覆盖既有 run 产物**）：

1. **入库 manifest** `experiments/<campaign>/<run_id>/config/evidence_manifest.json`
   —— 逐产物 `sha256` / 大小 / 类型 + 运行身份（`run_id` / `episode_id` / `git_commit` /
   `backend`）+ 生成器版本 + **访问位置**（归档路径）+ **保留期**。
2. **本地不可变归档** `experiments/_archive/<campaign>/<run_id>.tar` + 同名 `.tar.sha256`
   （只读 0444；归档文件哈希与取回验证记录见侧车文件）。
3. **跨路径取回验证**：归档解包到**另一路径**（临时目录）逐文件哈希比对，证明「**可取回**」，
   结果落 `experiments/_archive/<campaign>/<run_id>.retrieval.json`。

决策真源：[ADR-0064](../docs/adr/0064-evidence-immutable-archive-and-intake-manifest.md)；
产物纪律 [ADR-0007](../docs/adr/0007-experiment-artifacts-convention.md)；
运行身份 [ADR-0044](../docs/adr/0044-unique-run-identity-and-reproducible-evidence.md)。
R-030 证据丢失事实**只引用** [ADR-0045](../docs/adr/0045-resetfail-evidence-contradiction-registration-and-redo.md)，
本脚本**不**伪造 / **不**重建 / **不**冒充复核已丢失原件。

用法::

    python3 scripts/evidence_archive.py                 # 全部 run
    python3 scripts/evidence_archive.py --campaign w6-task-eval
    python3 scripts/evidence_archive.py --run-id 20261018-s06-geometry
    python3 scripts/evidence_archive.py --force         # 重建既有 manifest / 归档
    python3 scripts/evidence_archive.py --no-verify     # 跳过取回验证
    python3 scripts/evidence_archive.py --list          # 仅列出待处理 run
"""

import argparse
import datetime
import hashlib
import json
import os
import shutil
import sys
import tarfile
import tempfile

ROOT = '/home/chen/AGC'
EXPERIMENTS = os.path.join(ROOT, 'experiments')
ARCHIVE_ROOT = os.path.join(EXPERIMENTS, '_archive')

MANIFEST_REL = os.path.join('config', 'evidence_manifest.json')
GENERATOR = 'scripts/evidence_archive.py'
GENERATOR_VERSION = '1.0.0'
MANIFEST_VERSION = 1
RETENTION = '长期保留（≥ W8 对照冻结）；本地不可变归档（ADR-0064）'

KNOWN_TYPES = ('config', 'metrics', 'log', 'bag', 'figures')


def _now():
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def iter_artifacts(run_dir):
    """run_dir 下全部常规文件（相对路径，posix 分隔，排序），排除 manifest 自身。"""
    out = []
    for dirpath, dirnames, filenames in os.walk(run_dir, followlinks=False):
        dirnames.sort()
        for name in sorted(filenames):
            full = os.path.join(dirpath, name)
            if not os.path.isfile(full) or os.path.islink(full):
                continue
            rel = os.path.relpath(full, run_dir).replace(os.sep, '/')
            if rel == MANIFEST_REL.replace(os.sep, '/'):
                continue  # 自引用：manifest 不含自身哈希
            out.append(rel)
    out.sort()
    return out


def artifact_type(rel):
    top = rel.split('/', 1)[0]
    return top if top in KNOWN_TYPES else 'other'


def _read_text(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read().strip()


def _read_json(path):
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def read_identity(run_dir):
    cfg = os.path.join(run_dir, 'config')
    ep = _read_text(os.path.join(cfg, 'episode_id.txt'))
    episode_id = int(ep) if ep and ep.lstrip('-').isdigit() else (ep or None)
    commit = (_read_text(os.path.join(cfg, 'git_commit.txt')) or
              _read_text(os.path.join(cfg, 'git_head.txt')))
    scene = _read_json(os.path.join(cfg, 'scene_manifest.json'))
    backend = scene.get('backend') if isinstance(scene, dict) else None
    return {'episode_id': episode_id, 'git_commit': commit, 'backend': backend}


def build_manifest(run_dir, campaign, run_id):
    artifacts = []
    for rel in iter_artifacts(run_dir):
        full = os.path.join(run_dir, rel)
        artifacts.append({
            'path': rel,
            'type': artifact_type(rel),
            'size': os.path.getsize(full),
            'sha256': sha256_file(full),
        })
    ident = read_identity(run_dir)
    rel_archive = os.path.relpath(os.path.join(ARCHIVE_ROOT, campaign, run_id + '.tar'),
                                  ROOT).replace(os.sep, '/')
    return {
        'manifest_version': MANIFEST_VERSION,
        'generator': GENERATOR,
        'generator_version': GENERATOR_VERSION,
        'generated_at': _now(),
        'campaign': campaign,
        'run_id': run_id,
        'episode_id': ident['episode_id'],
        'git_commit': ident['git_commit'],
        'backend': ident['backend'],
        'retention': RETENTION,
        'self_excluded': MANIFEST_REL.replace(os.sep, '/'),
        'archive': {
            'path': rel_archive,
            'checksum_file': rel_archive + '.sha256',
            'retrieval_record': rel_archive + '.retrieval.json',
            'immutable': True,
            'note': '归档文件 sha256 与取回验证记录见 checksum_file / retrieval_record（侧车不在归档内）',
        },
        'artifacts': artifacts,
    }


def _unlock(path):
    """清除只读位并删除既有文件（归档侧车为 0444，--force 重建前须解锁）。"""
    if os.path.exists(path):
        try:
            os.chmod(path, 0o644)
        except OSError:
            pass
        os.remove(path)


def _tar_filter(ti):
    ti.uid = 0
    ti.gid = 0
    ti.uname = 'root'
    ti.gname = 'root'
    return ti


def pack_archive(run_dir, campaign, run_id):
    arch_dir = os.path.join(ARCHIVE_ROOT, campaign)
    os.makedirs(arch_dir, exist_ok=True)
    tar_path = os.path.join(arch_dir, run_id + '.tar')
    tmp_path = tar_path + '.tmp'
    _unlock(tmp_path)
    _unlock(tar_path)
    with tarfile.open(tmp_path, 'w') as tf:
        for rel in iter_artifacts(run_dir) + [MANIFEST_REL.replace(os.sep, '/')]:
            full = os.path.join(run_dir, rel)
            if os.path.exists(full):
                tf.add(full, arcname='%s/%s' % (run_id, rel), filter=_tar_filter)
    os.replace(tmp_path, tar_path)
    digest = sha256_file(tar_path)
    sha_path = tar_path + '.sha256'
    _unlock(sha_path)
    with open(sha_path, 'w', encoding='utf-8') as f:
        f.write('%s  %s\n' % (digest, os.path.basename(tar_path)))
    os.chmod(tar_path, 0o444)
    os.chmod(sha_path, 0o444)
    return tar_path, digest


def verify_retrieval(run_dir, tar_path, run_id):
    """解包到另一路径（临时目录）逐文件哈希比对，证明可取回。"""
    res = {'verified': False, 'method': 'extract_to_other_path_then_hash_compare',
           'verify_dir': None, 'files_checked': 0, 'mismatches': [], 'verified_at': _now()}
    tmp_root = tempfile.mkdtemp(prefix='evidence_retrieval_')
    try:
        with tarfile.open(tar_path, 'r') as tf:
            tf.extractall(tmp_root)
        res['verify_dir'] = os.path.join(tmp_root, run_id)
        expected = {rel: sha256_file(os.path.join(run_dir, rel))
                    for rel in iter_artifacts(run_dir)}
        expected[MANIFEST_REL.replace(os.sep, '/')] = sha256_file(
            os.path.join(run_dir, MANIFEST_REL))
        for rel, want in sorted(expected.items()):
            got_path = os.path.join(tmp_root, run_id, rel)
            if not os.path.isfile(got_path):
                res['mismatches'].append({'path': rel, 'reason': 'missing_in_archive'})
                continue
            got = sha256_file(got_path)
            res['files_checked'] += 1
            if got != want:
                res['mismatches'].append(
                    {'path': rel, 'reason': 'hash_mismatch', 'expected': want, 'actual': got})
        res['verified'] = not res['mismatches']
    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)
    return res


def list_runs(campaign_filter=None, run_filter=None):
    runs = []
    if not os.path.isdir(EXPERIMENTS):
        return runs
    for campaign in sorted(os.listdir(EXPERIMENTS)):
        cdir = os.path.join(EXPERIMENTS, campaign)
        if campaign == '_archive' or not os.path.isdir(cdir):
            continue
        if campaign_filter and campaign != campaign_filter:
            continue
        for run_id in sorted(os.listdir(cdir)):
            rdir = os.path.join(cdir, run_id)
            if not os.path.isdir(rdir):
                continue
            if run_filter and run_id != run_filter:
                continue
            runs.append((campaign, run_id, rdir))
    return runs


def process_run(campaign, run_id, run_dir, force=False, verify=True):
    manifest_path = os.path.join(run_dir, MANIFEST_REL)
    tar_path = os.path.join(ARCHIVE_ROOT, campaign, run_id + '.tar')
    already = os.path.exists(manifest_path) and os.path.exists(tar_path)
    if already and not force:
        return {'run_id': run_id, 'campaign': campaign, 'status': 'skipped_existing'}

    manifest = build_manifest(run_dir, campaign, run_id)
    os.makedirs(os.path.dirname(manifest_path), exist_ok=True)
    _unlock(manifest_path)
    with open(manifest_path, 'w', encoding='utf-8') as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
        f.write('\n')

    tar_path, digest = pack_archive(run_dir, campaign, run_id)

    retrieval = None
    if verify:
        retrieval = verify_retrieval(run_dir, tar_path, run_id)
        retrieval['archive'] = os.path.relpath(tar_path, ROOT).replace(os.sep, '/')
        retrieval['archive_sha256'] = digest
        _unlock(tar_path + '.retrieval.json')
        with open(tar_path + '.retrieval.json', 'w', encoding='utf-8') as f:
            json.dump(retrieval, f, indent=2, ensure_ascii=False)
            f.write('\n')
        os.chmod(tar_path + '.retrieval.json', 0o444)
    return {
        'run_id': run_id, 'campaign': campaign, 'status': 'archived',
        'artifacts': len(manifest['artifacts']),
        'archive': os.path.relpath(tar_path, ROOT).replace(os.sep, '/'),
        'archive_sha256': digest,
        'retrieval_verified': (retrieval or {}).get('verified'),
        'retrieval_files': (retrieval or {}).get('files_checked'),
        'retrieval_mismatches': (retrieval or {}).get('mismatches', []),
    }


def main():
    ap = argparse.ArgumentParser(description='证据不可变归档与入库 manifest（S-08）')
    ap.add_argument('--campaign', default=None, help='仅处理该 campaign')
    ap.add_argument('--run-id', default=None, help='仅处理该 run_id')
    ap.add_argument('--force', action='store_true', help='重建既有 manifest / 归档')
    ap.add_argument('--no-verify', action='store_true', help='跳过跨路径取回验证')
    ap.add_argument('--list', action='store_true', help='仅列出待处理 run')
    args = ap.parse_args()

    runs = list_runs(args.campaign, args.run_id)
    if args.list:
        for c, r, _ in runs:
            print('%s/%s' % (c, r))
        print('total=%d' % len(runs))
        return 0

    results = []
    for campaign, run_id, run_dir in runs:
        try:
            res = process_run(campaign, run_id, run_dir,
                              force=args.force, verify=not args.no_verify)
        except Exception as exc:  # noqa: BLE001
            res = {'campaign': campaign, 'run_id': run_id,
                   'status': 'error', 'error': repr(exc)}
        results.append(res)
        if res.get('status') == 'error':
            print('ERROR %s/%s: %s' % (campaign, run_id, res['error']))
        elif res.get('status') == 'skipped_existing':
            print('skip   %s/%s (既有 manifest / 归档，未覆盖；--force 可重建)' %
                  (campaign, run_id))
        else:
            print('ok     %s/%s artifacts=%d verify=%s files=%s mismatches=%s' % (
                campaign, run_id, res['artifacts'], res['retrieval_verified'],
                res['retrieval_files'], len(res['retrieval_mismatches'])))

    archived = [r for r in results if r.get('status') == 'archived']
    bad = [r for r in archived if not r.get('retrieval_verified')]
    print('\narchived=%d skipped=%d error=%d retrieval_failed=%d' % (
        len(archived), sum(1 for r in results if r.get('status') == 'skipped_existing'),
        sum(1 for r in results if r.get('status') == 'error'), len(bad)))
    return 1 if bad or any(r.get('status') == 'error' for r in results) else 0


if __name__ == '__main__':
    sys.exit(main())
