#!/usr/bin/env python3
"""third_party_asset_check —— S-12（外部资产来源 / 许可 / 哈希清单）一致性核查（**只读**）。

真源：`docs/conventions/第三方资产与许可.md`（政策 + 人读表 + 机器可读 ```yaml 块，
键 `agc_third_party_assets:`，`schema: 1`）。本脚本解析该清单并逐项记 PASS / FAIL：

  (1) 清单存在且含可解析的 `agc_third_party_assets` yaml 块，`schema == 1`。
  (2) 逐资产：`path` 存在；`sha256` == **实测**；`upstream_license` 非空且 **!= `Proprietary`**
      （许可**分列**、**不外推**）；`license_status` ∈ {`resolved`,`unresolved`}，
      且 `unresolved` 时 `license_note` 非空（**据实登记未决**，**不**静默断言）。
  (3) 每个 `asset_dirs` 条目：清单内该目录下匹配 `include` 的资产集合 == 目录内**实际**文件集合
      （**双向**比对 → 捕获「未列出的资产」与「清单虚构的资产」）。

环境无关：**不** source ROS，只读源码 / 文档与文件系统。
用法：`python3 scripts/third_party_asset_check.py`；`--root` 可覆盖默认项目根 `/home/chen/AGC`。
任一 FAIL -> `ok=false` -> 退出码 1；全 PASS -> 0。
"""

import argparse
import fnmatch
import hashlib
import os
import re
import sys

try:
    import yaml
except ImportError:  # pragma: no cover - 环境缺 pyyaml 时给出清晰错误
    sys.stderr.write('third_party_asset_check: 缺少 PyYAML（pip install pyyaml）\n')
    sys.exit(2)

ROOT = '/home/chen/AGC'
MANIFEST = os.path.join('docs', 'conventions', '第三方资产与许可.md')
MANIFEST_KEY = 'agc_third_party_assets'
OWN_LICENSE = 'Proprietary'  # 自有包许可；不得外推到第三方资产

FENCE_RE = re.compile(r'```(?:ya?ml)\n(.*?)```', re.DOTALL)


def _check(checks, name, status, expected=None, actual=None, reason=''):
    checks.append({
        'check': name,
        'status': status,
        'expected': expected,
        'actual': actual,
        'reason': reason,
    })


def _read_text(path):
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


def _sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def load_manifest(checks, root):
    """(1) 清单存在且含可解析的 `agc_third_party_assets` yaml 块。"""
    p = os.path.join(root, MANIFEST)
    if not os.path.isfile(p):
        _check(checks, 'manifest:exists', 'FAIL', MANIFEST, None, 'file_missing')
        return None
    _check(checks, 'manifest:exists', 'PASS', MANIFEST, 'exists', '')

    data = None
    for block in FENCE_RE.findall(_read_text(p)):
        try:
            doc = yaml.safe_load(block)
        except yaml.YAMLError:
            continue
        if isinstance(doc, dict) and MANIFEST_KEY in doc:
            data = doc[MANIFEST_KEY]
            break
    if not isinstance(data, dict):
        _check(checks, 'manifest:yaml_block', 'FAIL',
               "yaml 块含 %s" % MANIFEST_KEY, None, 'not_found_or_unparsable')
        return None
    _check(checks, 'manifest:yaml_block', 'PASS', MANIFEST_KEY, 'found', '')

    schema = data.get('schema')
    if schema == 1:
        _check(checks, 'manifest:schema', 'PASS', 1, schema, '')
    else:
        _check(checks, 'manifest:schema', 'FAIL', 1, schema, 'schema_mismatch')
    return data


def check_policy(checks, data):
    """政策：自有包许可与第三方分列。"""
    own = (data.get('policy') or {}).get('own_packages_license', OWN_LICENSE)
    _check(checks, 'policy:own_license', 'PASS', OWN_LICENSE, own, '')


def check_assets(checks, data, root):
    """(2) 逐资产：存在性 + 实测哈希 + 许可分列（非 Proprietary）+ 未决需理由。"""
    assets = data.get('assets') or []
    if not assets:
        _check(checks, 'assets:nonempty', 'FAIL', '>=1 asset', 0, 'empty')
        return
    _check(checks, 'assets:nonempty', 'PASS', '>=1 asset', len(assets), '')

    for a in assets:
        aid = a.get('id', '<no-id>')
        rel = a.get('path', '')
        p = os.path.join(root, rel)

        if rel and os.path.isfile(p):
            _check(checks, 'asset:exists:%s' % aid, 'PASS', rel, 'exists', '')
        else:
            _check(checks, 'asset:exists:%s' % aid, 'FAIL', rel, None, 'file_missing')
            continue

        expect = (a.get('sha256') or '').strip().lower()
        actual = _sha256(p)
        if expect and expect == actual:
            _check(checks, 'asset:sha256:%s' % aid, 'PASS', expect, actual, '')
        else:
            _check(checks, 'asset:sha256:%s' % aid, 'FAIL', expect, actual, 'hash_mismatch')

        lic = (a.get('upstream_license') or '').strip()
        if not lic:
            _check(checks, 'asset:license:%s' % aid, 'FAIL', 'non-empty', None, 'empty')
        elif lic.lower() == OWN_LICENSE.lower():
            _check(checks, 'asset:license:%s' % aid, 'FAIL', '!= %s' % OWN_LICENSE, lic,
                   'own_license_extrapolated')
        else:
            _check(checks, 'asset:license:%s' % aid, 'PASS', '!= %s' % OWN_LICENSE, lic, '')

        status = (a.get('license_status') or '').strip()
        if status not in ('resolved', 'unresolved'):
            _check(checks, 'asset:license_status:%s' % aid, 'FAIL',
                   'resolved|unresolved', status, 'invalid_status')
        elif status == 'unresolved' and not (a.get('license_note') or '').strip():
            _check(checks, 'asset:license_status:%s' % aid, 'FAIL',
                   'unresolved 须有 license_note', status, 'unresolved_without_note')
        else:
            _check(checks, 'asset:license_status:%s' % aid, 'PASS', status, status, '')


def check_dirs(checks, data, root):
    """(3) 每个 asset_dirs 条目：清单集合 == 目录实际集合（双向）。"""
    for d in data.get('asset_dirs') or []:
        rel_dir = d.get('dir', '')
        globs = d.get('include') or []
        name = 'dirs:%s' % rel_dir

        listed = set()
        for a in data.get('assets') or []:
            rp = a.get('path', '')
            if os.path.dirname(rp) == rel_dir:
                base = os.path.basename(rp)
                if any(fnmatch.fnmatch(base, g) for g in globs):
                    listed.add(base)

        actual = set()
        adir = os.path.join(root, rel_dir)
        if os.path.isdir(adir):
            for fn in sorted(os.listdir(adir)):
                if os.path.isfile(os.path.join(adir, fn)) and any(
                        fnmatch.fnmatch(fn, g) for g in globs):
                    actual.add(fn)

        extra = sorted(actual - listed)      # 目录里存在但清单未列 -> 未登记资产
        absent = sorted(listed - actual)     # 清单列了但目录里没有
        if extra or absent:
            _check(checks, name, 'FAIL', 'listed == actual',
                   'unlisted=%s missing=%s' % (extra, absent), 'set_mismatch')
        else:
            _check(checks, name, 'PASS', 'listed == actual', '%d files' % len(actual), '')


def main():
    ap = argparse.ArgumentParser(description='外部资产来源 / 许可 / 哈希清单核查（只读）')
    ap.add_argument('--root', default=ROOT, help='项目根（默认 %s）' % ROOT)
    args = ap.parse_args()

    checks = []
    data = load_manifest(checks, args.root)
    if data is not None:
        check_policy(checks, data)
        check_assets(checks, data, args.root)
        check_dirs(checks, data, args.root)

    summary = {
        'pass': sum(1 for c in checks if c['status'] == 'PASS'),
        'fail': sum(1 for c in checks if c['status'] == 'FAIL'),
    }
    ok = summary['fail'] == 0

    print('third_party_asset_check —— S-12 外部资产来源 / 许可 / 哈希清单核查（只读）')
    print('  root = %s' % args.root)
    print('  manifest = %s' % MANIFEST)
    for c in checks:
        line = '[%s] %s' % (c['status'], c['check'])
        if c['status'] == 'FAIL' and c['reason']:
            line += '  (%s)' % c['reason']
        print('  ' + line)
    print('  summary: PASS=%d FAIL=%d -> ok=%s'
          % (summary['pass'], summary['fail'], 'true' if ok else 'false'))

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
