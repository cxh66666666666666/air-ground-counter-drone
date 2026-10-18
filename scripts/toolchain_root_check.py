#!/usr/bin/env python3
"""toolchain_root_check —— S-11（工具链根显式注入 + 启动前验证）一致性核查（**只读**）。

真源：`AGENTS.md`「项目根 /home/chen/AGC」+ 契约（工具链在其下 `toolchain/`）。本脚本核查
两处 W6 入口的**根注入方式**与工具链二进制的**存在性**，逐项记 PASS / FAIL：

  静态检查（源码正则，只读）：
    (1) `src/agc_task/launch/w6.launch.py` 与 `src/agc_air/launch/air_bringup.launch.py`
        **不得**出现基于 `parents[...]` 的项目根深度上溯；命中 -> FAIL。
    (2) 两文件**均须**声明 launch 参数 `agc_root`，默认值 `/home/chen/AGC`；缺失 -> FAIL。

  功能检查（文件系统）：
    (3) `<agc_root>/toolchain/bin/MicroXRCEAgent` 与
        `<agc_root>/toolchain/PX4-Autopilot/build/px4_sitl_default/bin/px4` 存在；缺失 -> FAIL。

环境无关：**不** source ROS，只读源码与文件。用法：`python3 scripts/toolchain_root_check.py`；
`--agc-root` 可覆盖默认根 `/home/chen/AGC`。任一 FAIL -> 退出码 1；全 PASS -> 0。
"""

import argparse
import os
import re
import sys

ROOT = '/home/chen/AGC'
DEFAULT_AGC_ROOT = '/home/chen/AGC'

# 待核查的两处 W6 顶层入口（相对项目根）。
LAUNCH_FILES = [
    os.path.join('src', 'agc_task', 'launch', 'w6.launch.py'),
    os.path.join('src', 'agc_air', 'launch', 'air_bringup.launch.py'),
]

# 项目根上溯（深度索引）模式：`parents[3]` 之类。
PARENTS_RE = re.compile(r'parents\s*\[\s*\d')
# `agc_root` launch 参数声明：DeclareLaunchArgument('agc_root' ...)。
DECLARE_AGC_ROOT_RE = re.compile(r"""DeclareLaunchArgument\(\s*['"]agc_root['"]""")
# 声明处默认值须为固定项目根。
DEFAULT_ROOT_RE = re.compile(re.escape(DEFAULT_AGC_ROOT))


def _check(checks, name, status, expected=None, actual=None, reason=''):
    checks.append({
        'check': name,
        'status': status,
        'expected': expected,
        'actual': actual,
        'reason': reason,
    })


def _read(path):
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


def check_no_parents(checks, root):
    """(1) 两入口不得以 `parents[...]` 上溯推断项目根。"""
    for rel in LAUNCH_FILES:
        p = os.path.join(root, rel)
        if not os.path.isfile(p):
            _check(checks, 'no_parents:%s' % rel, 'FAIL', 'no parents[]', None, 'file_missing')
            continue
        hits = [m.group(0) for m in PARENTS_RE.finditer(_read(p))]
        if hits:
            _check(checks, 'no_parents:%s' % rel, 'FAIL', 'no parents[]', hits,
                   'parents_based_root')
        else:
            _check(checks, 'no_parents:%s' % rel, 'PASS', 'no parents[]', 'none', '')


def check_agc_root_declared(checks, root):
    """(2) 两入口均须声明 `agc_root`，默认值 /home/chen/AGC。"""
    for rel in LAUNCH_FILES:
        p = os.path.join(root, rel)
        if not os.path.isfile(p):
            _check(checks, 'agc_root:%s' % rel, 'FAIL', 'declared', None, 'file_missing')
            continue
        text = _read(p)
        m = DECLARE_AGC_ROOT_RE.search(text)
        if not m:
            _check(checks, 'agc_root:%s' % rel, 'FAIL',
                   "DeclareLaunchArgument('agc_root')", None, 'not_declared')
            continue
        # 取该声明所在行及其后 3 行判默认值（声明可能跨行书写）。
        lines = text.split('\n')
        start = text.count('\n', 0, m.start())
        window = '\n'.join(lines[start:start + 4])
        if DEFAULT_ROOT_RE.search(window):
            _check(checks, 'agc_root:%s' % rel, 'PASS', DEFAULT_AGC_ROOT, DEFAULT_AGC_ROOT, '')
        else:
            _check(checks, 'agc_root:%s' % rel, 'FAIL', DEFAULT_AGC_ROOT, window.strip(),
                   'default_missing')


def check_toolchain(checks, agc_root):
    """(3) 工具链二进制存在（PX4 / uXRCE Agent）。"""
    required = [
        os.path.join(agc_root, 'toolchain', 'bin', 'MicroXRCEAgent'),
        os.path.join(agc_root, 'toolchain', 'PX4-Autopilot', 'build',
                     'px4_sitl_default', 'bin', 'px4'),
    ]
    for p in required:
        name = 'toolchain:%s' % os.path.relpath(p, agc_root)
        if os.path.isfile(p):
            _check(checks, name, 'PASS', p, 'exists', '')
        else:
            _check(checks, name, 'FAIL', p, None, 'missing')


def main():
    ap = argparse.ArgumentParser(description='工具链根显式注入核查（只读）')
    ap.add_argument('--root', default=ROOT, help='项目根（默认 %s）' % ROOT)
    ap.add_argument('--agc-root', default=DEFAULT_AGC_ROOT,
                    help='工具链所在固定项目根（默认 %s）' % DEFAULT_AGC_ROOT)
    args = ap.parse_args()

    checks = []
    check_no_parents(checks, args.root)
    check_agc_root_declared(checks, args.root)
    check_toolchain(checks, args.agc_root)

    summary = {
        'pass': sum(1 for c in checks if c['status'] == 'PASS'),
        'fail': sum(1 for c in checks if c['status'] == 'FAIL'),
    }
    ok = summary['fail'] == 0

    print('toolchain_root_check —— S-11 工具链根显式注入核查（只读）')
    print('  agc_root = %s' % args.agc_root)
    for c in checks:
        line = '[%s] %s' % (c['status'], c['check'])
        if c['status'] == 'FAIL' and c['reason']:
            line += '  (%s)' % c['reason']
        print('  ' + line)
    print('  summary: PASS=%d FAIL=%d -> %s'
          % (summary['pass'], summary['fail'], 'OK' if ok else 'FAIL'))

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
