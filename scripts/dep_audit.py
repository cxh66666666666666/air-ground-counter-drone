#!/usr/bin/env python3
"""dep_audit —— 依赖声明 lint（评审 S-01「声明 ⊇ 真实直接使用」前置）。

对 `src/agc_*` 各包核对：
  1. `CMakeLists.txt` 的 `find_package(X)` → 必须在 `package.xml` 声明（含别名映射）；
  2. `launch/*.py` 与 `setup.py` 的 Python import（`launch` / `launch_ros` /
     `ament_index_python` / `yaml` / `rclpy` / `xacro`）→ 必须声明 `exec_depend`。

**仅**作前置（不足以证明可重建）；隔离最小环境验证另计。用法：`python3 scripts/dep_audit.py`。
"""

import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'src')

# `find_package` 名 → 声明键（rosdep / vendor 包）
FIND_ALIAS = {
    'Eigen3': 'eigen',
    'gz-sim8': 'gz_sim_vendor',
    'gz-plugin2': 'gz_plugin_vendor',
    'gz-transport13': 'gz_transport_vendor',
    'gz-msgs10': 'gz_msgs_vendor',
}

# Python 顶层模块名 → 声明键（exec）
PY_ALIAS = {
    'yaml': 'python3-yaml',
}

# 不要求声明为依赖的构建工具（buildtool / test 工具，另有 test_depend）
SKIP_FIND = {'ament_cmake', 'rosidl_default_generators'}


def declared_deps(pkg_xml):
    keys = set()
    tree = ET.parse(pkg_xml)
    for tag in ('depend', 'build_depend', 'build_export_depend', 'exec_depend',
                'test_depend', 'buildtool_depend', 'doc_depend', 'build_export_depend'):
        for el in tree.getroot().findall(tag):
            keys.add((el.text or '').strip())
    return keys


def find_packages(cmake):
    names = set()
    with open(cmake) as f:
        text = f.read()
    for m in re.finditer(r'find_package\(\s*([A-Za-z0-9_\-]+)', text):
        names.add(m.group(1))
    return names


def py_imports(pkg_dir):
    mods = set()
    files = glob.glob(os.path.join(pkg_dir, 'launch', '*.py'))
    files += glob.glob(os.path.join(pkg_dir, '**', '*.py'), recursive=True)
    files += [os.path.join(pkg_dir, 'setup.py')]
    for p in files:
        if not os.path.isfile(p):
            continue
        with open(p) as f:
            for line in f:
                m = re.match(r'\s*(?:import|from)\s+([A-Za-z_][A-Za-z0-9_\.]*)', line)
                if not m:
                    continue
                mods.add(m.group(1).split('.')[0])
    return mods


def main():
    problems = []
    pkgs = sorted(d for d in os.listdir(SRC)
                  if d.startswith('agc_') and os.path.isfile(os.path.join(SRC, d, 'package.xml')))
    for pkg in pkgs:
        pdir = os.path.join(SRC, pkg)
        deps = declared_deps(os.path.join(pdir, 'package.xml'))
        cmake = os.path.join(pdir, 'CMakeLists.txt')
        if os.path.isfile(cmake):
            for name in find_packages(cmake):
                if name in SKIP_FIND:
                    continue
                key = FIND_ALIAS.get(name, name)
                if key not in deps:
                    problems.append('%s: find_package(%s) → 未声明 %s' % (pkg, name, key))
        for mod in py_imports(pdir):
            key = PY_ALIAS.get(mod, mod)
            if key in ('launch', 'launch_ros', 'ament_index_python', 'rclpy', 'xacro',
                       'python3-yaml'):
                if key not in deps:
                    problems.append('%s: python import %s → 未声明 exec_depend %s' % (pkg, mod, key))
    if problems:
        print('依赖 lint 发现问题（声明 ⊉ 直接使用）：')
        for p in problems:
            print('  - ' + p)
        return 1
    print('依赖 lint 通过：%d 个 agc 包的「声明 ⊇ 直接使用」' % len(pkgs))
    return 0


if __name__ == '__main__':
    sys.exit(main())
