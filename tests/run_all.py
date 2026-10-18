#!/usr/bin/env python3
"""tests 验收核查的薄 runner。

约定：核查脚本退出码 0 = PASS、非 0 = FAIL（部分需拉起仿真 / SITL，故默认不接 CI）。

用法：
  python3 tests/run_all.py                       # 列出全部
  python3 tests/run_all.py --run w6_r034_check.py
  python3 tests/run_all.py --all --timeout 600
"""

import argparse
import subprocess
import sys
from pathlib import Path

TESTS = Path(__file__).resolve().parent


def discover() -> list[str]:
    return sorted(p.name for p in TESTS.glob("*.py") if p.name != "run_all.py")


def main() -> int:
    ap = argparse.ArgumentParser(description="tests 验收核查薄 runner")
    ap.add_argument("--run", nargs="+", metavar="NAME", help="运行指定核查")
    ap.add_argument("--all", action="store_true", help="运行全部核查")
    ap.add_argument("--timeout", type=int, default=600, help="单个核查超时秒数（默认 600）")
    args = ap.parse_args()

    tests = discover()
    if args.all:
        targets = tests
    elif args.run:
        unknown = [n for n in args.run if n not in tests]
        if unknown:
            print("未找到:", ", ".join(unknown))
            return 2
        targets = args.run
    else:
        for n in tests:
            print(n)
        print(f"\n共 {len(tests)} 个。用 --run NAME... 或 --all 运行。")
        return 0

    rc = 0
    for n in targets:
        print(f"\n=== {n} ===")
        try:
            p = subprocess.run([sys.executable, str(TESTS / n)], timeout=args.timeout)
        except subprocess.TimeoutExpired:
            rc = 1
            print(f"[TIMEOUT] {n}（>{args.timeout}s）")
            continue
        if p.returncode != 0:
            rc = 1
            print(f"[FAIL] {n} (exit {p.returncode})")
    return rc


if __name__ == "__main__":
    sys.exit(main())
