#!/usr/bin/env bash
# run_w7_core —— W7-a 核心库（T1 判定 / T2 obs 版本化 / T3 种子分解）一键核查。
#   用法：scripts/archive/run_w7_core.sh [run_id]
#   纯库 / 纯 Python（**无需仿真**）：调 tests/w7_core_check.py，证据落 experiments/w7-rl/<run_id>/。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"

RUN_ID="${1:-$(date +%Y%m%d)-w7-core-$(date +%H%M%S)-$(printf '%04d' $((RANDOM % 10000)))}"
echo "W7-a 核心库核查：run_id=${RUN_ID}"
python3 "${ROOT}/tests/w7_core_check.py" --run-id "${RUN_ID}"
