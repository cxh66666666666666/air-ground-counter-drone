#!/usr/bin/env bash
# run_m0 —— M0 最小端到端闭环入口（唯一跑法）
#   用法：scripts/archive/run_m0.sh [launch args...]
#   例：  scripts/archive/run_m0.sh                       # GUI
#         scripts/archive/run_m0.sh gz_args:="-s -r -v 3 <world>"   # 无头
set -euo pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"

exec ros2 launch agc_task m0_skeleton.launch.py "$@"