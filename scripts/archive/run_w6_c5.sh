#!/usr/bin/env bash
# run_w6_c5 —— W6-a R3/C5（相位复位触发）轻量验收：单独起 trajectory_tracker（无 PX4）+ 检查 + 清理
#   用法：scripts/run_w6_c5.sh [run_id]
#   例：  scripts/run_w6_c5.sh 20261014-w6-c5-0
#   说明：走**墙钟**（不设 use_sim_time），以保证相位随真实时间推进。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"

RUN_ID="${1:-$(date +%Y%m%d)-w6-c5-0}"
PARAMS="${ROOT}/install/agc_air/share/agc_air/config/air.yaml"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

setsid ros2 run agc_air trajectory_tracker \
  --ros-args --params-file "${PARAMS}" \
  > "${OUT}/log/tracker.log" 2>&1 &
PGID=$!
cleanup() {
  kill -TERM -"${PGID}" 2>/dev/null
  pkill -TERM -f "agc_air/trajectory_tracker" 2>/dev/null
  sleep 2
  kill -KILL -"${PGID}" 2>/dev/null
  pkill -KILL -f "agc_air/trajectory_tracker" 2>/dev/null
}
trap cleanup EXIT

sleep 2
python3 "${ROOT}/scripts/w6_c5_check.py" --run-id "${RUN_ID}"
RC=$?
echo "w6 c5 check rc=${RC}"
exit "${RC}"
