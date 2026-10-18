#!/usr/bin/env bash
# run_w6_intruder —— W6-a R4（目标 driver 三种运动）命令化验收（逐类型起 driver + 检查 + 清理）
#   用法：scripts/archive/run_w6_intruder.sh [run_id]
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"

RUN_ID="${1:-$(date +%Y%m%d)-w6-intruder-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

RC=0
for m in static linear maneuver; do
  echo "=== motion=$m ==="
  setsid ros2 run agc_task intruder_driver --ros-args \
    -p px4_ns:=px4_1 -p motion:="${m}" -p speed_mps:=1.0 -p radius_m:=6.0 \
    > "${OUT}/log/intruder_${m}.log" 2>&1 &
  DPID=$!
  sleep 2
  python3 "${ROOT}/tests/w6_intruder_check.py" --motion "${m}" --run-id "${RUN_ID}" || RC=1
  kill -9 "${DPID}" 2>/dev/null
  pkill -9 -f "intruder_driver" 2>/dev/null
  sleep 1
done
echo "w6 intruder check rc=${RC}"
exit "${RC}"
