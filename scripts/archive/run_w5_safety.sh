#!/usr/bin/env bash
# run_w5_safety —— W5-c R9/R4/R5 场景一键跑（launch 背景 + 检查 + 清理）
#   用法：scripts/archive/run_w5_safety.sh [headless|gui] [run_id]
#   例：  scripts/archive/run_w5_safety.sh headless
#         scripts/archive/run_w5_safety.sh gui 20261013-pole-approach-intro
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"

MODE="${1:-headless}"
RUN_ID="${2:-$(date +%Y%m%d)-pole-approach-0}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"

if [ "${MODE}" = "gui" ]; then
  GZ="-r -v 3 ${WORLD}"
else
  GZ="-s -r -v 3 ${WORLD}"
fi

OUT="${ROOT}/experiments/w5-ground-nav-safety/${RUN_ID}"
mkdir -p "${OUT}/log"

setsid ros2 launch agc_task w5_safety_scenario.launch.py gz_args:="${GZ}" \
  > "${OUT}/log/launch.log" 2>&1 &
PGID=$!
TAG="w5_safety_scenario.launch.py"
cleanup() {
  kill -TERM -"${PGID}" 2>/dev/null
  pkill -TERM -f "${TAG}" 2>/dev/null
  pkill -TERM -f "agc_world.sdf" 2>/dev/null
  sleep 3
  kill -KILL -"${PGID}" 2>/dev/null
  pkill -KILL -f "${TAG}" 2>/dev/null
  pkill -KILL -f "agc_world.sdf" 2>/dev/null
}
trap cleanup EXIT

echo "launch up (mode=${MODE}, pgid=${PGID}); 等待链路就绪 18 s ..."
sleep 18
python3 "${ROOT}/tests/w5_safety_check.py" --duration 30 --run-id "${RUN_ID}"
RC=$?
echo "safety check rc=${RC}"
exit "${RC}"
