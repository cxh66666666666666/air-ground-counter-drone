#!/usr/bin/env bash
# run_w6_r6 —— W6-b R6（出生净空 / seed 可复现）一键验收（launch 背景 + 检查 + 清理）
#   用法：scripts/archive/run_w6_r6.sh [headless|gui] [run_id] [params_file] [--expect-reject]
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"

MODE="${1:-headless}"
RUN_ID="${2:-$(date +%Y%m%d)-w6-r6-0}"
PARAMS_FILE="${3:-${ROOT}/src/agc_task/config/w6_r6_random.yaml}"
EXPECT="${4:-}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"

if [ "${MODE}" = "gui" ]; then GZ="-r -v 3 ${WORLD}"; else GZ="-s -r -v 3 ${WORLD}"; fi

OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

setsid ros2 launch agc_task m0_skeleton.launch.py \
  gz_args:="${GZ}" params_file:="${PARAMS_FILE}" \
  > "${OUT}/log/launch.log" 2>&1 &
PGID=$!
TAG="m0_skeleton.launch.py"
cleanup() {
  kill -INT -"${PGID}" 2>/dev/null; pkill -INT -f "${TAG}" 2>/dev/null
  pkill -INT -f "agc_world.sdf" 2>/dev/null
  sleep 4
  kill -TERM -"${PGID}" 2>/dev/null; pkill -TERM -f "${TAG}" 2>/dev/null
  pkill -TERM -f "agc_world.sdf" 2>/dev/null
  sleep 2
  kill -KILL -"${PGID}" 2>/dev/null; pkill -KILL -f "${TAG}" 2>/dev/null
  pkill -KILL -f "agc_world.sdf" 2>/dev/null
}
trap cleanup EXIT

echo "launch up (run_id=${RUN_ID}); 等待就绪 18 s ..."
sleep 18
python3 "${ROOT}/tests/w6_r6_check.py" --run-id "${RUN_ID}" ${EXPECT}
RC=$?
echo "w6 r6 check rc=${RC}"
exit "${RC}"
