#!/usr/bin/env bash
# run_w6_full —— W6 双 PX4 整链一键验收（launch w6.launch.py backend:=full + 机身层检查 + 清理）
#   用法：scripts/archive/run_w6_full.sh <scene> <motion> <run_id> [--jam]
#   例：  scripts/archive/run_w6_full.sh s02_linear_none_nominal_track linear 20261014-w6-full-s02
#         scripts/archive/run_w6_full.sh s03_maneuver_none_nominal_track maneuver 20261014-w6-full-s03
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"

SCENE="${1:?scene}"
MOTION="${2:?motion}"
RUN_ID="${3:-$(date +%Y%m%d)-w6-full-0}"
JAMFLAG="${4:-}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"

OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

setsid ros2 launch agc_task w6.launch.py backend:=full \
  scene:="${SCENE}" gz_args:="-s -r -v 3 ${WORLD}" \
  > "${OUT}/log/launch.log" 2>&1 &
PGID=$!
TAG="w6.launch.py"
cleanup() {
  kill -INT -"${PGID}" 2>/dev/null; pkill -INT -f "${TAG}" 2>/dev/null
  sleep 4
  kill -TERM -"${PGID}" 2>/dev/null; pkill -TERM -f "${TAG}" 2>/dev/null
  pkill -TERM -f "agc_world.sdf" 2>/dev/null; pkill -TERM -f "bin/px4" 2>/dev/null
  pkill -TERM -f "MicroXRCEAgent" 2>/dev/null
  sleep 3
  kill -KILL -"${PGID}" 2>/dev/null; pkill -KILL -f "${TAG}" 2>/dev/null
  pkill -KILL -f "agc_world.sdf" 2>/dev/null; pkill -KILL -f "bin/px4" 2>/dev/null
  pkill -KILL -f "MicroXRCEAgent" 2>/dev/null
}
trap cleanup EXIT

echo "launch up (scene=${SCENE}, run_id=${RUN_ID}); 等待双 PX4 + 驱动就绪 55 s ..."
sleep 55
EXTRA=""
if [ "${JAMFLAG}" = "--jam" ]; then EXTRA="--jam"; fi
python3 "${ROOT}/tests/w6_full_check.py" --motion "${MOTION}" --duration 8 \
  --run-id "${RUN_ID}" ${EXTRA}
RC=$?
echo "w6 full[${SCENE}] rc=${RC}"
exit "${RC}"
