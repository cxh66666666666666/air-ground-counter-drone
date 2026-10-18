#!/usr/bin/env bash
# run_w6_scene —— W6 R5 场景族：按 scene id 起 w6.launch.py backend:=lightweight 并校验期望终态
#   用法：scripts/run_w6_scene.sh <scene_id> <expect_terminal> [expect_reason] [headless|gui]
#   例：  scripts/run_w6_scene.sh s01_static_none_nominal_track TRACKING_SUCCESS
#         scripts/run_w6_scene.sh s10_maneuver_none_fov_timeout MISSION_FAILED TIMEOUT
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"

SCENE="${1:?scene id 必填}"
EXPECT_TERMINAL="${2:?expect terminal 必填}"
EXPECT_REASON="${3:-}"
MODE="${4:-headless}"
RUN_ID="$(date +%Y%m%d)-w6-scene-${SCENE%%_*}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"

if [ "${MODE}" = "gui" ]; then GZ="-r -v 3 ${WORLD}"; else GZ="-s -r -v 3 ${WORLD}"; fi

OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

setsid ros2 launch agc_task w6.launch.py \
  backend:=lightweight scene:="${SCENE}" gz_args:="${GZ}" \
  > "${OUT}/log/launch.log" 2>&1 &
PGID=$!
TAG="w6.launch.py"
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

echo "launch up (scene=${SCENE}, run_id=${RUN_ID}); 等待链路就绪 18 s ..."
sleep 18
EXTRA=""
if [ -n "${EXPECT_REASON}" ]; then EXTRA="--expect-reason ${EXPECT_REASON}"; fi
python3 "${ROOT}/scripts/w6_fsm_check.py" --run-id "${RUN_ID}" \
  --expect-terminal "${EXPECT_TERMINAL}" ${EXTRA}
RC=$?
echo "w6 scene[${SCENE}] rc=${RC}"
exit "${RC}"
