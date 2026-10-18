#!/usr/bin/env bash
# run_w5_fullchain —— W5-c R10 全链回归一键跑（launch 背景 + 检查 + 清理）
#   用法：scripts/archive/run_w5_fullchain.sh [headless|gui] [rl|baseline] [run_id]
#   例：  scripts/archive/run_w5_fullchain.sh headless rl
#         scripts/archive/run_w5_fullchain.sh gui baseline 20261013-fullchain-baseline
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"

MODE="${1:-headless}"
SOURCE="${2:-rl}"
RUN_ID="${3:-$(date +%Y%m%d)-fullchain-${SOURCE}}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"

if [ "${MODE}" = "gui" ]; then
  GZ="-r -v 3 ${WORLD}"
else
  GZ="-s -r -v 3 ${WORLD}"
fi

OUT="${ROOT}/experiments/w5-ground-nav-safety/${RUN_ID}"
mkdir -p "${OUT}/log"

setsid ros2 launch agc_task m0_skeleton.launch.py \
  gz_args:="${GZ}" command_source:="${SOURCE}" \
  > "${OUT}/log/launch.log" 2>&1 &
PGID=$!
TAG="m0_skeleton.launch.py"
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

echo "launch up (mode=${MODE}, command_source=${SOURCE}, pgid=${PGID}); 等待链路就绪 18 s ..."
sleep 18
python3 "${ROOT}/tests/w5_fullchain_check.py" --duration 8 --run-id "${RUN_ID}"
RC=$?
echo "fullchain check rc=${RC}"
exit "${RC}"
