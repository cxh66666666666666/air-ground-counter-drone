#!/usr/bin/env bash
# run_w6_r2 —— W6-a R2（门禁与 episode 起始时序）一键验收（launch 背景 + 尽早介入 + 清理）
#   用法：scripts/run_w6_r2.sh [headless|gui] [run_id]
#   例：  scripts/run_w6_r2.sh headless 20261014-w6-r2-0
#   注意：与 run_w6_fsm.sh 不同，本脚本**不等待**链路就绪 —— 需尽早调用 start 以观察"未就绪被拒"。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"

MODE="${1:-headless}"
RUN_ID="${2:-$(date +%Y%m%d)-w6-r2-0}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"

if [ "${MODE}" = "gui" ]; then
  GZ="-r -v 3 ${WORLD}"
else
  GZ="-s -r -v 3 ${WORLD}"
fi

OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

setsid ros2 launch agc_task m0_skeleton.launch.py \
  gz_args:="${GZ}" \
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

echo "launch up (mode=${MODE}, run_id=${RUN_ID}, pgid=${PGID}); 立即执行 R2 检查 ..."
python3 "${ROOT}/scripts/w6_r2_check.py" --run-id "${RUN_ID}"
RC=$?
echo "w6 r2 check rc=${RC}"
exit "${RC}"
