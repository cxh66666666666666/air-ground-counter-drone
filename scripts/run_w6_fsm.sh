#!/usr/bin/env bash
# run_w6_fsm —— W6-a R1/R3（任务状态机 / 成功失败判定）一键验收（launch 背景 + 检查 + 清理）
#   用法：scripts/run_w6_fsm.sh [headless|gui] [run_id] [params_file]
#   例：  scripts/run_w6_fsm.sh headless 20261014-w6-fsm-0
#         scripts/run_w6_fsm.sh headless 20261014-w6-c4-0 <agc_task>/config/w6_c4_timeout.yaml
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"

MODE="${1:-headless}"
RUN_ID="${2:-$(date +%Y%m%d)-w6-fsm-0}"
PARAMS_FILE="${3:-}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"

if [ "${MODE}" = "gui" ]; then
  GZ="-r -v 3 ${WORLD}"
else
  GZ="-s -r -v 3 ${WORLD}"
fi

OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

EXTRA=""
if [ -n "${PARAMS_FILE}" ]; then
  EXTRA="params_file:=${PARAMS_FILE}"
fi

setsid ros2 launch agc_task m0_skeleton.launch.py \
  gz_args:="${GZ}" ${EXTRA} \
  > "${OUT}/log/launch.log" 2>&1 &
PGID=$!
TAG="m0_skeleton.launch.py"
cleanup() {
  # 先 SIGINT（rclcpp 默认处理，令 run_logger 走到 finalize 落 summary.json），再退杀
  kill -INT -"${PGID}" 2>/dev/null
  pkill -INT -f "${TAG}" 2>/dev/null
  pkill -INT -f "agc_world.sdf" 2>/dev/null
  sleep 4
  kill -TERM -"${PGID}" 2>/dev/null
  pkill -TERM -f "${TAG}" 2>/dev/null
  pkill -TERM -f "agc_world.sdf" 2>/dev/null
  sleep 3
  kill -KILL -"${PGID}" 2>/dev/null
  pkill -KILL -f "${TAG}" 2>/dev/null
  pkill -KILL -f "agc_world.sdf" 2>/dev/null
}
trap cleanup EXIT

echo "launch up (mode=${MODE}, run_id=${RUN_ID}, pgid=${PGID}); 等待链路就绪 18 s ..."
sleep 18
python3 "${ROOT}/scripts/w6_fsm_check.py" --duration 150 --run-id "${RUN_ID}"
RC=$?
echo "w6 fsm check rc=${RC}"
exit "${RC}"
