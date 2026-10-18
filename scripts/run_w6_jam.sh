#!/usr/bin/env bash
# run_w6_jam —— W6 R8 干扰链（作用包络门控 + 命令输出）轻量验收（无 PX4）
#   用法：scripts/run_w6_jam.sh [run_id]
set -o pipefail
ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
RUN_ID="${1:-$(date +%Y%m%d)-w6-jam-0}"
PARAMS="${ROOT}/install/agc_task/share/agc_task/config/w6_base.yaml"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

setsid ros2 run agc_task capture_adjudicator --ros-args --params-file "${PARAMS}" \
  > "${OUT}/log/adjudicator.log" 2>&1 &
A=$!
setsid ros2 run agc_task jammer --ros-args --params-file "${PARAMS}" \
  > "${OUT}/log/jammer.log" 2>&1 &
J=$!
cleanup() {
  kill -9 "${A}" "${J}" 2>/dev/null
  pkill -9 -f "agc_task/capture_adjudicator" 2>/dev/null
  pkill -9 -f "agc_task/jammer" 2>/dev/null
}
trap cleanup EXIT
sleep 3
python3 "${ROOT}/scripts/w6_jam_check.py" --run-id "${RUN_ID}"
RC=$?
echo "w6 jam check rc=${RC}"
exit "${RC}"
