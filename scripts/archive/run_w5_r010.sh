#!/usr/bin/env bash
# run_w5_r010 —— W5 R-010 RL 输入许可与失效停车一键跑
#   用法：scripts/run_w5_r010.sh [gui|headless] [run_id] [scenario]
#     scenario: sim（默认，GUI 集成：真实节点故障注入）| node（独立进程：合成输入故障注入）
#   子测见 scripts/w5_r010_check.py（sim：S1–S3；node：U1 + N0–N5）。
#   与 R-009 不同：本项**不**杀 ground_policy_rl（RL 即命令源，须由其驱动）。
#   lifecycle bringup 偶发卡死 → 最多重试 3 次；node 场景不需 GZ，失败仅重试一次。
# 注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

MODE="${1:-gui}"
RUN_ID="${2:-20261015-w5-r010-0}"
SCENARIO="${3:-sim}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"

export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

if [ "${MODE}" = "gui" ]; then
  GZ="-r -v 3 ${WORLD}"
else
  GZ="-s -r -v 3 ${WORLD}"
fi

OUT="${ROOT}/experiments/w5-ground-nav-safety/${RUN_ID}"
mkdir -p "${OUT}/log"

TAG="w5_safety_scenario.launch.py"
cleanup() {
  kill -TERM -"${PGID}" 2>/dev/null
  pkill -TERM -f "[w]5_safety_scenario.launch.py" 2>/dev/null
  pkill -TERM -f "[a]gc_world.sdf" 2>/dev/null
  pkill -TERM -f "[i]nstall/agc_" 2>/dev/null
  pkill -TERM -f "[c]ollision_monitor" 2>/dev/null
  pkill -TERM -f "[n]av2_lifecycle_manager" 2>/dev/null
  pkill -TERM -f "[n]av2_costmap_2d" 2>/dev/null
  pkill -TERM -f "[r]os_gz_bridge" 2>/dev/null
  pkill -TERM -f "[s]tatic_transform_publisher" 2>/dev/null
  pkill -TERM -f "[g]z sim" 2>/dev/null
  sleep 3
  kill -KILL -"${PGID}" 2>/dev/null
  pkill -KILL -f "[w]5_safety_scenario.launch.py" 2>/dev/null
  pkill -KILL -f "[a]gc_world.sdf" 2>/dev/null
  pkill -KILL -f "[i]nstall/agc_" 2>/dev/null
  pkill -KILL -f "[c]ollision_monitor" 2>/dev/null
  pkill -KILL -f "[n]av2_lifecycle_manager" 2>/dev/null
  pkill -KILL -f "[n]av2_costmap_2d" 2>/dev/null
  pkill -KILL -f "[r]os_gz_bridge" 2>/dev/null
  pkill -KILL -f "[s]tatic_transform_publisher" 2>/dev/null
  pkill -KILL -f "[g]z sim" 2>/dev/null
}

if [ "${SCENARIO}" = "node" ]; then
  python3 "${ROOT}/scripts/w5_r010_check.py" --scenario node --run-id "${RUN_ID}"
  RC=$?
  [ "${RC}" -eq 0 ] && echo "r010 node PASS" || echo "r010 node FAIL (rc=${RC})"
  exit "${RC}"
fi

RC=1
for attempt in 1 2 3; do
  setsid ros2 launch agc_task "${TAG}" gz_args:="${GZ}" \
    > "${OUT}/log/launch-${attempt}.log" 2>&1 &
  PGID=$!
  trap cleanup EXIT
  echo "launch up (mode=${MODE}, scenario=${SCENARIO}, attempt=${attempt}, pgid=${PGID}); 等待链路就绪 18 s ..."
  sleep 18

  python3 "${ROOT}/scripts/w5_r010_check.py" --scenario sim --run-id "${RUN_ID}"
  RC=$?
  cleanup
  trap - EXIT
  if [ "${RC}" -eq 0 ]; then
    echo "r010 sim PASS (attempt ${attempt})"
    exit 0
  fi
  echo "r010 sim rc=${RC}（attempt ${attempt}）→ 重试"
  sleep 2
done
echo "r010 sim FAIL after retries (rc=${RC})"
exit "${RC}"
