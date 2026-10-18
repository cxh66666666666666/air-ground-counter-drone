#!/usr/bin/env bash
# run_w5_r011 —— W5 R-011 动作方向闭环 + C6 逃逸能力标定一键跑（launch 背景 + 杀 RL 命令源 + 脚本化命令源验收 + 清理）
#   用法：scripts/archive/run_w5_r011.sh [gui|headless] [run_id]
#   子测（另见 tests/w5_r011_check.py）：D 方向闭环（left→左 / right→右）、E C6 逃逸标定（T 内横向可达）。
#   与 R-008/R-010 同法：本脚本作"脚本化命令源"直接发 /agc/ground/cmd_vel_raw，故杀掉 RL 命令源避免争抢。
#   lifecycle bringup 偶发卡死（costmap change_state 超时）→ 最多重试 3 次。
# 注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

MODE="${1:-gui}"
RUN_ID="${2:-20261015-w5-r011-0}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"

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
  # 孤儿节点（launch 死后重挂 init）也需清掉，否则 DDS 上残留同名节点 → 发现失败
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

RC=1
for attempt in 1 2 3; do
  setsid ros2 launch agc_task "${TAG}" gz_args:="${GZ}" \
    > "${OUT}/log/launch-${attempt}.log" 2>&1 &
  PGID=$!
  trap cleanup EXIT
  echo "launch up (mode=${MODE}, attempt=${attempt}, pgid=${PGID}); 等待链路就绪 18 s ..."
  sleep 18

  # R-011：本脚本作"脚本化命令源"直接发 /agc/ground/cmd_vel_raw，故杀掉 RL 命令源避免争抢
  pkill -TERM -f "[g]round_policy_rl" 2>/dev/null
  sleep 2

  python3 "${ROOT}/tests/w5_r011_check.py" --run-id "${RUN_ID}" --mode "${MODE}" \
    2>&1 | tee "${OUT}/log/check-${attempt}.log"
  RC=${PIPESTATUS[0]}
  cleanup
  trap - EXIT
  if [ "${RC}" -eq 0 ]; then
    echo "r011 check PASS (attempt ${attempt})"
    exit 0
  fi
  echo "r011 check rc=${RC}（attempt ${attempt}）→ 重试"
  sleep 2
done
echo "r011 check FAIL after retries (rc=${RC})"
exit "${RC}"
