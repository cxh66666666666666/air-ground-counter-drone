#!/usr/bin/env bash
# run_w5_r012 —— W5 R-012 基线路径绑定与重规划失效安全故障注入一键跑（launch 背景 + 检查 + 清理）
#   用法：scripts/archive/run_w5_r012.sh [gui|headless] [run_id]
#   以 `command_source:=baseline` 起 W5 场景（离线对照基线为命令源），
#   子测（另见 tests/w5_r012_check.py）：d 不抖动 / a 地图增障重规划 / b 目标跳变 / c 不可达失效停车。
#   lifecycle bringup 偶发卡死（costmap change_state 超时）→ 最多重试 3 次。
# 注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

MODE="${1:-gui}"
RUN_ID="${2:-20261015-w5-r012-0}"
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
  pkill -TERM -f "[g]round_baseline" 2>/dev/null
  pkill -TERM -f "[r]os_gz_sim" 2>/dev/null
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
  pkill -KILL -f "[g]round_baseline" 2>/dev/null
  pkill -KILL -f "[r]os_gz_sim" 2>/dev/null
  pkill -KILL -f "[g]z sim" 2>/dev/null
}

RC=1
for attempt in 1 2 3; do
  setsid ros2 launch agc_task "${TAG}" gz_args:="${GZ}" command_source:=baseline \
    > "${OUT}/log/launch-${attempt}.log" 2>&1 &
  PGID=$!
  trap cleanup EXIT
  echo "launch up (mode=${MODE}, source=baseline, attempt=${attempt}, pgid=${PGID}); 等待链路就绪 18 s ..."
  sleep 18

  python3 "${ROOT}/tests/w5_r012_check.py" --run-id "${RUN_ID}" --mode "${MODE}" \
    2>&1 | tee "${OUT}/log/check-${attempt}.log"
  RC=${PIPESTATUS[0]}
  cleanup
  trap - EXIT
  if [ "${RC}" -eq 0 ]; then
    echo "r012 check PASS (attempt ${attempt})"
    exit 0
  fi
  echo "r012 check rc=${RC}（attempt ${attempt}）→ 重试"
  sleep 2
done
echo "r012 check FAIL after retries (rc=${RC})"
exit "${RC}"
