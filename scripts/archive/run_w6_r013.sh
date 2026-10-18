#!/usr/bin/env bash
# run_w6_r013 —— 评审 R-013（起始异步事务与新鲜门禁）一键验收（launch 背景 + 检查 + 清理）
#   用法：scripts/archive/run_w6_r013.sh [headless|gui] [run_id]
#   例：  scripts/archive/run_w6_r013.sh headless 20261016-w6-r013-0
#   以轻量 W6 栈（agc_task w6.launch.py backend:=lightweight，无 PX4）起全链，再执行
#   tests/w6_r013_check.py（子测 a 正常通过 / b 复位后新鲜数据缺席被拒 / c 世界名不符被拒 /
#   d 时钟冻结被拒）。链路就绪偶发较慢 → 最多重试 3 次。
# 注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

MODE="${1:-headless}"
RUN_ID="${2:-$(date +%Y%m%d)-w6-r013-0}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"

if [ "${MODE}" = "gui" ]; then
  GZ="-r -v 3 ${WORLD}"
else
  GZ="-s -r -v 3 ${WORLD}"
fi

OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

cleanup() {
  kill -TERM -"${PGID}" 2>/dev/null
  pkill -TERM -f "[w]6.launch.py" 2>/dev/null
  pkill -TERM -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -TERM -f "[a]gc_world.sdf" 2>/dev/null
  # 孤儿节点（launch 死后重挂 init）也需清掉，否则 DDS 上残留同名节点 → 发现失败
  pkill -TERM -f "[i]nstall/agc_" 2>/dev/null
  pkill -TERM -f "[p]arameter_bridge" 2>/dev/null
  pkill -TERM -f "[g]z sim" 2>/dev/null
  pkill -TERM -f "[r]os_gz_sim" 2>/dev/null
  sleep 3
  kill -KILL -"${PGID}" 2>/dev/null
  pkill -KILL -f "[w]6.launch.py" 2>/dev/null
  pkill -KILL -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -KILL -f "[a]gc_world.sdf" 2>/dev/null
  pkill -KILL -f "[i]nstall/agc_" 2>/dev/null
  pkill -KILL -f "[p]arameter_bridge" 2>/dev/null
  pkill -KILL -f "[g]z sim" 2>/dev/null
  pkill -KILL -f "[r]os_gz_sim" 2>/dev/null
}

RC=1
for attempt in 1 2 3; do
  setsid ros2 launch agc_task w6.launch.py backend:=lightweight gz_args:="${GZ}" \
    > "${OUT}/log/launch-${attempt}.log" 2>&1 &
  PGID=$!
  trap cleanup EXIT
  echo "launch up (mode=${MODE}, run_id=${RUN_ID}, attempt=${attempt}, pgid=${PGID}); 等待链路就绪 20 s ..."
  sleep 20

  python3 "${ROOT}/tests/w6_r013_check.py" --run-id "${RUN_ID}" \
    2>&1 | tee "${OUT}/log/check-${attempt}.log"
  RC=${PIPESTATUS[0]}
  cleanup
  trap - EXIT
  if [ "${RC}" -eq 0 ]; then
    echo "r013 check PASS (attempt ${attempt})"
    exit 0
  fi
  echo "r013 check rc=${RC}（attempt ${attempt}）→ 重试"
  sleep 2
done
echo "r013 check FAIL after retries (rc=${RC})"
exit "${RC}"
