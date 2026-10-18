#!/usr/bin/env bash
# run_w6_r029 —— 评审 R-029（唯一运行身份与可复现证据）一键验收
#   用法：scripts/archive/run_w6_r029.sh [run_id]
#   例：  scripts/archive/run_w6_r029.sh 20261016-r029-0
#
# 阶段：
#   Phase A 唯一不覆盖：同场景同 seed 连跑两次 run_logger → 落到**两个不同** run 目录（含时刻+随机后缀）。
#   Phase B 拒写已有目录：预建同 run_id 目录 + 哨兵 summary → logger 独占创建失败即退出码 2、哨兵不被重写。
#   Phase C episode_id 贯穿：订 /agc/task/episode 发 Int32=7 → summary/config 记录。
#   Phase D 身份齐备：拷 Phase A 的 config/（展开参数 + 代码身份 + 工具链/世界哈希）到 identity/。
#   Phase E runner 同一 run 关联：episode_runner 轻量栈跑 s01（新就绪逻辑、glob 关联）。
#   Phase F 自包含核查：w6_r029_check.py → <RUN_ID>/metrics/r029.json。返回码反映 overall_pass。
#
# 注：清理**禁止** pkill -f "<裸名>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

RUN_ID="${1:-20261016-r029-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"
PARAMS="${ROOT}/install/agc_task/share/agc_task/config/w6_base.yaml"
mkdir -p "${OUT}/log" "${OUT}/metrics" "${OUT}/dup/dup_a" "${OUT}/dup/dup_b" \
         "${OUT}/reject" "${OUT}/episode" "${OUT}/identity"

cleanup() {
  pkill -TERM -f "[e]pisode_runner" 2>/dev/null
  pkill -TERM -f "[w]6_task_eval.launch.py" 2>/dev/null
  pkill -TERM -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -TERM -f "[w]orld_bringup.launch.py" 2>/dev/null
  pkill -TERM -f "[a]gc_world.sdf" 2>/dev/null
  pkill -TERM -f "[p]arameter_bridge" 2>/dev/null
  pkill -TERM -f "[r]os_gz_bridge" 2>/dev/null
  pkill -TERM -f "[r]un_logger" 2>/dev/null
  sleep 1
  pkill -KILL -f "[e]pisode_runner" 2>/dev/null
  pkill -KILL -f "[w]6_task_eval.launch.py" 2>/dev/null
  pkill -KILL -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -KILL -f "[w]orld_bringup.launch.py" 2>/dev/null
  pkill -KILL -f "[a]gc_world.sdf" 2>/dev/null
  pkill -KILL -f "[p]arameter_bridge" 2>/dev/null
  pkill -KILL -f "[r]os_gz_bridge" 2>/dev/null
  pkill -KILL -f "[r]un_logger" 2>/dev/null
}
trap 'cleanup' EXIT

cleanup

# ---------------- Phase A：唯一不覆盖（同场景同 seed 连跑两次） ----------------
run_probe() {
  local label="$1"
  echo "[Phase A] probe ${label}：起 run_logger（scenario=r029-probe seed=0）"
  ros2 run agc_task run_logger --ros-args \
    -p campaign:=w6-task-eval -p scenario:=r029-probe -p seed:=0 \
    -p world_file:="${WORLD}" -p params_file:="${PARAMS}" \
    > "${OUT}/log/phase-a-${label}.log" 2>&1 &
  local pid=$!
  sleep 3
  # ros2 run 不转发 SIGINT → 直接对 run_logger 进程发（安全 [x]xx 写法）
  pkill -INT -f "[r]un_logger" 2>/dev/null
  wait "${pid}" 2>/dev/null
  sleep 2
  local d
  d="$(ls -dt ${ROOT}/experiments/w6-task-eval/*-r029-probe-0-*/ 2>/dev/null | head -1)"
  d="$(basename "${d%/}")"
  echo "${d}" > "${OUT}/dup/dup_${label}/run_dir.txt"
  if [ -f "${ROOT}/experiments/w6-task-eval/${d}/metrics/summary.json" ]; then
    cp "${ROOT}/experiments/w6-task-eval/${d}/metrics/summary.json" \
       "${OUT}/dup/dup_${label}/summary.json"
  fi
  echo "[Phase A] ${label} run_dir=${d}"
  cleanup
}
run_probe a
run_probe b

# ---------------- Phase B：拒写已有目录 ----------------
echo "[Phase B] 预建已有 run 目录 + 哨兵 → logger 应拒写、rc=2"
RDIR="$(date +%Y%m%d)-r029-probe-0-000000-0000"
mkdir -p "${ROOT}/experiments/w6-task-eval/${RDIR}/metrics"
printf '{"sentinel": true}\n' > "${ROOT}/experiments/w6-task-eval/${RDIR}/metrics/summary.json"
cp "${ROOT}/experiments/w6-task-eval/${RDIR}/metrics/summary.json" "${OUT}/reject/sentinel_before.json"
cp "${ROOT}/experiments/w6-task-eval/${RDIR}/metrics/summary.json" "${OUT}/reject/sentinel.json"
echo "${RDIR}" > "${OUT}/reject/existing_dir.txt"
ros2 run agc_task run_logger --ros-args \
  -p campaign:=w6-task-eval -p scenario:=r029-probe -p seed:=0 \
  -p run_id_override:="${RDIR}" -p params_file:="${PARAMS}" \
  > "${OUT}/log/phase-b-reject.log" 2>&1
echo "$?" > "${OUT}/reject/rc.txt"
echo "[Phase B] rc=$(cat "${OUT}/reject/rc.txt")（应 2）"
cleanup

# ---------------- Phase C：episode_id 贯穿 ----------------
echo "[Phase C] episode 身份：订 /agc/task/episode 发 Int32=7"
ros2 run agc_task run_logger --ros-args \
  -p campaign:=w6-task-eval -p scenario:=r029-episode -p seed:=0 \
  -p world_file:="${WORLD}" -p params_file:="${PARAMS}" \
  > "${OUT}/log/phase-c-episode.log" 2>&1 &
PID=$!
sleep 3
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable /agc/task/episode std_msgs/msg/Int32 "{data: 7}" \
  > "${OUT}/log/phase-c-pub.log" 2>&1 || true
sleep 1
pkill -INT -f "[r]un_logger" 2>/dev/null
wait "${PID}" 2>/dev/null
sleep 2
E="$(ls -dt ${ROOT}/experiments/w6-task-eval/*-r029-episode-0-*/ 2>/dev/null | head -1)"
mkdir -p "${OUT}/episode/summaries" "${OUT}/episode/config"
if [ -f "${E}metrics/summary.json" ]; then
  cp "${E}metrics/summary.json" "${OUT}/episode/summaries/summary.json"
fi
cp -r "${E}config/." "${OUT}/episode/config/" 2>/dev/null
echo "[Phase C] episode run_dir=$(basename "${E%/}")"
cleanup

# ---------------- Phase D：身份齐备（拷 Phase A dup_a 的 config/） ----------------
echo "[Phase D] 拷 dup_a run 的 config/ → identity/config/"
DA="$(cat "${OUT}/dup/dup_a/run_dir.txt")"
cp -r "${ROOT}/experiments/w6-task-eval/${DA}/config/." "${OUT}/identity/config/" 2>/dev/null
echo "[Phase D] identity run_dir=${DA}"

# ---------------- Phase E：runner 同一 run 关联（轻量栈 s01） ----------------
echo "[Phase E] episode_runner s01（新就绪逻辑 / glob 关联）→ ${RUN_ID}/runner/"
python3 "${ROOT}/scripts/episode_runner.py" \
  --scenes s01_static_none_nominal_track \
  --seeds 0 --mode headless --wall-watchdog-s 60 --run-id "${RUN_ID}/runner" \
  > "${OUT}/log/phase-e-runner.log" 2>&1
echo "[Phase E] runner rc=$?"
cleanup

# ---------------- Phase F：自包含核查 ----------------
echo "[Phase F] 核查 → metrics/r029.json"
python3 "${ROOT}/tests/w6_r029_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-f-check.log"
RC_F=${PIPESTATUS[0]}

if [ "${RC_F}" -eq 0 ]; then echo "r029 check PASS (overall_pass=true)"; else echo "r029 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{dup/,reject/,episode/,identity/,runner/,metrics/r029.json,log/}"
exit "${RC_F}"
