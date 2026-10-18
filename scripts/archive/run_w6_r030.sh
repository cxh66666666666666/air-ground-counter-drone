#!/usr/bin/env bash
# run_w6_r030 —— 评审 R-030（resetfail 证据矛盾登记与唯一新 run 重做）一键验收
#   用法：scripts/run_w6_r030.sh [run_id]
#   例：  scripts/run_w6_r030.sh 20261016-r030-0
#
# 背景：旧原件 experiments/w6-task-eval/20261014-c9-resetfail-0 已于 2026-10-16 因误删永久丢失
#   （不可恢复，见 docs/adr/0045-*.md）。本脚本**不**重建旧原件，改为在**唯一新 run**（R-029 不可
#   覆盖 run_id，见 ADR-0044）重做非法 spawn 验收，补原始日志 + 运行配置 + 产物身份。
#
# 阶段：
#   Phase 1 起无头 m0_skeleton（注入 w6_c9_resetfail.yaml + world_file），有界轮询 /agc/task/start 就绪。
#   Phase 2 调 /agc/task/start（非法 spawn：车(0,-1) 与 目标(0,0) 中心距 1.0 m < 6.0 m 净空 → 拒绝）；
#           预期应答 success=false、message="start rejected: reset_failed"。
#   Phase 3 优雅停止（SIGINT 让 run_logger finalize() 写 summary）→ 拷 summary / config 至
#           <RUN_ID>/resetfail/（**不**改动 run_logger/task_manager/scenario_reset 语义）。
#   Phase 4 自包含核查：w6_r030_check.py → <RUN_ID>/metrics/r030.json。返回码反映 overall_pass。
#
# 注：清理**禁止** pkill -f "<裸名>"（会匹配 shell 自身命令行 → 自杀）与任何 `rm -rf`；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

RUN_ID="${1:-20261016-r030-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"
PARAMS="${ROOT}/install/agc_task/share/agc_task/config/w6_c9_resetfail.yaml"
mkdir -p "${OUT}/log" "${OUT}/metrics" "${OUT}/resetfail/config"

cleanup() {
  pkill -TERM -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -TERM -f "[w]orld_bringup.launch.py" 2>/dev/null
  pkill -TERM -f "[a]gc_world.sdf" 2>/dev/null
  pkill -TERM -f "[p]arameter_bridge" 2>/dev/null
  pkill -TERM -f "[r]os_gz_bridge" 2>/dev/null
  pkill -TERM -f "[r]un_logger" 2>/dev/null
  pkill -TERM -f "[s]cenario_reset" 2>/dev/null
  pkill -TERM -f "[t]ask_manager" 2>/dev/null
  sleep 1
  pkill -KILL -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -KILL -f "[w]orld_bringup.launch.py" 2>/dev/null
  pkill -KILL -f "[a]gc_world.sdf" 2>/dev/null
  pkill -KILL -f "[p]arameter_bridge" 2>/dev/null
  pkill -KILL -f "[r]os_gz_bridge" 2>/dev/null
  pkill -KILL -f "[r]un_logger" 2>/dev/null
  pkill -KILL -f "[s]cenario_reset" 2>/dev/null
  pkill -KILL -f "[t]ask_manager" 2>/dev/null
}
trap 'cleanup' EXIT

cleanup

# ---------------- Phase 1：起无头 m0_skeleton + 有界就绪轮询 ----------------
echo "[Phase 1] 起 m0_skeleton（headless，params=${PARAMS##*/}，world=${WORLD##*/}）"
setsid ros2 launch agc_task m0_skeleton.launch.py \
  gz_args:="-s -r -v 3 ${WORLD}" params_file:="${PARAMS}" world_file:="${WORLD}" \
  > "${OUT}/log/launch.log" 2>&1 &
PGID=$!

READY=0
for _i in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -q "/agc/task/start"; then READY=1; break; fi
  sleep 1
done
if [ "${READY}" -ne 1 ]; then
  echo "[Phase 1] 超时：/agc/task/start 未就绪（60 s）；见 ${OUT}/log/launch.log"
  exit 1
fi
echo "[Phase 1] /agc/task/start 就绪"

# ---------------- Phase 2：调 start（非法 spawn → 预期 success=false / reset_failed） ----------------
echo "[Phase 2] 调 /agc/task/start（预期被拒：reset_failed）"
ros2 service call /agc/task/start std_srvs/srv/Trigger "{}" \
  > "${OUT}/log/start_call.log" 2>&1 || true
echo "[Phase 2] start 应答："; cat "${OUT}/log/start_call.log"

# 等待 reset 统计 publisher 与 run_logger 收到（transient_local 末值）
sleep 5

# ---------------- Phase 3：优雅停止 → 拷 run_logger 产物 ----------------
echo "[Phase 3] 优雅停止（SIGINT → finalize 写 summary）"
kill -INT -"${PGID}" 2>/dev/null
pkill -INT -f "[m]0_skeleton" 2>/dev/null
pkill -INT -f "[r]un_logger" 2>/dev/null
pkill -INT -f "[s]cenario_reset" 2>/dev/null
pkill -INT -f "[t]ask_manager" 2>/dev/null
sleep 4
kill -TERM -"${PGID}" 2>/dev/null
pkill -TERM -f "[m]0_skeleton" 2>/dev/null
pkill -TERM -f "[r]un_logger" 2>/dev/null
pkill -TERM -f "[s]cenario_reset" 2>/dev/null
pkill -TERM -f "[t]ask_manager" 2>/dev/null
sleep 2
kill -KILL -"${PGID}" 2>/dev/null
cleanup

# 发现 run_logger 的 run 目录（唯一不可覆盖 run_id：含时刻 + 随机后缀）
D="$(ls -dt ${ROOT}/experiments/w6-task-eval/*-c9-resetfail-0-*/ 2>/dev/null | head -1)"
D="$(basename "${D%/}")"
echo "${D}" > "${OUT}/resetfail/run_dir.txt"
echo "[Phase 3] run_dir=${D}"
if [ -n "${D}" ] && [ -f "${ROOT}/experiments/w6-task-eval/${D}/metrics/summary.json" ]; then
  cp "${ROOT}/experiments/w6-task-eval/${D}/metrics/summary.json" "${OUT}/resetfail/summary.json"
  cp -r "${ROOT}/experiments/w6-task-eval/${D}/config/." "${OUT}/resetfail/config/" 2>/dev/null
  echo "[Phase 3] summary/config 已拷入 ${OUT}/resetfail/"
else
  echo "[Phase 3] WARN 未找到 ${D:-<none>}/metrics/summary.json"
fi

# ---------------- Phase 4：自包含核查 ----------------
echo "[Phase 4] 核查 → metrics/r030.json"
python3 "${ROOT}/scripts/w6_r030_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/check.log"
RC_C=${PIPESTATUS[0]}

if [ "${RC_C}" -eq 0 ]; then echo "r030 check PASS (overall_pass=true)"; else echo "r030 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{log/,resetfail/,metrics/r030.json}"
exit "${RC_C}"
