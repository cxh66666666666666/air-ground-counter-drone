#!/usr/bin/env bash
# run_w6_r028 —— 评审 R-028（episode 窗口统计、终态锁存与有效样本掩码）一键验收
#   用法：scripts/run_w6_r028.sh [run_id]
#   例：  scripts/run_w6_r028.sh 20261016-r028-0
#
# 阶段：
#   Phase A 合成 harness：w6_r028_harness.py 以确定性合成诊断数据（/clock / odom / scan / cm /
#     state，**不**发布 cmd_vel）复现「陈旧 scan / 预热 / 终态锁存 / 无 scan / 无任务」6 个 case；
#     每个 case 的 run_logger summary 拷为 <RUN_ID>/summaries/harness-<case>.json。
#   Phase B 真实整链轻量栈：episode_runner 跑 s11 → run_logger summary 拷为 summaries/real_s11.json。
#   Phase C 自包含核查：w6_r028_check.py → <RUN_ID>/metrics/r028.json。
#   返回码反映 overall_pass。
#
# 注：清理**禁止** pkill -f "<裸名>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

RUN_ID="${1:-$(date +%Y%m%d)-r028-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log" "${OUT}/summaries" "${OUT}/metrics"

cleanup() {
  pkill -TERM -f "[w]6_r028_harness.py" 2>/dev/null
  pkill -TERM -f "[w]6_task_eval.launch.py" 2>/dev/null
  pkill -TERM -f "[w]5_safety_scenario.launch.py" 2>/dev/null
  pkill -TERM -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -TERM -f "[w]orld_bringup.launch.py" 2>/dev/null
  pkill -TERM -f "[a]gc_world.sdf" 2>/dev/null
  pkill -TERM -f "[p]arameter_bridge" 2>/dev/null
  pkill -TERM -f "[r]os_gz_bridge" 2>/dev/null
  pkill -TERM -f "[r]un_logger" 2>/dev/null
  sleep 1
  pkill -KILL -f "[w]6_r028_harness.py" 2>/dev/null
  pkill -KILL -f "[w]6_task_eval.launch.py" 2>/dev/null
  pkill -KILL -f "[w]5_safety_scenario.launch.py" 2>/dev/null
  pkill -KILL -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -KILL -f "[w]orld_bringup.launch.py" 2>/dev/null
  pkill -KILL -f "[a]gc_world.sdf" 2>/dev/null
  pkill -KILL -f "[p]arameter_bridge" 2>/dev/null
  pkill -KILL -f "[r]os_gz_bridge" 2>/dev/null
  pkill -KILL -f "[r]un_logger" 2>/dev/null
}
trap 'cleanup' EXIT

cleanup

# ---------------- Phase A：合成 harness（6 个确定性 case） ----------------
echo "[Phase A] w6_r028_harness.py（合成诊断数据）→ ${RUN_ID}/summaries/"
python3 "${ROOT}/scripts/w6_r028_harness.py" --out-dir "${OUT}/summaries" \
  > "${OUT}/log/phase-a-harness.log" 2>&1
echo "[Phase A] harness rc=$?"
tail -n 40 "${OUT}/log/phase-a-harness.log" || true
cleanup

# ---------------- Phase B：真实整链轻量栈（s11） ----------------
echo "[Phase B] episode_runner s11（轻量栈）→ ${RUN_ID}/lightweight/"
python3 "${ROOT}/scripts/episode_runner.py" \
  --scenes s11_static_static_none_unreachable \
  --seeds 0 --mode headless --wall-watchdog-s 60 --run-id "${RUN_ID}/lightweight" \
  > "${OUT}/log/phase-b-lightweight.log" 2>&1
echo "[Phase B] runner rc=$?"
cleanup

B_SUM="$(ls -t ${ROOT}/experiments/w6-task-eval/*-s11_static_static_none_unreachable-*/metrics/summary.json 2>/dev/null | head -1)"
if [ -n "${B_SUM}" ] && [ -f "${B_SUM}" ]; then
  cp "${B_SUM}" "${OUT}/summaries/real_s11.json"
  echo "[Phase B] summary ← ${B_SUM}"
else
  echo "[Phase B] WARN 未找到 s11 summary（*-s11_static_static_none_unreachable-*）"
fi

# ---------------- Phase C：自包含核查 ----------------
echo "[Phase C] 核查 → metrics/r028.json"
python3 "${ROOT}/scripts/w6_r028_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-c-check.log"
RC_C=${PIPESTATUS[0]}

if [ "${RC_C}" -eq 0 ]; then echo "r028 check PASS (overall_pass=true)"; else echo "r028 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{summaries/,metrics/r028.json,log/}"
exit "${RC_C}"
