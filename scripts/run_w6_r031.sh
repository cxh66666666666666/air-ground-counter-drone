#!/usr/bin/env bash
# run_w6_r031 —— 评审 R-031（runner 三层验收判定与完整性硬门）一键验收
#   用法：scripts/run_w6_r031.sh [run_id]
#   例：  scripts/run_w6_r031.sh 20261016-r031-0
#
# 阶段：
#   Phase A **实跑** episode_runner（轻量栈）跑 s01 两回合（seed 0/1），产物落 <RUN_ID>/runner/：
#           报告含三层验收（完整性 + 终止命题 + 指标有效性）+ metric_completeness（缺失计数 / 比例）。
#   Phase B 自包含核查：w6_r031_check.py → <RUN_ID>/metrics/r031.json。返回码反映 overall_pass。
#           （Case a/b/c 直接 import episode_runner 判定 / 汇总纯函数，验证缺 summary 不剔分母 /
#            空集合 not_applicable / 三层全必过。）
#
# 注：清理**禁止** pkill -f "<裸名>"（会匹配 shell 自身命令行 → 自杀）与任何 `rm -rf`；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

RUN_ID="${1:-20261016-r031-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log" "${OUT}/metrics" "${OUT}/runner"

cleanup() {
  pkill -TERM -f "[e]pisode_runner" 2>/dev/null
  pkill -TERM -f "[w]6_task_eval.launch.py" 2>/dev/null
  pkill -TERM -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -TERM -f "[w]orld_bringup.launch.py" 2>/dev/null
  pkill -TERM -f "[a]gc_world.sdf" 2>/dev/null
  pkill -TERM -f "[p]arameter_bridge" 2>/dev/null
  pkill -TERM -f "[r]os_gz_bridge" 2>/dev/null
  pkill -TERM -f "[r]un_logger" 2>/dev/null
  pkill -TERM -f "[c]apture_adjudicator" 2>/dev/null
  pkill -TERM -f "[j]ammer" 2>/dev/null
  sleep 1
  pkill -KILL -f "[e]pisode_runner" 2>/dev/null
  pkill -KILL -f "[w]6_task_eval.launch.py" 2>/dev/null
  pkill -KILL -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -KILL -f "[w]orld_bringup.launch.py" 2>/dev/null
  pkill -KILL -f "[a]gc_world.sdf" 2>/dev/null
  pkill -KILL -f "[p]arameter_bridge" 2>/dev/null
  pkill -KILL -f "[r]os_gz_bridge" 2>/dev/null
  pkill -KILL -f "[r]un_logger" 2>/dev/null
  pkill -KILL -f "[c]apture_adjudicator" 2>/dev/null
  pkill -KILL -f "[j]ammer" 2>/dev/null
}
trap 'cleanup' EXIT

cleanup

# ---------------- Phase A：实跑 episode_runner（轻量栈，同 on-known-good 场景两回合） ----------------
echo "[Phase A] episode_runner s01（seed 0）连跑两回合 → ${RUN_ID}/runner/（三层验收报告）"
python3 "${ROOT}/scripts/episode_runner.py" \
  --scenes s01_static_none_nominal_track --seeds 0,0 --mode headless \
  --wall-watchdog-s 90 --run-id "${RUN_ID}/runner" \
  > "${OUT}/log/phase-a-runner.log" 2>&1
echo "$?" > "${OUT}/runner/runner_rc.txt"
echo "[Phase A] runner rc=$(cat "${OUT}/runner/runner_rc.txt")"
cleanup

# ---------------- Phase B：自包含核查 ----------------
echo "[Phase B] 核查 → metrics/r031.json"
python3 "${ROOT}/scripts/w6_r031_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-b-check.log"
RC_B=${PIPESTATUS[0]}

if [ "${RC_B}" -eq 0 ]; then echo "r031 check PASS (overall_pass=true)"; else echo "r031 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{runner/,metrics/r031.json,log/}"
exit "${RC_B}"
