#!/usr/bin/env bash
# run_w6_r033 —— 评审 R-033（UAV 联合安全屏障与显式不可行路径）一键验收
#   用法：scripts/run_w6_r033.sh [run_id]
#   例：  scripts/run_w6_r033.sh 20261017-r033-0
#
# 阶段：
#   Phase A **实跑** R-033 逻辑台架（w6_r033_bench.py）：起真实 trajectory_tracker（configure/activate，
#           `safety.enabled=true` + 每场景静态障碍 / 目标几何）+ 合成驱动，观测 /fmu/in/trajectory_setpoint
#           （有效参考）与 /agc/air/tracking_status（含 R-033 显式不可行诊断）。
#           四场景：a 两障碍交叠→联合不可行 / b 参考安全但连线穿障 / c 目标闭合冲突 / d 真值最小净空。
#           产物落 <RUN_ID>/r033/bench.json。
#   Phase B 自包含核查（w6_r033_check.py）→ <RUN_ID>/metrics/r033.json。返回码反映 overall_pass。
#
# 注：清理**禁止** pkill -f "<裸名>"（会匹配 shell 自身命令行 → 自杀）与任何 `rm -rf`；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN_ID="${1:-20261017-r033-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log" "${OUT}/metrics" "${OUT}/r033"

cleanup() {
  # 括号规避：模式不会匹配本 shell 自身命令行；仅回收台架/节点残留。
  pkill -TERM -f "[t]rajectory_tracker" 2>/dev/null
  sleep 1
  pkill -KILL -f "[t]rajectory_tracker" 2>/dev/null
}
trap 'cleanup' EXIT

cleanup

# ---------------- Phase A：实跑 R-033 逻辑台架 ----------------
echo "[Phase A] R-033 逻辑台架（trajectory_tracker + 联合安全屏障）→ ${RUN_ID}/r033/bench.json"
python3 "${ROOT}/scripts/w6_r033_bench.py" --run-id "${RUN_ID}" \
  > "${OUT}/log/phase-a-bench.log" 2>&1
echo "$?" > "${OUT}/r033/bench_rc.txt"
echo "[Phase A] bench rc=$(cat "${OUT}/r033/bench_rc.txt")"
cleanup

# ---------------- Phase B：自包含核查 ----------------
echo "[Phase B] 核查 → metrics/r033.json"
python3 "${ROOT}/scripts/w6_r033_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-b-check.log"
RC_B=${PIPESTATUS[0]}

if [ "${RC_B}" -eq 0 ]; then echo "r033 check PASS (overall_pass=true)"; else echo "r033 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{r033/,metrics/r033.json,log/}"
exit "${RC_B}"
