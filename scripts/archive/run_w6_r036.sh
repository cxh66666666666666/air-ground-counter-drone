#!/usr/bin/env bash
# run_w6_r036 —— 评审 R-036（C1 高度语义统一与同时间证据曲线）一键验收
#   用法：scripts/archive/run_w6_r036.sh [run_id]
#   例：  scripts/archive/run_w6_r036.sh 20261017-w6-r036-0
#
# 阶段：
#   Phase A **实跑** R-036 逻辑台架（w6_r036_bench.py）：起**真实** agc_air 生命周期节点
#           （trajectory_tracker，standalone），同一 run 同一时间轴记录
#           **world GT / PX4 估计（/agc/air/odom）/ base（/agc/air/ref）/ effective / setpoint**
#           5 曲线；报告 P95 / max / 持续越限；回归 base↔effective 高度基准一致。
#           产物落 <RUN_ID>/r036/{bench.json,curves.csv}。
#   Phase B 自包含核查（w6_r036_check.py）→ <RUN_ID>/r036/check.json（+ metrics/r036.json）。
#
# 注：清理**禁止** `rm -rf` 与裸名 `pkill`（会匹配自身命令行 → 自杀）；用 `pgrep -f "路径片断[y]"`
#     写法定位真实进程再按 PID kill。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN_ID="${1:-20261017-w6-r036-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log" "${OUT}/metrics" "${OUT}/r036"

cleanup() {
  # [x]xx 写法 + **完整安装路径**：模式仅命中台架残留的**真实二进制进程**，不误杀其它会话 / 自身。
  for p in $(pgrep -f "install/agc_air/lib/agc_air/trajector[y]_tracker" 2>/dev/null); do
    kill -9 "${p}" 2>/dev/null
  done
}
trap 'cleanup' EXIT
cleanup

# ---------------- Phase A：实跑 R-036 逻辑台架 ----------------
echo "[Phase A] R-036 逻辑台架（真实节点，5 曲线同轴）→ ${RUN_ID}/r036/bench.json"
python3 "${ROOT}/scripts/archive/w6_r036_bench.py" --run-id "${RUN_ID}" \
  > "${OUT}/log/phase-a-bench.log" 2>&1
echo "$?" > "${OUT}/r036/bench_rc.txt"
echo "[Phase A] bench rc=$(cat "${OUT}/r036/bench_rc.txt")"
cleanup

# ---------------- Phase B：自包含核查 ----------------
echo "[Phase B] 核查 → r036/check.json + metrics/r036.json"
python3 "${ROOT}/tests/w6_r036_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-b-check.log"
RC_B=${PIPESTATUS[0]}

if [ "${RC_B}" -eq 0 ]; then echo "r036 check PASS (overall_pass=true)"; else echo "r036 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{r036/,metrics/r036.json,log/}"
exit "${RC_B}"
