#!/usr/bin/env bash
# run_w6_r035 —— 评审 R-035（空中控制唯一权威与控制许可）一键验收
#   用法：scripts/archive/run_w6_r035.sh [run_id]
#   例：  scripts/archive/run_w6_r035.sh 20261017-w6-r035-0
#
# 阶段：
#   Phase A **实跑** R-035 逻辑台架（w6_r035_bench.py）：起**真实** agc_air 生命周期节点
#           （trajectory_tracker）与**真实** Nav2 lifecycle_manager，覆盖
#           ① 迟到 release（未授权 ≈0 → 显式许可交权后 ≥20）
#           ② 重复节点（两真实例由显式 claim 互检竞争者 → 冲突后两侧停发 + conflict 可观测）
#           ③ manager 重启（重启后 setpoint ≈0 + claim permit=false，**不**残留旧权威）
#           ④ 静默 publisher（writers>=2 仅告警、不误报，setpoint 持续）
#           产物落 <RUN_ID>/r035/bench.json。
#   Phase B 自包含核查（w6_r035_check.py）→ <RUN_ID>/r035/check.json（+ metrics/r035.json）。
#
# 注：清理**禁止** `rm -rf` 与裸名 `pkill`（会匹配自身命令行 → 自杀）；用 `pgrep -f "路径片断[y]"`
#     写法定位真实进程再按 PID kill。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN_ID="${1:-20261017-w6-r035-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log" "${OUT}/metrics" "${OUT}/r035"

cleanup() {
  # [x]xx 写法 + **完整安装路径**：模式仅命中台架残留的**真实二进制进程**，不误杀其它会话 / 自身。
  for p in $(pgrep -f "install/agc_air/lib/agc_air/trajector[y]_tracker" 2>/dev/null); do
    kill -9 "${p}" 2>/dev/null
  done
  for p in $(pgrep -f "/opt/ros/jazzy/lib/nav2_lifecycle_manage[r]/lifecycle_manage[r]" 2>/dev/null); do
    kill -9 "${p}" 2>/dev/null
  done
  for p in $(pgrep -f "r035_silent_pu[b].py" 2>/dev/null); do
    kill -9 "${p}" 2>/dev/null
  done
}
trap 'cleanup' EXIT
cleanup

# ---------------- Phase A：实跑 R-035 逻辑台架 ----------------
echo "[Phase A] R-035 逻辑台架（真实节点 + Nav2 lifecycle_manager）→ ${RUN_ID}/r035/bench.json"
python3 "${ROOT}/scripts/archive/w6_r035_bench.py" --run-id "${RUN_ID}" \
  > "${OUT}/log/phase-a-bench.log" 2>&1
echo "$?" > "${OUT}/r035/bench_rc.txt"
echo "[Phase A] bench rc=$(cat "${OUT}/r035/bench_rc.txt")"
cleanup

# ---------------- Phase B：自包含核查 ----------------
echo "[Phase B] 核查 → r035/check.json + metrics/r035.json"
python3 "${ROOT}/tests/w6_r035_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-b-check.log"
RC_B=${PIPESTATUS[0]}

if [ "${RC_B}" -eq 0 ]; then echo "r035 check PASS (overall_pass=true)"; else echo "r035 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{r035/,metrics/r035.json,log/}"
exit "${RC_B}"
