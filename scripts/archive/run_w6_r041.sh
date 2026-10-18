#!/usr/bin/env bash
# run_w6_r041 —— 评审 R-041（目标失效策略与位移阈值化重规划）一键验收
#   用法：scripts/archive/run_w6_r041.sh [run_id]
#   例：  scripts/archive/run_w6_r041.sh 20261018-w6-r041-0
#
# 阶段：
#   Phase A **实跑**节点行为台架（w6_r041_bench.py，真实 task_manager / ground_baseline 二进制 + 合成输入）：
#           target 过期（失联） / 微噪声 goal / 失效安全停车 → <RUN_ID>/r041/bench.json
#   Phase B 自包含核查（w6_r041_check.py）→ <RUN_ID>/r041/check.json（+ metrics/r041.json）
#
# 注：清理**禁止** `rm -rf` 与裸名 `pkill`（会匹配自身命令行 → 自杀）；用 `pgrep -f "名[x]"` 写法定位。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN_ID="${1:-20261018-w6-r041-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log" "${OUT}/metrics" "${OUT}/r041"

cleanup() {
  # [x] 写法 + 完整安装路径：仅命中本台架真实二进制进程，不误杀其它会话 / 自身。
  for pat in "install/agc_task/lib/agc_task/task_manage[r]" \
             "install/agc_ground/lib/agc_ground/ground_baselin[e]" \
             "tf2_ros/static_transform_publishe[r]"; do
    for p in $(pgrep -f "${pat}" 2>/dev/null); do
      kill -9 "${p}" 2>/dev/null
    done
  done
}
trap 'cleanup' EXIT
cleanup

# ---------------- Phase A：节点行为台架（真实二进制） ----------------
echo "[Phase A] 台架（真实二进制）→ ${RUN_ID}/r041/bench.json"
python3 "${ROOT}/scripts/archive/w6_r041_bench.py" --run-id "${RUN_ID}" \
  > "${OUT}/log/phase-a-bench.log" 2>&1
echo "$?" > "${OUT}/r041/bench_rc.txt"
echo "[Phase A] bench rc=$(cat "${OUT}/r041/bench_rc.txt")"
cleanup

# ---------------- Phase B：自包含核查 ----------------
echo "[Phase B] 核查 → r041/check.json + metrics/r041.json"
python3 "${ROOT}/tests/w6_r041_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-b-check.log"
RC_C=${PIPESTATUS[0]}

if [ "${RC_C}" -eq 0 ]; then echo "r041 check PASS (overall_pass=true)"; else echo "r041 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{r041/,metrics/r041.json,log/,config/}"
exit "${RC_C}"
