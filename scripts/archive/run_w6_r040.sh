#!/usr/bin/env bash
# run_w6_r040 —— 评审 R-040（地图内容身份与单调更新序号）一键验收
#   用法：scripts/archive/run_w6_r040.sh [run_id]
#   例：  scripts/archive/run_w6_r040.sh 20261018-w6-r040-0
#
# 阶段：
#   Phase A **实跑**节点行为台架（w6_r040_bench.py，真实 map_adapter 二进制 + 合成障碍流）：
#           同字节不同 origin / 同分箱跨边界 / 单调序号 / R-006 → <RUN_ID>/r040/bench.json
#   Phase B 自包含核查（w6_r040_check.py）→ <RUN_ID>/r040/check.json（+ metrics/r040.json）
#
# 注：清理**禁止** `rm -rf` 与裸名 `pkill`（会匹配自身命令行 → 自杀）；用 `pgrep -f "名[x]"` 写法定位。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN_ID="${1:-20261018-w6-r040-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log" "${OUT}/metrics" "${OUT}/r040"

cleanup() {
  # [x] 写法 + 完整安装路径：仅命中本台架真实二进制进程，不误杀其它会话 / 自身。
  for pat in "install/agc_perception/lib/agc_perception/map_adapte[r]"; do
    for p in $(pgrep -f "${pat}" 2>/dev/null); do
      kill -9 "${p}" 2>/dev/null
    done
  done
}
trap 'cleanup' EXIT
cleanup

# ---------------- Phase A：节点行为台架（真实二进制） ----------------
echo "[Phase A] 台架（真实二进制）→ ${RUN_ID}/r040/bench.json"
python3 "${ROOT}/scripts/archive/w6_r040_bench.py" --run-id "${RUN_ID}" \
  > "${OUT}/log/phase-a-bench.log" 2>&1
echo "$?" > "${OUT}/r040/bench_rc.txt"
echo "[Phase A] bench rc=$(cat "${OUT}/r040/bench_rc.txt")"
cleanup

# ---------------- Phase B：自包含核查 ----------------
echo "[Phase B] 核查 → r040/check.json + metrics/r040.json"
python3 "${ROOT}/tests/w6_r040_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-b-check.log"
RC_C=${PIPESTATUS[0]}

if [ "${RC_C}" -eq 0 ]; then echo "r040 check PASS (overall_pass=true)"; else echo "r040 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{r040/,metrics/r040.json,log/,config/}"
exit "${RC_C}"
