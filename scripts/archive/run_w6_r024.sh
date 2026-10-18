#!/usr/bin/env bash
# run_w6_r024 —— 评审 R-024（三类时间口径与 RTF 记录）一键验收
#   用法：scripts/run_w6_r024.sh [headless|gui] [run_id]
#   例：  scripts/run_w6_r024.sh headless 20261016-w6-r024-0
#
# **本脚本不起 launch（自包含）**，原因：
#   * `scripts/w6_r024_check.py` 自己充当「假源 / reset 服务」桩环境，以 subprocess 直接起
#     `install/agc_task/lib/agc_task/task_manager` + 临时参数文件（用例 a 传 `use_sim_time: True`
#     并由 harness 自发 `/clock`），无需 gz / launch。
#   `headless|gui` 参数仅保留以兼容统一 CLI 形态（本脚本不使用 gz）。
# 注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

MODE="${1:-headless}"
RUN_ID="${2:-$(date +%Y%m%d)-w6-r024-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

# 清理残留 task_manager（"[x]xx" 写法规避匹配 shell 自身）
pkill -TERM -f "[t]ask_manager" 2>/dev/null
sleep 1

RC=1
for attempt in 1 2 3; do
  echo "r024 check (mode=${MODE}, run_id=${RUN_ID}, attempt=${attempt})"
  python3 "${ROOT}/scripts/w6_r024_check.py" --run-id "${RUN_ID}" \
    2>&1 | tee "${OUT}/log/check-${attempt}.log"
  RC=${PIPESTATUS[0]}
  # 收尾：杀掉本用例残留进程（正常由 check 的 Popen 句柄 kill）
  pkill -TERM -f "[t]ask_manager" 2>/dev/null
  if [ "${RC}" -eq 0 ]; then
    echo "r024 check PASS (attempt ${attempt})"
    exit 0
  fi
  echo "r024 check rc=${RC}（attempt ${attempt}）→ 重试"
  sleep 2
done
echo "r024 check FAIL after retries (rc=${RC})"
exit "${RC}"
