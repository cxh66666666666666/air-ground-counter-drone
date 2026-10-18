#!/usr/bin/env bash
# run_w6_r020 —— 评审 R-020（占位包络标注 + 高度含车体 + 独立干扰包络）一键验收
#   用法：scripts/archive/run_w6_r020.sh [headless|gui] [run_id]
#   例：  scripts/archive/run_w6_r020.sh headless 20261016-w6-r020-0
#
# **本脚本不起 launch（自包含）**，原因：
#   * `tests/w6_r020_check.py` 自己充当「假 odom / 目标源」桩环境，以 subprocess 直接起
#     `install/agc_task/lib/agc_task/{capture_adjudicator, jammer}` + 临时参数文件，无需 gz / launch。
#   `headless|gui` 参数仅保留以兼容统一 CLI 形态（本脚本不使用 gz）。
# 注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

MODE="${1:-headless}"
RUN_ID="${2:-$(date +%Y%m%d)-w6-r020-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

# 清理残留 capture_adjudicator / jammer（"[x]xx" 写法规避匹配 shell 自身）
pkill -TERM -f "[c]apture_adjudicator" 2>/dev/null
pkill -TERM -f "[j]ammer" 2>/dev/null
sleep 1

RC=1
for attempt in 1 2 3; do
  echo "r020 check (mode=${MODE}, run_id=${RUN_ID}, attempt=${attempt})"
  python3 "${ROOT}/tests/w6_r020_check.py" --run-id "${RUN_ID}" \
    2>&1 | tee "${OUT}/log/check-${attempt}.log"
  RC=${PIPESTATUS[0]}
  # 收尾：杀掉本用例残留进程（正常由 check 的 Popen 句柄 kill）
  pkill -TERM -f "[c]apture_adjudicator" 2>/dev/null
  pkill -TERM -f "[j]ammer" 2>/dev/null
  if [ "${RC}" -eq 0 ]; then
    echo "r020 check PASS (attempt ${attempt})"
    exit 0
  fi
  echo "r020 check rc=${RC}（attempt ${attempt}）→ 重试"
  sleep 2
done
echo "r020 check FAIL after retries (rc=${RC})"
exit "${RC}"
