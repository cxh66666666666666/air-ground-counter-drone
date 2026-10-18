#!/usr/bin/env bash
# run_w6_r017 —— 评审 R-017（干扰单一授权与已校验事件）一键验收（自包含 harness + 清理 + 重试）
#   用法：scripts/run_w6_r017.sh [headless|gui] [run_id]
#   例：  scripts/run_w6_r017.sh headless 20261016-w6-r017-0
#
# **本脚本不起 launch（自包含）**，原因：
#   * 轻量栈 `agc_task w6.launch.py backend:=lightweight` **不启动** `intruder_driver`（评审 R-025）；
#   * 其自带 `scenario_reset` 会发 epoch / car_start 干扰本项目用例；
#   故 `scripts/w6_r017_check.py` 自己充当「假 PX4」（发 odom/status、订 setpoint/command、发 epoch/包络/原始命令、
#   订 jam_state 已校验事件），并以 subprocess 直接起 `install/lib/agc_task/{jammer, intruder_driver}` +
#   临时参数文件，无需 gz / launch。
#   `headless|gui` 参数仅保留以兼容统一 CLI 形态（本脚本不使用 gz）。
# 注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

MODE="${1:-headless}"
RUN_ID="${2:-$(date +%Y%m%d)-w6-r017-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

# 清理残留 driver / jammer（"[x]xx" 写法规避匹配 shell 自身）
pkill -TERM -f "[i]ntruder_driver" 2>/dev/null
pkill -TERM -f "[j]ammer" 2>/dev/null
sleep 1

RC=1
for attempt in 1 2 3; do
  echo "r017 check (mode=${MODE}, run_id=${RUN_ID}, attempt=${attempt})"
  python3 "${ROOT}/scripts/w6_r017_check.py" --run-id "${RUN_ID}" \
    2>&1 | tee "${OUT}/log/check-${attempt}.log"
  RC=${PIPESTATUS[0]}
  # 收尾：杀掉本用例残留进程（正常由 check 的 Popen 句柄 kill）
  pkill -TERM -f "[i]ntruder_driver" 2>/dev/null
  pkill -TERM -f "[j]ammer" 2>/dev/null
  if [ "${RC}" -eq 0 ]; then
    echo "r017 check PASS (attempt ${attempt})"
    exit 0
  fi
  echo "r017 check rc=${RC}（attempt ${attempt}）→ 重试"
  sleep 2
done
echo "r017 check FAIL after retries (rc=${RC})"
exit "${RC}"
