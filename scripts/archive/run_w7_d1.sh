#!/usr/bin/env bash
# run_w7_d1 —— W7 入口前置 D1（时空与帧正确：R-002 / R-003）一键对拍。
#   用法：scripts/archive/run_w7_d1.sh [run_id]
#   只起 agc_air 的 px4_adapter（无需 gz / PX4），由 tests/w7_d1_check.py 以合成
#   VehicleOdometry 驱动并断言时间单位 / 原点 / 帧；证据落 experiments/w7-rl/<run_id>/。
#   就绪：本机 DDS 的 **graph 查询不可靠**（`get_node_names` / `wait_for_service` 常为空或"假真"），
#         但服务**数据面**可用 —— 首个调用常超时（发现预热），其后即通。故 probe 不用 graph 门禁，
#         直接对 ChangeState 重试；runner 另加整轮重试（同 run_w6_r013.sh 做法）。
# 注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
# 注意：**必须**用默认 `ROS_DOMAIN_ID`（0）。实测本机在非 0 域（77 / 81）下 lifecycle
# 服务数据面不通（ChangeState 一律超时、configure 不生效；日志无"就绪"），默认域正常。
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN_ID="${1:-$(date +%Y%m%d)-w7-d1-$(date +%H%M%S)-$(printf '%04d' $((RANDOM % 10000)))}"
OUT="${ROOT}/experiments/w7-rl/${RUN_ID}"
mkdir -p "${OUT}/log"

cleanup() {
  pkill -INT -f "[p]x4_adapter" 2>/dev/null
  sleep 2
  pkill -KILL -f "[p]x4_adapter" 2>/dev/null
}
trap cleanup EXIT

echo "D1 对拍：run_id=${RUN_ID}，domain=${ROS_DOMAIN_ID}"
RC=1
for attempt in 1 2 3; do
  cleanup
  echo "--- attempt ${attempt} ---"
  python3 "${ROOT}/tests/w7_d1_check.py" --run-id "${RUN_ID}" 2>&1 \
    | tee "${OUT}/log/check-${attempt}.log"
  RC=${PIPESTATUS[0]}
  [ "${RC}" -eq 0 ] && break
  echo "attempt ${attempt} rc=${RC} → 重试"
done
cleanup
trap - EXIT

if [ "${RC}" -eq 0 ]; then
  python3 "${ROOT}/scripts/evidence_archive.py" --campaign w7-rl --run-id "${RUN_ID}" \
    > "${OUT}/log/archive.log" 2>&1 || echo "（归档 manifest/archive 未生成，见 log/archive.log）"
  echo "D1 PASS（evidence: experiments/w7-rl/${RUN_ID}）"
else
  echo "D1 FAIL after retries (rc=${RC})"
fi
exit "${RC}"
