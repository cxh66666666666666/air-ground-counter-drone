#!/usr/bin/env bash
# run_w6_r038 —— 评审 R-038（数值模型暴露与配置合法性启动校验）一键验收
#   用法：scripts/run_w6_r038.sh [run_id]
#   例：  scripts/run_w6_r038.sh 20261017-w6-r038-0
#
# 阶段：
#   Phase A **实跑**失败型 gtest（真实二进制；不写镜像实现测试）：
#           agc_control/test_lqr_validation（非法 Q/R/dt 拒绝 / DARE 未收敛报告）
#           agc_control/test_lqr_controller（原有回归仍过）
#           agc_air/test_safety_shield_validation（平行数组长度不一致 / 非法几何拒绝 + 合法几何通过）
#           结果 → <RUN_ID>/r038/gtest/*.xml
#   Phase B **实跑**节点启动期拒绝台架（w6_r038_bench.py，真实 map_adapter / air_perception）：
#           合法配置存活 / 非法地图几何·障碍几何·seed → **拒绝启动** → <RUN_ID>/r038/bench.json
#   Phase C 自包含核查（w6_r038_check.py）→ <RUN_ID>/r038/check.json（+ metrics/r038.json）
#
# 注：清理**禁止** `rm -rf` 与裸名 `pkill`（会匹配自身命令行 → 自杀）；用 `pgrep -f "名[x]"` 写法定位。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN_ID="${1:-20261017-w6-r038-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log" "${OUT}/metrics" "${OUT}/r038/gtest"

cleanup() {
  # [x] 写法 + 完整安装路径：仅命中本台架真实二进制进程，不误杀其它会话 / 自身。
  for pat in "install/agc_perception/lib/agc_perception/map_adapte[r]" \
             "install/agc_perception/lib/agc_perception/air_perceptio[n]"; do
    for p in $(pgrep -f "${pat}" 2>/dev/null); do
      kill -9 "${p}" 2>/dev/null
    done
  done
}
trap 'cleanup' EXIT
cleanup

# ---------------- Phase A：失败型 gtest（直接运行真实可执行） ----------------
echo "[Phase A] 失败型 gtest → ${RUN_ID}/r038/gtest/"
A_RC=0
run_gtest() {
  local bin="$1" name="$2"
  if [ ! -x "${bin}" ]; then echo "  MISSING ${bin}"; A_RC=1; return; fi
  "${bin}" --gtest_output="xml:${OUT}/r038/gtest/${name}.xml" \
    > "${OUT}/log/gtest_${name}.log" 2>&1
  local rc=$?
  echo "  ${name} rc=${rc}"
  [ "${rc}" -ne 0 ] && A_RC=1
}
run_gtest "${ROOT}/build/agc_control/test_lqr_validation" test_lqr_validation
run_gtest "${ROOT}/build/agc_control/test_lqr_controller" test_lqr_controller
run_gtest "${ROOT}/build/agc_air/test_safety_shield_validation" test_safety_shield_validation
echo "${A_RC}" > "${OUT}/r038/gtest_rc.txt"

# ---------------- Phase B：节点启动期拒绝台架 ----------------
echo "[Phase B] 节点启动期拒绝台架（真实节点）→ ${RUN_ID}/r038/bench.json"
python3 "${ROOT}/scripts/w6_r038_bench.py" --run-id "${RUN_ID}" \
  > "${OUT}/log/phase-b-bench.log" 2>&1
echo "$?" > "${OUT}/r038/bench_rc.txt"
echo "[Phase B] bench rc=$(cat "${OUT}/r038/bench_rc.txt")"
cleanup

# ---------------- Phase C：自包含核查 ----------------
echo "[Phase C] 核查 → r038/check.json + metrics/r038.json"
python3 "${ROOT}/scripts/w6_r038_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-c-check.log"
RC_C=${PIPESTATUS[0]}

if [ "${RC_C}" -eq 0 ]; then echo "r038 check PASS (overall_pass=true)"; else echo "r038 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{r038/,metrics/r038.json,log/,config/}"
exit "${RC_C}"
