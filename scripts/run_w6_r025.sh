#!/usr/bin/env bash
# run_w6_r025 —— 评审 R-025（整链覆盖矩阵 + 同一 run 因果记录 + pass 命题化）一键验收
#   用法：scripts/run_w6_r025.sh [headless|gui] [run_id]
#   例：  scripts/run_w6_r025.sh headless 20261016-w6-r025-0
#
# 三阶段：
#   Phase A 轻量 campaign：episode_runner.py --run-id <RUN_ID>/lightweight
#     → 产物落 <RUN_ID>/lightweight/report.json（runner 原生输出路径本就是
#       experiments/w6-task-eval/<run_id>/report.json；此处以带 `/lightweight` 的 run_id
#       让**原生输出**恰落 <RUN_ID>/lightweight/，check 直接读该路径——**二选一，保持一致**）。
#   Phase B 整链（双 PX4，headless）：setsid 起 w6.launch.py backend:=full → 等 ~55 s →
#     w6_r025_full_probe.py → <RUN_ID>/same_run.json（同一 run 因果记录）。允许有限重试（≤3）。
#   Phase C 自包含核查：w6_r025_check.py → <RUN_ID>/metrics/r025.json + coverage_matrix.md。
#   返回码反映 overall_pass。
#
# 注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
#     整链世界需 GZ_SIM_SYSTEM_PLUGIN_PATH（world_bringup 追加而非覆盖），此处显式导出。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

MODE="${1:-headless}"
RUN_ID="${2:-$(date +%Y%m%d)-w6-r025-0}"
SCENE="s08_static_none_nominal_capture"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log"

LAUNCH_PGID=""
cleanup_full() {
  # 进程组 kill 为主（setsid 使后台作业成为独立进程组，PGID == $!）
  if [ -n "${LAUNCH_PGID}" ]; then kill -INT -"${LAUNCH_PGID}" 2>/dev/null; fi
  sleep 3
  if [ -n "${LAUNCH_PGID}" ]; then kill -TERM -"${LAUNCH_PGID}" 2>/dev/null; fi
  # 兜底：pkill 模式对可能命中自身命令行者加 [x] 规避
  pkill -TERM -f "[w]6.launch.py" 2>/dev/null
  pkill -TERM -f "[a]gc_world.sdf" 2>/dev/null
  pkill -TERM -f "[b]in/px4" 2>/dev/null
  pkill -TERM -f "[M]icroXRCEAgent" 2>/dev/null
  sleep 3
  if [ -n "${LAUNCH_PGID}" ]; then kill -KILL -"${LAUNCH_PGID}" 2>/dev/null; fi
  pkill -KILL -f "[w]6.launch.py" 2>/dev/null
  pkill -KILL -f "[a]gc_world.sdf" 2>/dev/null
  pkill -KILL -f "[b]in/px4" 2>/dev/null
  pkill -KILL -f "[M]icroXRCEAgent" 2>/dev/null
  LAUNCH_PGID=""
}
trap 'cleanup_full' EXIT

# 判定已产出「完整同一 run 因果记录」（full_chain_executed=true 且 5 问字段齐全）→ 可停止重试
record_valid() {
  python3 - "${OUT}/same_run.json" <<'PY'
import json, os, sys
p = sys.argv[1]
if not os.path.exists(p):
    sys.exit(1)
d = json.load(open(p))
c = d.get('causal_record') or {}
ok = bool(d.get('full_chain_executed')) and isinstance(c.get('start_ok'), bool) and \
    all(k in (c.get('own_takeover') or {}) for k in
        ('release_requested', 'takeover_ok', 'active_controller_before', 'active_controller_after')) and \
    isinstance((c.get('target_motion') or {}).get('classified'), str) and \
    isinstance((c.get('capture') or {}).get('observed'), bool) and \
    isinstance((c.get('jam') or {}).get('observed'), bool) and \
    isinstance((c.get('terminal') or {}).get('reached'), bool)
sys.exit(0 if ok else 1)
PY
}

# 清理残留（"[x]xx" 写法规避匹配 shell 自身）
pkill -TERM -f "[w]6.launch.py" 2>/dev/null
pkill -TERM -f "[b]in/px4" 2>/dev/null
pkill -TERM -f "[M]icroXRCEAgent" 2>/dev/null
sleep 1

# ---------------- Phase A：轻量 campaign ----------------
echo "[Phase A] 轻量 campaign（scene=s01,s08 seed=0 mode=${MODE}）→ ${RUN_ID}/lightweight/"
python3 "${ROOT}/scripts/episode_runner.py" \
  --scenes s01_static_none_nominal_track,s08_static_none_nominal_capture \
  --seeds 0 --mode "${MODE}" --run-id "${RUN_ID}/lightweight" \
  2>&1 | tee "${OUT}/log/phase-a-lightweight.log"
RC_A=${PIPESTATUS[0]}
echo "[Phase A] rc=${RC_A}"

# ---------------- Phase B：整链（双 PX4，headless） ----------------
BEST_ATTEMPT=""
for attempt in 1 2 3; do
  echo "[Phase B] 整链 attempt=${attempt}（scene=${SCENE}, headless）"
  setsid ros2 launch agc_task w6.launch.py backend:=full \
    scene:="${SCENE}" seed:=0 gz_args:="-s -r -v 3 ${WORLD}" \
    > "${OUT}/log/launch-attempt-${attempt}.log" 2>&1 &
  LAUNCH_PGID=$!
  echo "[Phase B] 等待双 PX4 + 驱动就绪 ~55 s ..."
  sleep 55
  python3 "${ROOT}/scripts/w6_r025_full_probe.py" --run-id "${RUN_ID}" --scene "${SCENE}" \
    --mode "${MODE}" 2>&1 | tee "${OUT}/log/probe-attempt-${attempt}.log"
  RC_P=${PIPESTATUS[0]}
  # 每次产物如实留档
  if [ -f "${OUT}/same_run.json" ]; then
    cp "${OUT}/same_run.json" "${OUT}/log/same_run-attempt-${attempt}.json"
  fi
  echo "[Phase B] probe rc=${RC_P}（attempt ${attempt}）"
  cleanup_full
  if [ "${RC_P}" -eq 0 ]; then
    BEST_ATTEMPT="${attempt}"
    echo "[Phase B] 整链 pass（attempt ${attempt}）"
    break
  fi
  if record_valid; then
    BEST_ATTEMPT="${attempt}"
    echo "[Phase B] 已有完整同一 run 因果记录（attempt ${attempt}，命题 pass=${RC_P}）→ 停止重试"
    break
  fi
  echo "[Phase B] 整链 attempt=${attempt} 未取得完整因果记录 → 重试"
  sleep 2
done
# 收束 same_run.json：优先用成功 attempt 的留档；否则保留最后一次（如实）
if [ -n "${BEST_ATTEMPT}" ] && [ -f "${OUT}/log/same_run-attempt-${BEST_ATTEMPT}.json" ]; then
  cp "${OUT}/log/same_run-attempt-${BEST_ATTEMPT}.json" "${OUT}/same_run.json"
fi

# ---------------- Phase C：自包含核查 ----------------
echo "[Phase C] 覆盖矩阵 / 因果记录核查 → metrics/r025.json"
python3 "${ROOT}/scripts/w6_r025_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-c-check.log"
RC_C=${PIPESTATUS[0]}

if [ "${RC_C}" -eq 0 ]; then
  echo "r025 check PASS (overall_pass=true)"
else
  echo "r025 check FAIL (overall_pass=false)"
fi
echo "产物：${RUN_ID}/{lightweight/,same_run.json,coverage_matrix.md,metrics/r025.json,log/}"
exit "${RC_C}"
