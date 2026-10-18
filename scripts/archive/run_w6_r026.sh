#!/usr/bin/env bash
# run_w6_r026 —— 评审 R-026（场景轴承载者校验与覆盖记账）一键验收
#   用法：scripts/archive/run_w6_r026.sh [run_id]
#   例：  scripts/archive/run_w6_r026.sh 20261016-r026-0
#
# 阶段：
#   Phase A 轻量实跑（逐场景短时 launch）：none/static/linear/maneuver/dropout/fov 各起一次，
#     采集顶层 launch 生成的 scene_manifest 与「none 世界生成」产物 → <RUN_ID>/manifests + worlds/。
#   Phase B dynamic 拒绝运行：scene=s05（obstacle=dynamic）→ 捕获显式原因日志 + 退出码。
#   Phase C 轻量 campaign：episode_runner.py --scenes s01,s05 → 覆盖账本（unsupported / not_carried）。
#   Phase D 自包含核查：w6_r026_check.py → <RUN_ID>/metrics/r026.json。
#   返回码反映 overall_pass。
#
# 注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

RUN_ID="${1:-$(date +%Y%m%d)-r026-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"
mkdir -p "${OUT}/log" "${OUT}/manifests" "${OUT}/worlds"

cleanup() {
  pkill -TERM -f "[w]6.launch.py" 2>/dev/null
  pkill -TERM -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -TERM -f "[a]gc_world.sdf" 2>/dev/null
  pkill -TERM -f "[r]os_gz_bridge" 2>/dev/null
  sleep 1
  pkill -KILL -f "[w]6.launch.py" 2>/dev/null
  pkill -KILL -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -KILL -f "[a]gc_world.sdf" 2>/dev/null
  pkill -KILL -f "[r]os_gz_bridge" 2>/dev/null
}
trap 'cleanup' EXIT

# 清理残留
cleanup

# ---------------- Phase A：轻量实跑（短时 launch → 采 manifest / 世界产物）---------------
run_scene() {
  local scene="$1"; local label="$2"; local secs="${3:-25}"
  echo "[Phase A] scene=${scene}（${label}）launch ~${secs}s"
  timeout -s INT "${secs}" ros2 launch agc_task w6.launch.py \
    backend:=lightweight scene:="${scene}" seed:=0 gz_args:="-s -r -v 3 ${WORLD}" \
    > "${OUT}/log/phase-a-${label}.log" 2>&1
  echo "[Phase A] rc=${?}（timeout 124 正常）"
  # 顶层 launch 生成的 scene_manifest（/tmp/w6_lightweight_manifest_<scene>_0_<token>.json）
  local mf="$(ls -t /tmp/w6_lightweight_manifest_${scene}_0_*.json 2>/dev/null | head -1)"
  if [ -f "${mf}" ]; then cp "${mf}" "${OUT}/manifests/${scene}.json"; else echo "[Phase A] WARN manifest 缺失 ${mf}"; fi
  # 障碍轴世界产物（S-06 / ADR-0062）：none = base world（不含 pole_*）；static = 真源注入生成的世界
  if [ "${label}" = "none" ] && [ -f "${WORLD}" ]; then
    cp "${WORLD}" "${OUT}/worlds/agc_world_no_obstacles.sdf"
  fi
  if [ "${label}" = "static" ] && [ -f /tmp/agc_world_static_obstacles.sdf ]; then
    cp /tmp/agc_world_static_obstacles.sdf "${OUT}/worlds/agc_world_static_obstacles.sdf"
  fi
  cleanup
}

run_scene s01_static_none_nominal_track none 25
run_scene s04_static_static_obs_track static 25
run_scene s02_linear_none_nominal_track linear 20
run_scene s03_maneuver_none_nominal_track maneuver 20
run_scene s06_static_none_dropout_track dropout 20
run_scene s07_linear_none_fov_track fov 20

# ---------------- Phase B：dynamic 轴无承载者 → 拒绝运行 ----------------
echo "[Phase B] scene=s05（obstacle=dynamic）→ 期望拒绝运行"
timeout -s INT 25 ros2 launch agc_task w6.launch.py \
  backend:=lightweight scene:=s05_linear_dynamic_obs_track seed:=0 gz_args:="-s -r -v 3 ${WORLD}" \
  > "${OUT}/log/dynamic_refusal.log" 2>&1
echo "$?" > "${OUT}/log/dynamic_refusal.rc"
echo "[Phase B] rc=$(cat "${OUT}/log/dynamic_refusal.rc")"
cleanup

# ---------------- Phase C：轻量 campaign（覆盖账本 unsupported / not_carried）----------------
echo "[Phase C] episode_runner（s01 + s05[unsupported]）→ ${RUN_ID}/lightweight/"
python3 "${ROOT}/scripts/episode_runner.py" \
  --scenes s01_static_none_nominal_track,s05_linear_dynamic_obs_track \
  --seeds 0 --mode headless --wall-watchdog-s 45 --run-id "${RUN_ID}/lightweight" \
  > "${OUT}/log/phase-c-lightweight.log" 2>&1
echo "[Phase C] rc=$?"
cleanup

# ---------------- Phase D：自包含核查 ----------------
echo "[Phase D] 核查 → metrics/r026.json"
python3 "${ROOT}/tests/w6_r026_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-d-check.log"
RC_D=${PIPESTATUS[0]}

if [ "${RC_D}" -eq 0 ]; then echo "r026 check PASS (overall_pass=true)"; else echo "r026 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{manifests/,worlds/,lightweight/,metrics/r026.json,log/}"
exit "${RC_D}"
