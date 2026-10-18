#!/usr/bin/env bash
# run_w6_r027 —— 评审 R-027（安全不可行率诚实口径与真实接触证据）一键验收
#   用法：scripts/run_w6_r027.sh [run_id]
#   例：  scripts/run_w6_r027.sh 20261016-r027-0
#
# 阶段：
#   Phase A 负对照（阈值外缓停不计碰撞）：episode_runner 跑 s11（车朝 pole_1 直行、安全层提前停车）
#     → 从 run_logger 的 run 目录拷 summary 为 <RUN_ID>/summaries/phase_a_safety_stop.json。
#   Phase B 正例（真实接触计事件）：world_bringup 把 agc_car spawn 到 pole_1（world (0,-6)）几何重叠
#     → GZ 物理产生真实接触；run_logger 订阅 /agc/ground/contact 记真实碰撞事件 →
#     <RUN_ID>/summaries/phase_b_real_contact.json。
#   Phase C 自包含核查：w6_r027_check.py → <RUN_ID>/metrics/r027.json。
#   返回码反映 overall_pass。
#
# 注：清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）；用 "[x]xx" 写法规避。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

RUN_ID="${1:-$(date +%Y%m%d)-r027-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"
mkdir -p "${OUT}/log" "${OUT}/summaries" "${OUT}/metrics"

cleanup() {
  pkill -TERM -f "[w]6_task_eval.launch.py" 2>/dev/null
  pkill -TERM -f "[w]5_safety_scenario.launch.py" 2>/dev/null
  pkill -TERM -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -TERM -f "[w]orld_bringup.launch.py" 2>/dev/null
  pkill -TERM -f "[a]gc_world.sdf" 2>/dev/null
  pkill -TERM -f "[p]arameter_bridge" 2>/dev/null
  pkill -TERM -f "[r]os_gz_bridge" 2>/dev/null
  pkill -TERM -f "[r]un_logger" 2>/dev/null
  sleep 1
  pkill -KILL -f "[w]6_task_eval.launch.py" 2>/dev/null
  pkill -KILL -f "[w]5_safety_scenario.launch.py" 2>/dev/null
  pkill -KILL -f "[m]0_skeleton.launch.py" 2>/dev/null
  pkill -KILL -f "[w]orld_bringup.launch.py" 2>/dev/null
  pkill -KILL -f "[a]gc_world.sdf" 2>/dev/null
  pkill -KILL -f "[p]arameter_bridge" 2>/dev/null
  pkill -KILL -f "[r]os_gz_bridge" 2>/dev/null
  pkill -KILL -f "[r]un_logger" 2>/dev/null
}
trap 'cleanup' EXIT

cleanup

# ---------------- Phase A：负对照——s11 车朝 pole_1 直行、安全层应在阈值外提前停车 ----------------
echo "[Phase A] episode_runner s11（阈值外缓停对照）→ ${RUN_ID}/lightweight/"
python3 "${ROOT}/scripts/episode_runner.py" \
  --scenes s11_static_static_none_unreachable \
  --seeds 0 --mode headless --wall-watchdog-s 60 --run-id "${RUN_ID}/lightweight" \
  > "${OUT}/log/phase-a-lightweight.log" 2>&1
echo "[Phase A] runner rc=$?"
cleanup

# 定位 s11 的 run_logger summary（run_id = <YYYYMMDD>-s11_static_static_none_unreachable-0）
A_SUM="$(ls -t ${ROOT}/experiments/w6-task-eval/*-s11_static_static_none_unreachable-0*/metrics/summary.json 2>/dev/null | head -1)"
if [ -n "${A_SUM}" ] && [ -f "${A_SUM}" ]; then
  cp "${A_SUM}" "${OUT}/summaries/phase_a_safety_stop.json"
  echo "[Phase A] summary ← ${A_SUM}"
else
  echo "[Phase A] WARN 未找到 s11 summary（*-s11_static_static_none_unreachable-0）"
fi

# ---------------- Phase B：正例——spawn 车与 pole_1 几何重叠 → GZ 真实接触 ----------------
echo "[Phase B] world_bringup + run_logger（spawn 重叠 pole_1）"
# 备选记录（据实报告）：本轮采用「spawn 与 pole 几何重叠」产生接触；**未**使用 set_pose 复位写位姿，
# **未**使用任何第二 ROS 命令者（gz topic -p cmd_vel 等一律禁止）。
ros2 launch agc_sim world_bringup.launch.py \
  gz_args:="-s -r -v 3 ${WORLD}" car_x:=0.0 car_y:=-6.0 \
  > "${OUT}/log/phase-b-world.log" 2>&1 &
sleep 8
# 实测并记录 gz contact 话题名（据实采用）
gz topic -l 2>/dev/null | grep -i contact > "${OUT}/log/phase-b-gz-contact-topics.txt" || true
echo "[Phase B] gz contact topics:"; cat "${OUT}/log/phase-b-gz-contact-topics.txt"
# 原始接触消息（ROS 侧）一次
timeout 6 ros2 topic echo --once /agc/ground/contact \
  > "${OUT}/log/phase-b-contact-echo.log" 2>&1 || true
# run_logger 采集约 15s
timeout -s INT 15 ros2 run agc_task run_logger --ros-args \
  -p campaign:=w6-task-eval -p scenario:=r027-contact -p seed:=0 \
  -p contact_topic:=/agc/ground/contact -p a_brake_mps2:=0.9 -p use_sim_time:=true \
  > "${OUT}/log/phase-b-run_logger.log" 2>&1
echo "[Phase B] logger rc=$?"
sleep 2
cleanup

B_SUM="$(ls -t ${ROOT}/experiments/w6-task-eval/*-r027-contact-0*/metrics/summary.json 2>/dev/null | head -1)"
if [ -n "${B_SUM}" ] && [ -f "${B_SUM}" ]; then
  cp "${B_SUM}" "${OUT}/summaries/phase_b_real_contact.json"
  echo "[Phase B] summary ← ${B_SUM}"
else
  echo "[Phase B] WARN 未找到 r027-contact summary"
fi

# ---------------- Phase C：自包含核查 ----------------
echo "[Phase C] 核查 → metrics/r027.json"
python3 "${ROOT}/scripts/w6_r027_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-c-check.log"
RC_C=${PIPESTATUS[0]}

if [ "${RC_C}" -eq 0 ]; then echo "r027 check PASS (overall_pass=true)"; else echo "r027 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{summaries/,metrics/r027.json,log/}"
exit "${RC_C}"
