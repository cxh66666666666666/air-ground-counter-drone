#!/usr/bin/env bash
# run_w6_r037 —— 评审 R-037（传感器/车体限制声明与能力标定）一键验收
#   用法：scripts/run_w6_r037.sh [run_id]
#   例：  scripts/run_w6_r037.sh 20261017-w6-r037-0
#
# 阶段：
#   Phase A **实跑** R-037 能力标定台架（w6_r037_bench.py）：起**最小 gz 场景**
#           （world_bringup：世界 + ros_gz 桥 + spawn **换装后** agc_car）+ **唯一执行口中继**
#           ground_drive_relay + **真值旁路** gt_bypass；经唯一执行口 /agc/ground/cmd_vel 标定
#           ① 满速制动距离/耗时 ② 转弯扫掠/摩擦敏感性 ③ 朝 pole_1 接触真值。
#           产物落 <RUN_ID>/r037/bench.json。
#   Phase B 自包含核查（w6_r037_check.py）→ <RUN_ID>/r037/check.json（+ metrics/r037.json）。
#
# 注：清理**禁止** `rm -rf` 与裸名 `pkill`（会匹配自身命令行 → 自杀）；用 `pgrep -f "路径片断[y]"`
#     写法定位真实进程再按 PID kill。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

RUN_ID="${1:-20261017-w6-r037-0}"
OUT="${ROOT}/experiments/w6-task-eval/${RUN_ID}"
mkdir -p "${OUT}/log" "${OUT}/metrics" "${OUT}/r037" "${OUT}/config"

# world_bringup.launch.py 以 EnvironmentVariable 追加 GZ_SIM_SYSTEM_PLUGIN_PATH / GZ_SIM_RESOURCE_PATH，
# 须先存在（照 scripts/run_w5_r009.sh）；显式导出 GZ 侧末端守门人插件库路径。
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"

WORLD="${ROOT}/install/agc_sim/share/agc_sim/worlds/agc_world.sdf"
GZ="-s -r -v 3 ${WORLD}"

# R-037 之前的安装模型基线（供 check (f) 核验物理/惯性/驱动插件未被改动）；
# 仅当缺失时从 install 拷贝，供本 run **首次**落基线（写入后不再覆盖）。
BASELINE="${OUT}/r037/xacro_baseline.sdf.xacro"
if [ ! -f "${BASELINE}" ]; then
  cp "${ROOT}/install/agc_sim/share/agc_sim/models/agc_car/agc_car.sdf.xacro" "${BASELINE}" 2>/dev/null || true
fi

cleanup() {
  # [x]xx 写法：模式仅命中真实进程，不误杀其它会话 / 自身。
  for pat in "world_bringup.launc[h].py" "agc_worl[d].sdf" "ros_gz_sim/[c]reate" \
             "ros_gz_bridge/[p]arameter_bridge" "install/agc_perception/lib/agc_perception/[g]t_bypass" \
             "install/agc_ground/lib/agc_ground/[g]round_drive_relay" "gz [s]im"; do
    for p in $(pgrep -f "${pat}" 2>/dev/null); do
      kill -9 "${p}" 2>/dev/null
    done
  done
}
trap 'cleanup' EXIT
cleanup
sleep 1

# ---------------- Phase A：起最小 gz 场景 + 台架 ----------------
echo "[Phase A] R-037 能力标定台架（最小 gz 场景）→ ${RUN_ID}/r037/bench.json"
setsid ros2 launch agc_sim world_bringup.launch.py gz_args:="${GZ}" \
  > "${OUT}/log/world_bringup.log" 2>&1 &
PGID=$!
echo "[Phase A] 世界/桥/spawn 起（pgid=${PGID}）；等待链路就绪 16 s ..."
sleep 16

setsid "${ROOT}/install/agc_ground/lib/agc_ground/ground_drive_relay" --ros-args \
  -p use_sim_time:=true -p in_topic:=/agc/ground/cmd_vel -p out_topic:=/model/agc_car/cmd_vel \
  -p watchdog_s:=0.3 -p expected_frame_id:=car/base_link -p max_sample_age_s:=0.5 \
  -p frame_id_tolerance:=0.05 -p max_linear_velocity:=1.5 -p max_angular_velocity:=2.0 \
  > "${OUT}/log/relay.log" 2>&1 &
setsid "${ROOT}/install/agc_perception/lib/agc_perception/gt_bypass" --ros-args \
  -p use_sim_time:=true -p world_name:=agc_world -p rate_hz:=30.0 \
  > "${OUT}/log/gt_bypass.log" 2>&1 &
sleep 3

python3 "${ROOT}/scripts/w6_r037_bench.py" --run-id "${RUN_ID}" \
  > "${OUT}/log/phase-a-bench.log" 2>&1
echo "$?" > "${OUT}/r037/bench_rc.txt"
echo "[Phase A] bench rc=$(cat "${OUT}/r037/bench_rc.txt")"
cleanup
trap - EXIT

# ---------------- Phase B：自包含核查 ----------------
echo "[Phase B] 核查 → r037/check.json + metrics/r037.json"
python3 "${ROOT}/scripts/w6_r037_check.py" --run-id "${RUN_ID}" \
  2>&1 | tee "${OUT}/log/phase-b-check.log"
RC_B=${PIPESTATUS[0]}

if [ "${RC_B}" -eq 0 ]; then echo "r037 check PASS (overall_pass=true)"; else echo "r037 check FAIL (overall_pass=false)"; fi
echo "产物：${RUN_ID}/{r037/,metrics/r037.json,log/}"
exit "${RC_B}"
