#!/usr/bin/env bash
# run_w7_baseline —— W7-a 规则基线可达性（T5 · R5）一键验收。
#   用法：scripts/archive/run_w7_baseline.sh <scene> [run_id]
#   复用 W5-c 的**离线对照基线**（Navfn + 自研 LQR，command_source:=baseline）起轻量栈，
#   由 episode_runner 跑一条 run 达任务出口（或明确的不可达 / 超时分类）。
#   命名纪律（R-012）：称「Navfn + 自研 LQR」，**不**称「Nav2 controller 完整配置」。
# 注：本机 DDS 坑——**必须**默认 ROS_DOMAIN_ID=0；graph 查询不可靠；首个服务调用常超时。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

SCENE="${1:-s01_static_none_nominal_track}"
RUN_ID="${2:-$(date +%Y%m%d)-w7-baseline-$(date +%H%M%S)-$(printf '%04d' $((RANDOM % 10000)))}"
echo "W7-a 规则基线可达性：scene=${SCENE} run_id=${RUN_ID} domain=${ROS_DOMAIN_ID}"
python3 "${ROOT}/scripts/episode_runner.py" \
  --scenes "${SCENE}" --seeds 0 --command-source baseline \
  --campaign w7-rl --run-id "${RUN_ID}" \
  --mode headless --timeout 150 --ready-timeout-s 90
