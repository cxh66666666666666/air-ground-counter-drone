#!/usr/bin/env bash
# run_w7_replay —— W7-a 单 worker 可重放（T4 · R4）一键验收。
#   用法：scripts/archive/run_w7_replay.sh [run_id]
#   起 w6.launch.py backend:=lightweight command_source:=train（外部适配层作命令源），
#   对同一 (scene, episode) 跑 2 个 attempt，比对 seed 计划 / spawn / 噪声 / 动作序列是否一致。
# 注：本机 DDS 坑——**必须**默认 ROS_DOMAIN_ID=0；graph 查询不可靠；首个服务调用常超时。
#     清理**禁止** pkill -f "<名字>"（会匹配 shell 自身命令行 → 自杀）。
set -o pipefail

ROOT=/home/chen/AGC
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# gz 系统插件（agc_sim 自研 SDF 插件）——与 W6 各 runner 同口径；缺失则 launch 立即报错。
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ROOT}/install/agc_sim/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

RUN_ID="${1:-$(date +%Y%m%d)-w7-replay-$(date +%H%M%S)-$(printf '%04d' $((RANDOM % 10000)))}"
echo "W7-a 可重放：run_id=${RUN_ID}，domain=${ROS_DOMAIN_ID}"
python3 "${ROOT}/scripts/w7_episode_runner.py" \
  --scene s01_static_none_nominal_track \
  --master-seed 7 --attempts 2 \
  --run-id "${RUN_ID}"
