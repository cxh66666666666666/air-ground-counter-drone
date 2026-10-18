#!/usr/bin/env bash
# lock_px4_failsafe_params —— R-019 / ADR-0034 运行时 PX4 failsafe 参数锁定。
#
#   用途：把 `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` / `NAV_DLL_ACT` 的**确定性值**注入 SITL 启动
#         参数文件，使**每次 SITL 启动**的 failsafe 行为确定化（不以源码默认代填运行值）。
#   真源：src/agc_task/config/px4_failsafe_lock.yaml（**单一真源**，非 ROS 节点参数）。
#   机制：w6.launch.py backend:=full 以 cwd=<build>/px4_sitl_default 起 PX4；rcS 末段 `. px4-rc.params`
#         （PATH 含 `${R}etc/init.d-posix`，R=<pwd>/）→ source 本脚本写出的
#         `<build>/etc/init.d-posix/px4-rc.params`。
#   本脚本只**运行时写入 build 产物**，**不**改 PX4 源码（只读）。
#
#   行为：写前若 `.agc-orig` 不存在则备份一次原文件；SITL 目录不存在则告警并 exit 0
#         （不阻断 launch）；最后把 NAME=VALUE 三行打印到 stdout（供 launch 日志 / 证据）。
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
YAML="${ROOT}/src/agc_task/config/px4_failsafe_lock.yaml"
PARAMS_DIR="${ROOT}/toolchain/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix"
OUT="${PARAMS_DIR}/px4-rc.params"

if [ ! -f "${YAML}" ]; then
  echo "WARN [lock_px4_failsafe_params] lock yaml not found: ${YAML} → skip (exit 0)"
  exit 0
fi
if [ ! -d "${PARAMS_DIR}" ]; then
  echo "WARN [lock_px4_failsafe_params] SITL params dir not found: ${PARAMS_DIR} → skip (exit 0)"
  exit 0
fi

# 备份原文件一次（.agc-orig）；已存在则不再覆盖备份（保留首次原始快照）
if [ -f "${OUT}" ] && [ ! -f "${OUT}.agc-orig" ]; then
  cp -a "${OUT}" "${OUT}.agc-orig"
fi

python3 - "${YAML}" "${OUT}" <<'PY'
import sys
import yaml

yaml_path, out_path = sys.argv[1], sys.argv[2]
with open(yaml_path) as f:
    cfg = yaml.safe_load(f)
cot = cfg['com_of_loss_t']
cob = cfg['com_obl_rc_act']
dll = cfg['nav_dll_act']
content = (
    "# 本文件由 scripts/lock_px4_failsafe_params.sh 生成（R-019 / ADR-0034）。\n"
    "# 单一真源：src/agc_task/config/px4_failsafe_lock.yaml（勿手改本文件）。\n"
    "param set COM_OF_LOSS_T %s\n"
    "param set COM_OBL_RC_ACT %s\n"
    "param set NAV_DLL_ACT %s\n"
    % (cot, cob, dll)
)
with open(out_path, 'w') as f:
    f.write(content)
print("COM_OF_LOSS_T=%s" % cot)
print("COM_OBL_RC_ACT=%s" % cob)
print("NAV_DLL_ACT=%s" % dll)
PY
