# PX4 链路采用 uXRCE-DDS + px4_msgs，弃用 MAVROS

## Status
accepted

## 背景与决策
空中链路以 Micro-XRCE-DDS Agent + `px4_msgs`（release/1.15，与固件同版）直连 PX4 uORB，弃用 MAVROS / MAVLink 中间层。

## Considered Options
- MAVROS(2) 桥接 —— 被否：多一层序列化、延迟更高、且非 PX4+ROS2 官方主推方向。

## Consequences
- 节点需自行处理 uORB 语义：微秒时间戳、NED/FRD ↔ ENU 坐标、QoS 配置。
- `px4_ros_com` 仅作示例参考；P0 已建成的 MicroXRCEAgent 与 `px4_msgs` 构建沿用。
