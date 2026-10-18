#!/usr/bin/env python3
"""ground_bringup —— W5 地面链路一键启动（spec W5-a/b/c）。

起：静态 TF（`car/base_link → car/lidar_link`）+ Nav2 `collision_monitor` 安全层
+ 地面执行中继（唯一执行口）+ **命令源**（`command_source` 单一选择，二者互斥）。

链路：命令源 → `/agc/ground/cmd_vel_raw` → `collision_monitor` → `/agc/ground/cmd_vel`
      → `ground_drive_relay` → GZ（`/model/agc_car/cmd_vel`）。

命令源（`command_source`，互斥；同一时刻仅一路下发，spec R7/R8）：
  * `rl`       —— `agc_rl` 的 RL 推理节点（主控，ADR-0004）。RL **不消费** costmap
                  （仅订 odom/scan/goal/state，见 `agc_rl.inference_node` / `agc_rl.interface`），
                  故**不**起独立 costmap 进程（按命令源裁剪，[ADR-0060](../../../docs/adr/0060-nav2-absorption-applicability-and-command-source-trimming.md)）；
                  仅安全层 + 唯一执行口中继。
  * `baseline` —— `agc_ground` 的离线对照基线（Nav2 planner 插件 + `agc_control` LQR 插件）；
                  基线自持内建 costmap（同一 `config/costmap.yaml` 参数源），故不再单起 costmap 节点。

参数单一来源：`config/ground.yaml`（安全层 / 中继）、`config/costmap.yaml`（分层 costmap，基线自持）、
`config/baseline.yaml`（基线）、`agc_rl/config/rl.yaml`（RL）（契约 #params）。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 传感器几何（与 agc_sim/models/agc_car/agc_car.sdf.xacro 的 car/lidar_link pose 一致）
LIDAR_X, LIDAR_Y, LIDAR_Z = 0.0, 0.0, 0.48
# gz 点云/LaserScan 的 header.frame_id 为 gz 作用域名（`<model>/<link>/<sensor>`），非契约帧名；
# 以单位静态 TF 将其接到契约帧 `car/lidar_link`（见 spec R2）。
GZ_LIDAR_FRAME = 'agc_car/car/lidar_link/lidar3d'


def _nodes(context, *args, **kwargs):
    command_source = LaunchConfiguration('command_source').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    ground_share = get_package_share_directory('agc_ground')
    rl_share = get_package_share_directory('agc_rl')
    params_file = os.path.join(ground_share, 'config', 'ground.yaml')
    costmap_file = os.path.join(ground_share, 'config', 'costmap.yaml')
    baseline_file = os.path.join(ground_share, 'config', 'baseline.yaml')

    common = [params_file, {'use_sim_time': use_sim_time}]

    actions = [
        # R2：传感器系静态 TF（collision_monitor 需将扫描变换到 base_frame）
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='car_lidar_tf',
            arguments=[
                '--x', str(LIDAR_X), '--y', str(LIDAR_Y), '--z', str(LIDAR_Z),
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'car/base_link', '--child-frame-id', 'car/lidar_link',
            ],
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='car_lidar_gz_frame_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'car/lidar_link', '--child-frame-id', GZ_LIDAR_FRAME,
            ],
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        ),
        # R-007：自体过滤（激光自返回置 inf）——安全层消费 /agc/ground/scan_filtered
        Node(
            package='agc_ground', executable='scan_self_filter',
            name='scan_self_filter', parameters=common, output='screen',
        ),
        # R1：安全层（Nav2 collision_monitor，scan 源 = 自滤后扫描；直接复用 Nav2 组件）
        Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', parameters=common, output='screen',
        ),
        # 唯一执行口中继
        Node(
            package='agc_ground', executable='ground_drive_relay',
            name='ground_drive_relay', parameters=common, output='screen',
        ),
    ]

    if command_source == 'baseline':
        # R8：离线对照基线；自持内建 costmap（参数取 costmap.yaml 的 `costmap:` 段）
        actions.append(Node(
            package='agc_ground', executable='ground_baseline',
            name='ground_baseline',
            parameters=[baseline_file, costmap_file, {'use_sim_time': use_sim_time}],
            output='screen',
        ))
        managed = ['collision_monitor', 'ground_baseline']
    else:
        # W5-b：命令源 = agc_rl 推理节点（include agc_rl 的 launch）。
        # S-04（ADR-0060）：RL 主控**不消费** costmap → **不**起闲置的独立 `nav2_costmap_2d` 进程；
        # 分层 costmap 仅由离线基线（`command_source=baseline`）自持（见上分支）。
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rl_share, 'launch', 'rl_inference.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ))
        managed = ['collision_monitor']

    # lifecycle 编排：复用 Nav2 lifecycle_manager 统一 configure→activate（ADR-0008 吸收点）
    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_ground',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': managed}],
        output='screen',
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'command_source', default_value='rl',
            description='命令源（互斥）：rl（RL 主控，ADR-0004）| baseline（离线对照基线，R8）'),
        OpaqueFunction(function=_nodes),
    ])
