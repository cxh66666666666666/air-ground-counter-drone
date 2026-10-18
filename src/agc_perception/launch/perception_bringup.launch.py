#!/usr/bin/env python3
"""perception_bringup —— W4 感知与融合一键启动（spec R3/R8）。

由上层 launch（air_bringup / m0_skeleton）**include**；参数单一来源 `config/perception.yaml`。
目标源节点（gt_adapter / air_fusion）由本 launch 提供，`/agc/target/source` 由此单一设置。

分段：
  * W4-a：gt_bypass（真值旁路）+ ground_odom_adapter + gt_adapter
  * W4-b：air_perception + air_fusion
  * W4-c：map_adapter
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('agc_perception')
    params_file = os.path.join(share, 'config', 'perception.yaml')
    default_sensors_file = os.path.join(share, 'config', 'perception_sensors.yaml')
    default_obstacles_file = os.path.join(share, 'config', 'obstacles.yaml')
    default_map_file = os.path.join(share, 'config', 'map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    target_source = LaunchConfiguration('target_source')
    obstacles_source = LaunchConfiguration('obstacles_source')
    # R-026：传感器通道 / 障碍真值文件由上层按场景轴选择并单一注入（契约 #params）。
    sensors_file = LaunchConfiguration('sensors_file')
    obstacles_file = LaunchConfiguration('obstacles_file')
    # S-06（ADR-0062）：地图几何文件由上层经**单一真源**派生注入（缺省 = 包内回退副本）。
    map_file = LaunchConfiguration('map_file')
    # R-040（ADR-0055）：场景 / 源版本 → 折入地图内容身份 hash（诊断，非 ABI）
    scenario_version = LaunchConfiguration('scenario_version')

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('target_source', default_value='gt_adapter',
                              description='契约 /agc/target/source：gt_adapter | air_fusion'),
        DeclareLaunchArgument('obstacles_source', default_value='air_fusion',
                              description='契约 /agc/obstacles/source：air_fusion | config'),
        DeclareLaunchArgument('sensors_file', default_value=default_sensors_file,
                              description='R-026 传感器通道文件（缺省 = 包内默认；由上层按场景轴选择）'),
        DeclareLaunchArgument('obstacles_file', default_value=default_obstacles_file,
                              description='R-026 障碍真值文件（缺省 = 包内默认；none 场景传空障碍文件）'),
        DeclareLaunchArgument('map_file', default_value=default_map_file,
                              description='S-06 地图几何文件（缺省 = 包内回退副本；由上层按真源派生注入）'),
        DeclareLaunchArgument('scenario_version', default_value='',
                              description='R-040 场景 / 源版本（由上层按场景轴选择；折入地图内容身份 hash）'),
    ]

    common = [params_file, {'use_sim_time': use_sim_time}]

    nodes = [
        # ---- W4-a · 适配层 ----
        Node(package='agc_perception', executable='gt_bypass',
             name='gt_bypass', parameters=common, output='screen'),
        Node(package='agc_perception', executable='ground_odom_adapter',
             name='ground_odom_adapter', parameters=common, output='screen'),
        Node(package='agc_perception', executable='gt_adapter',
             name='gt_adapter',
             parameters=common + [{'target_source': target_source}],
             output='screen'),
        # ---- W4-b · 感知与融合 ----
        Node(package='agc_perception', executable='air_perception',
             name='air_perception',
             parameters=common + [{'sensors_file': sensors_file,
                                   'obstacles_file': obstacles_file}],
             output='screen'),
        Node(package='agc_perception', executable='air_fusion',
             name='air_fusion',
             parameters=common + [{'target_source': target_source,
                                   'obstacles_source': obstacles_source}],
             output='screen'),
        # ---- W4-c · 地图 ----
        Node(package='agc_perception', executable='map_adapter',
             name='map_adapter',
             parameters=common + [{'map_file': map_file,
                                   'obstacles_file': obstacles_file,
                                   'obstacles_source': obstacles_source,
                                   'scenario_version': scenario_version}],
             output='screen'),
    ]

    ld = LaunchDescription()
    for a in declared:
        ld.add_action(a)
    for n in nodes:
        ld.add_action(n)
    return ld
