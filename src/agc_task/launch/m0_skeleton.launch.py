#!/usr/bin/env python3
"""m0_skeleton —— M0 最小端到端闭环顶层 launch。

一条命令起全链：include agc_sim 的世界+桥，再起 odom 适配、地面执行链、
门控/复位与 run logger。参数单一来源 src/agc_task/config/m0.yaml。
验收跑有头（GUI）；无头可传 gz_args:='-s -r -v 3 <world>'。

`command_source` 透传至 agc_ground 的 ground_bringup（W5 命令源互斥选择）：
  * rl（默认）—— `agc_rl` 推理节点为主控（ADR-0004）；
  * baseline  —— `agc_ground` 的离线对照基线（R8）。

W6 顶层入口已统一为 `w6.launch.py`（`backend` 参数；见 docs/adr/0061-*.md），
本文件保留为**骨架基座**供其 include（轻量栈）。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('agc_sim')
    task_share = get_package_share_directory('agc_task')
    perception_share = get_package_share_directory('agc_perception')
    params_file = LaunchConfiguration('params_file')
    world_file = os.path.join(sim_share, 'worlds', 'agc_world.sdf')
    default_params = os.path.join(task_share, 'config', 'm0.yaml')

    world_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'world_bringup.launch.py')),
        launch_arguments={
            'gz_args': LaunchConfiguration('gz_args'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # R-026：按障碍轴选择世界（none 生成不含 pole_* 的世界）
            'world_obstacles': LaunchConfiguration('world_obstacles'),
        }.items(),
    )

    # W4 感知适配层（真值旁路 + 地面 odom 归一 + 目标源）；ground_odom_adapter 自 W4 起
    # 由 agc_perception 提供（位姿来源改为真值旁路），M0 不再直接起该节点。
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'perception_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # W7-b（P1）：观测来源（lightweight 缺省 gt_adapter；可被上层覆盖）
            'target_source': LaunchConfiguration('target_source'),
            # R-026：传感器通道 / 障碍真值文件按场景轴由上层选择并单一注入
            'sensors_file': LaunchConfiguration('sensors_file'),
            'obstacles_file': LaunchConfiguration('obstacles_file'),
            # S-06（ADR-0062）：地图几何文件由上层经单一真源派生注入
            'map_file': LaunchConfiguration('map_file'),
        }.items(),
    )

    # W5 地面链路（安全层 collision_monitor + 中继 + 命令源）；替代 M0 的 safety stub
    ground_share = get_package_share_directory('agc_ground')
    ground_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ground_share, 'launch', 'ground_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'command_source': LaunchConfiguration('command_source'),
            # W7-b（T1/E1）：安全层开关透传（缺省 true；false = 无 shield 对照臂，纯仿真）
            'shield': LaunchConfiguration('shield'),
        }.items(),
    )

    common = [params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # ground_controller 字段随 command_source 切换（rl → agc_rl；baseline → agc_ground）
    controller_name = PythonExpression([
        "'agc_rl' if '", LaunchConfiguration('command_source'), "' == 'rl' else 'agc_ground'"])

    nodes = [
        Node(package='agc_task', executable='scenario_reset',
             name='scenario_reset', parameters=common, output='screen'),
        Node(package='agc_task', executable='task_manager',
             name='task_manager',
             parameters=common + [{'ground_controller_name': controller_name}],
             output='screen'),
        Node(package='agc_task', executable='run_logger',
             name='run_logger',
             parameters=common + [
                 # R-026：scene_manifest（声明轴 + 承载者 + 实际生效参数）与合并后参数文件
                 # 经 run_logger 拷入 run 的 config/（缺省空 = 不拷贝，不影响既有调用方）
                 {'scene_manifest_file': LaunchConfiguration('scene_manifest_file'),
                  # R-029（ADR-0044）：世界模型文件 → run_logger 记 sha256（工具链身份）
                  'world_file': LaunchConfiguration('world_file'),
                  'params_file': LaunchConfiguration('params_file'),
                  # W7-b（T1/E1）：安全层开关（字符串承载→节点内解析）——false 时 action 新鲜度门不适用
                  'shield_layer_enabled': LaunchConfiguration('shield')}],
             output='screen'),
    ]

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='任务层参数文件（默认 config/m0.yaml；W5 场景用 config/w5_scenario.yaml）'))
    ld.add_action(DeclareLaunchArgument(
        'gz_args', default_value=['-r -v 3 ', world_file],
        description='gz sim 参数（加 -s 可无头）'))
    ld.add_action(DeclareLaunchArgument(
        'command_source', default_value='rl',
        description='W5 地面命令源（互斥）：rl | baseline'))
    ld.add_action(DeclareLaunchArgument(
        'target_source', default_value='gt_adapter',
        description='W7-b P1：观测来源（air_fusion / gt_adapter）'))
    # W7-b（T1/E1）：安全层开关透传（缺省 true；false = 无 shield 对照臂，纯仿真）
    ld.add_action(DeclareLaunchArgument(
        'shield', default_value='true',
        description='安全层开关：true=起 collision_monitor（默认）；false=无 shield 对照臂'))
    # R-026：场景轴承载者接线（缺省 = 既有行为：static 世界 + 标称通道 + 默认障碍 + 无 manifest）
    ld.add_action(DeclareLaunchArgument(
        'world_obstacles', default_value='static',
        description='障碍轴世界生成：static | none（由上层按场景轴传入）'))
    ld.add_action(DeclareLaunchArgument(
        'sensors_file',
        default_value=os.path.join(perception_share, 'config', 'perception_sensors.yaml'),
        description='传感器通道文件（由上层按感知轴传入）'))
    ld.add_action(DeclareLaunchArgument(
        'obstacles_file',
        default_value=os.path.join(perception_share, 'config', 'obstacles.yaml'),
        description='障碍真值文件（由上层按障碍轴传入）'))
    ld.add_action(DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(perception_share, 'config', 'map.yaml'),
        description='S-06 地图几何文件（缺省 = 包内回退副本；由上层按真源派生传入）'))
    ld.add_action(DeclareLaunchArgument(
        'scene_manifest_file', default_value='',
        description='R-026 scene_manifest.json 路径（由上层生成；空 = 不落）'))
    ld.add_action(DeclareLaunchArgument(
        'world_file', default_value=world_file,
        description='R-029 世界模型文件路径（run_logger 记 sha256；缺省 = agc_world.sdf）'))
    ld.add_action(world_bringup)
    ld.add_action(perception)
    ld.add_action(ground_bringup)
    for n in nodes:
        ld.add_action(n)
    return ld
