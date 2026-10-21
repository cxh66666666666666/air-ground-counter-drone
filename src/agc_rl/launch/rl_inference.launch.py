#!/usr/bin/env python3
"""rl_inference —— W5-b RL 推理节点启动（供上层 launch include）。

参数单一来源 `config/rl.yaml`（契约 #params）。由 `agc_ground` 的 ground_bringup 在
`command_source: rl` 时 include。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *args, **kwargs):
    share = get_package_share_directory('agc_rl')
    params_file = os.path.join(share, 'config', 'rl.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # W7-b（P2 臂 B）：`mode=high_level` 起**高层决策节点**（只出目标点，低层另起）；
    # 缺省 / `mode=low_level` 起既有 RL 主控（写 cmd_vel_raw）。
    mode = LaunchConfiguration('mode').perform(context)
    if mode == 'high_level':
        return [Node(
            package='agc_rl', executable='agc_rl_high_level',
            name='agc_rl_high_level',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen',
        )]
    return [Node(
        package='agc_rl', executable='ground_policy_rl',
        name='ground_policy_rl',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mode', default_value='low_level',
                              description='low_level（RL 主控写 cmd_vel_raw）| high_level（只出目标点）'),
        OpaqueFunction(function=_nodes),
    ])
