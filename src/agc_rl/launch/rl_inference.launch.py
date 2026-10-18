#!/usr/bin/env python3
"""rl_inference —— W5-b RL 推理节点启动（供上层 launch include）。

参数单一来源 `config/rl.yaml`（契约 #params）。由 `agc_ground` 的 ground_bringup 在
`command_source: rl` 时 include。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('agc_rl')
    params_file = os.path.join(share, 'config', 'rl.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='agc_rl', executable='ground_policy_rl',
            name='ground_policy_rl',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
