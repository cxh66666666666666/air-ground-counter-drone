#!/usr/bin/env python3
"""w5_safety_scenario —— W5 脚本级最小场景（spec W5-c R9）。

复用 M0 顶层 launch（门控 `/agc/task/start`、`/agc/task/state` 与 `/agc/scenario/reset`），
仅替换任务层参数为 `config/w5_scenario.yaml`（车置于立柱前方 5 m、goal 越过立柱）。
**不**自建 W6 业务级 12 场景。

用法：
  ros2 launch agc_task w5_safety_scenario.launch.py                # GUI
  ros2 launch agc_task w5_safety_scenario.launch.py gz_args:="-s -r -v 3 <world>"   # 无头
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    task_share = get_package_share_directory('agc_task')
    sim_share = get_package_share_directory('agc_sim')
    params_file = os.path.join(task_share, 'config', 'w5_scenario.yaml')
    world_file = os.path.join(sim_share, 'worlds', 'agc_world.sdf')

    m0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(task_share, 'launch', 'm0_skeleton.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'gz_args': LaunchConfiguration('gz_args'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'command_source': LaunchConfiguration('command_source'),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'gz_args', default_value=['-r -v 3 ', world_file],
        description='gz sim 参数（加 -s 可无头）'))
    ld.add_action(DeclareLaunchArgument('command_source', default_value='rl'))
    ld.add_action(m0)
    return ld
