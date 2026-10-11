#!/usr/bin/env python3
"""m0_skeleton —— M0 最小端到端闭环顶层 launch。

一条命令起全链：include agc_sim 的世界+桥，再起 odom 适配、地面执行链、
门控/复位与 run logger。参数单一来源 src/agc_task/config/m0.yaml。
验收跑有头（GUI）；无头可传 gz_args:='-s -r -v 3 <world>'。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('agc_sim')
    task_share = get_package_share_directory('agc_task')
    params_file = os.path.join(task_share, 'config', 'm0.yaml')
    world_file = os.path.join(sim_share, 'worlds', 'agc_world.sdf')

    world_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'world_bringup.launch.py')),
        launch_arguments={
            'gz_args': LaunchConfiguration('gz_args'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    common = [params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]

    nodes = [
        Node(package='agc_perception', executable='ground_odom_adapter',
             name='ground_odom_adapter', parameters=common, output='screen'),
        Node(package='agc_ground', executable='ground_safety_stub',
             name='ground_safety_stub', parameters=common, output='screen'),
        Node(package='agc_ground', executable='ground_drive_relay',
             name='ground_drive_relay', parameters=common, output='screen'),
        Node(package='agc_ground', executable='ground_policy_placeholder',
             name='ground_policy_placeholder', parameters=common, output='screen'),
        Node(package='agc_task', executable='scenario_reset',
             name='scenario_reset', parameters=common, output='screen'),
        Node(package='agc_task', executable='task_manager',
             name='task_manager', parameters=common, output='screen'),
        Node(package='agc_task', executable='run_logger',
             name='run_logger', parameters=common, output='screen'),
    ]

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'gz_args', default_value=['-r -v 3 ', world_file],
        description='gz sim 参数（加 -s 可无头）'))
    ld.add_action(world_bringup)
    for n in nodes:
        ld.add_action(n)
    return ld