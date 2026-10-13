#!/usr/bin/env python3
"""air_bringup —— W3 空中链路一键启动（spec R1）。

一条 `ros2 launch` 起：agc_world（唯一 gz server，经 agc_sim/world_bringup）+ uXRCE-DDS Agent
+ PX4 SITL（`PX4_GZ_STANDALONE=1`、`PX4_GZ_WORLD=agc_world`、`PX4_SIM_MODEL=gz_x500`，
由 PX4 自身 gz_bridge 向 `/world/agc_world/create` spawn 机型）+ 空中链路节点。

参数单一来源：`config/air.yaml`（契约 [参数入口]）。链路连接层（agent 地址 / 端口 / PX4 目录）
集中在此文件与 launch 参数 —— 实机替换只动这一处（spec R11，薄抽象）。
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('agc_air')
    sim_share = get_package_share_directory('agc_sim')
    perception_share = get_package_share_directory('agc_perception')
    # install/agc_air/share/agc_air → 上溯 4 级为项目根（toolchain 在根下）
    agc_root = Path(share).parents[3]

    params_file = os.path.join(share, 'config', 'air.yaml')
    world_file = os.path.join(sim_share, 'worlds', 'agc_world.sdf')

    # ---- 链路连接层（唯一可替换处，spec R11）----
    px4_dir = str(agc_root / 'toolchain' / 'PX4-Autopilot')
    px4_build = os.path.join(px4_dir, 'build', 'px4_sitl_default')
    px4_bin = os.path.join(px4_build, 'bin', 'px4')
    agent_bin = str(agc_root / 'toolchain' / 'bin' / 'MicroXRCEAgent')
    gz_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    gz_worlds = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds')

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gz_args', default_value=['-r -v 3 ', world_file],
                              description='gz sim 参数（加 -s 可无头）'),
        DeclareLaunchArgument('px4_gz_model_pose', default_value='0,0,0.2,0,0,0',
                              description='己方无人机 spawn 位姿（world 系 ENU，逗号分隔）'),
        DeclareLaunchArgument('agent_port', default_value='8888'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='是否自动 configure/activate 空中节点'),
        DeclareLaunchArgument('target_source', default_value='gt_adapter',
                              description='契约 /agc/target/source（W4 目标源）：'
                                          'gt_adapter | air_fusion | none（none 用于单独录制 C1 八字轨迹）'),
    ]

    # ---- 世界 + 桥 + 车/目标 spawn（复用 agc_sim，唯一世界 / 唯一时钟）----
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'world_bringup.launch.py')),
        launch_arguments={
            'gz_args': LaunchConfiguration('gz_args'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # ---- W4 感知适配层（真值旁路 + 目标源）；目标源由本 launch 单一设置 ----
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'perception_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_source': LaunchConfiguration('target_source'),
        }.items(),
    )

    # ---- uXRCE-DDS Agent（PX4 与 ROS2 之间的桥）----
    agent = ExecuteProcess(
        cmd=[agent_bin, 'udp4', '-p', LaunchConfiguration('agent_port')],
        output='screen',
    )

    # ---- PX4 SITL（standalone：不自起 gz，挂入 agc_world）----
    px4_env = {
        'GZ_SIM_RESOURCE_PATH': gz_models + ':' + gz_worlds,
        'PX4_GZ_STANDALONE': '1',
        'PX4_GZ_WORLD': 'agc_world',
        'PX4_SIM_MODEL': 'gz_x500',
        'PX4_GZ_MODEL_POSE': LaunchConfiguration('px4_gz_model_pose'),
    }
    # 用 PX4 官方启动脚本（与官方 gz_x500 流程一致；不注入任何自定义参数）
    px4 = ExecuteProcess(
        cmd=[px4_bin, '-d', '-s', 'etc/init.d-posix/rcS'],
        cwd=px4_build,
        additional_env=px4_env,
        output='screen',
    )

    # ---- 空中链路节点（长跑控制节点为 lifecycle managed node）----
    common = [params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    nodes = [
        Node(package='agc_air', executable='px4_adapter',
             name='px4_adapter', parameters=common, output='screen'),
        Node(package='agc_air', executable='takeoff_controller',
             name='takeoff_controller', parameters=common, output='screen'),
        Node(package='agc_air', executable='trajectory_tracker',
             name='trajectory_tracker', parameters=common, output='screen'),
        Node(package='agc_air', executable='standoff_follower',
             name='standoff_follower', parameters=common, output='screen'),
    ]

    # lifecycle 编排：**复用 Nav2 `lifecycle_manager`**（ADR-0008 吸收点）。
    # 它先 wait_for_service 等节点就绪，再统一 configure → activate，天然避免"节点未就绪就下发"
    # 与固定延时竞态（原先手写 TimerAction + `ros2 lifecycle set` 重试的做法已废弃）。
    # 说明：本节点未创建 bond 连接（非 nav2_util::LifecycleNode），故 bond_timeout=0 关闭 bond 监控。
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_air',
        parameters=[{
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['px4_adapter', 'takeoff_controller', 'trajectory_tracker',
                           'standoff_follower'],
            'bond_timeout': 0.0,
        }],
        output='screen',
    )

    ld = LaunchDescription()
    for a in declared:
        ld.add_action(a)
    # gz 服务器在**运行期**经 `/world/agc_world/create` 解析 PX4 机型 `model://x500`
    # （PX4 自身 gz_bridge 只给文件名，资源解析靠 gz 侧），故本变量必须对整个 launch 生效。
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_models + ':' + gz_worlds))
    ld.add_action(world)
    ld.add_action(perception)
    ld.add_action(agent)
    ld.add_action(TimerAction(period=12.0, actions=[px4]))
    for n in nodes:
        ld.add_action(n)
    ld.add_action(lifecycle_manager)
    return ld
