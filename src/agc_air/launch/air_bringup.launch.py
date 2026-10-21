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

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _generate(context, *args, **kwargs):
    share = get_package_share_directory('agc_air')
    sim_share = get_package_share_directory('agc_sim')
    perception_share = get_package_share_directory('agc_perception')
    # S-11：项目根**显式注入**（launch 参数 agc_root，默认固定根 /home/chen/AGC），
    # 不再由包 share 目录深度上溯推断；ROS 资产仍经 ament index。
    agc_root = Path(LaunchConfiguration('agc_root').perform(context))

    params_file = os.path.join(share, 'config', 'air.yaml')
    # S-06（ADR-0062）：静态障碍几何来自**单一真源** `agc_sim/config/world_geometry.yaml`，
    # 注入空中安全列表（`trajectory_tracker.safety.obstacle_*`）；air.yaml 不再硬编码几何。
    geometry = yaml.safe_load(open(os.path.join(sim_share, 'config', 'world_geometry.yaml')))
    safety_geometry = {
        'safety.obstacle_names': [o['id'] for o in geometry['obstacles']],
        'safety.obstacle_x': [float(o['x']) for o in geometry['obstacles']],
        'safety.obstacle_y': [float(o['y']) for o in geometry['obstacles']],
    }

    # ---- 链路连接层（唯一可替换处，spec R11）----
    px4_dir = str(agc_root / 'toolchain' / 'PX4-Autopilot')
    px4_build = os.path.join(px4_dir, 'build', 'px4_sitl_default')
    px4_bin = os.path.join(px4_build, 'bin', 'px4')
    agent_bin = str(agc_root / 'toolchain' / 'bin' / 'MicroXRCEAgent')
    # S-11：启动前校验工具链二进制存在（缺失即拒启动，清晰报错）。
    _required = [agc_root / 'toolchain' / 'bin' / 'MicroXRCEAgent', Path(px4_bin)]
    _missing = [str(p) for p in _required if not p.exists()]
    if _missing:
        raise RuntimeError(
            '工具链缺失，拒绝启动：%s\n请确认 agc_root 指向固定项目根（默认 /home/chen/AGC）；'
            '亦可用 agc_root:=<path> 覆盖' % ', '.join(_missing))
    gz_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    gz_worlds = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds')

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
    # `origin_pose`（R-003）：PX4 局部系原点（世界 ENU）= 己方机 spawn 位姿，经 launch 单一注入；
    # px4_adapter（局部→世界）与 OffboardInterface（世界→局部 setpoint）共用该值。
    common = [params_file, {'use_sim_time': LaunchConfiguration('use_sim_time'),
                            'origin_pose': LaunchConfiguration('px4_gz_model_pose')}]
    nodes = [
        Node(package='agc_air', executable='px4_adapter',
             name='px4_adapter', parameters=common, output='screen'),
        Node(package='agc_air', executable='takeoff_controller',
             name='takeoff_controller', parameters=common, output='screen'),
        Node(package='agc_air', executable='trajectory_tracker',
             name='trajectory_tracker', parameters=common + [safety_geometry], output='screen'),
        Node(package='agc_air', executable='standoff_follower',
             name='standoff_follower', parameters=common, output='screen'),
    ]

    # ADR-0080：manager 启动**前**预热被管节点的 lifecycle 服务连接，避免其阻塞式
    # `change_state` 在 DDS 发现预热期永久挂起（实测停在 `Configuring takeoff_controller`）。
    prewarm_bin = os.path.join(
        get_package_prefix('agc_air'), 'lib', 'agc_air', 'air_lifecycle_prewarm')
    prewarm = ExecuteProcess(
        cmd=[prewarm_bin, 'px4_adapter', 'takeoff_controller', 'trajectory_tracker',
             'standoff_follower'],
        output='screen')

    # lifecycle 编排：**复用 Nav2 `lifecycle_manager`**（ADR-0008 吸收点）。
    # 它先 wait_for_service 等节点就绪，再统一 configure → activate，天然避免"节点未就绪就下发"
    # 与固定延时竞态（原先手写 TimerAction + `ros2 lifecycle set` 重试的做法已废弃）。
    # R-034：被管空中节点已实现 Nav2 bond（topic=`bond`、id=节点名、`bond_heartbeat_period`）→
    # `bond_timeout` **不**再置 0，启用 bond 失联检测（被管节点死亡 → manager 硬复位，见 ADR-0049）。
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_air',
        parameters=[{
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['px4_adapter', 'takeoff_controller', 'trajectory_tracker',
                           'standoff_follower'],
            'bond_timeout': 5.0,
        }],
        output='screen',
    )

    # gz 服务器在**运行期**经 `/world/agc_world/create` 解析 PX4 机型 `model://x500`
    # （PX4 自身 gz_bridge 只给文件名，资源解析靠 gz 侧），故本变量必须对整个 launch 生效。
    # OpaqueFunction 要求函数返回实体**列表**（LaunchDescription 本身不可迭代），
    # 故直接返回按原顺序组装的实体清单。
    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_models + ':' + gz_worlds),
        world,
        perception,
        agent,
        TimerAction(period=12.0, actions=[px4]),
        *nodes,
        prewarm,
        RegisterEventHandler(OnProcessExit(target_action=prewarm, on_exit=[lifecycle_manager])),
    ]


def generate_launch_description():
    sim_share = get_package_share_directory('agc_sim')
    world_file = os.path.join(sim_share, 'worlds', 'agc_world.sdf')

    # 所有 launch 参数在**顶层**声明，保证 `ros2 launch ... --show-args` 可内省；
    # `_generate`（OpaqueFunction）只负责按参数组装 actions。
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

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'agc_root', default_value='/home/chen/AGC',
        description='固定项目根（AGENTS.md）；工具链在其下 toolchain/'))
    for a in declared:
        ld.add_action(a)
    ld.add_action(OpaqueFunction(function=_generate))
    return ld
