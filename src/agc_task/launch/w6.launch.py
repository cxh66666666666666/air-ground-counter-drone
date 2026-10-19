#!/usr/bin/env python3
"""w6 —— W6 顶层统一入口（结构评审 S-05；ADR-0061；backend 参数化单一入口）。

一条 `ros2 launch` 按 `backend` 组装 W6 场景：
  * backend:=lightweight（默认）—— 轻量栈：include `m0_skeleton.launch.py`（世界 + 桥 + 感知 +
    地面链路 + 任务层 scenario_reset/task_manager/run_logger）+ `capture_adjudicator` + `jammer`；
    **无** PX4 / 空中链路。
  * backend:=full —— 双 PX4 整链：`world_bringup` + `perception_bringup` + `ground_bringup`
    (command_source=rl) + 2×uXRCE-DDS Agent（8888/8889）+ 己方 PX4(实例0) + 入侵者 PX4(实例1, attach)
    + 空中链路 4 节点 + `lifecycle_manager_air` + 任务层 5 节点 + `intruder_driver`。

场景配置解析（base/registry/carriers 加载、carrier 校验、参数合并、轴断言、manifest 生成）
**单一真源**在共享模块 `launch/w6_composition.py`；本入口不含重复解析逻辑。约束类信息
（参数入口 / 唯一执行口 / 命名空间 / 硬约束）**只引用不复述**——真源 `docs/ROS2迁移交接文档.md`。

用法：
  ros2 launch agc_task w6.launch.py backend:=lightweight scene:=s01_static_none_nominal_track
  ros2 launch agc_task w6.launch.py backend:=full scene:=s02_linear_none_nominal_track \\
    gz_args:="-s -r -v 3 <world>"
"""

import os
import sys
import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory

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
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 共享配置解析模块与本入口同目录（随包安装到 share/agc_task/launch）。
_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
from w6_composition import (  # noqa: E402
    BACKENDS,
    apply_air_geometry,
    assertions,
    build_manifest,
    compose_params,
    load_scene,
    new_run_token,
    write_manifest,
    write_temp_params,
)


def _compose(context, *args, **kwargs):
    task_share = get_package_share_directory('agc_task')
    perc_share = get_package_share_directory('agc_perception')
    sim_share = get_package_share_directory('agc_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gz_args = LaunchConfiguration('gz_args')

    backend = LaunchConfiguration('backend').perform(context)
    # W7-a（R5/R6）：命令源单一选择由本入口透传至 ground_bringup——
    #   rl（RL 主控，默认）| baseline（Navfn + 自研 LQR 离线对照基线）| train（W7 外部训练/评估适配层作命令源）。
    command_source = LaunchConfiguration('command_source').perform(context)
    # W7-a（R3）：训练视图开关——显式设置时按 seed 随机化出生位（spawn 流）；缺省 None 不改既有行为。
    _sr = LaunchConfiguration('spawn_randomize').perform(context)
    spawn_randomize = None if _sr in (None, '', 'None') else (str(_sr).lower() in ('1', 'true', 'yes'))
    if backend not in BACKENDS:
        raise RuntimeError(
            '未知 backend: %s（可用：%s）→ 拒绝运行' % (backend, list(BACKENDS)))
    scene = LaunchConfiguration('scene').perform(context)
    seed = int(LaunchConfiguration('seed').perform(context))

    info = load_scene(task_share, perc_share, sim_share, scene, seed)

    # ---------------- backend:=lightweight ----------------
    if backend == 'lightweight':
        merged = compose_params(info['base'], info['sc'], backend, seed,
                                geometry=info['geometry'], spawn_randomize=spawn_randomize)
        tmp = write_temp_params(merged, backend, scene, seed)
        asserts = assertions(scene, merged, info)
        manifest_path = write_manifest(build_manifest(backend, info, merged, asserts),
                                       backend, scene, seed)
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(task_share, 'launch', 'm0_skeleton.launch.py')),
                launch_arguments={
                    'params_file': tmp,
                    'gz_args': gz_args,
                    'use_sim_time': use_sim_time,
                    'world_obstacles': info['world_obstacles'],
                    'sensors_file': info['sensors_file'],
                    # S-06：障碍先验 / 地图几何由**真源派生**（不再读 agc_perception 包内副本）
                    'obstacles_file': info['obstacles_file'],
                    'map_file': info['derived_map_file'],
                    # R-029（ADR-0044）：世界模型文件（run_logger 记 sha256）单一注入
                    'world_file': info['world_file'],
                    'scene_manifest_file': manifest_path,
                }.items(),
            ),
            # W6 拦截裁决（地面侧，无需 PX4）：发射式网捕获裁决器 + 行为级干扰注入
            Node(package='agc_task', executable='capture_adjudicator',
                 name='capture_adjudicator',
                 parameters=[tmp, {'use_sim_time': use_sim_time}], output='screen'),
            Node(package='agc_task', executable='jammer', name='jammer',
                 parameters=[tmp, {'use_sim_time': use_sim_time}], output='screen'),
        ]

    # ---------------- backend:=full ----------------
    air_share = get_package_share_directory('agc_air')
    ground_share = get_package_share_directory('agc_ground')
    # S-11：项目根**显式注入**（launch 参数 agc_root，默认固定根 /home/chen/AGC），
    # 不再由包 share 目录深度上溯推断；ROS 资产仍经 ament index（get_package_share_directory）。
    agc_root = Path(LaunchConfiguration('agc_root').perform(context))

    own_pose = LaunchConfiguration('px4_own_pose').perform(context)
    # R-014：己方机 spawn 位（world ENU）透传给 scenario_reset 做出生净空校验
    try:
        own_parts = [float(v) for v in own_pose.split(',')]
        own_uav_x, own_uav_y = own_parts[0], own_parts[1]
    except (ValueError, IndexError):
        print('WARN: w6 px4_own_pose 解析失败 %r → 按 (0,0) 处理' % (own_pose,))
        own_uav_x, own_uav_y = 0.0, 0.0
    agent0_port = LaunchConfiguration('agent0_port').perform(context)
    agent1_port = LaunchConfiguration('agent1_port').perform(context)

    merged = compose_params(info['base'], info['sc'], backend, seed, (own_uav_x, own_uav_y),
                            geometry=info['geometry'], spawn_randomize=spawn_randomize)
    tmp = write_temp_params(merged, backend, scene, seed)

    # R-026：空中链路节点参数 = air.yaml 基参 + scenarios.yaml 场景覆盖**合并后注入**
    air_base = yaml.safe_load(open(os.path.join(air_share, 'config', 'air.yaml')))
    air_merged = {n: dict(sec.get('ros__parameters', {})) for n, sec in air_base.items()}
    for n, over in (info['sc'].get('params') or {}).items():
        if n in air_merged:
            air_merged[n].update(over)
    # S-06：静态障碍几何由**真源**注入空中安全列表（obstacle_names/x/y；半径 / margin 仍来自 air.yaml）。
    apply_air_geometry(air_merged, info['geometry'])
    air_doc = {n: {'ros__parameters': p} for n, p in air_merged.items()}
    air_tmp = os.path.join(tempfile.gettempdir(),
                           'w6_%s_air_%s_%d_%s.yaml' % (backend, scene, seed, new_run_token()))
    with open(air_tmp, 'w') as f:
        yaml.safe_dump(air_doc, f, allow_unicode=True, sort_keys=True)

    # R-026 + S-06 断言（含空中安全列表 == 真源）在 air 合并后做；不符即拒绝运行。
    asserts = assertions(scene, merged, info, air_merged)
    manifest_path = write_manifest(build_manifest(backend, info, merged, asserts),
                                   backend, scene, seed)

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

    common = [tmp, {'use_sim_time': use_sim_time,
                    'scene_manifest_file': manifest_path, 'params_file': tmp}]
    # `origin_pose`（R-003）：己方 PX4 局部系原点（世界 ENU）= 己方机 spawn 位姿（同一 launch 参数）；
    # px4_adapter（局部→世界）与 OffboardInterface（世界→局部 setpoint）共用该值。
    air_common = [air_tmp, {'use_sim_time': use_sim_time, 'origin_pose': own_pose}]

    # 入侵者 PX4（实例 1）：attach 已生 `intruder_uav`。其进程 start 事件驱动 `intruder_driver`
    # （见下方 RegisterEventHandler）——不再用绝对时间盲计时。
    intruder_px4 = ExecuteProcess(
        cmd=[px4_bin, '-i', '1', '-d', '-s', 'etc/init.d-posix/rcS'], cwd=px4_build,
        additional_env={
            'GZ_SIM_RESOURCE_PATH': gz_models + ':' + gz_worlds,
            'PX4_GZ_STANDALONE': '1', 'PX4_GZ_WORLD': 'agc_world',
            'PX4_SIM_MODEL': 'gz_x500', 'PX4_GZ_MODEL_NAME': 'intruder_uav',
        }, output='screen')

    actions = [
        # R-019（ADR-0034）：运行时 PX4 failsafe 参数锁定——须在 PX4（12 s / 25 s 定时器）**之前**执行，
        # 写入 SITL 启动参数文件 `px4-rc.params`（由 rcS source），使每次 SITL 启动确定化。
        ExecuteProcess(
            cmd=['bash', str(agc_root / 'scripts' / 'lock_px4_failsafe_params.sh')],
            output='screen'),
        # PX4 模型库须先入资源路径：world_bringup 生成 `intruder_uav` 时 merge `model://x500`
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_models + ':' + gz_worlds),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_share, 'launch', 'world_bringup.launch.py')),
            launch_arguments={
                'gz_args': gz_args, 'use_sim_time': use_sim_time,
                'target_controlled': 'true',
                # R-026：按障碍轴选择世界（none 生成不含 pole_* 的世界）
                'world_obstacles': info['world_obstacles'],
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perc_share, 'launch', 'perception_bringup.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                # R-026：整链以 air_fusion 承载感知轴 + 按轴选择通道 / 障碍文件
                'target_source': BACKENDS[backend]['target_source'],
                'sensors_file': info['sensors_file'],
                # S-06：障碍先验 / 地图几何由**真源派生**（不再读 agc_perception 包内副本）
                'obstacles_file': info['obstacles_file'],
                'map_file': info['derived_map_file'],
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ground_share, 'launch', 'ground_bringup.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'command_source': command_source}.items()),
        # 两个 uXRCE-DDS Agent（实例各一，端口独立）
        ExecuteProcess(cmd=[agent_bin, 'udp4', '-p', agent0_port], output='screen'),
        ExecuteProcess(cmd=[agent_bin, 'udp4', '-p', agent1_port], output='screen'),
        # 己方 PX4（实例 0）：standalone，自 spawn gz_x500 自机。
        # 12 s 计时**保留**理由：属 gz 引擎 spawn 时序（世界/桥先起，PX4 才具 gz 资源与时钟）；
        # 本层**无可达就绪信号**（该 PX4 进程本身即被 spawn），故保留最小计时。
        TimerAction(period=12.0, actions=[ExecuteProcess(
            cmd=[px4_bin, '-d', '-s', 'etc/init.d-posix/rcS'], cwd=px4_build,
            additional_env={
                'GZ_SIM_RESOURCE_PATH': gz_models + ':' + gz_worlds,
                'PX4_GZ_STANDALONE': '1', 'PX4_GZ_WORLD': 'agc_world',
                'PX4_SIM_MODEL': 'gz_x500', 'PX4_GZ_MODEL_POSE': own_pose,
            }, output='screen')]),
        # 入侵者 PX4（实例 1）：attach 已生 `intruder_uav`（world_bringup 于 gz+3 s spawn）。
        # 25 s 计时**保留**理由：同属 gz 引擎 spawn 时序（须待世界 spawn 出入侵者模型），
        # 本层无可达就绪信号，故保留最小计时。
        TimerAction(period=25.0, actions=[intruder_px4]),
    ]

    # 空中链路节点 + lifecycle（复用 Nav2 lifecycle_manager）
    for exe in ('px4_adapter', 'takeoff_controller', 'trajectory_tracker', 'standoff_follower'):
        actions.append(Node(package='agc_air', executable=exe, name=exe,
                            parameters=air_common, output='screen'))
    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_air',
        # R-034（ADR-0049）：bond_timeout **不**无条件置 0——被管空中节点已实现 Nav2 bond
        #   （topic=bond、id=节点名），故启用 bond 失联检测（被管节点死亡 → manager "IS DOWN" 硬复位）。
        parameters=[{'autostart': True, 'bond_timeout': 5.0, 'use_sim_time': use_sim_time,
                     'node_names': ['px4_adapter', 'takeoff_controller',
                                    'trajectory_tracker', 'standoff_follower']}],
        output='screen'))

    # W6 任务层（地面侧）
    for exe in ('scenario_reset', 'task_manager', 'run_logger', 'capture_adjudicator', 'jammer'):
        # R-029：run_logger 另注入世界模型文件路径（记 sha256 作工具链 / 世界身份）
        params = common + ([{'world_file': info['world_file']}] if exe == 'run_logger' else [])
        actions.append(Node(package='agc_task', executable=exe, name=exe,
                            parameters=params, output='screen'))

    # 入侵者驱动：**就绪依赖**（事件驱动）替代绝对时间盲计时——入侵者 PX4 进程 start 即起
    # intruder_driver；其自身已就绪门控（无 odom 先 no-op），故无需再叠加延时。
    actions.append(RegisterEventHandler(
        OnProcessStart(target_action=intruder_px4, on_start=[
            Node(package='agc_task', executable='intruder_driver', name='intruder_driver',
                 parameters=common, output='screen')])))

    return actions


def generate_launch_description():
    sim_share = get_package_share_directory('agc_sim')
    world_file = os.path.join(sim_share, 'worlds', 'agc_world.sdf')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'backend', default_value='lightweight',
        description='W6 后端：lightweight（轻量栈，无 PX4）| full（双 PX4 整链）'))
    ld.add_action(DeclareLaunchArgument(
        'scene', default_value='s01_static_none_nominal_track',
        description='场景 id（见 config/scenarios.yaml）'))
    ld.add_action(DeclareLaunchArgument('seed', default_value='0',
                                        description='episode 种子（run_logger 记录 + spawn 生成）'))
    # W7-a（R5/R6）：命令源单一选择（透传 ground_bringup）——
    #   rl | baseline（Navfn + 自研 LQR 离线对照基线）| train（W7 外部适配层作命令源，仅 lightweight）。
    ld.add_action(DeclareLaunchArgument(
        'command_source', default_value='rl',
        description='命令源（互斥）：rl（RL 主控）| baseline（离线对照基线）| train（W7 外部适配层）'))
    # W7-a（R3）：训练视图——按 seed 随机化出生位；缺省空（不改既有 W6 行为）。
    ld.add_action(DeclareLaunchArgument(
        'spawn_randomize', default_value='',
        description='W7 训练视图：spawn 随机化开关（空=不改既有行为；true/false=显式覆盖）'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'gz_args', default_value=['-r -v 3 ', world_file],
        description='gz sim 参数（加 -s 可无头）'))
    ld.add_action(DeclareLaunchArgument(
        'px4_own_pose', default_value='0,0,0.2,0,0,0',
        description='己方机 spawn 位（world ENU；仅 backend:=full 生效）'))
    ld.add_action(DeclareLaunchArgument(
        'agent0_port', default_value='8888', description='uXRCE-DDS Agent 端口（仅 backend:=full）'))
    ld.add_action(DeclareLaunchArgument(
        'agent1_port', default_value='8889', description='uXRCE-DDS Agent 端口（仅 backend:=full）'))
    ld.add_action(DeclareLaunchArgument(
        'agc_root', default_value='/home/chen/AGC',
        description='固定项目根（AGENTS.md；colcon 根即项目根）；工具链在其下 toolchain/'))
    ld.add_action(OpaqueFunction(function=_compose))
    return ld
