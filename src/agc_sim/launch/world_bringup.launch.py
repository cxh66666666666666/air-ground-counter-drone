#!/usr/bin/env python3
"""world_bringup —— W2-min 世界启动编排（世界 + 桥 + spawn 车/目标）。

单一入口，供 M0 顶层 launch include。参数单一来源：
  * 车 / 目标模型与 spawn 位默认值取自 config/car.yaml、config/target.yaml；
  * 桥配置取自 config/bridge.yaml；
  * 静态障碍几何取自**单一真源** config/world_geometry.yaml（S-06；ADR-0062）——
    `agc_world.sdf` 为不含 `pole_*` 的 base world，本 launch 按 `world_obstacles`
    （`static` 注入 pole / `none` 不注入）由真源生成运行期世界；
  * 全部经 launch 单一设置（契约 #params），节点不得覆写。
"""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _load_geometry(path):
    """读**单一几何真源** config/world_geometry.yaml（S-06；ADR-0062）。"""
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _inject_static_obstacles(src_path, dst_path, geometry):
    """S-06：由**单一真源**（`world_geometry.yaml` 的 obstacles）向 base world 注入静态障碍
    `<model>` 块，生成运行期世界写入临时文件。

    **不**手写第二份完整 world：base world（光源/地面/物理/插件/`spherical_coordinates`）
    原样保留，仅把真源障碍**逐值**注入到 `</world>` 之前——保证几何**只**来自真源、
    「无障碍」只改障碍因子（`none` 直接用 base，不注入）、**不改世界其它物理**。
    """
    with open(src_path, 'r') as f:
        text = f.read()
    if '</world>' not in text:
        raise RuntimeError('world 生成失败：base world %s 无 </world> 锚点' % src_path)
    blocks = []
    for ob in (geometry.get('obstacles') or []):
        size = ob['size']
        color = ob.get('color') or [0.7, 0.7, 0.7]
        blocks.append(
            '\n    <model name="%(id)s">\n'
            '      <static>true</static>\n'
            '      <pose>%(x)s %(y)s %(z)s 0 0 %(yaw)s</pose>\n'
            '      <link name="link">\n'
            '        <collision name="collision">\n'
            '          <geometry><box><size>%(sx)s %(sy)s %(sz)s</size></box></geometry>\n'
            '        </collision>\n'
            '        <visual name="visual">\n'
            '          <geometry><box><size>%(sx)s %(sy)s %(sz)s</size></box></geometry>\n'
            '          <material>\n'
            '            <ambient>%(r)s %(g)s %(b)s 1</ambient>\n'
            '            <diffuse>%(r)s %(g)s %(b)s 1</diffuse>\n'
            '          </material>\n'
            '        </visual>\n'
            '      </link>\n'
            '    </model>' % {
                'id': ob['id'], 'x': ob['x'], 'y': ob['y'], 'z': ob['z'],
                'yaw': ob.get('yaw', 0.0),
                'sx': size[0], 'sy': size[1], 'sz': size[2],
                'r': color[0], 'g': color[1], 'b': color[2]})
    injected = '\n'.join(blocks)
    idx = text.rfind('</world>')
    out = text[:idx] + injected + '\n\n  ' + text[idx:]
    with open(dst_path, 'w') as f:
        f.write(out)
    return dst_path


def _make_gz_sim(world_file, ros_gz_sim_launch, geometry):
    """按 `world_obstacles` 选择世界（S-06）：
      * `static` —— base world + 由真源注入 `pole_*` 的生成世界（临时文件）；
      * `none`   —— base world 本身（**已无** `pole_*`，不注入）。
    世界路径经 gz_args token（按 basename 匹配替换，兼容相对/绝对）单一指定，不由上层手写。
    """
    def _opaque(context, *args, **kwargs):
        gz_args = LaunchConfiguration('gz_args').perform(context)
        world_obstacles = LaunchConfiguration('world_obstacles').perform(context)
        if world_obstacles == 'static':
            world = os.path.join(tempfile.gettempdir(), 'agc_world_static_obstacles.sdf')
            _inject_static_obstacles(world_file, world, geometry)
        elif world_obstacles == 'none':
            world = world_file
        else:
            raise RuntimeError(
                '未知 world_obstacles=%s（可用：static | none）→ 拒绝运行' % world_obstacles)
        base = os.path.basename(world_file)
        tokens = gz_args.split()
        replaced = [world if os.path.basename(t) == base else t for t in tokens]
        if not any(os.path.basename(t) == base for t in tokens):
            replaced.append(world)
        gz_args = ' '.join(replaced)
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros_gz_sim_launch),
            launch_arguments={'gz_args': gz_args}.items())]
    return OpaqueFunction(function=_opaque)


def generate_launch_description():
    share = get_package_share_directory('agc_sim')
    car = _load_yaml(os.path.join(share, 'config', 'car.yaml'))['agc_sim']['car']
    target = _load_yaml(os.path.join(share, 'config', 'target.yaml'))['agc_sim']['target']
    # S-06：静态障碍几何单一真源（物理世界由此派生注入）。
    geometry = _load_geometry(os.path.join(share, 'config', 'world_geometry.yaml'))

    world_file = os.path.join(share, 'worlds', 'agc_world.sdf')
    car_xacro = os.path.join(share, 'models', 'agc_car', 'agc_car.sdf.xacro')
    target_xacro = os.path.join(share, 'models', 'intruder_uav', 'intruder_uav.sdf.xacro')
    bridge_cfg = os.path.join(share, 'config', 'bridge.yaml')
    ros_gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    car_m = car['model']
    car_s = car['spawn']
    tgt_m = target['model']
    tgt_s = target['spawn']

    declared = [
        DeclareLaunchArgument(
            'gz_args', default_value=['-r -v 3 ', world_file],
            description='Arguments passed to gz sim（更 -s 可无头）'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # 车
        DeclareLaunchArgument('car_x', default_value=str(car_s['x'])),
        DeclareLaunchArgument('car_y', default_value=str(car_s['y'])),
        DeclareLaunchArgument('car_z', default_value=str(car_s['z'])),
        DeclareLaunchArgument('car_yaw', default_value=str(car_s['yaw'])),
        DeclareLaunchArgument('car_wheel_separation', default_value=str(car_m['wheel_separation'])),
        DeclareLaunchArgument('car_wheel_radius', default_value=str(car_m['wheel_radius'])),
        DeclareLaunchArgument('car_wheel_width', default_value=str(car_m['wheel_width'])),
        DeclareLaunchArgument('car_max_wheel_torque', default_value=str(car_m['max_wheel_torque'])),
        DeclareLaunchArgument('car_max_wheel_acceleration', default_value=str(car_m['max_wheel_acceleration'])),
        DeclareLaunchArgument('car_max_linear_velocity', default_value=str(car_m['max_linear_velocity'])),
        DeclareLaunchArgument('car_max_angular_velocity', default_value=str(car_m['max_angular_velocity'])),
        DeclareLaunchArgument('car_body_length', default_value=str(car_m['body_length'])),
        DeclareLaunchArgument('car_body_width', default_value=str(car_m['body_width'])),
        DeclareLaunchArgument('car_body_height', default_value=str(car_m['body_height'])),
        DeclareLaunchArgument('car_body_mass', default_value=str(car_m['body_mass'])),
        DeclareLaunchArgument('car_capture_net_radius', default_value=str(car_m['capture_net_radius'])),
        DeclareLaunchArgument('car_payload_mass', default_value=str(car_m['payload_mass'])),
        DeclareLaunchArgument('car_cmd_deadman_timeout_s', default_value=str(car_m['cmd_deadman_timeout_s'])),
        # 目标
        DeclareLaunchArgument('target_x', default_value=str(tgt_s['x'])),
        DeclareLaunchArgument('target_y', default_value=str(tgt_s['y'])),
        DeclareLaunchArgument('target_z', default_value=str(tgt_s['z'])),
        DeclareLaunchArgument('target_yaw', default_value=str(tgt_s['yaw'])),
        DeclareLaunchArgument('target_body_length', default_value=str(tgt_m['body_length'])),
        DeclareLaunchArgument('target_body_width', default_value=str(tgt_m['body_width'])),
        DeclareLaunchArgument('target_body_height', default_value=str(tgt_m['body_height'])),
        DeclareLaunchArgument('target_body_mass', default_value=str(tgt_m['body_mass'])),
        DeclareLaunchArgument(
            'target_controlled', default_value='false',
            description='目标模型模式（W6 前置 P）：false=被动失重（M0/W5）；'
                        'true=PX4 可控机型（merge x500，由第二 PX4 实例控制）'),
        # R-026 / S-06：按场景障碍轴选择世界。
        #   static —— base world 由真源注入 pole_*（含 pole_*）；
        #   none   —— base world 本身（不含 pole_*）。
        DeclareLaunchArgument(
            'world_obstacles', default_value='static',
            description='障碍轴世界生成：static（由真源注入 pole_*）| none（base world，不含 pole_*）'),
    ]

    # gz sim（世界）：按 `world_obstacles` 由**单一几何真源**注入 / 不注入生成
    gz_sim = _make_gz_sim(world_file, ros_gz_sim_launch, geometry)

    # ros_gz 桥（话题，声明式 YAML）
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_cfg,
            'expand_gz_topic_names': True,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )

    # ros_gz 桥（set_pose 服务，CLI 形式）
    # 本机 ros_gz_bridge(1.0.24) 的 YAML 服务条目在 factory 侧会因空 topic 名崩溃，
    # 故该服务以 CLI 形式桥接（等价），见 config/bridge.yaml 注释。
    set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='set_pose_bridge',
        arguments=['/world/agc_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    # spawn 车（xacro → SDF 字符串）
    car_desc = Command([
        FindExecutable(name='xacro'), ' ', car_xacro,
        ' wheel_separation:=', LaunchConfiguration('car_wheel_separation'),
        ' wheel_radius:=', LaunchConfiguration('car_wheel_radius'),
        ' wheel_width:=', LaunchConfiguration('car_wheel_width'),
        ' max_wheel_torque:=', LaunchConfiguration('car_max_wheel_torque'),
        ' max_wheel_acceleration:=', LaunchConfiguration('car_max_wheel_acceleration'),
        ' max_linear_velocity:=', LaunchConfiguration('car_max_linear_velocity'),
        ' max_angular_velocity:=', LaunchConfiguration('car_max_angular_velocity'),
        ' body_length:=', LaunchConfiguration('car_body_length'),
        ' body_width:=', LaunchConfiguration('car_body_width'),
        ' body_height:=', LaunchConfiguration('car_body_height'),
        ' body_mass:=', LaunchConfiguration('car_body_mass'),
        ' capture_net_radius:=', LaunchConfiguration('car_capture_net_radius'),
        ' payload_mass:=', LaunchConfiguration('car_payload_mass'),
        ' cmd_deadman_timeout_s:=', LaunchConfiguration('car_cmd_deadman_timeout_s'),
    ])
    spawn_car = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=[
            '-world', 'agc_world', '-name', 'agc_car', '-string', car_desc,
            '-x', LaunchConfiguration('car_x'),
            '-y', LaunchConfiguration('car_y'),
            '-z', LaunchConfiguration('car_z'),
            '-Y', LaunchConfiguration('car_yaw'),
        ],
    )

    # spawn 目标
    target_desc = Command([
        FindExecutable(name='xacro'), ' ', target_xacro,
        ' controlled:=', LaunchConfiguration('target_controlled'),
        ' body_length:=', LaunchConfiguration('target_body_length'),
        ' body_width:=', LaunchConfiguration('target_body_width'),
        ' body_height:=', LaunchConfiguration('target_body_height'),
        ' body_mass:=', LaunchConfiguration('target_body_mass'),
    ])
    spawn_target = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=[
            '-world', 'agc_world', '-name', 'intruder_uav', '-string', target_desc,
            '-x', LaunchConfiguration('target_x'),
            '-y', LaunchConfiguration('target_y'),
            '-z', LaunchConfiguration('target_z'),
            '-Y', LaunchConfiguration('target_yaw'),
        ],
    )

    ld = LaunchDescription()
    for a in declared:
        ld.add_action(a)
    # 模型资源路径：agc_car 的 mesh 用 `model://agc_car/...`，需将此 models 目录加入 gz 资源搜索路径
    # （**追加**而非覆盖：air_bringup 会另行追加 PX4 模型库，二者需共存）。
    sim_models = os.path.join(share, 'models')
    ld.add_action(SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [sim_models, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH')]))
    # R-009（ADR-0025）：gz 进程加载 GZ 侧末端失效停车插件 agc_cmd_deadman。
    # 插件库装于 <pkg prefix>/lib（install/agc_sim/lib），追加到 GZ_SIM_SYSTEM_PLUGIN_PATH
    # （gz_sim.launch.py 会再并入 LD_LIBRARY_PATH，故内置系统插件仍可加载）。**追加**而非覆盖。
    plugin_dir = os.path.join(os.path.dirname(share), 'lib')
    ld.add_action(SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        [plugin_dir, ':', EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH')]))
    ld.add_action(gz_sim)
    ld.add_action(bridge)
    ld.add_action(set_pose_bridge)
    ld.add_action(TimerAction(period=3.0, actions=[spawn_car, spawn_target]))
    return ld