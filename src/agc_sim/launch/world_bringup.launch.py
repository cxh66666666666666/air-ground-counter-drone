#!/usr/bin/env python3
"""world_bringup —— W2-min 世界启动编排（世界 + 桥 + spawn 车/目标）。

单一入口，供 M0 顶层 launch include。参数单一来源：
  * 车 / 目标模型与 spawn 位默认值取自 config/car.yaml、config/target.yaml；
  * 桥配置取自 config/bridge.yaml；
  * 全部经 launch 单一设置（契约 #params），节点不得覆写。
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    share = get_package_share_directory('agc_sim')
    car = _load_yaml(os.path.join(share, 'config', 'car.yaml'))['agc_sim']['car']
    target = _load_yaml(os.path.join(share, 'config', 'target.yaml'))['agc_sim']['target']

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
        DeclareLaunchArgument('car_body_length', default_value=str(car_m['body_length'])),
        DeclareLaunchArgument('car_body_width', default_value=str(car_m['body_width'])),
        DeclareLaunchArgument('car_body_height', default_value=str(car_m['body_height'])),
        DeclareLaunchArgument('car_body_mass', default_value=str(car_m['body_mass'])),
        # 目标
        DeclareLaunchArgument('target_x', default_value=str(tgt_s['x'])),
        DeclareLaunchArgument('target_y', default_value=str(tgt_s['y'])),
        DeclareLaunchArgument('target_z', default_value=str(tgt_s['z'])),
        DeclareLaunchArgument('target_yaw', default_value=str(tgt_s['yaw'])),
        DeclareLaunchArgument('target_body_length', default_value=str(tgt_m['body_length'])),
        DeclareLaunchArgument('target_body_width', default_value=str(tgt_m['body_width'])),
        DeclareLaunchArgument('target_body_height', default_value=str(tgt_m['body_height'])),
        DeclareLaunchArgument('target_body_mass', default_value=str(tgt_m['body_mass'])),
    ]

    # gz sim（世界）
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_gz_sim_launch),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
    )

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
        ' body_length:=', LaunchConfiguration('car_body_length'),
        ' body_width:=', LaunchConfiguration('car_body_width'),
        ' body_height:=', LaunchConfiguration('car_body_height'),
        ' body_mass:=', LaunchConfiguration('car_body_mass'),
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
    ld.add_action(gz_sim)
    ld.add_action(bridge)
    ld.add_action(set_pose_bridge)
    ld.add_action(TimerAction(period=3.0, actions=[spawn_car, spawn_target]))
    return ld