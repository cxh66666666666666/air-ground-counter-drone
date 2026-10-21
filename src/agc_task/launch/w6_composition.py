#!/usr/bin/env python3
"""w6_composition —— W6 场景配置解析的**单一真源**（结构评审 S-05；ADR-0061）。

本模块集中承载 W6 顶层 composition 的**全部**场景配置解析逻辑，供统一入口
`w6.launch.py` 复用（`backend` ∈ `lightweight` | `full`）：base/registry/carriers 加载、
carrier 承载者校验、参数合并、轴断言、manifest 生成、临时文件落盘。**不再**在多入口内联重复。

约束类信息（参数入口 / 唯一执行口 / 命名空间 / 硬约束 C1–C10 / 终态语义）**只引用不复述**，
真源见 `docs/ROS2迁移交接文档.md` 的对应锚点。

设计要点（S-05）：
  * `BACKENDS` 为 **backend 能力表**：每 backend 声明 `air_link` / `px4` / `target_source` /
    `required_nodes`（完整展开节点名）/ `supported_axes`；`coverage_status` 的 `not_carried`
    状态**由能力表派生**（不再逐文件硬编码「轻量栈无空中链路」）。
  * manifest 身份与入口名一致：`launch='w6.launch.py'` + 实际 `backend` + `nodes`（展开节点）
    + 生效参数（`scene` / `seed` / `support_status` / `declared_axes` / `expected_terminal` /
    `expected_reason` / `carriers` / `effective_params` / `assertions`）。

设计要点（S-06）：单一几何真源 `agc_sim/config/world_geometry.yaml` 的读取（`load_geometry`）
与内容哈希（`geometry_sha256`）；按域**派生**——感知障碍先验（`write_derived_obstacles`）、
感知地图几何（`write_derived_map`）、`scenario_reset` 障碍 / 边界与空中安全列表注入
（`compose_params` / `apply_air_geometry`）；manifest 记 `geometry` 块；`assertions` 断言
「派生几何 == 真源」。
"""

import hashlib
import json
import os
import random
import tempfile
import time

import yaml

# 统一入口文件名（manifest `launch` 字段须与入口名一致，S-05）。
LAUNCH = 'w6.launch.py'

# S-06：静态障碍 + 作业区几何的**单一真源**（位于 agc_sim share 的 config/ 下）。
GEOMETRY_FILE = 'world_geometry.yaml'


def new_run_token():
    """S-07（ADR-0063）：每次调用生成唯一 token（时刻 + 进程 + 随机），供临时文件命名避免并发 / 重复覆盖。"""
    return '%s-%s-%d-%04d' % (time.strftime('%Y%m%d'), time.strftime('%H%M%S'),
                              os.getpid(), random.randint(0, 9999))


def load_geometry(sim_share):
    """读**单一几何真源** `agc_sim/config/world_geometry.yaml`（S-06；ADR-0062）。"""
    with open(os.path.join(sim_share, 'config', GEOMETRY_FILE)) as f:
        return yaml.safe_load(f)


def geometry_sha256(sim_share):
    """真源文件**字节**的 sha256（几何版本身份；写入 manifest）。"""
    with open(os.path.join(sim_share, 'config', GEOMETRY_FILE), 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def geometry_ids(geometry):
    return [o['id'] for o in (geometry.get('obstacles') or [])]


def geometry_xs(geometry):
    return [float(o['x']) for o in (geometry.get('obstacles') or [])]


def geometry_ys(geometry):
    return [float(o['y']) for o in (geometry.get('obstacles') or [])]


def work_area_bounds(geometry):
    """由真源 `work_area` 派生作业区边界（`map_*`，map 系）。"""
    wa = geometry['work_area']
    return {
        'map_x_min': float(wa['origin_x']),
        'map_x_max': float(wa['origin_x']) + float(wa['size_x']),
        'map_y_min': float(wa['origin_y']),
        'map_y_max': float(wa['origin_y']) + float(wa['size_y']),
    }


# backend 能力表：每 backend 声明能力事实，驱动覆盖状态派生与 manifest 节点展开。
#   * air_link      —— 是否含空中链路（PX4 / 空中节点）；False → requires_air_link 承载者 not_carried。
#   * px4           —— 是否起 PX4 SITL 实例。
#   * target_source —— 感知轴目标源：gt_adapter（地面真值旁路）/ air_fusion（空中链路融合）。
#   * required_nodes —— 该 backend 完整展开的节点 / 进程名（manifest `nodes` 字段）。
#   * supported_axes —— 该 backend 实际执行的轴（不含即如实标注 not_carried / N/A）。
BACKENDS = {
    'lightweight': {
        'air_link': False,
        'px4': False,
        'target_source': 'gt_adapter',
        'required_nodes': [
            'world_bringup', 'ros_gz_bridge', 'set_pose_bridge',
            'perception_bringup', 'ground_bringup',
            'scenario_reset', 'task_manager', 'run_logger',
            'capture_adjudicator', 'jammer',
        ],
        # 轻量栈无 PX4 / 空中链路：仅承载地面侧轴（obstacle / outcome）。
        'supported_axes': ['obstacle', 'outcome'],
    },
    'full': {
        'air_link': True,
        'px4': True,
        'target_source': 'air_fusion',
        'required_nodes': [
            'world_bringup', 'ros_gz_bridge', 'set_pose_bridge',
            'perception_bringup', 'ground_bringup',
            'uxrce_dds_agent0', 'uxrce_dds_agent1', 'px4_own', 'px4_intruder',
            'px4_adapter', 'takeoff_controller', 'trajectory_tracker', 'standoff_follower',
            'lifecycle_manager_air',
            'scenario_reset', 'task_manager', 'run_logger',
            'capture_adjudicator', 'jammer', 'intruder_driver',
        ],
        # 整链含全部 4 轴（obstacle / outcome + target_motion / perception 的空中侧）。
        'supported_axes': ['obstacle', 'outcome', 'target_motion', 'perception'],
    },
}


def carrier_entry(carriers, axis, value, scene):
    """R-026：声明轴值必须有承载者；无承载者 → 显式拒绝运行（不以命名当执行）。"""
    ent = (carriers.get(axis) or {}).get(value)
    if ent is None:
        raise RuntimeError(
            '场景 %s 声明 %s=%s：scenario_carriers.yaml 无承载者登记 → 拒绝运行' % (
                scene, axis, value))
    if not ent.get('supported', False):
        raise RuntimeError(
            '场景 %s 声明 %s=%s 无承载者（%s）→ 拒绝运行（不支持轴，不以命名推定已覆盖）' % (
                scene, axis, value, ent.get('reason', 'unsupported')))
    return ent


def coverage_status(entry, backend):
    """承载者状态：**由 backend 能力表派生**——需空中链路而该 backend 无 → not_carried。

    S-05：不再硬编码 `backend == 'lightweight'`，而以 `BACKENDS[backend]['air_link']` 判定。
    """
    if entry.get('requires_air_link') and not BACKENDS[backend]['air_link']:
        return 'not_carried', 'backend=%s 无空中链路 / PX4（能力表 air_link=False），承载者不生效' % (
            backend,)
    return 'covered', ''


def write_derived_obstacles(geometry, mode, scene, seed, token=None):
    """S-06：由真源派生**感知障碍先验文件**（`obstacles.yaml` 形态），返回 (路径, id 列表)。

    `static` → 真源障碍实体（`id/x/y/z/yaw/size`）；`none` → 空数组（与「无 pole_* 世界」一致）。
    w6 路径**不再**读 agc_perception 包内副本（副本仅作 standalone 回退）。
    """
    token = token or new_run_token()
    if mode == 'static':
        obstacles = [{
            'id': ob['id'], 'x': float(ob['x']), 'y': float(ob['y']), 'z': float(ob['z']),
            'yaw': float(ob.get('yaw', 0.0)), 'size': [float(s) for s in ob['size']],
        } for ob in (geometry.get('obstacles') or [])]
    elif mode == 'none':
        obstacles = []
    else:
        raise RuntimeError('未知障碍模式 %s（可用：static | none）→ 拒绝运行' % mode)
    path = os.path.join(tempfile.gettempdir(), 'w6_derived_obstacles_%s_%d_%s.yaml' % (scene, seed, token))
    with open(path, 'w') as f:
        yaml.safe_dump({'obstacles': obstacles}, f, allow_unicode=True, sort_keys=True)
    return path, [o['id'] for o in obstacles]


def write_derived_map(geometry, scene, seed, token=None):
    """S-06：由真源 `work_area` 派生**感知地图几何文件**（`map.yaml` 形态），返回路径。"""
    token = token or new_run_token()
    wa = geometry['work_area']
    doc = {'resolution': float(wa['resolution']), 'size_x': float(wa['size_x']),
           'size_y': float(wa['size_y']), 'origin_x': float(wa['origin_x']),
           'origin_y': float(wa['origin_y'])}
    path = os.path.join(tempfile.gettempdir(), 'w6_derived_map_%s_%d_%s.yaml' % (scene, seed, token))
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
    return path


def load_scene(task_share, perc_share, sim_share, scene, seed, token=None):
    """加载场景配置：base / registry / carriers，校验 scene 存在与 support_status，逐轴取承载者。

    返回结构化 dict（含 base / sc / axes / support_status / entries 与按轴选择的文件路径）；
    S-06：额外返回几何真源（`geometry` / `geometry_sha256`）与**按域派生文件**
    （`derived_obstacles_file` / `derived_map_file`）。
    """
    token = token or new_run_token()
    base = yaml.safe_load(open(os.path.join(task_share, 'config', 'w6_base.yaml')))
    registry = yaml.safe_load(open(os.path.join(task_share, 'config', 'scenarios.yaml')))
    carriers = yaml.safe_load(
        open(os.path.join(task_share, 'config', 'scenario_carriers.yaml')))['axes']
    scenes = registry.get('scenes', {})
    if scene not in scenes:
        raise RuntimeError('unknown scene: %s; available=%s' % (scene, list(scenes)))
    sc = scenes[scene] or {}
    axes = sc.get('axes') or {}
    support_status = sc.get('support_status', 'supported')

    # R-026 启动时校验：每个声明轴值必须有承载者（obstacle=dynamic → 拒绝运行）
    entries = {ax: carrier_entry(carriers, ax, val, scene) for ax, val in axes.items()}
    if str(support_status).startswith('unsupported'):
        raise RuntimeError(
            '场景 %s support_status=%s（声明轴无承载者）→ 拒绝运行' % (scene, support_status))

    # 按轴选择承载者文件
    sensors_file_name = entries['perception']['sensors_file']
    obstacles_file_name = entries['obstacle']['obstacles_file']
    world_obstacles = entries['obstacle']['world_obstacles']
    sensors_file = os.path.join(perc_share, 'config', sensors_file_name)

    # S-06：几何真源 + 按域派生（感知障碍先验 / 感知地图）。障碍文件改由真源派生，
    #   不再读 agc_perception 包内副本。
    geometry = load_geometry(sim_share)
    geometry_hash = geometry_sha256(sim_share)
    derived_obstacles_file, derived_obstacles_ids = write_derived_obstacles(
        geometry, world_obstacles, scene, seed, token)
    derived_map_file = write_derived_map(geometry, scene, seed, token)
    world_file = os.path.join(sim_share, 'worlds', 'agc_world.sdf')

    return {
        'scene': scene, 'seed': seed,
        'base': base, 'sc': sc, 'axes': axes,
        'support_status': support_status, 'entries': entries,
        'sensors_file_name': sensors_file_name, 'obstacles_file_name': obstacles_file_name,
        'world_obstacles': world_obstacles,
        'sensors_file': sensors_file,
        'obstacles_file': derived_obstacles_file,          # S-06：派生路径（非包内副本）
        'derived_obstacles_file': derived_obstacles_file,
        'derived_obstacles_ids': derived_obstacles_ids,
        'derived_map_file': derived_map_file,
        'geometry': geometry,
        'geometry_source': os.path.join(sim_share, 'config', GEOMETRY_FILE),
        'geometry_sha256': geometry_hash,
        'world_file': world_file,
    }


def compose_params(base, sc, backend, seed, own_uav=(0.0, 0.0), geometry=None,
                   spawn_randomize=None):
    """合并参数（base + 场景覆盖）并按 backend 注入运行模式。

    lightweight：`task_manager.air_present=False`（无空中域，相位复位门禁 C5 不适用，R-001）。
    full：`task_manager.air_present=True` + `scenario_reset.own_uav_present/x/y`（己方机在场 + 出生净空校验透传）。
    seed 经 launch 单一覆盖（run_logger 记录 + scenario_reset 生成 spawn，C7 可复现）。
    `spawn_randomize`（W7-a · R3）：`None` = 不改（沿用 base / 场景值）；显式 `True/False` = 覆盖——
    训练视图按 seed 随机化出生位（spawn 流），缺省 `None` → 既有 W6 行为不变。
    S-06：`scenario_reset` 障碍清单 / 作业区边界由**单一几何真源**注入（w6_base 不再硬编码几何）。
    """
    merged = {node: dict(sec.get('ros__parameters', {})) for node, sec in base.items()}
    for node, over in (sc.get('params') or {}).items():
        merged.setdefault(node, {}).update(over)
    if backend == 'lightweight':
        # 轻量栈不起空中链路：task_manager 显式声明无空中域，不得由服务缺席推定（R-001）。
        merged['task_manager']['air_present'] = False
    else:
        # 整链起真实空中链路：相位复位门禁（C5）强制（R-001）。
        merged['task_manager']['air_present'] = True
        # R-014：己方机在出生净空校验中视为在场（显式运行模式），并透传其 spawn 位（world ENU）。
        merged['scenario_reset']['own_uav_present'] = True
        merged['scenario_reset']['own_uav_x'] = own_uav[0]
        merged['scenario_reset']['own_uav_y'] = own_uav[1]
    merged['run_logger']['seed'] = seed
    merged['scenario_reset']['spawn_seed'] = seed
    # W7-a（R3）：训练视图按 seed 随机化出生位（显式覆盖；缺省 None → 沿用 base / 场景值）。
    if spawn_randomize is not None:
        merged['scenario_reset']['spawn_randomize'] = bool(spawn_randomize)
    # S-06：静态障碍清单 + 作业区边界由**真源**派生注入（物理世界为 static 时注入 pole_*）。
    if geometry is not None:
        merged['scenario_reset']['obstacle_names'] = geometry_ids(geometry)
        merged['scenario_reset']['obstacle_x'] = geometry_xs(geometry)
        merged['scenario_reset']['obstacle_y'] = geometry_ys(geometry)
        merged['scenario_reset'].update(work_area_bounds(geometry))
    return merged


def apply_air_geometry(air_merged, geometry):
    """S-06：把真源静态障碍几何注入空中安全列表（`trajectory_tracker.safety`）。

    `obstacle_names/x/y` 由真源派生；`obstacle_safety_radius` / margin 等**安全余量**仍来自
    `air.yaml`（安全参数，非几何）。原地修改并返回 `air_merged`。
    """
    safety = air_merged.setdefault('trajectory_tracker', {}).setdefault('safety', {})
    safety['obstacle_names'] = geometry_ids(geometry)
    safety['obstacle_x'] = geometry_xs(geometry)
    safety['obstacle_y'] = geometry_ys(geometry)
    return air_merged


def assertions(scene, merged, info, air_merged=None):
    """R-026 + S-06 断言：实际生效参数 == 声明轴、**派生几何 == 真源**；任一失败 → 拒绝运行。"""
    axes = info['axes']
    entries = info['entries']
    geometry = info['geometry']
    gids = geometry_ids(geometry)
    gxs = geometry_xs(geometry)
    gys = geometry_ys(geometry)
    bounds = work_area_bounds(geometry)
    sr = merged.get('scenario_reset', {})
    obs_ids = info['derived_obstacles_ids']
    res = {
        'target_motion_effective_equals_declared':
            merged.get('intruder_driver', {}).get('motion') == axes.get('target_motion'),
        'obstacle_world_effective_matches':
            info['world_obstacles'] == entries['obstacle']['world_obstacles'],
        'obstacles_file_effective_matches':
            info['obstacles_file_name'] == entries['obstacle']['obstacles_file'],
        'perception_sensors_file_effective_matches':
            info['sensors_file_name'] == entries['perception']['sensors_file'],
        # S-06：派生几何 == 真源（scenario_reset 障碍清单 / 作业区边界）。
        'scenario_reset_obstacles_match_geometry':
            list(sr.get('obstacle_names', [])) == gids
            and [float(v) for v in sr.get('obstacle_x', [])] == gxs
            and [float(v) for v in sr.get('obstacle_y', [])] == gys,
        'scenario_reset_bounds_match_work_area':
            all(k in sr for k in bounds)
            and all(float(sr[k]) == v for k, v in bounds.items()),
        # S-06：感知障碍先验按轴派生（static=真源；none=空），无第二份几何。
        'derived_obstacles_match_axis':
            (obs_ids == gids) if info['world_obstacles'] == 'static' else (obs_ids == []),
    }
    # ADR-0076 不变量：**已授权射击**的场景必须关闭「跟踪里程碑即终态」——
    #   否则 `capture_adjudicator` 的交火窗口（`task_state == GROUND_APPROACH`）会在
    #   `TRACKING_SUCCESS` 锁存时被提前关闭，捕获（`INTERCEPT_CONFIRMED/CAPTURE_NET`）**永不发生**
    #   （实测 s08 协同臂 20/20 落 TRACKING_SUCCESS）。此断言把该配置矛盾挡在启动前。
    authorized = bool((merged.get('capture_adjudicator') or {}).get('interception_authorized', False))
    milestone_terminal = bool(
        (merged.get('task_manager') or {}).get('tracking_milestone_is_terminal', True))
    res['capture_authorized_requires_nonterminal_milestone'] = (not authorized) or (not milestone_terminal)
    if air_merged is not None:
        safety = air_merged.get('trajectory_tracker', {}).get('safety', {})
        res['air_obstacles_match_geometry'] = (
            list(safety.get('obstacle_names', [])) == gids
            and [float(v) for v in safety.get('obstacle_x', [])] == gxs
            and [float(v) for v in safety.get('obstacle_y', [])] == gys)
    bad = [k for k, v in res.items() if not v]
    if bad:
        raise RuntimeError(
            '场景 %s 实际生效参数 != 声明轴 或 派生几何 != 真源（断言失败：%s）→ 拒绝运行' % (
                scene, bad))
    return res


def build_manifest(backend, info, merged, assertions_result):
    """产出 manifest dict（身份与入口名一致 + 完整展开节点 + 生效参数）。

    `launch` == `LAUNCH`（入口名）；`backend` = 实际 backend；`nodes` 由 backend 能力表派生。
    """
    scene = info['scene']
    seed = info['seed']
    axes = info['axes']
    entries = info['entries']
    sc = info['sc']
    support_status = info['support_status']
    world_obstacles = info['world_obstacles']
    sensors_file_name = info['sensors_file_name']
    obstacles_file_name = info['obstacles_file_name']
    sensors_file = info['sensors_file']
    obstacles_file = info['obstacles_file']
    world_file = info['world_file']
    target_source = BACKENDS[backend]['target_source']

    # 实际障碍集：从**真源派生** obstacles_file 读取 id（none → []；static → pole_1/pole_2）
    _obs_doc = yaml.safe_load(open(obstacles_file)) or {}
    obstacle_ids = [o.get('id') for o in (_obs_doc.get('obstacles') or [])]

    # S-06：几何版本身份——真源路径 + 内容 sha256 + work_area + 障碍（id/pose/size）。
    geometry = info['geometry']
    geometry_block = {
        'source': info['geometry_source'],
        'sha256': info['geometry_sha256'],
        'work_area': geometry.get('work_area'),
        'obstacles': [
            {
                'id': ob['id'],
                'pose': [float(ob['x']), float(ob['y']), float(ob['z']), 0.0, 0.0,
                         float(ob.get('yaw', 0.0))],
                'size': [float(s) for s in ob['size']],
            }
            for ob in (geometry.get('obstacles') or [])
        ],
    }

    # R-026 运行时证据：声明轴 + 承载者（状态由能力表派生）+ 实际生效参数/文件
    carriers_ledger = {}
    for ax, val in axes.items():
        st, why = coverage_status(entries[ax], backend)
        eff = {}
        if ax == 'obstacle':
            eff = {'world_obstacles': world_obstacles, 'obstacles_file': obstacles_file_name,
                   'obstacle_ids': obstacle_ids}
        elif ax == 'perception':
            eff = {'sensors_file': sensors_file_name, 'target_source': target_source}
        elif ax == 'target_motion':
            eff = {'intruder_driver.motion': merged.get('intruder_driver', {}).get('motion')}
        carriers_ledger[ax] = {'value': val, 'carrier': entries[ax].get('carrier'),
                               'status': st, 'reason': why, 'effective': eff}

    return {
        'scene': scene, 'seed': seed, 'backend': backend, 'launch': LAUNCH,
        'support_status': support_status,
        'declared_axes': axes,
        'expected_terminal': sc.get('expected_terminal'),
        'expected_reason': sc.get('expected_reason'),
        'nodes': list(BACKENDS[backend]['required_nodes']),
        'carriers': carriers_ledger,
        'effective_params': {
            'task_manager': {k: merged.get('task_manager', {}).get(k) for k in (
                'air_present', 'goal_follow_target', 'goal_x', 'goal_y', 't_max_s')},
            'intruder_driver': {k: merged.get('intruder_driver', {}).get(k) for k in (
                'motion', 'speed_mps', 'radius_m')},
            'run_logger': {'scenario': merged.get('run_logger', {}).get('scenario'), 'seed': seed},
            'target_source': target_source,
            'sensors_file': sensors_file, 'obstacles_file': obstacles_file,
            'world_obstacles': world_obstacles, 'world_file': world_file,
        },
        'geometry': geometry_block,
        'assertions': assertions_result,
    }


def write_temp_params(merged, backend, scene, seed, token=None):
    """把合并后参数写入临时 yaml（`w6_<backend>_<scene>_<seed>_<token>.yaml`），返回路径。"""
    token = token or new_run_token()
    doc = {node: {'ros__parameters': p} for node, p in merged.items()}
    tmp = os.path.join(tempfile.gettempdir(), 'w6_%s_%s_%d_%s.yaml' % (backend, scene, seed, token))
    with open(tmp, 'w') as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
    return tmp


def write_manifest(manifest, backend, scene, seed, token=None):
    """把 manifest 写入临时 json（`w6_<backend>_manifest_<scene>_<seed>_<token>.json`），返回路径。"""
    token = token or new_run_token()
    path = os.path.join(
        tempfile.gettempdir(), 'w6_%s_manifest_%s_%d_%s.json' % (backend, scene, seed, token))
    with open(path, 'w') as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
    return path
