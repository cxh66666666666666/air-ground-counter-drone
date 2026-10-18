#!/usr/bin/env python3
"""w6_geometry_check —— S-06（场景几何单一真源 + 按域派生）一致性核查（只读）。

断言（任一失败即整体 FAIL，退出码 1）：
  (a) 真源 `agc_sim/config/world_geometry.yaml` 存在且 schema 合法；
  (b) `world_bringup`「static 世界」中 pole_1/pole_2 的 pose / size == 真源；`none` 世界不含 pole；
  (c) `agc_perception/config/obstacles.yaml` / `map.yaml`（standalone 回退副本）与真源一致（不漂移）；
  (d) `agc_air/config/air.yaml` 已不含硬编码 `safety.obstacle_names/x/y`（保留安全半径）；
  (e) `agc_task/config/w6_base.yaml` 已去硬编码几何（派生入口），且 `w6_composition` 派生注入 == 真源；
  (f) `geometry_sha256` 稳定；`w6_composition` 派生链（load_scene→compose→assertions→manifest）
      的 manifest `geometry` 块 == 真源。
  (g) 「谁可见何种信息」权限未被真源混用：`obstacles.yaml` 静态先验成文假设仍在。

输出 JSON 摘要（stdout）；`--out <path>` 落盘。用法：`python3 scripts/w6_geometry_check.py`。
"""

import argparse
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml

ROOT = '/home/chen/AGC'


def _share(pkg):
    """优先取 install share；失败回退 src/<pkg>。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:  # noqa: BLE001
        return os.path.join(ROOT, 'src', pkg)


def _load_yaml(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _generate_static_world(wb_path, src, dst, geometry):
    """调用 `world_bringup.launch.py` 的 `_inject_static_obstacles` 生成 static 世界（**单一实现**）。

    经子进程加载 launch 模块：独立把 ROS lib 目录并入 LD_LIBRARY_PATH，使未 `source` ROS 环境
    （缺少 `librcl_action.so`）时 `launch_ros` 亦可导入；主进程本身**不**导入 launch（无 ROS 依赖）。
    """
    lib = '/opt/ros/%s/lib' % os.environ.get('ROS_DISTRO', 'jazzy')
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = lib + ':' + env.get('LD_LIBRARY_PATH', '')
    code = (
        "import importlib.util, json, sys\n"
        "spec = importlib.util.spec_from_file_location('wb_launch', sys.argv[1])\n"
        "m = importlib.util.module_from_spec(spec)\n"
        "spec.loader.exec_module(m)\n"
        "m._inject_static_obstacles(sys.argv[2], sys.argv[3], json.loads(sys.argv[4]))\n"
    )
    subprocess.run([sys.executable, '-c', code, wb_path, src, dst, json.dumps(geometry)],
                   check=True, env=env, capture_output=True)
    return dst


def _sdf_models(world_path):
    """返回 {model_name: {'pose': [6 floats], 'size': [3 floats] | None}}。"""
    root = ET.parse(world_path).getroot()
    out = {}
    for world in root.iter('world'):
        for m in world.findall('model'):
            name = m.get('name')
            pose_el = m.find('pose')
            pose = [float(v) for v in pose_el.text.split()] if pose_el is not None else None
            size_el = m.find('.//box/size')
            size = [float(v) for v in size_el.text.split()] if size_el is not None else None
            out[name] = {'pose': pose, 'size': size}
    return out


def _close(a, b, eps=1e-9):
    return a is not None and b is not None and len(a) == len(b) and all(
        abs(x - y) <= eps for x, y in zip(a, b))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=None, help='JSON 摘要落盘路径（可选）')
    args = ap.parse_args()

    sim_share = _share('agc_sim')
    perc_share = _share('agc_perception')
    air_share = _share('agc_air')
    task_share = _share('agc_task')

    geom_path = os.path.join(sim_share, 'config', 'world_geometry.yaml')
    world_path = os.path.join(sim_share, 'worlds', 'agc_world.sdf')
    wb_path = os.path.join(sim_share, 'launch', 'world_bringup.launch.py')

    res = {'tests': {}}
    try:
        geometry = _load_yaml(geom_path)
        res['geometry_source'] = geom_path
        res['geometry_sha256'] = hashlib.sha256(open(geom_path, 'rb').read()).hexdigest()
        obstacles = geometry.get('obstacles') or []
        wa = geometry.get('work_area') or {}
        src_ids = [o['id'] for o in obstacles]
        src_x = [float(o['x']) for o in obstacles]
        src_y = [float(o['y']) for o in obstacles]
        src_pose = {o['id']: [float(o['x']), float(o['y']), float(o['z']), 0.0, 0.0,
                              float(o.get('yaw', 0.0))] for o in obstacles}
        src_size = {o['id']: [float(s) for s in o['size']] for o in obstacles}

        # (a) 真源 schema
        a = {
            'exists': os.path.exists(geom_path),
            'work_area_keys_ok': all(k in wa for k in (
                'resolution', 'size_x', 'size_y', 'origin_x', 'origin_y')),
            'obstacles_nonempty': bool(obstacles),
            'obstacle_keys_ok': all(all(k in o for k in ('id', 'x', 'y', 'z', 'yaw', 'size'))
                                    for o in obstacles),
        }
        a['pass'] = all(a.values())
        res['tests']['a_source_schema'] = a

        # (b) world_bringup 派生世界
        tmp_static = os.path.join(tempfile.gettempdir(), 'w6_geometry_check_static.sdf')
        _generate_static_world(wb_path, world_path, tmp_static, geometry)
        static_models = _sdf_models(tmp_static)
        none_models = _sdf_models(world_path)  # base world（none 直接用）
        b = {
            'static_has_all_ids': all(i in static_models for i in src_ids),
            'static_pose_size_match': all(
                i in static_models
                and _close(static_models[i]['pose'], src_pose[i])
                and _close(static_models[i]['size'], src_size[i])
                for i in src_ids),
            'none_has_no_pole': not any(str(n).startswith('pole') for n in none_models),
            'static_world_wellformed': True,
        }
        b['pass'] = all(b.values())
        res['tests']['b_world_generation'] = b

        # (c) standalone 回退副本与真源一致
        fb_obs = _load_yaml(os.path.join(perc_share, 'config', 'obstacles.yaml'))
        fb_map = _load_yaml(os.path.join(perc_share, 'config', 'map.yaml'))
        fb_list = fb_obs.get('obstacles') or []

        def _fb_ok(o):
            return (_close([float(o['x']), float(o['y']), float(o['z']),
                            float(o.get('yaw', 0.0))],
                           [src_pose[o['id']][0], src_pose[o['id']][1], src_pose[o['id']][2],
                            src_pose[o['id']][5]])
                    and _close([float(s) for s in o['size']], src_size[o['id']]))

        c = {
            'obstacles_ids_match': [o['id'] for o in fb_list] == src_ids,
            'obstacles_geoms_match': all(_fb_ok(o) for o in fb_list),
            'map_matches_work_area': _close(
                [float(fb_map[k]) for k in
                 ('resolution', 'size_x', 'size_y', 'origin_x', 'origin_y')],
                [float(wa[k]) for k in
                 ('resolution', 'size_x', 'size_y', 'origin_x', 'origin_y')]),
        }
        c['pass'] = all(c.values())
        res['tests']['c_fallback_copies_match'] = c

        # (d) air.yaml 无硬编码障碍几何
        air = _load_yaml(os.path.join(air_share, 'config', 'air.yaml'))
        safety = air['trajectory_tracker']['ros__parameters']['safety']
        d = {
            'no_obstacle_names': 'obstacle_names' not in safety,
            'no_obstacle_x': 'obstacle_x' not in safety,
            'no_obstacle_y': 'obstacle_y' not in safety,
            'keeps_safety_radius': 'obstacle_safety_radius' in safety,
        }
        d['pass'] = all(d.values())
        res['tests']['d_air_no_hardcoded_geometry'] = d

        # (e) w6_base 去硬编码 + compose 派生 == 真源
        sys.path.insert(0, os.path.join(task_share, 'launch'))
        comp = _load_module(os.path.join(task_share, 'launch', 'w6_composition.py'),
                            'w6_composition')
        base = _load_yaml(os.path.join(task_share, 'config', 'w6_base.yaml'))
        sr_base = base['scenario_reset']['ros__parameters']
        merged = comp.compose_params(base, {}, 'lightweight', 0, geometry=geometry)
        sr = merged['scenario_reset']
        bounds = comp.work_area_bounds(geometry)
        e = {
            'w6_base_no_obstacle_names': 'obstacle_names' not in sr_base,
            'w6_base_no_map_bounds': not any(k in sr_base for k in bounds),
            'compose_obstacles_match': (
                list(sr['obstacle_names']) == src_ids
                and [float(v) for v in sr['obstacle_x']] == src_x
                and [float(v) for v in sr['obstacle_y']] == src_y),
            'compose_bounds_match': all(float(sr[k]) == v for k, v in bounds.items()),
        }
        e['pass'] = all(e.values())
        res['tests']['e_w6_base_derived'] = e

        # (f) geometry_sha256 稳定 + 派生链 manifest geometry 块
        h1 = comp.geometry_sha256(sim_share)
        h2 = comp.geometry_sha256(sim_share)
        info = comp.load_scene(task_share, perc_share, sim_share,
                               's04_static_static_obs_track', 0)
        asserts = comp.assertions('s04_static_static_obs_track',
                                  comp.compose_params(base, info['sc'], 'lightweight', 0,
                                                      geometry=info['geometry']),
                                  info)
        man = comp.build_manifest('lightweight', info, merged, asserts)
        gb = man.get('geometry') or {}
        f = {
            'sha256_stable': h1 == h2 and len(h1) == 64 and h1 == res['geometry_sha256'],
            'derived_obstacles_match': [o['id'] for o in (gb.get('obstacles') or [])] == src_ids,
            'manifest_sha256_match': gb.get('sha256') == res['geometry_sha256'],
            'assertions_all_pass': all(asserts.values()),
        }
        f['pass'] = all(f.values())
        res['tests']['f_derivation_and_hash'] = f

        # (g) 权限：静态先验成文假设仍在
        obs_txt = open(os.path.join(perc_share, 'config', 'obstacles.yaml'),
                       encoding='utf-8').read()
        g = {
            'static_prior_wording_present': '静态先验' in obs_txt,
            'no_truth_advantage_clause_present': '不得静默引入真值优势' in obs_txt,
        }
        g['pass'] = all(g.values())
        res['tests']['g_permission_assumption'] = g

        res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
    except Exception as exc:  # noqa: BLE001
        res['error'] = repr(exc)
        res['overall_pass'] = False

    for name, t in res.get('tests', {}).items():
        print('%-32s %s' % (name, 'PASS' if t.get('pass') else 'FAIL'))
        for k, v in t.items():
            if k != 'pass':
                print('    %s = %s' % (k, v))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s (geometry_sha256=%s)' % (
        'PASS' if res.get('overall_pass') else 'FAIL', res.get('geometry_sha256')))

    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, 'w', encoding='utf-8') as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        print('-> %s' % args.out)
    return 0 if res.get('overall_pass') else 1


if __name__ == '__main__':
    sys.exit(main())
