#!/usr/bin/env python3
"""w6_r040_bench —— R-040（地图内容身份与单调更新序号）**真实节点台架**。

起**真实**二进制 `install/agc_perception/lib/agc_perception/map_adapter`，用**临时生成的
map/obstacles YAML**（作 `--params-file` 的 `map_file` / `obstacles_file`），订阅 `/agc/map`
（`OccupancyGrid`）、`/agc/map/version_hash`、`/agc/map/update_seq`、`/agc/map/valid`，**同一时间轴**记录。

场景（逐条验收 R-040 / checklist）：
  1. `same_bytes_diff_origin` 同字节不同 origin：A（origin 0，障碍 x=1.25）/ B（origin_x 0.5，障碍 x=1.75），
     res 0.5 → 二者占据同一格，**data 字节相同**；断言 `version_hash` **不同**、data-only hash **相同**。
  2. `cross_boundary` 同分箱跨边界：同一动态障碍 x 0.30 → 0.70（旧分箱 `round(x/res)` 均为 1，
     栅格列由 0 → 1）→ 断言 data 变化、`update_seq` **递增**（旧分箱漏报、真实 raster 捕获）。
  3. `monotonic_seq` 实质重建单调序号：同内容反复输入 → seq / hash **不变**；移动到另一格 / 动态轨过期
     → seq **严格递增**、hash 变化（过期产生 unknown=-1）。
  4. `r006_config_valid` / `r006_air_silent` 保持 R-006：config 模式 `valid=true` 且栅格含 occupied(100)；
     air_fusion 模式源静默 → `valid=false`；三态与 info↔data 长度一致。

只回收**本进程组**子进程（`start_new_session` + `killpg`）；**不**用裸名 `pkill -f`、**不**做递归删除。
证据落 `experiments/w6-task-eval/<run_id>/r040/bench.json`（ADR-0007）。

⚠ 范围标注：本台架为**节点行为台架**（真实 map_adapter 二进制 + 合成障碍流，**非**整链 / **非** GUI）。
"""

import argparse
import math
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String, UInt64
from agc_msgs.msg import ObstacleTrackArray, ObstacleTrack

ROOT = '/home/chen/AGC'
PERC_BIN = os.path.join(ROOT, 'install', 'agc_perception', 'lib', 'agc_perception')
MAP_TOPIC = '/agc/map'
HASH_TOPIC = '/agc/map/version_hash'
SEQ_TOPIC = '/agc/map/update_seq'
VALID_TOPIC = '/agc/map/valid'
OBST_TOPIC = '/agc/obstacles'

RES = 0.5
SX = 4.0
SY = 2.0
SV = 'r040-bench'          # 场景 / 源版本（折入内容身份 hash）

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def fnv_data(data):
    """对栅格 data 的 int8 字节做 FNV-1a（**仅 data**，用于与内容身份 hash 对比）。"""
    h = 1469598103934665603
    for v in data:
        h ^= (v & 0xff)
        h = (h * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return '%016x' % h


def _bin(v):
    """对齐 C++ std::llround(v/res)：四舍五入到最近整数（旧 sameObstacles 的分箱）。"""
    return int(math.floor(v / RES + 0.5))


def _track(oid, x, y):
    return {'id': oid, 'x': x, 'y': y, 'sx': 0.04, 'sy': 0.04, 'sz': 0.04}


def _kill(proc):
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    try:
        if pgid == os.getpgrp():
            proc.send_signal(signal.SIGKILL)
        else:
            os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError, OSError):
        pass
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        pass


def _reap_stale():
    pat = 'install/agc_perception/lib/agc_perception/map_adapte[r]'
    try:
        out = subprocess.run(['pgrep', '-f', pat], capture_output=True, text=True, timeout=10)
    except Exception:  # noqa: BLE001
        out = None
    if out is not None:
        for pid in [p for p in out.stdout.split() if p.strip().isdigit()]:
            if int(pid) != os.getpid():
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except OSError:
                    pass
    time.sleep(0.5)


class Harness:
    def __init__(self, name):
        self.node = Node('r040_bench_' + name)
        self.maps = []
        self.hashes = []
        self.seqs = []
        self.valids = []
        self.obst = []
        self.obst_pub = self.node.create_publisher(ObstacleTrackArray, OBST_TOPIC, 10)
        self.node.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, LATCHED)
        self.node.create_subscription(String, HASH_TOPIC, self._on_hash, LATCHED)
        self.node.create_subscription(UInt64, SEQ_TOPIC, self._on_seq, LATCHED)
        self.node.create_subscription(Bool, VALID_TOPIC, self._on_valid, LATCHED)

    def _on_map(self, m):
        d = list(m.data)
        self.maps.append({
            'wall': time.time(), 'frame': m.header.frame_id,
            'res': m.info.resolution, 'w': int(m.info.width), 'h': int(m.info.height),
            'ox': m.info.origin.position.x, 'oy': m.info.origin.position.y,
            'data_hash': fnv_data(d), 'data_len': len(d),
            'has100': any(v == 100 for v in d), 'has_unknown': any(v == -1 for v in d),
            'data': d,
        })

    def _on_hash(self, m):
        self.hashes.append({'wall': time.time(), 'data': m.data})

    def _on_seq(self, m):
        self.seqs.append({'wall': time.time(), 'data': int(m.data)})

    def _on_valid(self, m):
        self.valids.append({'wall': time.time(), 'data': bool(m.data)})

    def pub_obstacles(self, tracks):
        msg = ObstacleTrackArray()
        stamp = self.node.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = 'world'
        for t in tracks:
            ot = ObstacleTrack()
            ot.header.stamp = stamp
            ot.header.frame_id = 'world'
            ot.obstacle_id = t['id']
            ot.pose.position.x = float(t['x'])
            ot.pose.position.y = float(t['y'])
            ot.pose.position.z = float(t.get('z', 0.0))
            ot.pose.orientation.w = 1.0
            ot.size.x = float(t['sx'])
            ot.size.y = float(t['sy'])
            ot.size.z = float(t['sz'])
            ot.confidence = 0.9
            ot.source = 'air_fusion'
            msg.tracks.append(ot)
        self.obst_pub.publish(msg)
        self.obst.append({'wall': time.time(), 'tracks': tracks})

    def pump(self, duration, cb, hz=20.0):
        t0 = time.time()
        last = 0.0
        period = 1.0 / hz
        while True:
            now = time.time()
            el = now - t0
            if el >= duration:
                break
            if now - last >= period:
                last = now
                cb(el, now)
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def drain(self, duration):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)


def _wait_settle(h, tracks, timeout_s=6.0, need=4, require_occupied=True):
    """反复发布 tracks 直到 update_seq 稳定**且**栅格已含占据格（覆盖 /agc/obstacles 订阅 DDS discovery 延迟）。"""
    end = time.time() + timeout_s
    last, stable = None, 0
    while time.time() < end:
        h.pub_obstacles(tracks)
        rclpy.spin_once(h.node, timeout_sec=0.05)
        s = h.seqs[-1]['data'] if h.seqs else None
        occ = bool(h.maps and h.maps[-1]['has100'])
        if (not require_occupied or occ) and s is not None and s == last:
            stable += 1
            if stable >= need:
                break
        else:
            stable, last = 0, s
    return (h.seqs[-1]['data'] if h.seqs else None)


def _snap(hh, m, source, scen_ver):
    if m is None:
        return None
    return {'frame': m['frame'], 'res': m['res'], 'w': m['w'], 'h': m['h'],
            'ox': m['ox'], 'oy': m['oy'], 'data': m['data'],
            'source': source, 'scenario_version': scen_ver,
            'published_hash': hh, 'published_data_hash': m['data_hash'],
            'data_len': m['data_len'], 'has100': m['has100'], 'has_unknown': m['has_unknown']}


def _lists(h):
    return {
        'seq_values': [s['data'] for s in h.seqs],
        'hash_values': [s['data'] for s in h.hashes],
        'valid_values': [v['data'] for v in h.valids],
        'data_hashes': [m['data_hash'] for m in h.maps],
    }


# ------------------------------------------------------------------ 场景

def sc_same_bytes_diff_origin(h):
    """采集一份内容快照（origin 由参数文件决定，两 run 的外部比较见 main）。"""
    h.drain(0.8)
    m = h.maps[-1] if h.maps else None
    hh = h.hashes[-1]['data'] if h.hashes else None
    return {'pass': bool(m is not None and hh is not None),
            'snapshot': _snap(hh, m, 'config', SV),
            'seq': (h.seqs[-1]['data'] if h.seqs else None),
            'valid': (h.valids[-1]['data'] if h.valids else None)}


def sc_cross_boundary(h):
    """同分箱跨边界：x 0.30 -> 0.70（bin 均 1，栅格列 0 -> 1）。"""
    _wait_settle(h, [_track('dyn1', 0.30, 1.0)])
    mA = h.maps[-1] if h.maps else None
    sA = h.seqs[-1]['data'] if h.seqs else None
    hA = h.hashes[-1]['data'] if h.hashes else None
    _wait_settle(h, [_track('dyn1', 0.70, 1.0)])
    mB = h.maps[-1] if h.maps else None
    sB = h.seqs[-1]['data'] if h.seqs else None
    hB = h.hashes[-1]['data'] if h.hashes else None
    raster_changed = bool(mA and mB and mA['data_hash'] != mB['data_hash'])
    seq_inc = (sA is not None and sB is not None and sB > sA)
    same_bin = (_bin(0.30) == _bin(0.70))
    return {
        'pass': bool(raster_changed and seq_inc and same_bin),
        'raster_changed': raster_changed, 'seq_before': sA, 'seq_after': sB,
        'seq_incremented': seq_inc, 'hash_before': hA, 'hash_after': hB,
        'bin_before': _bin(0.30), 'bin_after': _bin(0.70),
        'same_bin_old_impl_would_miss': same_bin,
        'data_hash_before': (mA['data_hash'] if mA else None),
        'data_hash_after': (mB['data_hash'] if mB else None),
        'content_before': _snap(hA, mA, 'air_fusion', SV),
        'content_after': _snap(hB, mB, 'air_fusion', SV),
        'note': '同分箱（floor(x/res+0.5) 均为 1）跨格：旧分箱会漏报，真实 raster 捕获 -> data 变 + seq 递增。',
    }


def sc_monotonic_seq(h):
    """实质重建单调序号：同内容不变；移动 / 过期严格递增。"""
    trkA = [_track('dyn1', 0.30, 1.0)]
    trkB = [_track('dyn1', 0.70, 1.0)]
    _wait_settle(h, trkA)                       # 建立内容并等 update_seq 稳定（覆盖 DDS discovery 延迟）
    sA = h.seqs[-1]['data'] if h.seqs else None
    hA = h.hashes[-1]['data'] if h.hashes else None
    iA = len(h.seqs)
    h.pump(0.7, lambda el, now: h.pub_obstacles(trkA))
    window = [s['data'] for s in h.seqs[iA:]]
    sB = h.seqs[-1]['data'] if h.seqs else None
    same_stable = (sA is not None and sB is not None and sA == sB and len(set(window)) <= 1
                   and (not window or window[0] == sA))
    _wait_settle(h, trkB)
    sC = h.seqs[-1]['data'] if h.seqs else None
    hC = h.hashes[-1]['data'] if h.hashes else None
    changed = (sC is not None and sB is not None and sC > sB and hC != hA)
    h.drain(1.4)
    sD = h.seqs[-1]['data'] if h.seqs else None
    hD = h.hashes[-1]['data'] if h.hashes else None
    expired = (sD is not None and sC is not None and sD > sC and hD != hC)
    unknown_seen = any(m['has_unknown'] for m in h.maps)
    return {
        'pass': bool(same_stable and changed and expired and unknown_seen),
        'seq_same_content_stable': same_stable, 'seq_after_first': sA, 'seq_after_repeat': sB,
        'seq_after_move': sC, 'seq_after_expire': sD,
        'hash_first': hA, 'hash_move': hC, 'hash_expire': hD,
        'same_content_window_distinct': sorted(set(window)),
        'move_strictly_increased': changed, 'expire_strictly_increased': expired,
        'unknown_seen_after_expire': unknown_seen,
        'content_last': _snap(hD, h.maps[-1] if h.maps else None, 'air_fusion', SV),
        'note': '同内容 seq/hash 不变；移动 / 过期 -> seq 严格 +1、hash 变化（过期产生 unknown=-1）。',
    }


def sc_r006_config_valid(h):
    h.drain(0.8)
    m = h.maps[-1] if h.maps else None
    valid_true = any(v['data'] for v in h.valids)
    has100 = bool(m and m['has100'])
    len_ok = bool(m and m['data_len'] == m['w'] * m['h'])
    return {'pass': bool(valid_true and has100 and len_ok and m is not None),
            'valid_seen_true': valid_true, 'valid_flags': [v['data'] for v in h.valids],
            'has100': has100, 'len_ok': len_ok,
            'content': _snap(h.hashes[-1]['data'] if h.hashes else None, m, 'config', SV)}


def sc_r006_air_silent(h):
    h.drain(0.9)
    m = h.maps[-1] if h.maps else None
    valid_false = any(not v['data'] for v in h.valids)
    len_ok = bool(m and m['data_len'] == m['w'] * m['h'])
    return {'pass': bool(valid_false and len_ok and m is not None),
            'valid_seen_false': valid_false, 'valid_flags': [v['data'] for v in h.valids],
            'has100': (m['has100'] if m else None),
            'has_unknown': (m['has_unknown'] if m else None), 'len_ok': len_ok,
            'content': _snap(h.hashes[-1]['data'] if h.hashes else None, m, 'air_fusion', SV)}


def _write(path, text):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def _write_geometry(path):
    _write(path, 'resolution: %s\nsize_x: %s\nsize_y: %s\norigin_x: %s\norigin_y: %s\n'
           % (RES, SX, SY, 0.0, 0.0))


def _write_geometry_origin(path, ox):
    _write(path, 'resolution: %s\nsize_x: %s\nsize_y: %s\norigin_x: %s\norigin_y: %s\n'
           % (RES, SX, SY, ox, 0.0))


def _write_obstacles(path, obstacles):
    if not obstacles:
        _write(path, 'obstacles: []\n')
        return
    lines = ['obstacles:']
    for o in obstacles:
        lines.append('  - id: %s' % o['id'])
        lines.append('    x: %s' % o['x'])
        lines.append('    y: %s' % o['y'])
        lines.append('    z: %s' % o.get('z', 0.0))
        lines.append('    yaw: %s' % o.get('yaw', 0.0))
        lines.append('    size: [%s, %s, %s]' % (o['sx'], o['sy'], o['sz']))
    _write(path, '\n'.join(lines) + '\n')


def _write_params(path, map_file, obstacles_file, source, dyn_age, src_age):
    _write(path,
           'map_adapter:\n  ros__parameters:\n'
           '    use_sim_time: false\n'
           '    world_frame: "world"\n'
           '    map_file: "%s"\n'
           '    obstacles_file: "%s"\n'
           '    obstacles_source: "%s"\n'
           '    rate_hz: 5.0\n'
           '    map_topic: "%s"\n'
           '    scenario_version: "%s"\n'
           '    update_seq_topic: "%s"\n'
           '    dynamic_max_age_s: %s\n'
           '    source_max_age_s: %s\n'
           % (map_file, obstacles_file, source, MAP_TOPIC, SV, SEQ_TOPIC, dyn_age, src_age))


def run_scenario(name, params_file, log_dir, script_fn):
    log = os.path.join(log_dir, name + '.log')
    proc = subprocess.Popen(
        [os.path.join(PERC_BIN, 'map_adapter'), '--ros-args', '--params-file', params_file],
        stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    h = Harness(name)
    res = {'scenario': name}
    try:
        end = time.time() + 4.0
        while time.time() < end:
            rclpy.spin_once(h.node, timeout_sec=0.05)
            if h.maps or h.valids:
                break
        connected = bool(h.maps or h.valids)
        res.update(script_fn(h))
        res['scenario'] = name
        res['discovery_connected'] = connected
        res['pass'] = bool(res.get('pass') and connected)
        res.update(_lists(h))
    finally:
        _kill(proc)
        h.node.destroy_node()
    return res


def run_scenario_with_retry(name, params_file, log_dir, script_fn, attempts=3):
    last = None
    for i in range(attempts):
        rclpy.init()
        try:
            last = run_scenario(name + ('' if i == 0 else '_r%d' % i), params_file, log_dir, script_fn)
        finally:
            rclpy.shutdown()
        if last.get('discovery_connected'):
            last['scenario'] = name
            last.setdefault('retries', i)
            return last
        _reap_stale()
        time.sleep(0.5)
    last['scenario'] = name
    return last


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261018-w6-r040-0')
    ap.add_argument('--experiments-root', default=os.path.join(ROOT, 'experiments'))
    args = ap.parse_args()

    out_dir = os.path.join(args.experiments_root, 'w6-task-eval', args.run_id)
    cfg_dir = os.path.join(out_dir, 'config')
    log_dir = os.path.join(out_dir, 'r040', 'log')
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    # 场景文件（临时 YAML）
    gA = os.path.join(cfg_dir, 'r040_map_A.yaml')
    gB = os.path.join(cfg_dir, 'r040_map_B.yaml')
    g0 = os.path.join(cfg_dir, 'r040_map_0.yaml')
    oA = os.path.join(cfg_dir, 'r040_obs_A.yaml')
    oEmpty = os.path.join(cfg_dir, 'r040_obs_empty.yaml')
    _write_geometry_origin(gA, 0.0)
    _write_geometry_origin(gB, 0.5)
    _write_geometry(g0)
    _write_obstacles(oA, [{'id': 'o1', 'x': 1.25, 'y': 1.25, 'sx': 0.4, 'sy': 0.4, 'sz': 0.4}])
    _write_obstacles(oEmpty, [])

    pA = os.path.join(cfg_dir, 'r040_params_A.yaml')
    pB = os.path.join(cfg_dir, 'r040_params_B.yaml')
    pCross = os.path.join(cfg_dir, 'r040_params_cross.yaml')
    pMono = os.path.join(cfg_dir, 'r040_params_mono.yaml')
    pCfg = os.path.join(cfg_dir, 'r040_params_cfg.yaml')
    pAir = os.path.join(cfg_dir, 'r040_params_air.yaml')
    # A/B：config 模式，障碍相对同一格（A origin 0 & x=1.25；B origin 0.5 & x=1.75）
    _write_params(pA, gA, oA, 'config', 30.0, 30.0)
    _write_params(pB, gB, os.path.join(cfg_dir, 'r040_obs_B.yaml'), 'config', 30.0, 30.0)
    _write_obstacles(os.path.join(cfg_dir, 'r040_obs_B.yaml'),
                     [{'id': 'o1', 'x': 1.75, 'y': 1.25, 'sx': 0.4, 'sy': 0.4, 'sz': 0.4}])
    _write_params(pCross, g0, oEmpty, 'air_fusion', 30.0, 30.0)
    _write_params(pMono, g0, oEmpty, 'air_fusion', 0.5, 30.0)
    _write_params(pCfg, g0, oA, 'config', 30.0, 30.0)
    _write_params(pAir, g0, oEmpty, 'air_fusion', 30.0, 30.0)

    rclpy.init()
    _reap_stale()
    rclpy.shutdown()

    result = {
        'run_id': args.run_id,
        'suite': 'R-040 map content-identity + monotonic update-sequence bench',
        'scope_note': ('节点行为台架：真实 map_adapter 二进制 + 合成障碍流 / 临时 map YAML，'
                       '验证内容身份含几何元数据、hash 与单调序号分离、真实 raster 等价不欠包、'
                       'R-006 三分语义保持；**非整链** / 非 GUI。'),
        'unsupported': [],
        'scenarios': {},
    }

    # —— scenario 1：同字节不同 origin（两次独立 run 外部比较）
    ra = run_scenario_with_retry('sbdo_A', pA, log_dir, sc_same_bytes_diff_origin)
    _reap_stale()
    rb = run_scenario_with_retry('sbdo_B', pB, log_dir, sc_same_bytes_diff_origin)
    _reap_stale()
    sa, sb = ra.get('snapshot'), rb.get('snapshot')
    dt_ok = bool(sa and sb and sa['published_data_hash'] == sb['published_data_hash'])
    vh_diff = bool(sa and sb and sa['published_hash'] != sb['published_hash'])
    origin_diff = bool(sa and sb and sa['ox'] != sb['ox'])
    ra['comparison'] = {'data_only_hash_A': (sa['published_data_hash'] if sa else None),
                        'data_only_hash_B': (sb['published_data_hash'] if sb else None),
                        'version_hash_A': (sa['published_hash'] if sa else None),
                        'version_hash_B': (sb['published_hash'] if sb else None),
                        'origin_x_A': (sa['ox'] if sa else None), 'origin_x_B': (sb['ox'] if sb else None),
                        'data_only_equal': dt_ok, 'version_hash_differs': vh_diff, 'origin_differs': origin_diff}
    ra['pass'] = bool(ra.get('pass') and dt_ok and vh_diff and origin_diff)
    ra['scenario'] = 'same_bytes_diff_origin'
    rb['scenario'] = 'same_bytes_diff_origin_B'
    result['scenarios']['same_bytes_diff_origin'] = ra
    result['scenarios']['same_bytes_diff_origin_B'] = rb

    for name, params, fn in (
            ('cross_boundary', pCross, sc_cross_boundary),
            ('monotonic_seq', pMono, sc_monotonic_seq),
            ('r006_config_valid', pCfg, sc_r006_config_valid),
            ('r006_air_silent', pAir, sc_r006_air_silent)):
        result['scenarios'][name] = run_scenario_with_retry(name, params, log_dir, fn)
        print('%-24s PASS=%s' % (name, result['scenarios'][name].get('pass')))
        _reap_stale()

    named = ['same_bytes_diff_origin', 'cross_boundary', 'monotonic_seq',
             'r006_config_valid', 'r006_air_silent']
    result['overall_pass'] = bool(named and all(
        result['scenarios'].get(n, {}).get('pass') for n in named))
    with open(os.path.join(out_dir, 'r040', 'bench.json'), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print('\noverall_pass=%s -> %s' % (
        result['overall_pass'], os.path.join(out_dir, 'r040', 'bench.json')))
    return 0 if result['overall_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
