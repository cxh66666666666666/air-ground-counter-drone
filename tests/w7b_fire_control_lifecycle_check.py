#!/usr/bin/env python3
"""w7b_fire_control_lifecycle_check —— FL-1..FL-4 节点级合成话题回归（ADR-0091）。

**只起** `capture_adjudicator`（不启 GZ / PX4 / 整链 launch），用合成话题驱动：
  /agc/ground/odom      nav_msgs/Odometry（车世界位姿）
  /agc/targets          agc_msgs/TargetTrackArray（target_id=intruder_1，conf=0.9）
  /agc/task/state       agc_msgs/AgcTaskState（state=GROUND_APPROACH=3）
  /agc/task/episode     std_msgs/Int32（transient_local；每个用例新 id → 复位火控）
监视 /agc/task/fire_control 与 /agc/task/capture_evidence。

用例（每个用例新 episode，目标先预热 >= 1.2 s 使 TargetMotionFilter 就绪）：
  A 发射后停止目标                → 恰好一个 SHOT_MISS，无 SHOT_HIT
  B 发射后 conf=0.1               → SHOT_MISS
  C 发射后目标 z=3.0（越出上限）  → SHOT_MISS
  D 发射后停止 odom（目标横向移出）→ SHOT_MISS（车位姿缺失不得冻结在飞）
  E 在飞时切换 episode            → SHOT_ABORTED reason=episode_reset；旧 shot 无后续终局
  F 失手后保持输入有效            → 冷却前无新 SHOT_FIRED；冷却后（+3 s 内）可再射

退出码 0 = 全部 PASS。仅起本测试自己的进程组，清理用 os.killpg（不按名杀进程）。
"""

import argparse
import os
import signal
import subprocess
import sys
import time

# 本机 DDS 发现偶发抖动：测试节点与子进程都固定到 LOCALHOST / domain 0。
os.environ['ROS_DOMAIN_ID'] = '0'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import Int32, String  # noqa: E402

from agc_msgs.msg import AgcTaskState, TargetTrack, TargetTrackArray  # noqa: E402

ROOT = '/home/chen/AGC'
EXE = os.path.join(ROOT, 'install/agc_task/lib/agc_task/capture_adjudicator')

CAR_XYZ = (0.0, 0.0, 0.0)
# 目标放在可交战边界附近（h=0.1、水平 2.35 m；阈值 ≈ 2.413 m）：
# 网在落地前才接近目标（发射后 ≈1.23 s），使「停止目标 / 低置信度」用例在
# target_max_age_s=0.5 s 失效后仍不会被中途命中（命中只可能发生在目标仍有效时）。
TARGET_XYZ = (2.35, 0.0, 0.1)
TARGET_ID = 'intruder_1'
GROUND_APPROACH = 3

RATE_HZ = 20.0
WARM_S = 1.2                # TargetMotionFilter 就绪所需（min_fit_s=0.75 + 8 样本）
FIRE_TIMEOUT_S = 6.0
RESOLVE_TIMEOUT_S = 5.0
COOLDOWN_S = 2.0
FLIGHT_TIMEOUT_S = 3.0

TERMINALS = ('SHOT_HIT', 'SHOT_MISS', 'SHOT_ABORTED')


class Event:
    __slots__ = ('t', 'kind', 'shot', 'reason', 'raw')

    def __init__(self, raw):
        self.t = time.monotonic()
        self.raw = raw
        parts = raw.split()
        self.kind = parts[0] if parts else ''
        self.shot = None
        self.reason = None
        for p in parts:
            if p.startswith('shot='):
                self.shot = p[len('shot='):]
            elif p.startswith('reason='):
                self.reason = p[len('reason='):]

    def __repr__(self):
        return '%s(shot=%s%s)' % (
            self.kind, self.shot,
            '' if self.reason is None else ', reason=%s' % self.reason)


class CaseResult:
    def __init__(self, name):
        self.name = name
        self.checks = []
        self.infra = False
        self.detail = ''

    def check(self, desc, ok):
        self.checks.append((desc, bool(ok)))
        return bool(ok)

    @property
    def passed(self):
        return (not self.infra) and bool(self.checks) and all(ok for _, ok in self.checks)


class Monitor(Node):
    def __init__(self):
        super().__init__('w7b_fire_control_lifecycle_check')
        self.fire = []
        self.evidence = []
        self.episode_id = None
        self.publish_odom = True
        self.publish_target = True
        self.target = TARGET_XYZ
        self.confidence = 0.9
        self.last_pub = 0.0

        self.pub_odom = self.create_publisher(Odometry, '/agc/ground/odom', 10)
        self.pub_targets = self.create_publisher(TargetTrackArray, '/agc/targets', 10)
        self.pub_state = self.create_publisher(AgcTaskState, '/agc/task/state', 10)
        ep_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.pub_episode = self.create_publisher(Int32, '/agc/task/episode', ep_qos)
        self.create_subscription(String, '/agc/task/fire_control', self._on_fire, 20)
        self.create_subscription(String, '/agc/task/capture_evidence', self._on_evidence, 20)

    # —— 订阅 ——
    def _on_fire(self, m):
        self.fire.append(Event(m.data))

    def _on_evidence(self, m):
        self.evidence.append(Event(m.data))

    # —— 发布 ——
    def set_episode(self, ep):
        self.episode_id = ep
        self.pub_episode.publish(Int32(data=int(ep)))

    def _publish_inputs(self):
        t = time.monotonic()
        if t - self.last_pub < 1.0 / RATE_HZ:
            return
        self.last_pub = t
        stamp = self.get_clock().now().to_msg()

        if self.publish_odom:
            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = 'world'
            od.pose.pose.position.x = CAR_XYZ[0]
            od.pose.pose.position.y = CAR_XYZ[1]
            od.pose.pose.position.z = CAR_XYZ[2]
            self.pub_odom.publish(od)

        st = AgcTaskState()
        st.header.stamp = stamp
        st.state = GROUND_APPROACH
        st.state_name = 'GROUND_APPROACH'
        self.pub_state.publish(st)

        if self.publish_target:
            arr = TargetTrackArray()
            arr.header.stamp = stamp
            arr.header.frame_id = 'world'
            tr = TargetTrack()
            tr.target_id = TARGET_ID
            tr.confidence = float(self.confidence)
            tr.source = 'gt_adapter'
            tr.pose = PoseWithCovariance()
            tr.pose.pose.position.x = float(self.target[0])
            tr.pose.pose.position.y = float(self.target[1])
            tr.pose.pose.position.z = float(self.target[2])
            tr.twist = TwistWithCovariance()
            arr.tracks = [tr]
            self.pub_targets.publish(arr)

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            self._publish_inputs()

    def pump_until(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            self._publish_inputs()
            if pred():
                return True
        return False

    # —— 用例辅助 ——
    def begin_case(self, ep):
        self.publish_odom = True
        self.publish_target = True
        self.target = TARGET_XYZ
        self.confidence = 0.9
        self.set_episode(ep)
        self.pump(0.3)   # 让复位 / episode / 首帧输入落到节点

    def wait_fire(self, ep, timeout=FIRE_TIMEOUT_S):
        """预热 >= WARM_S，再等首个属于该 episode 的 SHOT_FIRED；返回 (event, 起始索引)。"""
        prefix = '%d-' % ep
        idx0 = len(self.fire)
        warm_end = time.monotonic() + WARM_S
        while time.monotonic() < warm_end:
            rclpy.spin_once(self, timeout_sec=0.01)
            self._publish_inputs()

        def find():
            for e in self.fire[idx0:]:
                if e.kind == 'SHOT_FIRED' and e.shot and e.shot.startswith(prefix):
                    return e
            return None

        self.pump_until(lambda: find() is not None, timeout)
        return find(), idx0

    def terminals(self, shot, since=0):
        return [e for e in self.fire[since:] if e.kind in TERMINALS and e.shot == shot]

    def evidence_for(self, kind, shot):
        return [e for e in self.evidence if e.kind == kind and e.shot == shot]


def resolve_case(node, ep, name, mutate, expect, forbid_hit=True):
    """通用 A–D：发射 → mutate 输入 → 唯一终局。"""
    r = CaseResult(name)
    node.begin_case(ep)
    fired, idx0 = node.wait_fire(ep)
    if fired is None:
        r.infra = True
        r.check('SHOT_FIRED received', False)
        r.detail = '预热后 %.0f s 内无 SHOT_FIRED' % FIRE_TIMEOUT_S
        return r
    r.check('SHOT_FIRED received', True)

    mutate(node)
    node.pump_until(lambda: len(node.terminals(fired.shot, idx0)) > 0, RESOLVE_TIMEOUT_S)
    node.pump(0.15)   # 给「重复终局」缺陷留一个曝光窗口
    terms = node.terminals(fired.shot, idx0)

    r.check('exactly one terminal', len(terms) == 1)
    r.check('terminal is %s' % expect, len(terms) == 1 and terms[0].kind == expect)
    if forbid_hit:
        r.check('no SHOT_HIT', not any(e.kind == 'SHOT_HIT' for e in terms))
    r.check('capture_evidence marks %s' % expect,
            bool(node.evidence_for(expect, fired.shot)))
    r.detail = 'shot=%s terminals=%s' % (fired.shot, terms)
    return r


def case_e(node, ep):
    """在飞时切换 episode → SHOT_ABORTED reason=episode_reset；旧 shot 无后续终局。"""
    r = CaseResult('E')
    node.begin_case(ep)
    fired, idx0 = node.wait_fire(ep)
    if fired is None:
        r.infra = True
        r.check('SHOT_FIRED received', False)
        r.detail = '预热后无 SHOT_FIRED'
        return r
    r.check('SHOT_FIRED received', True)

    shot = fired.shot
    node.publish_target = False          # 复位后不再有新目标 → 不会新发射
    node.set_episode(ep + 1)             # 在飞时切换到新 episode
    node.pump_until(
        lambda: any(e.kind == 'SHOT_ABORTED' and e.shot == shot for e in node.fire[idx0:]),
        3.0)
    aborts = [e for e in node.fire[idx0:] if e.kind == 'SHOT_ABORTED' and e.shot == shot]
    r.check('exactly one SHOT_ABORTED', len(aborts) == 1)
    r.check('reason=episode_reset', bool(aborts) and aborts[0].reason == 'episode_reset')

    node.pump(2.0)   # 目标已停：新 episode 内不得出现新发射，旧 shot 不得再有终局
    # 证据串与火控串是两次独立发布，稍晚于火控串到达 → 在静置窗口后再核对。
    r.check('capture_evidence marks SHOT_ABORTED',
            bool(node.evidence_for('SHOT_ABORTED', shot)))
    terms = node.terminals(shot, idx0)
    r.check('old shot has no later HIT/MISS',
            len(terms) == 1 and terms[0].kind == 'SHOT_ABORTED')
    r.check('no new SHOT_FIRED without target',
            not any(e.kind == 'SHOT_FIRED' and e.shot != shot for e in node.fire[idx0:]))
    r.detail = 'shot=%s terminals=%s' % (shot, terms)
    return r


def case_f(node, ep):
    """失手后冷却：冷却前不得再射；冷却 + 3 s 内可再射。"""
    r = CaseResult('F')
    node.begin_case(ep)
    fired1, idx0 = node.wait_fire(ep)
    if fired1 is None:
        r.infra = True
        r.check('SHOT_FIRED received', False)
        r.detail = '预热后无 SHOT_FIRED'
        return r
    r.check('first SHOT_FIRED received', True)

    node.publish_target = False          # 目标失效 → 落地 SHOT_MISS
    node.pump_until(lambda: len(node.terminals(fired1.shot, idx0)) > 0, RESOLVE_TIMEOUT_S)
    terms1 = node.terminals(fired1.shot, idx0)
    if not (len(terms1) == 1 and terms1[0].kind == 'SHOT_MISS'):
        r.check('first shot resolved as single SHOT_MISS', False)
        r.detail = 'shot=%s terminals=%s' % (fired1.shot, terms1)
        return r
    r.check('first shot resolved as single SHOT_MISS', True)
    t_miss = terms1[0].t

    # 失手后恢复有效目标 / odom：允许冷却结束再射。
    node.publish_target = True
    node.target = TARGET_XYZ
    node.confidence = 0.9

    def new_fired():
        return [e for e in node.fire[idx0:]
                if e.kind == 'SHOT_FIRED' and e.shot != fired1.shot]

    node.pump_until(lambda: len(new_fired()) > 0, COOLDOWN_S + 3.5)
    new = new_fired()
    r.check('re-fired after cooldown', bool(new))
    if new:
        delta = new[0].t - t_miss
        early = [e for e in node.fire[idx0:]
                 if e.kind == 'SHOT_FIRED' and e.shot != fired1.shot
                 and e.t < t_miss + (COOLDOWN_S - 0.3)]
        r.check('no re-fire before cooldown (tolerance 0.3 s)', not early)
        r.check('re-fire within cooldown + 3 s', delta <= COOLDOWN_S + 3.0)
        r.detail = 'delta=%.2f s' % delta
    else:
        r.detail = 'no re-fire within %.1f s' % (COOLDOWN_S + 3.5)
    return r


def global_check(node):
    """每次 SHOT_FIRED 恰有一个终局（HIT / MISS / ABORTED）。"""
    r = CaseResult('GLOBAL')
    fired = [e for e in node.fire if e.kind == 'SHOT_FIRED']
    terminals = [e for e in node.fire if e.kind in TERMINALS]
    counts = {e.shot: sum(1 for t in terminals if t.shot == e.shot) for e in fired}
    r.check('every SHOT_FIRED has exactly one terminal',
            bool(fired) and all(n == 1 for n in counts.values()))
    extra = sorted(s for s, n in counts.items() if n != 1)
    r.check('no duplicate/missing terminal', not extra)
    r.detail = 'fired=%d terminals=%d odd=%s' % (len(fired), len(terminals), extra)
    return r


def run_suite(base_ep):
    node = Monitor()
    try:
        results = [
            resolve_case(node, base_ep + 1, 'A',
                         lambda n: setattr(n, 'publish_target', False), 'SHOT_MISS'),
            resolve_case(node, base_ep + 11, 'B',
                         lambda n: setattr(n, 'confidence', 0.1), 'SHOT_MISS'),
            resolve_case(node, base_ep + 21, 'C',
                         lambda n: setattr(n, 'target', (2.35, 0.0, 3.0)), 'SHOT_MISS'),
            # D：停 odom；同时把目标横向移出网弹道（否则网会在目标有效期内命中，
            #    从而无法验证「车位姿缺失仍结算」这一路径）。
            resolve_case(node, base_ep + 31, 'D',
                         lambda n: (setattr(n, 'publish_odom', False),
                                    setattr(n, 'target', (2.35, 2.0, 0.1))), 'SHOT_MISS'),
            case_e(node, base_ep + 41),
            case_f(node, base_ep + 51),
        ]
        # 收尾：切换 episode，中止仍可能残留的在飞网，使全局唯一终局判据成立。
        node.set_episode(base_ep + 61)
        node.pump(0.6)
        results.append(global_check(node))
    finally:
        node.destroy_node()
    return results


def start_child():
    args = ([EXE] if os.path.exists(EXE) else ['ros2', 'run', 'agc_task', 'capture_adjudicator'])
    args += [
        '--ros-args',
        '-r', '__node:=capture_adjudicator_lifecycle_test',
        '-p', 'interception_authorized:=true',
        '-p', 'rate_hz:=%.1f' % RATE_HZ,
        '-p', 'net_flight_timeout_s:=%.1f' % FLIGHT_TIMEOUT_S,
        '-p', 'fire_cooldown_s:=%.1f' % COOLDOWN_S,
        '-p', 'target_max_age_s:=0.5',
        '-p', 'confidence_min:=0.5',
        '-p', 'use_sim_time:=false',
    ]
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = '0'
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    return subprocess.Popen(
        args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True, env=env)


def stop_child(proc):
    """只回收本进程组（start_new_session=True）；不按进程名匹配、不杀其它会话。"""
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        pgid = None

    if pgid is not None:
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                break
            except PermissionError:
                break
            time.sleep(0.05)

    try:
        proc.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def print_results(results):
    print('\n%-8s %-6s %s' % ('CASE', 'RESULT', 'DETAIL / FAILED CHECKS'))
    for r in results:
        print('%-8s %-6s %s' % (r.name, 'PASS' if r.passed else 'FAIL', r.detail))
        for desc, ok in r.checks:
            if not ok:
                print('           - FAIL: %s' % desc)


def main():
    ap = argparse.ArgumentParser(description='火控在飞生命周期节点级回归（ADR-0091）')
    ap.add_argument('--attempts', type=int, default=2,
                    help='首个用例未收到 SHOT_FIRED 时的总尝试次数（含首次，默认 2）')
    args = ap.parse_args()

    rclpy.init()
    results = []
    try:
        for attempt in range(max(1, args.attempts)):
            proc = start_child()
            try:
                results = run_suite(base_ep=1000 + attempt * 1000)
            finally:
                stop_child(proc)
            if not (results and results[0].infra):
                break
            print('[retry %d/%d] 首个用例未收到 SHOT_FIRED（疑似启动 / DDS 抖动），重启后重试'
                  % (attempt + 1, args.attempts))
    finally:
        rclpy.shutdown()

    print_results(results)
    ok = bool(results) and all(r.passed for r in results)
    print('\n%s -> tests/w7b_fire_control_lifecycle_check.py' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
