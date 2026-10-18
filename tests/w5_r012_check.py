#!/usr/bin/env python3
"""w5_r012_check —— W5 R-012 基线路径绑定与重规划失效安全故障注入验收。

在 `agc_task w5_safety_scenario.launch.py command_source:=baseline`（GUI，硬约束 C3）运行中执行，
对离线对照基线 `ground_baseline` 做故障注入，观察 `/agc/ground/path`（重规划事件）与
`/agc/ground/cmd_vel_raw`（基线输出）/ `/agc/ground/odom`（真实运动）。

先学后做（导航取证）：本机 Jazzy `nav2_navfn_planner::NavfnPlanner::createPlan` 在 makePlan 失败
（目标不可达 / 出界）时 **throw nav2_core::PlannerException**（`strings libnav2_navfn_planner.so`：
"Failed to create plan with tolerance of:"）——故"目标不可达"落到本节点 `catch` 异常分支；
空路径分支为对其它 planner 的防御分支。语义与本节点 `plan()` 的两处 `path_valid_ = false` 对应。

目标注入方式：`task_manager` 把 `goal_x/goal_y` 在构造时快照为成员且每 1 s 复发布场景目标
（`goal_y=-9`），运行时 `ros2 param set` 不会改变其输出目标。故本脚本在任务进入 `GROUND_APPROACH`
后 **SIGSTOP** `task_manager`（其最后一条 `/agc/task/state` 保持 `GROUND_APPROACH` 供基线门控），
改由本脚本以 ~10 Hz 直接发布 `/agc/ground/goal` 控制目标。

子测：
  d. 正常不抖动    ：目标 / 地图未变、未超时 → 重规划仅按周期（`replan_period_s`）发生，
                     不逐帧抖动（路径发布率 << 控制频率 20 Hz）。
  a. 地图增障      ：沿路径运行中，在车前 ~5 m 注入静态障碍（GZ 生成）→ costmap 内容签名持续偏离
                     → 触发**提前**重规划（gap < `replan_period_s`），新路径绕障。
  b. 目标跳变      ：改目标为**可达的新点** → 触发重规划且新路径端点更新（目标身份绑定）。
  c. 规划失败停车  ：改目标为**不可达点** → planner 抛异常 → 路径失效（`path_valid_=false`）、
                     发布零速、车停住，**不**继续驶向旧目标；记录日志分支。

结果落 experiments/w5-ground-nav-safety/<run_id>/metrics/r012.json。
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from agc_msgs.msg import AgcTaskState
from geometry_msgs.msg import PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from std_srvs.srv import Trigger

RAW = '/agc/ground/cmd_vel_raw'
OUT = '/agc/ground/cmd_vel'
ODOM = '/agc/ground/odom'
GOAL_TOPIC = '/agc/ground/goal'
PATH_TOPIC = '/agc/ground/path'
STATE_TOPIC = '/agc/task/state'
PERIOD_S = 2.0              # replan_period_s（baseline.yaml）
GOAL0 = (0.0, -30.0)        # 远可达目标：保持长距离行驶
GOAL_B = (10.0, -30.0)      # 目标跳变（可达的新点）
GOAL_UNREACH = (0.0, 100.0)  # 出界不可达（map ∈ [-40,40] @0.5）
GROUND_APPROACH = 3


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pgrep(pattern):
    try:
        out = subprocess.check_output(['pgrep', '-f', pattern], text=True)
    except subprocess.CalledProcessError:
        return []
    return [int(p) for p in out.split() if p.strip()]


class R012(Node):
    def __init__(self):
        super().__init__('w5_r012_check')
        self.x = self.y = self.yaw = 0.0
        self.ovx = self.ovy = 0.0
        self.raw_v = self.raw_w = 0.0
        self.out_v = self.out_w = 0.0
        self.state = 0
        self.state_name = ''
        self.paths = []            # (monotonic_t, n_poses, end_x, end_y)
        self.last_path_xy = []     # 最近一条路径的点序列（供沿路径注入障碍）
        self.logs = []             # (t, level, text) from logger ground_baseline
        self.gx, self.gy = GOAL0
        self.allow_goal = False   # 冻结 task_manager 前不注入目标（避免与其 1 Hz 复发布冲突）
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
        self.create_subscription(Odometry, ODOM, self._on_odom, 50)
        self.create_subscription(TwistStamped, RAW, self._on_raw, 50)
        self.create_subscription(TwistStamped, OUT, self._on_out, 50)
        self.create_subscription(Path, PATH_TOPIC, self._on_path, 10)
        self.create_subscription(AgcTaskState, STATE_TOPIC, self._on_state, 20)
        self.create_subscription(Log, '/rosout', self._on_log, 50)
        self.start_cli = self.create_client(Trigger, '/agc/task/start')
        self.state_cli = self.create_client(GetState, '/collision_monitor/get_state')

    def _on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_from_quat(m.pose.pose.orientation)
        self.ovx = m.twist.twist.linear.x
        self.ovy = m.twist.twist.linear.y

    def _on_raw(self, m):
        self.raw_v = m.twist.linear.x
        self.raw_w = m.twist.angular.z

    def _on_out(self, m):
        self.out_v = m.twist.linear.x
        self.out_w = m.twist.angular.z

    def _on_path(self, m):
        end = m.poses[-1].pose.position if m.poses else None
        self.paths.append((time.monotonic(), len(m.poses),
                           end.x if end else float('nan'), end.y if end else float('nan')))
        self.last_path_xy = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def _on_state(self, m):
        self.state = m.state
        self.state_name = m.state_name

    def _on_log(self, m):
        if m.name == 'ground_baseline':
            self.logs.append((time.monotonic(), m.level, m.msg))

    def speed(self):
        return math.hypot(self.ovx, self.ovy)

    def set_goal(self, x, y):
        self.gx, self.gy = float(x), float(y)

    def _pub_goal(self):
        if not self.allow_goal:
            return
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'world'
        g.pose.position.x = self.gx
        g.pose.position.y = self.gy
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    def spin(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self._pub_goal()
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_until(self, cond, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self._pub_goal()
            rclpy.spin_once(self, timeout_sec=0.02)
            if cond():
                return True, time.monotonic() - t0
        return False, time.monotonic() - t0

    def start_task(self):
        if not self.start_cli.wait_for_service(timeout_sec=15.0):
            return False, 'service /agc/task/start unavailable'
        fut = self.start_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        if not fut.done():
            return False, 'start call timeout'
        return fut.result().success, fut.result().message

    def cm_active(self, timeout=70.0):
        if not self.state_cli.wait_for_service(timeout_sec=15.0):
            return False
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            fut = self.state_cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.done() and fut.result() is not None and fut.result().current_state.id == 3:
                return True
            time.sleep(1.0)
        return False

    def next_path_after(self, t, timeout):
        ok, _ = self.spin_until(lambda: any(p[0] > t for p in self.paths), timeout)
        if not ok:
            return None
        return next(p for p in self.paths if p[0] > t)

    def last_path_before(self, t):
        cand = [p for p in self.paths if p[0] <= t]
        return cand[-1] if cand else None


def spawn_box(name, x, y, sx=0.6, sy=0.6, sz=1.2, z=0.6):
    sdf = ('<sdf version="1.9"><model name="%s"><static>true</static>'
           '<link name="link">'
           '<collision name="c"><geometry><box><size>%g %g %g</size></box></geometry></collision>'
           '<visual name="v"><geometry><box><size>%g %g %g</size></box></geometry>'
           '<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>'
           '</link></model></sdf>' % (name, sx, sy, sz, sx, sy, sz))
    try:
        out = subprocess.run(
            ['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'agc_world', '-string', sdf,
             '-name', name, '-x', str(x), '-y', str(y), '-z', str(z)],
            timeout=25, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        return out.returncode == 0, (out.stdout or '').strip()[:400]
    except Exception as ex:  # noqa: BLE001
        return False, str(ex)


def remove_box(name):
    try:
        subprocess.run(
            ['gz', 'service', '-s', '/world/agc_world/remove', '--reqtype', 'gz.msgs.Entity',
             '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
             '--req', 'name: "%s", type: 2' % name],
            timeout=20, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-id', default='20261015-w5-r012-0')
    ap.add_argument('--mode', default='gui')
    ap.add_argument('--experiments-root', default='/home/chen/AGC/experiments')
    args = ap.parse_args()

    rclpy.init()
    n = R012()
    res = {'run_id': args.run_id, 'mode': args.mode, 'tests': {}, 'params': {},
           'logs': [], 'path_events': []}
    frozen = []
    try:
        tw = time.monotonic()
        while time.monotonic() - tw < 12.0 and not n.paths:
            rclpy.spin_once(n, timeout_sec=0.05)
        active = n.cm_active()
        res['collision_monitor_active'] = active
        res['params'] = {'replan_period_s': PERIOD_S, 'goal0': list(GOAL0),
                         'goal_b': list(GOAL_B), 'goal_unreach': list(GOAL_UNREACH)}
        if not active:
            res['overall_pass'] = False
            res['error'] = 'collision_monitor 未 active（infra）'
            return dump(res, args, infra=True)

        ok, msg = n.start_task()
        res['start_ok'] = bool(ok)
        res['start_message'] = msg
        reached, _ = n.spin_until(lambda: n.state == GROUND_APPROACH, 30.0)
        res['reached_ground_approach'] = bool(reached)
        if not reached:
            res['overall_pass'] = False
            res['error'] = 'task 未进入 GROUND_APPROACH'
            return dump(res, args, infra=True)

        # 冻结 task_manager：其最后一条 /agc/task/state 保持 GROUND_APPROACH；改由本脚本发 goal
        frozen = pgrep('[t]ask_manager')
        for p in frozen:
            try:
                os.kill(p, signal.SIGSTOP)
            except Exception:
                pass
        res['task_manager_frozen_pids'] = frozen
        time.sleep(0.5)
        n.allow_goal = True

        def ensure_moving(dist=25.0, timeout=20.0):
            """若车已停/接近目标，则重设一个车前 dist 的目标并等待行驶。"""
            if n.speed() >= 0.4:
                return True
            n.set_goal(n.x, n.y - dist)
            ok, _ = n.spin_until(lambda: n.speed() > 0.4, timeout)
            return ok

        # 设置远可达目标并等待行驶
        n.set_goal(*GOAL0)
        moving, _ = n.spin_until(lambda: n.speed() > 0.4, 20.0)
        res['pre_moving'] = bool(moving)

        # ---------------- d. 正常不抖动 ----------------
        n.paths.clear()
        n.spin(8.0)
        rate = len(n.paths) / 8.0
        res['tests']['d_no_churn'] = {
            'window_s': 8.0, 'path_publishes': len(n.paths),
            'rate_hz': round(rate, 3), 'control_hz': 20.0,
            'pass': bool(0.15 <= rate <= 3.0),
        }

        # ---------------- a. 地图增障 → 触发重规划 ----------------
        ensure_moving(30.0)
        if not n.paths:
            n.spin(2.5)
        a = {}
        if n.speed() < 0.2 or not n.paths:
            a['pass'] = False
            a['error'] = '车未动 / 无锚定路径，跳过增障注入'
        else:
            prev = n.paths[-1]
            # 沿**当前路径**在车前 ~5 m 处注入（确保落在 costmap 内且位于规划走廊上）
            bx, by = n.x + 5.0 * math.cos(n.yaw), n.y + 5.0 * math.sin(n.yaw)
            pts = n.last_path_xy
            if len(pts) >= 2:
                i0 = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - n.x, pts[i][1] - n.y))
                acc = 0.0
                for i in range(i0, len(pts) - 1):
                    seg = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                    acc += seg
                    if acc >= 5.0:
                        bx, by = pts[i + 1][0], pts[i + 1][1]
                        break
            a['spawn'] = {'name': 'obs_r012', 'x': round(bx, 3), 'y': round(by, 3)}
            log_mark = len(n.logs)
            ok_spawn, spawn_msg = spawn_box('obs_r012', bx, by)
            a['spawn_ok'] = bool(ok_spawn)
            a['spawn_msg'] = spawn_msg
            # 节点自证："重规划触发：地图变化"（costmap 硬障碍签名偏离）
            trig, _ = n.spin_until(
                lambda: any('重规划触发：地图变化' in t for (_, _, t) in n.logs[log_mark:]), 6.0)
            t_log = next((lt for (lt, _, t) in n.logs[log_mark:]
                          if '重规划触发：地图变化' in t), None)
            a['map_trigger_logged'] = bool(trig)
            n.spin(1.5)  # 收集触发后由 plan() 发布的路径（rosout 与 path 到达顺序不定）
            newp = next((p for p in n.paths if t_log and p[0] > t_log), None)
            if newp:
                a['replan_latency_s'] = round(newp[0] - t_log, 3)
                a['replan_prompt'] = bool(newp[0] - t_log <= 1.0)
                a['path_changed'] = bool(newp[1] != prev[1]
                                         or abs(newp[2] - prev[2]) > 0.3
                                         or abs(newp[3] - prev[3]) > 0.3)
                a['pass'] = bool(a['map_trigger_logged'] and a['replan_prompt'] and a['path_changed'])
            else:
                a['stopped_after_trigger'] = bool(abs(n.raw_v) < 0.05)
                a['pass'] = bool(a['map_trigger_logged'] and a['stopped_after_trigger'])
            remove_box('obs_r012')
        res['tests']['a_map_change'] = a

        # ---------------- b. 目标跳变（可达的新点）→ 重规划 ----------------
        ensure_moving(30.0)
        # 跳变目标 = 车当前位置正前 22 m（可达、身份明确的新点）
        gb = (round(n.x, 3), round(n.y - 22.0, 3))
        path_mark = len(n.paths)
        n.set_goal(*gb)
        n.spin_until(
            lambda: any(abs(p[2] - gb[0]) < 1.0 and abs(p[3] - gb[1]) < 1.0
                        for p in n.paths[path_mark:]), 6.0)
        b = {'goal': list(gb)}
        matched = [p for p in n.paths[path_mark:]
                   if abs(p[2] - gb[0]) < 1.0 and abs(p[3] - gb[1]) < 1.0]
        if matched:
            b['new_path_end'] = [round(matched[-1][2], 3), round(matched[-1][3], 3)]
            b['end_near_new_goal'] = True
            b['pass'] = True
        else:
            last = n.paths[-1] if n.paths else None
            b['new_path_end'] = [round(last[2], 3), round(last[3], 3)] if last else None
            b['end_near_new_goal'] = False
            b['pass'] = False
            b['error'] = '目标跳变后未见指向新目标的新路径'
        res['tests']['b_goal_jump'] = b

        # ---------------- c. 不可达目标 → planner 异常 → 失效停车 ----------------
        ensure_moving(30.0)
        v0 = n.speed()
        log_mark = len(n.logs)
        n.set_goal(*GOAL_UNREACH)
        # 等待失效分支出现（planner 抛异常 / 空路径）
        got_fail, _ = n.spin_until(
            lambda: any(('路径失效' in t) for (_, _, t) in n.logs[log_mark:]), 8.0)
        fail_t = next((lt for (lt, _, t) in n.logs[log_mark:] if '路径失效' in t), None)
        stopped, _ = n.spin_until(lambda: abs(n.raw_v) < 0.05 and n.speed() < 0.05, 8.0)
        n.spin(2.5)  # 覆盖至少一个失败重试周期，确认失效后不再产出路径
        paths_after_fail = [p for p in n.paths if fail_t and p[0] > fail_t]
        texts = [t for (_, _, t) in n.logs[log_mark:]]
        branch = 'exception' if any('异常' in t for t in texts) else (
            'empty' if any('为空' in t for t in texts) else 'none')
        c = {
            'goal': list(GOAL_UNREACH), 'pre_speed_mps': round(v0, 3),
            'fail_observed': bool(got_fail), 'stopped': bool(stopped),
            'raw_v_at_stop': round(n.raw_v, 3), 'odom_speed_at_stop': round(n.speed(), 3),
            'new_path_published_after_fail': len(paths_after_fail),
            'fail_branch': branch, 'invalid_failstop_logged': bool(got_fail),
            'pass': bool(got_fail and stopped and abs(n.raw_v) < 0.05 and n.speed() < 0.05
                         and len(paths_after_fail) == 0 and branch in ('exception', 'empty')),
        }
        res['tests']['c_unreachable_failstop'] = c

        res['logs'] = [{'t': round(t, 2), 'level': lv, 'text': tx} for (t, lv, tx) in n.logs]
        res['path_events'] = [{'t': round(p[0], 2), 'n': p[1],
                               'end': [round(p[2], 3), round(p[3], 3)]} for p in n.paths]
        res['overall_pass'] = all(t.get('pass') for t in res['tests'].values())
        return dump(res, args)
    finally:
        for p in frozen:
            try:
                os.kill(p, signal.SIGCONT)
            except Exception:
                pass
        n.destroy_node()
        rclpy.shutdown()


def dump(res, args, infra=False):
    out_dir = os.path.join(args.experiments_root, 'w5-ground-nav-safety',
                           args.run_id, 'metrics')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'r012.json')
    with open(path, 'w') as fh:
        json.dump(res, fh, indent=2, ensure_ascii=False)
    for name, t in res.get('tests', {}).items():
        print('%-26s %s  %s' % (name, 'PASS' if t.get('pass') else 'FAIL',
                                {k: v for k, v in t.items() if k != 'pass'}))
    if 'error' in res:
        print('error:', res['error'])
    print('\n%s -> %s' % ('PASS' if res.get('overall_pass') else 'FAIL', path))
    if infra:
        return 2
    return 0 if res.get('overall_pass') else 1


if __name__ == '__main__':
    sys.exit(main())
