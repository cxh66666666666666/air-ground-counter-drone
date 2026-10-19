"""Gymnasium 兼容的环境适配层（W7-a · spec R1）。

**无 gymnasium 依赖**：实现 Gymnasium v0.26+ 约定的最小 API（`reset` / `step` / `close`），
`reset() -> (obs, info)`；`step(action) -> (obs, reward, terminated, truncated, info)`。

`terminated / truncated / info` 由**单一判定来源**生成 —— `agc_rl.termination`
（**与评估 adapter `scripts/episode_runner.py` 共用**，spec R1 判据：评估摘要与训练 adapter
的 end / reason 一致）：

  * **任务终态**（`/agc/task/state` ∈ `TERMINALS`）→ `terminated=True`（**含超时失败**，契约 C4）；
  * **外部截断**（墙钟看门狗 / 训练 horizon / 无终态）→ `truncated=True`（须 bootstrap）；
  * `INFRA_FAILURE` → `terminated=True` 且 `info.classification='infra'`（**不吞**入成功率，ADR-0046）。

架构（与 [ADR-0069](../../docs/adr/0069-w7-entry-and-exit-definition.md) 决定一致）：
  * 环境 = **外部命令源**：本适配层独占发布 `/agc/ground/cmd_vel_raw`；由 `w6.launch.py`
    `command_source:=train` 保证图内**不**另起 rl / baseline 发布者（安全层仍是唯一 shield）。
  * 链路与 `rl` 同构：`适配层 → cmd_vel_raw → collision_monitor → cmd_vel → 中继 → GZ`。
  * 步进模型 = **异步实时**（复用现栈，RTF≈1）：`step` 边界 = 仿真时钟推进 `control_dt_s`；
    **不承诺** bitwise 物理一致（R3 边界，成文）。
  * 仿真栈须**由 runner 预先拉起**（spawn 由 `scenario_reset` 按注入的 spawn seed 生成）；
    `reset` 只做任务起始事务（`/agc/task/start` → 相位 t=0 → 新鲜门禁），**不**重建世界。

**奖励**：属 W7-b（spec 明确本段不产出奖励）；本层返回占位 `0.0` 并在 `info.reward_defined=False`
如实标注 —— **不**以占位值冒充奖励设计。
"""

import math
import os
import threading
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import SingleThreadedExecutor

from agc_msgs.msg import AgcTaskState
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from agc_rl import interface
from agc_rl import obs_spec
from agc_rl import seeding
from agc_rl import termination

# action → cmd_vel 映射默认参数（与 `agc_rl/config/rl.yaml` 的 control 段一致，[ADR-0026]）。
DEFAULT_CONTROL = {
    'reference_speed': 1.0, 'fast_speed': 1.3, 'min_offset': 0.3, 'max_offset': 1.0,
    'k_heading': 1.5, 'k_lateral': 0.8, 'max_yaw_rate': 1.0, 'clearance_ref_m': 1.0,
}

# 内置回落默认（`config/*.yaml` 不可读时用）——正常运行以 share 的 `rl.yaml` + `w7.yaml`
# 为**单一来源**（spec R6；契约 #params）。obs / 控制 / 寿命参数取自 `rl.yaml`（与 agent 共用），
# W7 平台参数取自 `w7.yaml`。
_ENV_DEFAULTS = {
    'base_frame': 'car/base_link', 'rate_hz': 20.0,
    'odom_topic': '/agc/ground/odom', 'scan_topic': '/agc/ground/scan',
    'goal_topic': '/agc/ground/goal', 'state_topic': '/agc/task/state',
    'clock_topic': '/clock', 'gt_car_topic': '/agc/gt/agc_car',
    'cmd_raw_topic': '/agc/ground/cmd_vel_raw', 'start_service': '/agc/task/start',
    'num_sectors': 12, 'lidar_max_range': 10.0, 'pose_norm_m': 10.0, 'track_norm_m': 5.0,
    'input_timeout': {'odom_s': 0.5, 'scan_s': 0.5, 'goal_s': 2.0, 'task_auth_s': 2.0},
}
_W7_DEFAULTS = {'obs_version': 1, 'control_dt_s': 0.1, 'max_steps': 0,
                'wall_watchdog_s': 150.0, 'obs_noise_sigma': 0.0, 'spawn_randomize': False}


def _load_share_yaml():
    """读 `agc_rl` share 的 `config/rl.yaml` + `config/w7.yaml`（**单一来源**，spec R6）。

    返回 `(rl_ros_parameters | None, w7 | None)`；未安装 / 缺失 → `(None, None)`（回落内置默认）。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('agc_rl')
        with open(os.path.join(share, 'config', 'rl.yaml'), encoding='utf-8') as f:
            rl = yaml.safe_load(f) or {}
        with open(os.path.join(share, 'config', 'w7.yaml'), encoding='utf-8') as f:
            w7 = yaml.safe_load(f) or {}
        return (rl.get('ground_policy_rl', {}).get('ros__parameters', {}) or {},
                w7.get('w7', {}) or {})
    except Exception:  # noqa: BLE001
        return None, None


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class _EnvNode(Node):
    """ROS 侧：订阅 obs / 任务态 / 时钟；定频发布 `cmd_vel_raw`（默认零速，保持链路活跃）。"""

    def __init__(self, cfg):
        super().__init__('w7_env_adapter')
        self.cfg = cfg

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(TwistStamped, cfg['cmd_raw_topic'], 10)
        self.create_subscription(Odometry, cfg['odom_topic'], self._on_odom, 20)
        self.create_subscription(LaserScan, cfg['scan_topic'], self._on_scan, sensor_qos)
        self.create_subscription(PoseStamped, cfg['goal_topic'], self._on_goal, 20)
        self.create_subscription(AgcTaskState, cfg['state_topic'], self._on_state, 20)
        self.create_subscription(Clock, cfg['clock_topic'], self._on_clock, 10)
        self.gt_sub = None
        if cfg.get('gt_car_topic'):
            self.gt_sub = self.create_subscription(
                PoseStamped, cfg['gt_car_topic'], self._on_gt, 10)
        self.start_cli = self.create_client(Trigger, cfg['start_service'])

        self.x = self.y = self.yaw = 0.0
        self.goal_x = self.goal_y = 0.0
        self.state = AgcTaskState.IDLE
        self.reason = ''
        self.sectors = [1.0] * cfg['num_sectors']
        self.scan_valid = False
        self.sim_s = None
        self.gt = None
        self._cmd = (0.0, 0.0)
        self._t_odom = None
        self._t_scan = None
        self._t_goal = None
        self._t_state = None

        self.create_timer(1.0 / max(cfg['rate_hz'], 1e-3), self._on_timer)

    def _on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = _yaw_from_quat(m.pose.pose.orientation)
        self._t_odom = time.monotonic()

    def _on_scan(self, m):
        ranges = list(m.ranges)
        angles = [m.angle_min + i * m.angle_increment for i in range(len(ranges))]
        self.sectors = interface.sectors_from_angles(
            angles, ranges, self.cfg['num_sectors'], self.cfg['lidar_max_range'])
        self.scan_valid = interface.scan_has_valid_reading(ranges)
        self._t_scan = time.monotonic()

    def _on_goal(self, m):
        self.goal_x = m.pose.position.x
        self.goal_y = m.pose.position.y
        self._t_goal = time.monotonic()

    def _on_state(self, m):
        self.state = m.state
        self.reason = m.reason_code
        self._t_state = time.monotonic()

    def _on_clock(self, m):
        self.sim_s = m.clock.sec + m.clock.nanosec * 1e-9

    def _on_gt(self, m):
        self.gt = (m.pose.position.x, m.pose.position.y, m.pose.position.z)

    def _on_timer(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.cfg['base_frame']
        cmd.twist.linear.x = float(self._cmd[0])
        cmd.twist.angular.z = float(self._cmd[1])
        self.pub.publish(cmd)

    def set_cmd(self, v, w):
        self._cmd = (float(v), float(w))

    def fresh(self, t_arrival, limit_s):
        return t_arrival is not None and (time.monotonic() - t_arrival) <= limit_s


class GroundAgcEnv:
    """地面 RL 环境适配层（Gymnasium 兼容；无 gymnasium 依赖）。"""

    metadata = {'render_modes': [], 'spec_version': 1}

    def __init__(self, scene, seed_plan=None, obs_version=None, **overrides):
        # 单一来源：`rl.yaml`（obs/控制/寿命，与 agent 共用）+ `w7.yaml`（W7 平台参数）。
        rl_p, w7_p = _load_share_yaml()
        cfg = dict(_ENV_DEFAULTS)
        w7cfg = dict(_W7_DEFAULTS)
        if isinstance(rl_p, dict):
            for k in ('base_frame', 'rate_hz', 'odom_topic', 'scan_topic', 'goal_topic',
                      'state_topic', 'cmd_raw_topic', 'num_sectors', 'lidar_max_range',
                      'pose_norm_m', 'track_norm_m'):
                if k in rl_p:
                    cfg[k] = rl_p[k]
            if isinstance(rl_p.get('input_timeout'), dict):
                cfg['input_timeout'] = dict(rl_p['input_timeout'])
        if isinstance(w7_p, dict):
            w7cfg.update({k: v for k, v in w7_p.items() if k in w7cfg})
        cfg.update(w7cfg)
        cfg['scene'] = str(scene)
        cfg.update(overrides)
        cfg['control'] = dict(
            DEFAULT_CONTROL, **(rl_p.get('control') if isinstance(rl_p, dict) else {}) or {},
            **(overrides.get('control') or {}))
        self.cfg = cfg
        self.obs_version = int(w7cfg['obs_version'] if obs_version is None else obs_version)
        self.scene = cfg['scene']
        self.action_space = obs_spec.ActionSpaceDescriptor()
        self.observation_space = obs_spec.get_obs_spec(
            self.obs_version, cfg['num_sectors'],
            lidar_max_range=cfg['lidar_max_range'], pose_norm_m=cfg['pose_norm_m'],
            track_norm_m=cfg['track_norm_m'])

        self._seed_plan = seed_plan or seeding.make_seed_plan(0, self.scene)
        self._noise_rng = self._seed_plan.rng('sensor')
        self._steps = 0
        self._ep_idle = None

        # 若调用方未初始化 rclpy（如被 runner 独立调用），本层自持初始化（close 时对应 shutdown）。
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()

        self._node = _EnvNode(cfg)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin_thread.start()

    # ---------------- 内部 ----------------
    def _spin_until(self, pred, timeout, period=0.01):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            time.sleep(period)
            if pred():
                return True
        return False

    def _fresh_inputs(self, n):
        to = self.cfg['input_timeout']
        return (n.state == AgcTaskState.GROUND_APPROACH
                and n.fresh(n._t_state, to['task_auth_s'])
                and n.fresh(n._t_odom, to['odom_s'])
                and n.fresh(n._t_goal, to['goal_s'])
                and n.scan_valid and n.fresh(n._t_scan, to['scan_s']))

    def _sim_now(self):
        return self._node.sim_s

    def _build_obs(self, n):
        dx = n.goal_x - n.x
        dy = n.goal_y - n.y
        dist = math.hypot(dx, dy)
        obs = obs_spec.build_obs_vector(
            self.obs_version, n.sectors, n.x, n.y, dist, self._last_action,
            self.cfg['num_sectors'], self.cfg['pose_norm_m'], self.cfg['track_norm_m'])
        sigma = float(self.cfg['obs_noise_sigma'])
        if sigma > 0.0:
            obs = [v + self._noise_rng.gauss(0.0, sigma) for v in obs]
        return obs, dist

    def _info(self, n, extra=None):
        info = {
            'scene': self.scene, 'obs_version': self.obs_version,
            'seed_plan': self._seed_plan.to_dict(),
            'reward_defined': False,
            'task_state': n.state, 'reason_code': n.reason,
            'sim_s': n.sim_s, 'steps': self._steps,
            'spawn': n.gt,
        }
        if extra:
            info.update(extra)
        return info

    # ---------------- Gymnasium API ----------------
    def reset(self, *, seed=None, options=None):
        """任务起始事务：`/agc/task/start` → 相位 t=0 → 新鲜门禁（复现 D3 语义）。"""
        options = options or {}
        if seed is not None:
            self._seed_plan = seeding.make_seed_plan(
                int(seed), self.scene, options.get('episode_index', 0),
                options.get('attempt_index', 0))
        self._noise_rng = self._seed_plan.rng('sensor')
        self._steps = 0
        self._last_action = 0
        n = self._node
        n.set_cmd(0.0, 0.0)

        # 本机 DDS：graph 查询不可靠 + 首个服务调用常超时 → 先等就绪、再长超时**重试若干次**。
        cli = n.start_cli
        ready = self._spin_until(lambda: cli.service_is_ready(), float(options.get('service_wait_s', 40.0)))
        ok, msg = False, 'timeout'
        for _ in range(4):
            fut = cli.call_async(Trigger.Request())
            if self._spin_until(lambda: fut.done(), 20.0) and fut.result() is not None:
                ok = bool(fut.result().success)
                msg = fut.result().message
                if ok:
                    break
            self._spin_until(lambda: False, 0.5)

        # 新鲜门禁：等四类输入就绪（task 授权 / odom / goal / scan）
        self._spin_until(lambda: self._fresh_inputs(n), float(self.cfg['wall_watchdog_s']))
        fresh = self._fresh_inputs(n)
        self._t_reset_sim = n.sim_s
        self._ep_wall_start = time.monotonic()   # episode 墙钟起点（看门狗口径，ADR-0039）
        obs, dist = self._build_obs(n)
        info = self._info(n, {'started': ok, 'start_message': msg, 'service_ready': ready,
                              'fresh_gate': fresh, 'reset': True, 'dist_to_goal': dist})
        return (obs, info) if ok and fresh else (obs, info)

    def step(self, action):
        """发布动作 → 推进 `control_dt_s`（仿真时钟）→ 取下一观测与终止判定。

        `terminated` = 任务终态（含超时失败，C4）；`truncated` = **外部截断**（墙钟看门狗 / 训练
        horizon / 输入失效）；回合**进行中**时两者皆 False，`info.outcome=None`（不误判为结束）。
        """
        n = self._node
        self._steps += 1
        dt_sim = float(self.cfg['control_dt_s'])
        wd = float(self.cfg['wall_watchdog_s'])

        if not self._fresh_inputs(n):
            # 输入失效 → 零速（R-010 语义），并如实标注（外部截断，非任务终态）
            n.set_cmd(0.0, 0.0)
            obs, dist = self._build_obs(n)
            cls = {'outcome_class': 'input_invalid', 'classification': 'infra',
                   'budget_kind': None, 'counted_as_success': False,
                   'is_task_terminal': False, 'is_truncation': True, 'reason': 'input_invalid'}
            info = self._info(n, {'outcome': cls, 'dist_to_goal': dist, 'input_valid': False})
            info['terminal_observation'] = obs
            return obs, 0.0, False, True, info

        heading_err = _wrap_pi(math.atan2(n.goal_y - n.y, n.goal_x - n.x) - n.yaw)
        forward_clearance_m = interface.forward_clearance(
            n.sectors, self.cfg['num_sectors'], 1) * self.cfg['lidar_max_range']
        v, w = interface.action_to_twist(action, heading_err, forward_clearance_m, self.cfg['control'])
        n.set_cmd(v, w)
        self._last_action = int(action)

        t0 = n.sim_s
        if t0 is None:
            # 仿真时钟缺席 → 以墙钟近似推进（如实标注 time_source）
            self._spin_until(lambda: False, dt_sim)
            time_source = 'wall_fallback'
            sim_elapsed = None
        else:
            self._spin_until(lambda: (n.sim_s is not None and n.sim_s - t0 >= dt_sim)
                             or (n.state in termination.TERMINALS),
                             wd)
            time_source = 'ros_sim'
            sim_elapsed = (None if n.sim_s is None else (n.sim_s - t0))

        obs, dist = self._build_obs(n)
        elapsed_wall = time.monotonic() - self._ep_wall_start
        state_name = termination.TERMINAL_NAMES.get(n.state, 'ACTIVE')
        terminated = n.state in termination.TERMINALS
        truncated = False
        cls = None                                   # 回合进行中：无结束判定
        if terminated:
            cls = termination.classify_episode_outcome(
                state_name, n.reason, None, None, elapsed_wall, wd)
        elif elapsed_wall >= wd:
            truncated = True                         # 外部截断（墙钟看门狗）
            cls = termination.classify_episode_outcome('ACTIVE', n.reason, None, None, elapsed_wall, wd)
        if (not terminated) and self.cfg['max_steps'] and self._steps >= int(self.cfg['max_steps']):
            truncated = True                         # 外部截断（训练 horizon）
            cls = {'outcome_class': 'training_horizon', 'classification': 'infra',
                   'budget_kind': 'episode_wall_watchdog', 'counted_as_success': False,
                   'is_task_terminal': False, 'is_truncation': True, 'reason': 'training_horizon'}
        info = self._info(n, {'outcome': cls, 'dist_to_goal': dist, 'input_valid': True,
                              'time_source': time_source, 'step_sim_elapsed': sim_elapsed,
                              'cmd': [v, w], 'elapsed_wall_s': elapsed_wall})
        info['terminal_observation'] = obs
        return obs, 0.0, terminated, truncated, info

    def close(self):
        try:
            self._exec.shutdown()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        self._spin_thread.join(timeout=2.0)
        if getattr(self, '_owns_rclpy', False):
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass

    @property
    def seed_plan(self):
        return self._seed_plan

    def robot_pose(self):
        """当前车体位姿（world ENU）——供 runner 记录轨迹 / 分叉边界。"""
        n = self._node
        return (n.x, n.y, n.yaw)

    def noise_sequence(self, n=20):
        """外生 `sensor` 流的前 n 个抽样（供 R3「噪声」证据；确定性可复现）。"""
        rng = self._seed_plan.rng('sensor')
        return [rng.gauss(0.0, 1.0) for _ in range(n)]


def _wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a
