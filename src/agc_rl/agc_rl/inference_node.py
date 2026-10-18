"""agc_rl 地面 RL 推理节点（W5-b · spec R6/R7）。

链路位置：`ground_policy_rl → cmd_vel_raw → collision_monitor → 唯一执行口 → ground_drive_relay → GZ`。
本节点**只**写中间话题 `cmd_vel_raw`（`/agc/ground/cmd_vel_raw`），**SHALL NOT** 直接写契约
唯一执行口（spec R6/R7；契约 [唯一执行口](../../docs/ROS2迁移交接文档.md#execution-port)）。

- 订阅：`/agc/ground/odom`（world 位姿/速度）、`/agc/ground/scan`（2D LaserScan）、`/agc/ground/goal`、
  `/agc/task/state`（仅 `GROUND_APPROACH` 允许驱动）。
- 输出：`TwistStamped`（车体系）至 `cmd_vel_raw`，定频（默认 20 Hz）。
- 失效语义（R-010）：四类必需输入（task 授权 / odom / goal / scan）各持**独立寿命**（`rl.yaml`
  `input_timeout.*` 单一注入），任一过期 / 无效即输出**零速**；`scan` 的「未知」（NaN）与
  「合法无回波」（`+inf`）区分见 `agc_rl.interface`；**从未收到 / 全 NaN** 的扫描**不**默认无遮挡。
- 策略由 config `policy_path` 加载（`agc_rl.policy`）；obs/action 语义由 `agc_rl.interface` 冻结。
- 语言：Python（RL 域，[ADR-0011](../../docs/adr/0011-node-language-policy.md)）。

RL 节点**不写文件**（W5 spec R13）：run 级日志 / 产物归 W6 `agc_run_logger` 与 `experiments/`。
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from ament_index_python.packages import get_package_share_directory

from agc_msgs.msg import AgcTaskState
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from agc_rl import interface
from agc_rl.policy import load_policy


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GroundPolicyRL(Node):
    def __init__(self):
        super().__init__('ground_policy_rl')

        self.base_frame = self.declare_parameter('base_frame', 'car/base_link').value
        self.rate_hz = self.declare_parameter('rate_hz', 20.0).value

        self.odom_topic = self.declare_parameter('odom_topic', '/agc/ground/odom').value
        self.scan_topic = self.declare_parameter('scan_topic', '/agc/ground/scan').value
        self.goal_topic = self.declare_parameter('goal_topic', '/agc/ground/goal').value
        self.state_topic = self.declare_parameter('state_topic', '/agc/task/state').value
        self.cmd_raw_topic = self.declare_parameter(
            'cmd_raw_topic', '/agc/ground/cmd_vel_raw').value

        self.num_sectors = int(self.declare_parameter('num_sectors', 12).value)
        self.lidar_max_range = self.declare_parameter('lidar_max_range', 10.0).value
        self.pose_norm_m = self.declare_parameter('pose_norm_m', 10.0).value
        self.track_norm_m = self.declare_parameter('track_norm_m', 5.0).value

        # 策略加载路径（config/*.yaml 单一注入，spec R6）：config 给文件名，节点在
        # `agc_rl/policies/` 下解析为绝对路径；空则用内置默认确定性策略。
        self.policy_file = self.declare_parameter('policy_file', '').value

        # action → cmd_vel 映射参数（`interface.action_to_twist`）
        self.control = {
            'reference_speed': self.declare_parameter('control.reference_speed', 1.0).value,
            'fast_speed': self.declare_parameter('control.fast_speed', 1.3).value,
            'min_offset': self.declare_parameter('control.min_offset', 0.3).value,
            'max_offset': self.declare_parameter('control.max_offset', 1.0).value,
            'k_heading': self.declare_parameter('control.k_heading', 1.5).value,
            'k_lateral': self.declare_parameter('control.k_lateral', 0.8).value,
            'max_yaw_rate': self.declare_parameter('control.max_yaw_rate', 1.0).value,
            # 净空参考值（**米**，与车前净空同量纲）；R-011 量纲一致化，动作强度与 lidar_max_range 解耦。
            'clearance_ref_m': self.declare_parameter('control.clearance_ref_m', 1.0).value,
        }

        # R-010：四类必需输入的**独立寿命**（秒，由 config 单一注入）。到达时刻按
        # **单调时钟**记（与 R-009 同口径：不受 ROS 时钟冻结 / 回拨影响）。
        self.timeouts = {
            'task_auth': self.declare_parameter('input_timeout.task_auth_s', 2.0).value,
            'odom': self.declare_parameter('input_timeout.odom_s', 0.5).value,
            'goal': self.declare_parameter('input_timeout.goal_s', 2.0).value,
            'scan': self.declare_parameter('input_timeout.scan_s', 0.5).value,
        }

        policy_path = ''
        if self.policy_file:
            policy_path = os.path.join(
                get_package_share_directory('agc_rl'), 'policies', self.policy_file)
        self.policy = load_policy(policy_path)
        self.get_logger().info(
            "agc_rl 推理节点：policy=%s path='%s' obs_dim=%d n_actions=%d"
            % (self.policy.name, policy_path,
               interface.obs_dim(self.num_sectors), interface.N_ACTIONS))

        # 传感器数据用 SENSOR_DATA（best-effort）QoS，对齐 Nav2（ADR-0008 吸收点）
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(TwistStamped, self.cmd_raw_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, sensor_qos)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 20)
        self.create_subscription(AgcTaskState, self.state_topic, self._on_state, 20)

        self.x = self.y = self.yaw = 0.0
        self.goal_x = self.goal_y = 0.0
        self.state = AgcTaskState.IDLE
        self.sectors = [1.0] * self.num_sectors
        self.scan_valid = False
        self.last_action = 0
        # 各输入的**到达时刻**（单调钟）；`None` = 从未收到（R-010 独立寿命）。
        self._t_odom = None
        self._t_goal = None
        self._t_state = None
        self._t_scan = None

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1e-3), self._tick)

    def _on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._t_odom = time.monotonic()

    def _on_scan(self, msg):
        ranges = list(msg.ranges)
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(ranges))]
        self.sectors = interface.sectors_from_angles(
            angles, ranges, self.num_sectors, self.lidar_max_range)
        # 全 NaN / 非法读数 → 该扫描**无效**（不得据以判「无遮挡」）
        self.scan_valid = interface.scan_has_valid_reading(ranges)
        self._t_scan = time.monotonic()

    def _on_goal(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self._t_goal = time.monotonic()

    def _on_state(self, msg):
        self.state = msg.state
        self._t_state = time.monotonic()

    def _publish(self, v, w):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)
        self.pub.publish(cmd)

    def _fresh(self, t_arrival, limit_s):
        """到达时刻（单调钟）是否在寿命内；`None`（从未收到）恒为 False（R-010）。"""
        return t_arrival is not None and (time.monotonic() - t_arrival) <= limit_s

    def _authorized(self):
        """任务授权有效：`GROUND_APPROACH` 且状态消息在授权寿命内。"""
        return (self.state == AgcTaskState.GROUND_APPROACH
                and self._fresh(self._t_state, self.timeouts['task_auth']))

    def _stop(self):
        self.last_action = 0
        self._publish(0.0, 0.0)

    def _tick(self):
        # R-010：task 授权 / odom / goal / scan 各自**独立寿命与无效语义**；
        # 任一必需输入过期 / 无效 → 输出零速（不延续旧状态）。
        if not self._authorized():
            self._stop()
            return
        if not self._fresh(self._t_odom, self.timeouts['odom']):
            self._stop()
            return
        if not self._fresh(self._t_goal, self.timeouts['goal']):
            self._stop()
            return
        if not (self.scan_valid and self._fresh(self._t_scan, self.timeouts['scan'])):
            self._stop()
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)
        heading_err = _wrap_pi(math.atan2(dy, dx) - self.yaw)

        obs = interface.build_observation(
            self.sectors, self.x, self.y, dist, self.last_action,
            self.num_sectors, self.pose_norm_m, self.track_norm_m)
        action = self.policy.act(obs, self.num_sectors)
        self.last_action = action

        # 车前净空（米）：扇区为 range / lidar_max_range 的归一值，乘量程还原为米（R-011 量纲一致）。
        forward_clearance_m = interface.forward_clearance(
            self.sectors, self.num_sectors, 1) * self.lidar_max_range
        v, w = interface.action_to_twist(
            action, heading_err, forward_clearance_m, self.control)
        self._publish(v, w)


def main(args=None):
    rclpy.init(args=args)
    node = GroundPolicyRL()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
