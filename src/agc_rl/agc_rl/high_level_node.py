#!/usr/bin/env python3
"""high_level_node —— W7-b P2 臂 B：**高层（拦截点）+ 成熟低层**（[ADR-0070](b)(f)）。

角色（P2「RL 职责分层」的**臂 B**）：本节点只出**高层决策**——把目标预测拦截点作为**目标点**
发布到低层（`ground_baseline` = Navfn 规划 + 自研 LQR，见 [R-012](docs/reviews/.../10-风险.md#r-012)，
**准确命名**）；**不**直接写 `cmd_vel`、**不**写唯一执行口——低层与安全层仍是唯一运动通道。

**诚实标注**：本段的高层策略为**规则实现**（目标线性外推 + 提前量），**非**已训练 RL；
「臂 B」考察的是**职责分层**（高层决策 + 成熟低层），RL 训练属后续（[F-02]）。

I/O：订 `/agc/ground/odom`、`/agc/targets`、`/agc/task/state`；发 `PoseStamped`（world）至
`/agc/ground/goal_hl`（由 `ground_baseline` 的 `goal_topic` 消费；**非**命令者）。
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from agc_msgs.msg import AgcTaskState, TargetTrackArray


class HighLevelNode(Node):
    def __init__(self):
        super().__init__('agc_rl_high_level')
        self.goal_topic = self.declare_parameter('goal_topic', '/agc/ground/goal_hl').value
        self.odom_topic = self.declare_parameter('odom_topic', '/agc/ground/odom').value
        self.targets_topic = self.declare_parameter('targets_topic', '/agc/targets').value
        self.state_topic = self.declare_parameter('state_topic', '/agc/task/state').value
        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0).value)
        # 提前量（s）：拦截点 = 目标位姿 + 目标速度 × lead_s（规则线性外推，非学习策略）
        self.lead_s = float(self.declare_parameter('lead_s', 1.0).value)
        self.target_max_age_s = float(self.declare_parameter('target_max_age_s', 0.5).value)
        self.confidence_min = float(self.declare_parameter('confidence_min', 0.5).value)

        self.pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)
        self.create_subscription(TargetTrackArray, self.targets_topic, self._on_target, 20)
        self.create_subscription(AgcTaskState, self.state_topic, self._on_state, 20)

        self.x = self.y = 0.0
        self.tgt = None
        self.tgt_vel = (0.0, 0.0)
        self.tgt_rx = None
        self.conf = 0.0
        self.state = 0
        self.create_timer(1.0 / max(self.rate_hz, 1e-3), self._tick)
        self.get_logger().info(
            'agc_rl 高层（规则拦截点）就绪：goal_topic=%s lead_s=%.2f（低层 = Navfn + 自研 LQR）'
            % (self.goal_topic, self.lead_s))

    def _on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y

    def _on_target(self, m):
        if not m.tracks:
            return
        t = m.tracks[0]
        self.tgt = (t.pose.pose.position.x, t.pose.pose.position.y)
        self.tgt_vel = (t.twist.twist.linear.x, t.twist.twist.linear.y)
        self.conf = float(t.confidence)
        self.tgt_rx = time.monotonic()

    def _on_state(self, m):
        self.state = m.state

    def _tick(self):
        if self.tgt is None:
            return
        if self.tgt_rx is None or (time.monotonic() - self.tgt_rx) > self.target_max_age_s:
            return
        if self.conf < self.confidence_min:
            return
        # 高层决策：目标预测拦截点（规则线性外推）——非命令者，仅出目标点
        gx = self.tgt[0] + self.tgt_vel[0] * self.lead_s
        gy = self.tgt[1] + self.tgt_vel[1] * self.lead_s
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HighLevelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
