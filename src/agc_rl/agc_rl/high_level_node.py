#!/usr/bin/env python3
"""high_level_node —— W7-b P2 臂 B：**高层（拦截点）+ 成熟低层**（[ADR-0070](b)(f)）。

角色（P2「RL 职责分层」的**臂 B**）：本节点只出**高层决策**——把目标预测拦截点作为**目标点**
发布到低层（`ground_baseline` = Navfn 规划 + 自研 LQR，见 [R-012](docs/reviews/.../10-风险.md#r-012)，
**准确命名**）；**不**直接写 `cmd_vel`、**不**写唯一执行口——低层与安全层仍是唯一运动通道。

**诚实标注**：本段的高层策略为**规则实现**（目标线性外推 + 提前量），**非**已训练 RL；
「臂 B」考察的是**职责分层**（高层决策 + 成熟低层），RL 训练属后续（[F-02]）。
P2 v1 以规则高层承载的裁定见 [ADR-0082](docs/adr/0082-w7b-p2-arm-b-rule-high-level-first.md)。

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
        # ADR-0083(g)：感知跳变抑制——目标位置/速度一阶低通 + 前向位移限幅。
        #   不滤波时 twist 的单帧差分尖峰 × lead_s 会把目标点甩出数米，令低层每 0.25 s 重规划。
        self.target_tau_s = float(self.declare_parameter('target_tau_s', 0.5).value)
        self.target_gap_reset_s = float(self.declare_parameter('target_gap_reset_s', 1.0).value)
        self.max_lead_m = float(self.declare_parameter('max_lead_m', 1.0).value)

        self.pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)
        self.create_subscription(TargetTrackArray, self.targets_topic, self._on_target, 20)
        self.create_subscription(AgcTaskState, self.state_topic, self._on_state, 20)

        self.x = self.y = 0.0
        self.tgt = None
        self.tgt_vel = (0.0, 0.0)
        self.tgt_rx = None
        self._filt_rx = None   # 上一帧观测的单调时刻（滤波 dt）
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
        # 取置信度最高的航迹（避免 front() 在噪声航迹间跳变）。
        t = max(m.tracks, key=lambda tr: float(tr.confidence))
        mx, my = t.pose.pose.position.x, t.pose.pose.position.y
        mvx, mvy = t.twist.twist.linear.x, t.twist.twist.linear.y
        now = time.monotonic()
        # 首帧 / 目标中断超过 gap → 直接接管，避免把旧滤波状态拖进来。
        if self.tgt is None or self._filt_rx is None or \
                (now - self._filt_rx) > self.target_gap_reset_s:
            self.tgt = (mx, my)
            self.tgt_vel = (mvx, mvy)
        else:
            dt = max(1e-3, now - self._filt_rx)
            alpha = 1.0 - math.exp(-dt / max(1e-3, self.target_tau_s))
            self.tgt = (self.tgt[0] + alpha * (mx - self.tgt[0]),
                        self.tgt[1] + alpha * (my - self.tgt[1]))
            self.tgt_vel = (self.tgt_vel[0] + alpha * (mvx - self.tgt_vel[0]),
                            self.tgt_vel[1] + alpha * (mvy - self.tgt_vel[1]))
        self._filt_rx = now
        self.conf = float(t.confidence)
        self.tgt_rx = now

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
        # 前向位移限幅：感知速度尖峰不得把目标点甩出 max_lead_m。
        lx = self.tgt_vel[0] * self.lead_s
        ly = self.tgt_vel[1] * self.lead_s
        ln = math.hypot(lx, ly)
        if ln > self.max_lead_m and ln > 1e-9:
            k = self.max_lead_m / ln
            lx, ly = lx * k, ly * k
        gx = self.tgt[0] + lx
        gy = self.tgt[1] + ly
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
