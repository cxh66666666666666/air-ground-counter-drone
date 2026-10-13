#!/usr/bin/env python3
"""w3_target_driver —— W3 临时目标驱动脚本（spec Task 9.2）

按速度指令驱动 `intruder_uav`（模型挂 GZ VelocityControl，经 ros_gz 桥接
`/agc/sim/target_cmd_vel` → `/model/intruder_uav/cmd_vel`）。正式 driver / 干扰注入属 W6。

用法：
  python3 scripts/w3_target_driver.py --mode circle --speed 1.0 --radius 6.0
  python3 scripts/w3_target_driver.py --mode line   --speed 1.0 --heading-deg 138 --duration 35
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TargetDriver(Node):
    def __init__(self, topic: str, mode: str, speed: float, radius: float, rate: float,
                 heading_deg: float = 0.0):
        super().__init__('w3_target_driver')
        self.pub_ = self.create_publisher(Twist, topic, 10)
        self.mode_ = mode
        self.speed_ = speed
        self.radius_ = max(radius, 1e-3)
        self.heading_ = math.radians(heading_deg)
        self.dt_ = 1.0 / rate
        self.t0_ = time.monotonic()
        self.timer_ = self.create_timer(self.dt_, self.tick)

    def tick(self) -> None:
        t = time.monotonic() - self.t0_
        msg = Twist()
        if self.mode_ == 'line':
            msg.linear.x = self.speed_ * math.cos(self.heading_)
            msg.linear.y = self.speed_ * math.sin(self.heading_)
        else:  # circle：速度方向以 ω = v / r 匀速旋转 → 半径 r、线速度 v 的圆周
            omega = self.speed_ / self.radius_
            msg.linear.x = self.speed_ * math.cos(omega * t)
            msg.linear.y = self.speed_ * math.sin(omega * t)
        self.pub_.publish(msg)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/agc/sim/target_cmd_vel')
    ap.add_argument('--mode', choices=['line', 'circle'], default='circle')
    ap.add_argument('--speed', type=float, default=1.0, help='目标巡航速度 m/s（ADR-0019：1.0）')
    ap.add_argument('--radius', type=float, default=6.0, help='circle 模式半径 m')
    ap.add_argument('--heading-deg', type=float, default=0.0,
                    help='line 模式航向（world ENU，度；0 = +x）')
    ap.add_argument('--rate', type=float, default=20.0, help='指令频率 Hz')
    ap.add_argument('--duration', type=float, default=0.0, help='>0 则到时退出，0 为常发')
    args = ap.parse_args()

    rclpy.init(args=[])
    node = TargetDriver(args.topic, args.mode, args.speed, args.radius, args.rate,
                        args.heading_deg)
    try:
        if args.duration > 0.0:
            end = time.monotonic() + args.duration
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
