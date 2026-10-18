#!/usr/bin/env python3
"""intruder_driver —— 入侵者 PX4 脚本化驱动（W6 前置 P / spec R4；R-016 episode 起点协议）。

入侵者为**第二个 PX4 SITL 实例**（attach 到模型 `intruder_uav`）。本节点经 uXRCE / `px4_msgs`
以 Offboard 驱动其运动：`static`（悬停）/ `linear`（匀速直线）/ `maneuver`（圆周机动）。
仅命令**入侵者**，不触碰己方机 / 车（契约 [唯一执行口]）。

话题：`/<ns>/fmu/{in,out}/*`（ns 由 launch 单一设置，默认 `px4_1`）。
NED 约定：PX4 本地系为 NED（z 向下），故高度目标用 `-alt_m`（相对 boot 原点）。

R-016 episode 起点协议（ADR-0031）：
  * **预热独立于 episode 计时**：节点启动即记 `warmup_t0_`；`tw = now - warmup_t0_`，`tw > 1 s` 才请求
    ARM，已解锁且非 OFFBOARD(`14`) 则请求 `SET_NAV_STATE=OFFBOARD`（按 `arm_retry_s` 重试）。
    **预热时间不进入轨迹参数 t**。
  * **episode epoch**：订阅 `<epoch_topic>`（`std_msgs/Int32`，depth1 + RELIABLE + TRANSIENT_LOCAL）。
    **epoch 值变化 = 新 episode**：重锚 `origins_`（下条 odom）、清 `t0_`、清干扰态、解终止、置 `episode_armed_`。
  * **起点协议**：预热 → 进入起始位姿（`_target(0)`）→ 确认稳定（`dist(pos, start) <= start_tol_m`
    且 `speed <= start_vel_eps_mps` 持续 `start_dwell_s`）→ 记 `t0_`（episode t=0）。
    **未进入 episode（`t0_ is None`）setpoint 恒定 = 起始位姿（悬停保持，不推进）**。
  * **切入 / 运行区边界**：`maneuver` 的 t=0 setpoint 落在圆周定义点 `(ox+r, oy)`（不从原点起跳）；
    setpoint 水平离原点超过 `run_region_radius_m`（>0 生效）→ 夹持到边界并置 `terminated_`（越界终止，不外推）。
  * **CLEAR 重基准**：干扰解除时把暂停时长加回 `t0_`，使参考从干扰发生时刻**连续**续上（不跳变）。

R-017（ADR-0032）：干扰授权收敛到**单一裁决者** `jammer`。本节点**只消费**其产出的**已校验授权事件**
（`/agc/task/jam_state`，`std_msgs/String`）；**不直订**任何原始干扰命令话题（原直订路径已移除）。
`jammed_`（→ 停发 Offboard 流）**仅由该已校验事件驱动**。R-019（ADR-0034）：停发 Offboard 流触发
PX4 的 **offboard-loss** 行为（由运行时锁定的 `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` 决定），**非**
data-link-loss（`NAV_DLL_ACT`）；二者为不同机制，本节点不主张「数据链被切断」。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)
from std_msgs.msg import Int32, String

OFFBOARD_NAV_STATE = VehicleStatus.NAVIGATION_STATE_OFFBOARD  # 14


def _qos():
    q = QoSProfile(depth=5)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    return q


def _epoch_qos():
    """R-016：epoch 订阅 QoS —— depth=1 + RELIABLE + TRANSIENT_LOCAL（对齐 scenario_reset 发布端）。"""
    q = QoSProfile(depth=1)
    q.reliability = ReliabilityPolicy.RELIABLE
    q.history = HistoryPolicy.KEEP_LAST
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return q


class IntruderDriver(Node):
    def __init__(self):
        super().__init__('intruder_driver')
        self.ns_ = self.declare_parameter('px4_ns', 'px4_1').value
        self.motion_ = self.declare_parameter('motion', 'static').value
        self.alt_m_ = self.declare_parameter('alt_m', 2.0).value
        self.v_ = self.declare_parameter('speed_mps', 1.0).value
        self.radius_ = self.declare_parameter('radius_m', 6.0).value
        self.rate_hz_ = self.declare_parameter('rate_hz', 20.0).value
        self.arm_retry_s_ = self.declare_parameter('arm_retry_s', 2.0).value
        # 目标 sys id：入侵者 PX4 实例 1 → MAV_SYS_ID = px4_instance+1 = 2
        self.target_system_ = int(self.declare_parameter('target_system', 2).value)
        # R-016：起点协议参数（契约 #params 单一注入，节点不得覆写）
        self.start_tol_m_ = float(self.declare_parameter('start_tol_m', 0.4).value)
        self.start_vel_eps_mps_ = float(self.declare_parameter('start_vel_eps_mps', 0.35).value)
        self.start_dwell_s_ = float(self.declare_parameter('start_dwell_s', 0.5).value)
        self.run_region_radius_m_ = float(self.declare_parameter('run_region_radius_m', 15.0).value)
        self.epoch_topic_ = self.declare_parameter('epoch_topic', '/agc/scenario/reset_epoch').value

        p = f'/{self.ns_}/fmu'
        self.pub_ocm_ = self.create_publisher(OffboardControlMode, f'{p}/in/offboard_control_mode', _qos())
        self.pub_sp_ = self.create_publisher(TrajectorySetpoint, f'{p}/in/trajectory_setpoint', _qos())
        self.pub_cmd_ = self.create_publisher(VehicleCommand, f'{p}/in/vehicle_command', _qos())
        self.sub_odom_ = self.create_subscription(VehicleOdometry, f'{p}/out/vehicle_odometry', self._on_odom, _qos())
        self.sub_st_ = self.create_subscription(VehicleStatus, f'{p}/out/vehicle_status', self._on_status, _qos())
        # R-017：只消费 jammer 的**已校验授权事件**（非原始命令）→ 停止 Offboard 流
        # （R-019：停发 Offboard 流 → 触发 PX4 offboard-loss 行为，非 data-link-loss）
        self.jam_state_topic_ = self.declare_parameter(
            'jam_state_topic', '/agc/task/jam_state').value
        self.sub_jam_state_ = self.create_subscription(
            String, self.jam_state_topic_, self._on_jam_state, 10)
        # R-016：episode epoch（新 episode 起点 / 时间 / 干扰态复位）
        self.sub_epoch_ = self.create_subscription(Int32, self.epoch_topic_, self._on_epoch, _epoch_qos())

        # 预热时钟：**独立于** episode 计时（节点启动即起算）
        self.warmup_t0_ = self._now_us()
        # episode 状态
        self.origins_ = None
        self.t0_ = None
        self.episode_armed_ = False
        self.last_epoch_ = None
        self.stable_since_ = None
        self.terminated_ = False
        # 观测
        self.have_odom_ = False
        self.pos_ = None
        self.speed_ = None
        self.armed_ = False
        self.nav_state_ = 0
        self.offboard_at_ = None
        self.last_arm_ = None
        # 干扰
        self.jammed_ = False
        self.jam_on_us_ = None
        self.dt_ = 1.0 / self.rate_hz_

        self.create_timer(self.dt_, self._tick)
        self.get_logger().info(
            f'intruder_driver: ns={self.ns_} motion={self.motion_} alt={self.alt_m_} '
            f'speed={self.v_} radius={self.radius_} R_run={self.run_region_radius_m_}')

    # ---- callbacks ----
    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _on_odom(self, m: VehicleOdometry):
        self.have_odom_ = True
        self.pos_ = (m.position[0], m.position[1], m.position[2])
        self.speed_ = math.sqrt(m.velocity[0] ** 2 + m.velocity[1] ** 2 + m.velocity[2] ** 2)
        if self.origins_ is None:
            # 新 episode 后**重新锚定**：原点是进入 episode 时的当前位姿（boot 点或复位后的 spawn 位）
            self.origins_ = self.pos_
            self.get_logger().info(f'odom origin (NED) = {self.origins_}')

    def _on_status(self, m: VehicleStatus):
        self.nav_state_ = m.nav_state
        self.armed_ = (m.arming_state == 2)  # ARMING_STATE_ARMED

    def _on_epoch(self, m: Int32):
        """新 episode：epoch 值变化即重开回合（起点 / 时间 / 干扰态复位）。"""
        val = int(m.data)
        if self.last_epoch_ is None or val != self.last_epoch_:
            self.last_epoch_ = val
            self.origins_ = None      # 下一条 odom 重新锚定
            self.t0_ = None
            self.episode_armed_ = True
            self.stable_since_ = None
            self.terminated_ = False
            self.jammed_ = False
            self.jam_on_us_ = None
            self.get_logger().info(f'episode armed (epoch={val})')

    def _on_jam_state(self, m: String):
        """R-017：消费 jammer 的**已校验授权事件**（`JAM_*` = 授权，`JAM_NONE`/空 = 释放）。"""
        key = m.data.strip().upper()
        jammed = key not in ('', 'JAM_NONE')
        now = self._now_us()
        if jammed and not self.jammed_:
            self.jam_on_us_ = now
            self.get_logger().warn(
                f'jam event {key}: STOP offboard stream '
                f'(→ PX4 offboard-loss, per COM_OF_LOSS_T/COM_OBL_RC_ACT; NOT data-link-loss NAV_DLL_ACT)')
        elif not jammed and self.jammed_:
            # R-016 CLEAR 重基准：把暂停时长加回 t0_，恢复后参考从干扰发生时刻连续续上（不跳变）
            if self.jam_on_us_ is not None and self.t0_ is not None:
                pause = now - self.jam_on_us_
                self.t0_ += pause
                self.get_logger().warn(f'jam CLEAR (rebaseline pause={pause * 1e-6:.3f}s)')
            else:
                self.get_logger().warn(f'jam off ({key}): resume')
            self.jam_on_us_ = None
        self.jammed_ = jammed

    # ---- control ----
    def _send_cmd(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        c = VehicleCommand()
        c.timestamp = self._now_us()
        c.command = command
        c.param1, c.param2, c.param3, c.param4 = p1, p2, p3, p4
        c.param5, c.param6, c.param7 = p5, p6, p7
        c.target_system = self.target_system_     # 本实例 sys id（px4_instance+1）
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.pub_cmd_.publish(c)

    def _target(self, t):
        """返回基于 `origins_` 的 Offboard 位置目标（NED）：static 悬停 / linear 直线 / maneuver 圆周。"""
        ox, oy, oz = self.origins_
        z = oz - self.alt_m_          # NED 向下 → -alt 表示上升 alt（相对 boot 点）
        if self.motion_ == 'linear':
            return ox, oy + self.v_ * t, z
        if self.motion_ == 'maneuver':
            th = (self.v_ / max(self.radius_, 1e-3)) * t
            # t=0 落在圆周定义点 (ox+r, oy)——切入连续（不从原点起跳）
            return ox + self.radius_ * math.cos(th), oy + self.radius_ * math.sin(th), z
        return ox, oy, z              # static：悬停

    def _apply_region(self, tgt):
        """运行区边界：水平离 `origins_` 超过 R（>0 生效）→ 夹持到边界并置 `terminated_`（仅日志一次）。"""
        R = self.run_region_radius_m_
        if R is None or R <= 0.0:
            return tgt
        ox, oy = self.origins_[0], self.origins_[1]
        dx, dy = tgt[0] - ox, tgt[1] - oy
        d = math.hypot(dx, dy)
        if d > R:
            if not self.terminated_:
                self.terminated_ = True
                self.get_logger().error(f'[boundary] run region reached R={R:.2f} (d={d:.2f})')
            k = R / d
            return ox + dx * k, oy + dy * k, tgt[2]
        return tgt

    def _start_pose(self):
        """起始位姿 = `_target(0)`，并经受同一运行区夹持（保证与推进轨迹一致）。"""
        return self._apply_region(self._target(0.0))

    def _maybe_start(self, ts):
        """起点协议：起始位姿稳定持续 `start_dwell_s` → 记 episode t=0。"""
        if self.pos_ is None or self.speed_ is None:
            return
        sx, sy, sz = self._start_pose()
        d = math.sqrt((self.pos_[0] - sx) ** 2 + (self.pos_[1] - sy) ** 2 + (self.pos_[2] - sz) ** 2)
        if d <= self.start_tol_m_ and self.speed_ <= self.start_vel_eps_mps_:
            if self.stable_since_ is None:
                self.stable_since_ = ts
            elif (ts - self.stable_since_) * 1e-6 >= self.start_dwell_s_:
                self.t0_ = ts
                self.get_logger().info(f'episode t=0 (epoch={self.last_epoch_})')
        else:
            self.stable_since_ = None

    def _tick(self):
        if not self.have_odom_ or self.origins_ is None:
            return
        if self.jammed_:
            return  # 干扰授权生效：停发 Offboard → PX4 offboard-loss（R8/R-019；非数据链失联）
        ts = self._now_us()
        tw = (ts - self.warmup_t0_) * 1e-6     # 预热时间（**独立于** episode）

        # ① OffboardControlMode（位置）——切 OFFBOARD 前须持续流 > 1 s
        ocm = OffboardControlMode()
        ocm.timestamp = ts
        ocm.position = True
        self.pub_ocm_.publish(ocm)

        # ② 位置 setpoint：未进入 episode → 恒定悬停保持（不推进）；否则按 episode t 推进
        if self.t0_ is None:
            tgt = self._start_pose()
        else:
            tgt = self._apply_region(self._target((ts - self.t0_) * 1e-6))
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float(tgt[0]), float(tgt[1]), float(tgt[2])]
        sp.yaw = 0.0
        self.pub_sp_.publish(sp)

        # ③ 起点协议：稳定判定 → episode t=0（未进入 episode 且已武装且未越界终止）
        if self.t0_ is None and self.episode_armed_ and not self.terminated_:
            self._maybe_start(ts)

        # ④ 预热（独立计时）→ arm + 切 OFFBOARD（失败则按 arm_retry_s_ 重试）
        if tw > 1.0 and not self.armed_:
            if self.last_arm_ is None or tw - self.last_arm_ > self.arm_retry_s_:
                self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)  # ARM
                self.last_arm_ = tw
                self.get_logger().warn(f'[tw={tw:.1f}s] requesting ARM (armed={self.armed_})')
        if self.armed_ and self.nav_state_ != OFFBOARD_NAV_STATE:
            if self.offboard_at_ is None or tw - self.offboard_at_ > self.arm_retry_s_:
                self._send_cmd(VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, p1=float(OFFBOARD_NAV_STATE))
                self.offboard_at_ = tw
                self.get_logger().info(f'[tw={tw:.1f}s] requesting OFFBOARD (nav_state={self.nav_state_})')


def main():
    rclpy.init()
    node = IntruderDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
