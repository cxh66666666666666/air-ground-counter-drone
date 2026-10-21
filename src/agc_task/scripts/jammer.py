#!/usr/bin/env python3
"""jammer —— 行为级电子干扰注入（W6 R8 / spec R8；R-017 单一授权 + R-018 真实确认 + R-019 因子分离）。

对**入侵者** PX4 经 uXRCE 发 `px4_msgs/VehicleCommand`（`VEHICLE_CMD_SET_NAV_STATE`）强制
`RTL / LAND / HOLD`；"失联"（LOST）以**停发 Offboard 流**触发 PX4 自身 failsafe。
**不**触碰己方机 / 车（契约 [唯一执行口]）。

R-019（[ADR-0034]）failsafe **因子分离**（三因子，独立可观测 / 标注）：
  ① **强制模式**（`forced_mode`）：`RTL / LAND / HOLD` 发 `SET_NAV_STATE`（nav_state 5 / 18 / 4）。
  ② **仅断 Offboard**（`offboard_only`）：`LOST` **只**停发 Offboard 流，**不**发 `SET_NAV_STATE`；
     其后的行为由运行时锁定的 `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` 解释（见
     `src/agc_task/config/px4_failsafe_lock.yaml` 真源与 `scripts/lock_px4_failsafe_params.sh` 注入的
     SITL 启动参数 `px4-rc.params`），**非**数据链失联。
  ③ **数据链失联**（`NAV_DLL_ACT`）：本轮**未实现**（如实限制，见 [ADR-0034]）。
  **停发 Offboard 流 ≠ `NAV_DLL_ACT`**：前者是 offboard-loss（`COM_OF_LOSS_T` + `COM_OBL_RC_ACT`），
  后者才是 data-link-loss（`NAV_DLL_ACT`），两者为**不同机制**。
  `LOST` 的确认期望 nav_state = `lost_expected_nav_state`（= 运行时锁定 `COM_OBL_RC_ACT` 的映射；
  默认 `0` → POSCTL `2`），使 LOST 的 R-018 确认仍可成立（观测目标进入该 failsafe 模式且持续）。

R-017（[ADR-0032]）：本节点是干扰授权的**唯一裁决者**。仅当「命令合法」**且**「作用包络**新鲜**」
（最近 `envelope_max_age_s` 内收到包络为真）才产出**已校验授权事件** `JAM_*`（发布到 `state_topic`）；
未知命令 / 包络外 / 包络**过期（未就绪）** 一律**不产出**事件。`JAM_NONE` 是**释放事件**（非注入），
`''/NONE/CLEAR` **始终发出**。

R-018（[ADR-0033]）：授权 / 发送**不等于**效果。证据链拆为
**意图 → 授权 → 发送 → ACK → 真实 nav 状态 → 持续行为**：
  * 授权事件（`/agc/task/jam_state`）语义**保留**（R-017，驱动 `intruder_driver` 停发 Offboard）。
  * 发送后开 **确认窗口**（`jam_confirm_window_s`）；订阅入侵者 PX4 `VehicleStatus`（`vehicle_status_topic`），
    **仅当**实际 `nav_state` **达到所命令模式**（`LOST` 为 `lost_expected_nav_state`）且
    **持续 ≥ `jam_sustain_s`** 才产出**已确认事件** `/agc/task/jam_confirm`
    （`std_msgs/String`，恰为 `JAM_RTL|JAM_LAND|JAM_HOLD|JAM_LOST`；**不**追加 factor）。
  * ACK（收到状态流）单独 **不** 构成确认。全部证据 / 负例落 `/agc/task/jam_evidence`，
    证据串携带 R-019 **factor 标签**：
    `JAM_SENT <reason> factor=<factor>` / `JAM_ACK <reason> nav_state=N factor=<factor>` /
    `JAM_CONFIRMED <reason> hold=T factor=<factor>` / `JAM_RELEASE`；
    负例 `JAM_FAIL_NO_FEEDBACK`（窗口内无状态，曾见过）/
    `JAM_FAIL_TARGET_ABSENT`（从未见过目标状态）/
    `JAM_FAIL_MODE_UNREACHED`（窗口内有状态但未达目标模式）/
    `JAM_FAIL_NOT_SUSTAINED`（达到但未持续 `jam_sustain_s`）——负例亦携带 `factor`。

R-020（[ADR-0035]）：干扰**作用包络**已与**捕获作用包络**分离。
  * 可交战包络 `/agc/task/envelope_in` 由 `capture_adjudicator` 的**最小动态火控交战门**给出
    （W7-b T2/E2：逐帧弹道；高度含车体 `h = target.z − car.z − launcher_h`）。
    （高度含车体 `h = target.z − car.z − launcher_h`）。
  * 干扰包络 `/agc/task/envelope_jam` 为**独立模型**（`behavioral_range_gate`：高度带 + 水平距离门），
    **不**与网弹道共源。本节点默认门控话题即为该独立干扰包络。
  `envelope_max_age_s` 新鲜度门控沿用（R-017）。

触发：订阅 `/agc/task/jam_cmd`（`std_msgs/String`，取值 `RTL|LAND|HOLD|LOST|NONE`）。
下游（`intruder_driver`）**只消费** `state_topic` 授权事件；`task_manager` 的 `INTERCEPT_CONFIRMED`
**只**由 `confirm_topic` 已确认事件驱动。
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleCommand, VehicleStatus
from std_msgs.msg import Bool, String

# 干扰方式 → R-019 因子 + PX4 nav_state（NED；见 VehicleStatus.msg）。
# factor：`forced_mode`（发 SET_NAV_STATE 强制模式）/ `offboard_only`（仅断 Offboard，**不**发命令）。
# nav_state 为 `None` 表示**不**发 `SET_NAV_STATE`（LOST 的后续行为由运行时锁定的
# COM_OF_LOSS_T / COM_OBL_RC_ACT 解释，非 NAV_DLL_ACT）。
JAM_MAP = {
    'RTL':  {'factor': 'forced_mode',   'nav_state': VehicleStatus.NAVIGATION_STATE_AUTO_RTL,
             'reason': 'JAM_RTL'},       # 5
    'LAND': {'factor': 'forced_mode',   'nav_state': VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
             'reason': 'JAM_LAND'},      # 18
    'HOLD': {'factor': 'forced_mode',   'nav_state': VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
             'reason': 'JAM_HOLD'},      # 4
    'LOST': {'factor': 'offboard_only', 'nav_state': None,
             'reason': 'JAM_LOST'},      # 仅断 Offboard（**不**镜像强制 RTL）
}

# 正向确认集合（R-018）：仅此集合可构成「已确认事件」。
POSITIVE = ('JAM_RTL', 'JAM_LAND', 'JAM_HOLD', 'JAM_LOST')

# VehicleStatus 新鲜度上限（s）：超过即视为「无当前状态反馈」（模式不可信）。
_STATUS_STALE_S = 0.5

# 确认窗口评估节拍（s）。
_TICK_S = 0.05


def _qos():
    q = QoSProfile(depth=5)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.history = HistoryPolicy.KEEP_LAST
    return q


class Jammer(Node):
    def __init__(self):
        super().__init__('jammer')
        self.ns_ = self.declare_parameter('px4_ns', 'px4_1').value
        self.cmd_topic_ = self.declare_parameter('cmd_topic', '/agc/task/jam_cmd').value
        self.state_topic_ = self.declare_parameter('state_topic', '/agc/task/jam_state').value
        # 作用包络（R8/9.3；R-020 分离）：干扰门控改用**独立干扰作用包络**
        # `/agc/task/envelope_jam`（由 capture_adjudicator 以独立模型计算并发布，与捕获包络
        # `/agc/task/envelope_in` 分离，不再共源）。
        self.envelope_topic_ = self.declare_parameter(
            'envelope_topic', '/agc/task/envelope_jam').value
        # R-017：包络新鲜度上限（s）——最近该时长内收到「包络为真」才算有效；过期视同包络外
        self.envelope_max_age_s_ = float(
            self.declare_parameter('envelope_max_age_s', 0.5).value)
        # 目标 sys id：入侵者 PX4 实例 1 → MAV_SYS_ID = 2
        self.target_system_ = int(self.declare_parameter('target_system', 2).value)
        # R-018：真实行为确认（契约 #params 单一注入，节点不得覆写）
        self.vehicle_status_topic_ = self.declare_parameter(
            'vehicle_status_topic', f'/{self.ns_}/fmu/out/vehicle_status').value
        self.confirm_topic_ = self.declare_parameter(
            'confirm_topic', '/agc/task/jam_confirm').value
        self.evidence_topic_ = self.declare_parameter(
            'evidence_topic', '/agc/task/jam_evidence').value
        # 确认窗口（s）：发送后须在此时段内达到并持续目标模式，否则负例。
        self.jam_confirm_window_s_ = float(
            self.declare_parameter('jam_confirm_window_s', 3.0).value)
        # 持续时长（s）：目标模式须持续达到此值才确认。
        self.jam_sustain_s_ = float(self.declare_parameter('jam_sustain_s', 1.0).value)
        # R-019：`LOST`（仅断 Offboard，**不**发 SET_NAV_STATE）的**确认期望** nav_state。
        # 它 = 运行时锁定 `COM_OBL_RC_ACT` 的映射（默认 0 = Position → POSCTL = 2）；见
        # `src/agc_task/config/px4_failsafe_lock.yaml` 真源与 run 证据记录。
        self.lost_expected_nav_state_ = int(
            self.declare_parameter('lost_expected_nav_state', 2).value)

        self.envelope_in_ = False
        self.last_env_rx_ = None   # 最近一次收到包络的墙钟时刻（monotonic）

        # 确认状态机（R-018）
        self.target_nav_state_ = None   # 当前 pending 注入的目标 nav_state（None = 无待确认）
        self.pending_reason_ = None
        self.pending_factor_ = None      # R-019 因子标签（forced_mode / offboard_only）
        self.window_open_t_ = None
        self.sustain_start_t_ = None
        self.acked_ = False
        self.reached_ = False
        self.confirmed_ = False
        # 观测（R-018）：目标 PX4 状态流
        self.ever_status_ = False       # 启动以来是否见过任何 VehicleStatus
        self.nav_state_ = None
        self.last_status_rx_ = None

        self.pub_cmd_ = self.create_publisher(
            VehicleCommand, f'/{self.ns_}/fmu/in/vehicle_command', _qos())
        self.pub_state_ = self.create_publisher(String, self.state_topic_, 10)
        self.pub_confirm_ = self.create_publisher(String, self.confirm_topic_, 10)
        self.pub_evidence_ = self.create_publisher(String, self.evidence_topic_, 10)
        self.sub_ = self.create_subscription(String, self.cmd_topic_, self._on_cmd, 10)
        self.sub_env_ = self.create_subscription(Bool, self.envelope_topic_, self._on_env, 10)
        self.sub_status_ = self.create_subscription(
            VehicleStatus, self.vehicle_status_topic_, self._on_status, _qos())
        self.timer_ = self.create_timer(_TICK_S, self._tick)
        self.get_logger().info(
            f'jammer ready: ns={self.ns_} cmd={self.cmd_topic_} '
            f'status={self.vehicle_status_topic_} confirm={self.confirm_topic_}')

    # ---- 输入 ----
    def _on_env(self, m: Bool):
        self.envelope_in_ = bool(m.data)
        self.last_env_rx_ = time.monotonic()

    def _envelope_ok(self):
        """R-017：授权判据——包络为真**且**在 `envelope_max_age_s` 内新鲜。"""
        return (
            self.envelope_in_ and
            self.last_env_rx_ is not None and
            (time.monotonic() - self.last_env_rx_) <= self.envelope_max_age_s_)

    def _on_status(self, m: VehicleStatus):
        """R-018：真实 nav 状态反馈。ACK = 窗口内首次收到状态流（**不**构成确认）。"""
        self.ever_status_ = True
        self.nav_state_ = int(m.nav_state)
        self.last_status_rx_ = time.monotonic()
        if self.target_nav_state_ is None or self.confirmed_:
            return
        if not self.acked_:
            self.acked_ = True
            self._evidence(
                f'JAM_ACK {self.pending_reason_} nav_state={self.nav_state_} '
                f'factor={self.pending_factor_}')
        if self.nav_state_ == self.target_nav_state_:
            self.reached_ = True

    def _on_cmd(self, m: String):
        key = m.data.strip().upper()
        # 释放事件（非注入）：''/NONE/CLEAR **始终发出** JAM_NONE；取消待确认并出释放备注。
        if key in ('', 'NONE', 'CLEAR'):
            self._cancel_window()
            self.pub_state_.publish(String(data='JAM_NONE'))
            self._evidence('JAM_RELEASE')
            return
        if key not in JAM_MAP:
            self.get_logger().warn(f'unknown jam cmd "{m.data}"')
            return
        if not self._envelope_ok():
            if not self.envelope_in_:
                self.get_logger().warn(
                    f'jam {key} ignored: target out of engagement envelope')
            else:
                age = (time.monotonic() - self.last_env_rx_
                       if self.last_env_rx_ is not None else float('inf'))
                self.get_logger().warn(
                    f'jam {key} ignored: engagement envelope stale/not ready '
                    f'(age={age:.3f}s > {self.envelope_max_age_s_}s)')
            return
        spec = JAM_MAP[key]
        factor = spec['factor']
        nav_state = spec['nav_state']
        reason = spec['reason']
        # R-019 因子分离：仅 `forced_mode` 发 SET_NAV_STATE；`offboard_only`（LOST）**不**发命令。
        if nav_state is not None:
            self._send_nav_state(nav_state)
        # R-017 授权事件（语义保留，驱动 intruder_driver）——幂等：重复命令仍发授权事件
        self.pub_state_.publish(String(data=reason))
        # R-018 开确认窗口；LOST 的期望 nav_state = 运行时锁定 COM_OBL_RC_ACT 的映射值
        expected = nav_state if nav_state is not None else self.lost_expected_nav_state_
        # 幂等去重：同一命令在**待确认窗口内**重复到达（上游重发 / 网络去抖）**不**重开窗口，
        # 以免重置「持续行为」计时（否则重复命令会不断推迟 / 阻止确认）。
        if (self.window_open_t_ is not None and
                self.target_nav_state_ == expected and
                self.pending_reason_ == reason):
            self.get_logger().info(
                f're-inject {reason} (duplicate within window; window not reset)')
            return
        self._open_window(expected, reason, factor)
        self.get_logger().warn(
            f'inject {reason} (factor={factor}, expected_nav_state={expected})')

    # ---- R-018 确认状态机 ----
    def _open_window(self, nav_state, reason, factor):
        self.target_nav_state_ = nav_state
        self.pending_reason_ = reason
        self.pending_factor_ = factor
        self.window_open_t_ = time.monotonic()
        self.sustain_start_t_ = None
        self.acked_ = False
        self.reached_ = False
        self.confirmed_ = False
        self._evidence(f'JAM_SENT {reason} factor={factor}')

    def _cancel_window(self):
        self.target_nav_state_ = None
        self.pending_reason_ = None
        self.pending_factor_ = None
        self.window_open_t_ = None
        self.sustain_start_t_ = None
        self.acked_ = False
        self.reached_ = False
        self.confirmed_ = False

    def _tick(self):
        if self.target_nav_state_ is None or self.confirmed_:
            return
        now = time.monotonic()
        fresh = (self.last_status_rx_ is not None and
                 (now - self.last_status_rx_) <= _STATUS_STALE_S)
        holding = fresh and self.nav_state_ == self.target_nav_state_
        if holding:
            if self.sustain_start_t_ is None:
                self.sustain_start_t_ = now
            elif now - self.sustain_start_t_ >= self.jam_sustain_s_:
                self._confirm(now)
                return
        else:
            self.sustain_start_t_ = None
        # 窗口到期 → 负例分类
        if now - self.window_open_t_ >= self.jam_confirm_window_s_:
            self._fail(now)

    def _confirm(self, now):
        hold = now - self.sustain_start_t_
        reason = self.pending_reason_
        factor = self.pending_factor_
        self._evidence(f'JAM_CONFIRMED {reason} hold={hold:.2f} factor={factor}')
        self.pub_confirm_.publish(String(data=reason))
        self.get_logger().warn(f'confirmed {reason} (hold={hold:.2f}s, factor={factor})')
        self.confirmed_ = True
        self._cancel_window()

    def _fail(self, now):
        reason = self.pending_reason_
        factor = self.pending_factor_
        if not self.reached_:
            if not self.ever_status_:
                tag = 'JAM_FAIL_TARGET_ABSENT'
            elif self.last_status_rx_ is None or self.last_status_rx_ < self.window_open_t_:
                tag = 'JAM_FAIL_NO_FEEDBACK'
            else:
                tag = 'JAM_FAIL_MODE_UNREACHED'
        else:
            tag = 'JAM_FAIL_NOT_SUSTAINED'
        self._evidence(f'{tag} {reason} factor={factor}')
        self.get_logger().warn(f'jam not confirmed: {tag} ({reason})')
        self._cancel_window()

    def _evidence(self, text):
        self.pub_evidence_.publish(String(data=text))

    # ---- 输出 ----
    def _send_nav_state(self, nav_state):
        c = VehicleCommand()
        c.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        c.command = VehicleCommand.VEHICLE_CMD_SET_NAV_STATE
        c.param1 = float(nav_state)
        c.target_system = self.target_system_      # 本实例 sys id（px4_instance+1）
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.pub_cmd_.publish(c)


def main():
    rclpy.init()
    node = Jammer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
