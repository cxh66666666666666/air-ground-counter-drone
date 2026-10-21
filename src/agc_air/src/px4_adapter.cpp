// px4_adapter —— PX4 uORB → 契约口径 的归一化适配（spec R2，ADR-0005）
//
// 唯一直接消费 `/fmu/out/*` 的节点：集中完成微秒时间戳→ROS 时间、NED/FRD↔ENU/FLU、
// QoS 标定，并归一化到契约口径 `/agc/air/*`。上层控制节点只消费归一化话题。
// 帧名依契约 [命名空间与坐标系]：`frame_id=world`、`child_frame_id=drone/base_link`（禁裸 base_link）。
// 帧约定（R-003）：pose 为世界 ENU（已含 EKF 局部原点平移）；twist.linear 为**机体系 FLU**
// （nav_msgs Odometry 要求 twist 在 child_frame），twist.angular 同为机体系 FLU。

#include <cmath>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "agc_air/agc_lifecycle_node.hpp"
#include "agc_air/px4_conversions.hpp"

using agc_air::AgcLifecycleNode;

namespace
{
const char * kOdomOut = "/agc/air/odom";
const char * kStatusOut = "/agc/air/status";
}  // namespace

class Px4Adapter : public AgcLifecycleNode
{
public:
  Px4Adapter()
  : AgcLifecycleNode("px4_adapter")
  {
  }

protected:
  CallbackReturn on_configure_impl() override
  {
    odom_in_ = declare_param_idempotent<std::string>("odom_in_topic", "/fmu/out/vehicle_odometry");
    status_in_ = declare_param_idempotent<std::string>("status_in_topic", "/fmu/out/vehicle_status");
    odom_out_ = declare_param_idempotent<std::string>("odom_out_topic", kOdomOut);
    status_out_ = declare_param_idempotent<std::string>("status_out_topic", kStatusOut);
    world_frame_ = declare_param_idempotent<std::string>("world_frame", "world");
    base_frame_ = declare_param_idempotent<std::string>("base_frame", "drone/base_link");
    time_alpha_ = declare_param_idempotent<double>("time_offset_alpha", 0.05);
    // W7-b 取证：PX4→ROS 时间映射的**合理性界**（s）——映射结果与 `now()` 的偏差超过它即判
    // 不可信（拒发该帧 + 重锚）。默认 5.0 s：远大于正常帧龄（≤ 数十 ms），又能拦住尺度跳变。
    stamp_max_skew_s_ = declare_param_idempotent<double>("stamp_max_skew_s", 5.0);
    // 局部原点（R-003）：PX4 odom 的 position 相对 EKF 全局原点，非 gz 世界坐标。
    // 该原点 = 首次定位时刻的机体位置（SITL 下即 spawn 位），由 launch 以 spawn 位姿单一注入。
    const auto origin_pose = declare_param_idempotent<std::string>("origin_pose", "0,0,0.2,0,0,0");
    if (!agc_air::parseSpawnPositionXyz(origin_pose, origin_enu_)) {
      RCLCPP_WARN(
        get_logger(), "origin_pose='%s' 解析失败 → 局部原点按 (0,0,0) 处理", origin_pose.c_str());
      origin_enu_ = agc_air::Vector3::Zero();
    }

    mapper_ = agc_air::Px4TimeMapper(time_alpha_);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      odom_out_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    status_pub_ = create_publisher<px4_msgs::msg::VehicleStatus>(
      status_out_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    // PX4 uXRCE-DDS 的 `/fmu/out/*` writer 为 best_effort，订阅端必须同档（否则 QoS 不兼容、收不到数据）；
    // 归一化后的对外话题再按契约口径给 QoS（odom→SENSOR_DATA 档、状态→内部可靠）。
    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      odom_in_, rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleOdometry::SharedPtr m) {onOdom(*m);});

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      status_in_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](px4_msgs::msg::VehicleStatus::SharedPtr m) {status_pub_->publish(*m);});

    RCLCPP_INFO(get_logger(), "px4_adapter 就绪：%s → %s", odom_in_.c_str(), odom_out_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate_impl() override
  {
    odom_pub_->on_activate();
    status_pub_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate_impl() override
  {
    odom_pub_->on_deactivate();
    status_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup_impl() override
  {
    odom_sub_.reset();
    status_sub_.reset();
    odom_pub_.reset();
    status_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void onOdom(const px4_msgs::msg::VehicleOdometry & m)
  {
    // 帧校验（R-003）：按输入自报的 pose_frame / velocity_frame 处理，不默认按 NED。
    // PX4 EKF2 恒发 POSE_FRAME_NED + VELOCITY_FRAME_NED（源码确认）；其它帧（mocap/视觉里程计等）
    // 不得静默当 NED 使用 —— 本适配层暂不支持，丢弃该帧并节流告警。
    if (m.pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED ||
      m.velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "vehicle_odometry 帧不符（pose_frame=%u, velocity_frame=%u）：仅支持 NED，已丢弃（R-003）",
        m.pose_frame, m.velocity_frame);
      return;
    }

    const std::uint64_t px4_us = m.timestamp;
    // EKF 复位检测（R-003）：reset_counter 变化意味着局部原点/姿态可能跳变，世界坐标须重新标定。
    // W7-b 取证：EKF 复位同时**丢弃时间锚定**（局部原点与时间基准同源，都会随之跳变）。
    if (m.reset_counter != reset_counter_) {
      RCLCPP_WARN(
        get_logger(), "EKF 复位计数 %u → %u：局部原点可能跳变，世界坐标需重新标定（R-003）；"
        "同时重锚 PX4→ROS 时间映射", reset_counter_, m.reset_counter);
      reset_counter_ = m.reset_counter;
      mapper_.reset();
    }

    // W7-b 取证（整链 full）：先用**当前**估计映射并做**合理性校验**——非负且与 `now()` 的偏差
    // 在 `stamp_max_skew_s` 内。实测 uXRCE-DDS 桥时间同步与仿真 `/clock` 打架时，会产出
    // `sec≈−1.7e9` / `+8.5e7` 的脏 stamp，令下游 `rclcpp::Time(stamp)` 抛
    // "cannot store a negative time point"（**整节点崩溃**）与年龄门失效。
    // 处置：**拒发该帧** + 一次性如实告警 + **重锚**（丢弃被污染的偏移，下一帧自愈）——
    // **不**静默替换为其它时刻（R-004 源采样时刻语义不许伪装）。
    const std::int64_t now_ns = now().nanoseconds();
    if (mapper_.valid()) {
      const std::int64_t ros_ns = mapper_.toRosNanoseconds(px4_us);
      const double skew_s =
        std::fabs(static_cast<double>(ros_ns - now_ns)) * 1e-9;
      if (ros_ns < 0 || skew_s > stamp_max_skew_s_) {
        if (!warned_bad_stamp_) {
          warned_bad_stamp_ = true;
          RCLCPP_WARN(
            get_logger(),
            "PX4 odom 时间映射不可信（px4_us=%llu → ros=%.3f s，与 now 偏差 %.3f s > %.1f s）"
            "→ 拒发该帧并重锚时间映射（时间源尺度可能跳变；R-004 不静默替换）",
            static_cast<unsigned long long>(px4_us), ros_ns * 1e-9, skew_s, stamp_max_skew_s_);
        }
        mapper_.reset();
        return;
      }
    }
    mapper_.observe(px4_us, now_ns / 1000);
    if (!mapper_.valid()) {return;}

    const agc_air::Vector3 pos_ned(m.position[0], m.position[1], m.position[2]);
    const agc_air::Vector3 vel_ned(m.velocity[0], m.velocity[1], m.velocity[2]);
    const agc_air::Vector3 ang_frd(m.angular_velocity[0], m.angular_velocity[1],
      m.angular_velocity[2]);
    // 无效估计按 NaN 上报（PX4 约定），不得当有效位姿/速度发布
    if (!pos_ned.allFinite() || !vel_ned.allFinite()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "vehicle_odometry 位置/速度为 NaN：已丢弃该帧");
      return;
    }
    const auto q_enu = agc_air::px4AttitudeToEnu(m.q);

    nav_msgs::msg::Odometry out;
    // R-002：rclcpp::Time(单整数) 按纳秒解释 → 必须用 toRosNanoseconds（误用 toRosMicros 会缩小 1000 倍）
    out.header.stamp = rclcpp::Time(mapper_.toRosNanoseconds(px4_us), RCL_ROS_TIME);
    out.header.frame_id = world_frame_;
    out.child_frame_id = base_frame_;
    // 位置：局部 NED → 世界 ENU（含 EKF 原点平移，R-003）
    const agc_air::Vector3 p = agc_air::localNedToWorldEnu(pos_ned, origin_enu_);
    out.pose.pose.position.x = p.x();
    out.pose.pose.position.y = p.y();
    out.pose.pose.position.z = p.z();
    out.pose.pose.orientation.x = q_enu.x();
    out.pose.pose.orientation.y = q_enu.y();
    out.pose.pose.orientation.z = q_enu.z();
    out.pose.pose.orientation.w = q_enu.w();

    // 速度：世界 ENU → **机体系 FLU**（nav_msgs Odometry 约定：twist 在 child_frame_id，
    // 与 frame_id 的世界位姿区分；消费端用 px4_conversions 的 bodyFluToEnu 还原世界速度，R-003）
    const agc_air::Vector3 v_flu = agc_air::enuToBodyFlu(q_enu, agc_air::nedToEnu(vel_ned));
    out.twist.twist.linear.x = v_flu.x();
    out.twist.twist.linear.y = v_flu.y();
    out.twist.twist.linear.z = v_flu.z();

    // 角速度：PX4 为 body-fixed FRD → 机体系 FLU（本就在 child frame，无需换基到世界）
    const agc_air::Vector3 w = agc_air::frdToFlu(ang_frd);
    out.twist.twist.angular.x = w.x();
    out.twist.twist.angular.y = w.y();
    out.twist.twist.angular.z = w.z();

    odom_pub_->publish(out);
  }

  std::string odom_in_, status_in_, odom_out_, status_out_, world_frame_, base_frame_;
  double time_alpha_{0.05};
  double stamp_max_skew_s_{5.0};
  bool warned_bad_stamp_{false};
  agc_air::Vector3 origin_enu_{agc_air::Vector3::Zero()};
  std::uint8_t reset_counter_{0};
  agc_air::Px4TimeMapper mapper_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<Px4Adapter>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
