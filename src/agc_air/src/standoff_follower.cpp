// standoff_follower —— 对目标**常开** standoff 跟踪（spec R6，C10）
//
// 契约 [硬约束] C10：无人机对目标的跟踪**常开**（非可选），保持安全间距（不紧贴），
// 目的是不丢失目标；目标须新鲜（ref_max_age_s=0.5、confidence≥0.5），**过期即冻结**。
// 行为对齐旧 `air_follow` 的**距离带策略**（WHAT）；实现为全新重写（HOW 不继承）。
// 输出 `/agc/air/tracking_ref`，**覆盖**基础轨迹参考（由 trajectory_tracker 执行覆盖）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>

#include <agc_msgs/msg/target_track_array.hpp>

#include "agc_air/agc_lifecycle_node.hpp"
#include "agc_air/px4_conversions.hpp"
#include "agc_air/reference_msg.hpp"

using agc_air::AgcLifecycleNode;

class StandoffFollower : public AgcLifecycleNode
{
public:
  StandoffFollower()
  : AgcLifecycleNode("standoff_follower")
  {
  }

protected:
  CallbackReturn on_configure_impl() override
  {
    world_frame_ = declare_param_idempotent<std::string>("world_frame", "world");
    odom_topic_ = declare_param_idempotent<std::string>("odom_topic", "/agc/air/odom");
    target_topic_ = declare_param_idempotent<std::string>("target_topic", "/agc/targets");
    out_topic_ = declare_param_idempotent<std::string>("tracking_ref_topic", "/agc/air/tracking_ref");

    safe_min_ = declare_param_idempotent<double>("safe_min", 8.0);
    safe_max_ = declare_param_idempotent<double>("safe_max", 15.0);
    // 跟踪高度：相对**参考基准**（trajectory_tracker 统一叠加高度基准偏移，见其接管处说明）
    height_m_ = declare_param_idempotent<double>("height", 2.5);
    hysteresis_m_ = declare_param_idempotent<double>("hysteresis", 1.0);
    max_step_m_ = declare_param_idempotent<double>("max_step_m", 0.1);
    rate_hz_ = declare_param_idempotent<double>("rate_hz", 20.0);
    ref_max_age_s_ = declare_param_idempotent<double>("ref_max_age_s", 0.5);
    confidence_min_ = declare_param_idempotent<double>("confidence_min", 0.5);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        drone_pos_ = agc_air::Vector3(m->pose.pose.position.x, m->pose.pose.position.y,
            m->pose.pose.position.z);
        have_odom_ = true;
      });

    target_sub_ = create_subscription<agc_msgs::msg::TargetTrackArray>(
      target_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](agc_msgs::msg::TargetTrackArray::SharedPtr m) {
        if (m->tracks.empty()) {have_target_ = false; return;}
        const auto & tr = m->tracks.front();
        const auto & p = tr.pose.pose.position;
        target_pos_ = agc_air::Vector3(p.x, p.y, p.z);
        target_conf_ = tr.confidence;
        // 新鲜度以消息自身时刻衡量（契约 C10）
        target_stamp_ = rclcpp::Time(tr.header.stamp, RCL_ROS_TIME);
        have_target_ = true;
      });

    ref_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      out_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    RCLCPP_INFO(get_logger(), "standoff_follower 就绪：距离带 [%.1f, %.1f] m，高度 %.1f m",
      safe_min_, safe_max_, height_m_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate_impl() override
  {
    ref_pub_->on_activate();
    frozen_ = false;
    last_ref_ = agc_control::Reference();
    last_ref_.pos = drone_pos_;
    last_ref_.pos.z() = height_m_;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_), [this]() {tick();});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate_impl() override
  {
    if (timer_) {timer_->cancel();}
    timer_.reset();
    if (ref_pub_) {ref_pub_->on_deactivate();}
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup_impl() override
  {
    timer_.reset();
    odom_sub_.reset();
    target_sub_.reset();
    ref_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  /// 目标是否满足新鲜度要求（契约 C10：ref_max_age_s 与 confidence 门槛）
  bool targetFresh() const
  {
    if (!have_target_) {return false;}
    const double age = (now() - target_stamp_).seconds();
    return age >= 0.0 && age < ref_max_age_s_ && target_conf_ >= confidence_min_;
  }

  void tick()
  {
    if (!have_odom_ || !ref_pub_ || !ref_pub_->is_activated()) {return;}

    if (!targetFresh()) {
      // 契约 C10 的"过期即冻结"针对**已捕获目标后**失联的情形；
      // 若从未捕获过目标，则不应占用跟踪参考（否则会覆盖基础轨迹参考，飞机只会原地悬停）。
      if (!acquired_) {return;}
      // 过期即冻结：保持上一有效跟踪参考，不再更新（既不掉高也不追）
      if (!frozen_) {
        frozen_ = true;
        RCLCPP_WARN(get_logger(), "目标过期/置信度不足 → 冻结跟踪（hold）");
      }
      // 冻结语义 = **保持位置**：速度/加速度前馈清零，否则残留的参考速度会被 PX4 位置环
      // 折算成稳态偏置（实测 vel.z 残留使机体稳定高出参考 ~3 m）。
      agc_control::Reference hold = last_ref_;
      hold.vel = agc_air::Vector3::Zero();
      hold.acc = agc_air::Vector3::Zero();
      ref_pub_->publish(agc_air::toMsg(hold, world_frame_, now()));
      return;
    }
    acquired_ = true;
    frozen_ = false;

    // 水平距离（世界系 ENU，XY 平面）
    const agc_air::Vector3 delta = drone_pos_ - target_pos_;
    const double r = std::hypot(delta.x(), delta.y());

    // 距离带 + 滞回：决定目标距离
    double d_target = r;
    if (r > safe_max_) {
      d_target = safe_max_;
      approaching_ = true;
    } else if (r < safe_min_) {
      d_target = safe_min_;
      approaching_ = false;
    } else if (approaching_ && r <= safe_max_ - hysteresis_m_) {
      approaching_ = false;   // 进入带内并越过滞回下沿 → 停止趋近
    } else if (!approaching_ && r >= safe_min_ + hysteresis_m_) {
      approaching_ = false;
    }
    if (r > safe_max_) {approaching_ = true;}

    // 站定参考：目标→机体的射线方向上取 d_target
    agc_control::Reference ref;
    if (r < 1e-3) {
      ref.pos = drone_pos_;
    } else {
      const agc_air::Vector3 w(delta.x() / r, delta.y() / r, 0.0);
      agc_air::Vector3 standoff = target_pos_ + w * d_target;
      standoff.z() = height_m_;
      // 单拍限幅（0.1 m@20 Hz → 最大水平速度 2.0 m/s）
      agc_air::Vector3 move = standoff - drone_pos_;
      const double step = std::hypot(move.x(), move.y());
      if (step > max_step_m_) {move *= max_step_m_ / step;}
      ref.pos = drone_pos_ + move;
      ref.pos.z() = height_m_;
      // 速度前馈只取**水平**分量：高度参考 `height_m_` 是常量，其“参考速度”必须为 0。
      // 若照搬 `move * rate_hz_`，z 误差会被放大成 ±20·e_z 的垂直速度前馈（实测 ±13 m/s），
      // 与位置环重复控制高度 → 高增益双环 → 机体上下大幅振荡（高度环全靠 LQR 的 PD 项即可）。
      ref.vel = agc_air::Vector3(move.x() * rate_hz_, move.y() * rate_hz_, 0.0);
    }

    last_ref_ = ref;
    ref_pub_->publish(agc_air::toMsg(ref, world_frame_, now()));
  }

  std::string world_frame_, odom_topic_, target_topic_, out_topic_;
  double safe_min_{8.0}, safe_max_{15.0}, height_m_{2.5}, hysteresis_m_{1.0};
  double max_step_m_{0.1}, rate_hz_{20.0}, ref_max_age_s_{0.5}, confidence_min_{0.5};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<agc_msgs::msg::TargetTrackArray>::SharedPtr target_sub_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr ref_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  agc_air::Vector3 drone_pos_{agc_air::Vector3::Zero()};
  agc_air::Vector3 target_pos_{agc_air::Vector3::Zero()};
  double target_conf_{0.0};
  rclcpp::Time target_stamp_{0, 0, RCL_ROS_TIME};
  bool have_odom_{false};
  bool have_target_{false};
  bool acquired_{false};   // 是否曾捕获过目标（决定是否占用跟踪参考）
  bool frozen_{false};
  bool approaching_{false};
  agc_control::Reference last_ref_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<StandoffFollower>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
