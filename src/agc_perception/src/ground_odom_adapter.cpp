// ground_odom_adapter —— 地面 odom 归一（W4-a 加固 spec R2；R-039 源时刻分离 + 复位 epoch 速度历史）
//
// 位姿来源：**W4 真值旁路**（`/agc/gt/agc_car`，gz 物理引擎产生的世界真值）。
// 归一为 world 系 `/agc/ground/odom`（`world`→`car/base_link`）+ TF + `/agc/ground/odom_valid`
// + 诊断 `/agc/ground/odom_diag`。
//
// R-039（口径见 docs/adr/0054-ground-odom-source-time-and-reset-epoch-velocity.md）修复三项时间口径缺口：
//   * **源采样时刻 = 发布时刻**：`Header.stamp` 采用 `gt` 样本自带 `Header.stamp`，且**仅在有新样本时**
//     发布 odom / TF；**不**用 `now()` 冒充测量时刻（本实现**无静默外推**——TF 连续性由「源率 = odom 率」
//     承载，无新样本时停发并由 `valid=false` 显式表达，符合 tf2「缺变换即显式错误」惯例）。
//   * **负年龄 / 时钟回拨 → 显式无效**：`age = now − 源采样时刻`，**不钳 0**；`age < 0` 记 `clock_anomaly`，
//     `age > odom_max_age_s_` 记 `stale`，二者均 `valid=false`。
//   * **速度历史按 reset epoch 清空**：订阅 `/agc/scenario/reset_epoch`（Int32，transient_local）；epoch 变化
//     → 清 `history_`；另有**位置跳变守卫**（teleport 不连续）双保险。重建 ≥2 个新样本前速度输出 0。
//
// twist 仍按 ROS Odometry 约定表达在 child frame：world 速度短窗差分后旋转到车体系。
// 帧名遵循契约 #namespace：机体系固定 car/base_link（禁止裸 base_link）。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "agc_perception/math_utils.hpp"

namespace
{
const size_t kHistoryMax = 500;  // 与旧树行为对齐（deque(maxlen=500)）

struct Sample
{
  double t, x, y, yaw;
};

std::string fmt3(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3f", v);
  return std::string(buf);
}
}  // namespace

class GroundOdomAdapter : public rclcpp::Node
{
public:
  GroundOdomAdapter()
  : Node("ground_odom_adapter")
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    base_frame_ = declare_parameter<std::string>("base_frame", "car/base_link");
    gt_topic_ = declare_parameter<std::string>("gt_topic", "/agc/gt/agc_car");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/agc/ground/odom");
    valid_topic_ = declare_parameter<std::string>("valid_topic", "/agc/ground/odom_valid");
    diag_topic_ = declare_parameter<std::string>("diag_topic", "/agc/ground/odom_diag");
    reset_epoch_topic_ =
      declare_parameter<std::string>("reset_epoch_topic", "/agc/scenario/reset_epoch");
    rate_hz_ = declare_parameter<double>("rate_hz", 30.0);
    diff_window_s_ = declare_parameter<double>("diff_window_s", 0.15);
    odom_max_age_s_ = declare_parameter<double>("odom_max_age_s", 0.5);
    reset_jump_clear_m_ = declare_parameter<double>("reset_jump_clear_m", 0.5);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(valid_topic_, 10);
    // 诊断 latched（transient_local）：任一时刻可读到最近一次 state / age / epoch（非 ABI）。
    diag_pub_ = create_publisher<std_msgs::msg::String>(
      diag_topic_, rclcpp::QoS(1).transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    gt_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      gt_topic_, 10,
      std::bind(&GroundOdomAdapter::onGt, this, std::placeholders::_1));

    // R-039：reset epoch（transient_local，可拿到启动前的最近值）；epoch 变化 → 速度历史作废。
    reset_epoch_sub_ = create_subscription<std_msgs::msg::Int32>(
      reset_epoch_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&GroundOdomAdapter::onResetEpoch, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1e-3));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GroundOdomAdapter::tick, this));

    RCLCPP_INFO(
      get_logger(), "ground_odom_adapter：真值源 %s → %s（%.1f Hz）；源采样时刻=发布时刻，epoch 源 %s",
      gt_topic_.c_str(), odom_topic_.c_str(), rate_hz_, reset_epoch_topic_.c_str());
  }

private:
  void onGt(const geometry_msgs::msg::PoseStamped & msg)
  {
    const rclcpp::Time t_src(msg.header.stamp);
    const double x = msg.pose.position.x;
    const double y = msg.pose.position.y;

    // teleport / 不连续守卫（保守）：相邻样本位移跳变 → 速度历史清空（与 reset epoch 双保险）。
    if (!history_.empty()) {
      const Sample & last = history_.back();
      const double jump = std::hypot(x - last.x, y - last.y);
      if (jump > reset_jump_clear_m_) {
        history_.clear();
        RCLCPP_WARN(
          get_logger(), "ground odom：位置跳变 %.3f m > %.3f m → 速度历史清空（teleport?）",
          jump, reset_jump_clear_m_);
      }
    }

    cur_pos_ = {x, y, msg.pose.position.z};
    cur_quat_ = {
      msg.pose.orientation.x, msg.pose.orientation.y,
      msg.pose.orientation.z, msg.pose.orientation.w};
    src_stamp_ = t_src;
    history_.push_back(Sample{t_src.seconds(), x, y, agc::yawFromQuat(msg.pose.orientation)});
    if (history_.size() > kHistoryMax) {history_.pop_front();}
    have_src_ = true;
  }

  void onResetEpoch(const std_msgs::msg::Int32 & msg)
  {
    if (!epoch_seen_ || msg.data != reset_epoch_) {
      reset_epoch_ = msg.data;
      epoch_seen_ = true;
      history_.clear();
      RCLCPP_INFO(
        get_logger(), "ground odom：reset epoch=%d → 速度历史清空，等待新稳定样本", reset_epoch_);
    }
  }

  // 年龄 = now − 源采样时刻（**不钳 0**；负值即 future stamp / 时钟回拨）。
  double signedAge() const
  {
    return now().seconds() - src_stamp_.seconds();
  }

  void velocity(std::array<double, 3> * v_world, double * wz) const
  {
    *v_world = {{0.0, 0.0, 0.0}};
    *wz = 0.0;
    if (history_.size() < 2) {return;}
    const Sample & s1 = history_.back();
    Sample s0 = history_.front();
    for (const Sample & e : history_) {
      if (s1.t - e.t <= diff_window_s_) {s0 = e; break;}
    }
    const double dt = s1.t - s0.t;
    if (dt <= 1e-6) {return;}
    (*v_world)[0] = (s1.x - s0.x) / dt;
    (*v_world)[1] = (s1.y - s0.y) / dt;
    (*v_world)[2] = 0.0;
    *wz = agc::normalizeAngle(s1.yaw - s0.yaw) / dt;
  }

  void publishValid(bool ok)
  {
    std_msgs::msg::Bool m;
    m.data = ok;
    valid_pub_->publish(m);
  }

  void publishDiag(const std::string & state, double age_s)
  {
    std_msgs::msg::String m;
    m.data = "state=" + state + " age=" + fmt3(age_s) + " epoch=" + std::to_string(reset_epoch_);
    diag_pub_->publish(m);
  }

  void tick()
  {
    if (!have_src_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "ground odom：等待真值 %s", gt_topic_.c_str());
      publishDiag("no_sample", 0.0);
      publishValid(false);
      return;
    }

    const double age_s = signedAge();
    if (age_s < 0.0) {
      // future stamp / 仿真时钟回拨：显式无效（**不**钳 0 让坏时钟仍算新鲜）。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ground odom clock_anomaly: age=%.3f s < 0（future stamp / 时钟回拨）", age_s);
      publishDiag("clock_anomaly", age_s);
      publishValid(false);
      return;
    }
    if (age_s > odom_max_age_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ground odom stale: age=%.3f s > %.3f s",
        age_s, odom_max_age_s_);
      publishDiag("stale", age_s);
      publishValid(false);
      return;
    }

    // 仅有**新样本**时发布；`Header.stamp` = **源采样时刻**（(= src_stamp_)），非 now()。
    if (src_stamp_ != last_pub_stamp_) {
      std::array<double, 3> v_world;
      double wz = 0.0;
      velocity(&v_world, &wz);  // history < 2（含 reset/teleport 后重建期）→ 0，不产生尖峰

      // world 速度 → 车体系（R(q)^T · v_world）
      tf2::Quaternion q(cur_quat_[0], cur_quat_[1], cur_quat_[2], cur_quat_[3]);
      tf2::Matrix3x3 R(q);
      const tf2::Vector3 v_w(v_world[0], v_world[1], v_world[2]);
      const tf2::Vector3 v_b = R.transpose() * v_w;

      const rclcpp::Time stamp = src_stamp_;  // 源采样时刻——**不**用 now() 冒充测量时刻

      nav_msgs::msg::Odometry o;
      o.header.stamp = stamp;
      o.header.frame_id = world_frame_;
      o.child_frame_id = base_frame_;
      o.pose.pose.position.x = cur_pos_[0];
      o.pose.pose.position.y = cur_pos_[1];
      o.pose.pose.position.z = cur_pos_[2];
      o.pose.pose.orientation.x = cur_quat_[0];
      o.pose.pose.orientation.y = cur_quat_[1];
      o.pose.pose.orientation.z = cur_quat_[2];
      o.pose.pose.orientation.w = cur_quat_[3];
      o.twist.twist.linear.x = v_b.x();
      o.twist.twist.linear.y = v_b.y();
      o.twist.twist.linear.z = v_b.z();
      o.twist.twist.angular.x = 0.0;
      o.twist.twist.angular.y = 0.0;
      o.twist.twist.angular.z = wz;
      odom_pub_->publish(o);

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = world_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = cur_pos_[0];
      tf.transform.translation.y = cur_pos_[1];
      tf.transform.translation.z = cur_pos_[2];
      tf.transform.rotation.x = cur_quat_[0];
      tf.transform.rotation.y = cur_quat_[1];
      tf.transform.rotation.z = cur_quat_[2];
      tf.transform.rotation.w = cur_quat_[3];
      tf_broadcaster_->sendTransform(tf);

      last_pub_stamp_ = src_stamp_;
      has_last_pub_ = true;
    }

    publishValid(true);
    publishDiag(history_.size() >= 2 ? "ok" : "reset_rebuild", age_s);
  }

  std::string world_frame_, base_frame_, gt_topic_, odom_topic_, valid_topic_, diag_topic_;
  std::string reset_epoch_topic_;
  double rate_hz_{30.0}, diff_window_s_{0.15}, odom_max_age_s_{0.5}, reset_jump_clear_m_{0.5};

  bool have_src_{false};
  std::array<double, 3> cur_pos_{{0.0, 0.0, 0.0}};
  std::array<double, 4> cur_quat_{{0.0, 0.0, 0.0, 1.0}};
  rclcpp::Time src_stamp_{0, 0, RCL_ROS_TIME};
  std::deque<Sample> history_;
  int reset_epoch_{0};
  bool epoch_seen_{false};
  rclcpp::Time last_pub_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_pub_{false};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reset_epoch_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundOdomAdapter>());
  rclcpp::shutdown();
  return 0;
}
