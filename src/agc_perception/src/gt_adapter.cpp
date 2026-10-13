// gt_adapter —— 真值目标适配器（W4-a，spec R3）
//
// 消费 W4 真值旁路 `/agc/gt/<target_model>`（world 系 PoseStamped），产出契约
// `/agc/targets`（`agc_msgs/TargetTrackArray`）：
//   * `source=gt_adapter`、`confidence=1.0`、无噪声基线（协方差取 std²）；
//   * pose/twist 均在 world 系；twist 由位置短窗差分得 world 速度，不做机体系旋转
//     （与 Odometry 的 child-frame twist 约定不同）；
//   * 数组 header.stamp = 发布时刻；track.header.stamp = 最后一次真实观测时刻，
//     本拍无新观测则 tracks 为空（绝不刷新旧观测时间）。
//
// 目标源由契约参数 `/agc/target/source` 在 run 前固定（launch 单一设置）：仅当本节点为
// 选中源（`gt_adapter`）时才发布 `/agc/targets`，避免与 `air_fusion` 双发布者。

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <agc_msgs/msg/target_track.hpp>
#include <agc_msgs/msg/target_track_array.hpp>

#include "agc_perception/math_utils.hpp"

namespace
{
const size_t kHistoryMax = 500;
const char * const kSource = "gt_adapter";

void diagCov(double sigma, std::array<double, 36> & c)
{
  c.fill(0.0);
  const double s2 = sigma * sigma;
  c[0] = c[7] = c[14] = s2;
}

struct Sample
{
  double t, x, y, z, yaw;
};
}  // namespace

class GtAdapter : public rclcpp::Node
{
public:
  GtAdapter()
  : Node("gt_adapter")
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    target_model_ = declare_parameter<std::string>("target_model", "intruder_uav");
    target_id_ = declare_parameter<std::string>("target_id", "intruder_uav");
    gt_topic_ = declare_parameter<std::string>("gt_topic", "/agc/gt/intruder_uav");
    target_topic_ = declare_parameter<std::string>("target_topic", "/agc/targets");
    target_source_ = declare_parameter<std::string>("target_source", kSource);
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);
    pose_std_m_ = declare_parameter<double>("pose_std_m", 0.01);
    twist_std_m_ = declare_parameter<double>("twist_std_m", 0.05);
    diff_window_s_ = declare_parameter<double>("diff_window_s", 0.15);

    if (target_source_ == kSource) {
      pub_ = create_publisher<agc_msgs::msg::TargetTrackArray>(
        target_topic_, rclcpp::QoS(10));
      has_pub_ = true;
    } else {
      RCLCPP_WARN(
        get_logger(), "gt_adapter：/agc/target/source=%s，本节点不发布 %s",
        target_source_.c_str(), target_topic_.c_str());
    }

    gt_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      gt_topic_, 10, std::bind(&GtAdapter::onGt, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1e-3));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GtAdapter::tick, this));

    RCLCPP_INFO(
      get_logger(), "gt_adapter：source=%s；真值 %s → %s；%.1f Hz",
      target_source_.c_str(), gt_topic_.c_str(), target_topic_.c_str(), rate_hz_);
  }

private:
  void onGt(const geometry_msgs::msg::PoseStamped & msg)
  {
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    history_.push_back(
      Sample{t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
        agc::yawFromQuat(msg.pose.orientation)});
    if (history_.size() > kHistoryMax) {history_.pop_front();}
    last_pos_ = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
    last_quat_ = {msg.pose.orientation.x, msg.pose.orientation.y,
      msg.pose.orientation.z, msg.pose.orientation.w};
    last_obs_stamp_ = rclcpp::Time(msg.header.stamp);
    new_obs_ = true;
    has_pose_ = true;
  }

  void velocity(std::array<double, 3> & v, double & wz) const
  {
    v = {{0.0, 0.0, 0.0}};
    wz = 0.0;
    if (history_.size() < 2) {return;}
    const Sample & a = history_.back();
    Sample b = history_.front();
    for (const Sample & e : history_) {
      if (a.t - e.t <= diff_window_s_) {b = e; break;}
    }
    const double dt = a.t - b.t;
    if (dt <= 1e-6) {return;}
    v = {{(a.x - b.x) / dt, (a.y - b.y) / dt, (a.z - b.z) / dt}};
    wz = agc::normalizeAngle(a.yaw - b.yaw) / dt;
  }

  agc_msgs::msg::TargetTrack buildTrack()
  {
    agc_msgs::msg::TargetTrack tr;
    tr.header.stamp = last_obs_stamp_;  // 最后一次真实观测时刻
    tr.header.frame_id = world_frame_;
    tr.target_id = target_id_;
    tr.pose.pose.position.x = last_pos_[0];
    tr.pose.pose.position.y = last_pos_[1];
    tr.pose.pose.position.z = last_pos_[2];
    tr.pose.pose.orientation.x = last_quat_[0];
    tr.pose.pose.orientation.y = last_quat_[1];
    tr.pose.pose.orientation.z = last_quat_[2];
    tr.pose.pose.orientation.w = last_quat_[3];
    diagCov(pose_std_m_, tr.pose.covariance);
    std::array<double, 3> v;
    double wz = 0.0;
    velocity(v, wz);
    tr.twist.twist.linear.x = v[0];
    tr.twist.twist.linear.y = v[1];
    tr.twist.twist.linear.z = v[2];
    tr.twist.twist.angular.x = 0.0;
    tr.twist.twist.angular.y = 0.0;
    tr.twist.twist.angular.z = wz;
    diagCov(twist_std_m_, tr.twist.covariance);
    tr.confidence = 1.0f;
    tr.source = kSource;
    return tr;
  }

  void tick()
  {
    agc_msgs::msg::TargetTrackArray arr;
    arr.header.stamp = now();
    arr.header.frame_id = world_frame_;
    if (new_obs_ && has_pose_) {arr.tracks.push_back(buildTrack());}
    new_obs_ = false;  // 本拍观测已消费：下拍若无新消息则发空数组
    if (has_pub_) {pub_->publish(arr);}
  }

  std::string world_frame_, target_model_, target_id_, gt_topic_, target_topic_, target_source_;
  double rate_hz_{10.0}, pose_std_m_{0.01}, twist_std_m_{0.05}, diff_window_s_{0.15};

  bool has_pub_{false};
  std::deque<Sample> history_;
  std::array<double, 3> last_pos_{{0.0, 0.0, 0.0}};
  std::array<double, 4> last_quat_{{0.0, 0.0, 0.0, 1.0}};
  rclcpp::Time last_obs_stamp_;
  bool has_pose_{false}, new_obs_{false};

  rclcpp::Publisher<agc_msgs::msg::TargetTrackArray>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GtAdapter>());
  rclcpp::shutdown();
  return 0;
}