// target_sim_source —— 目标真值源（spec R8，W3 占位）
//
// 目标源由契约参数 `/agc/target/source` 选择；W3 接 **sim 真值源**（读 gz 经桥出的
// 目标位姿），发布契约已有 `TargetTrackArray`（对齐旧 `/agc/targets`）。W4 到位后切感知源。
// 非长跑控制节点，故为普通节点（无需 lifecycle）。

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <agc_msgs/msg/target_track.hpp>
#include <agc_msgs/msg/target_track_array.hpp>

class TargetSimSource : public rclcpp::Node
{
public:
  TargetSimSource()
  : Node("target_sim_source")
  {
    source_ = declare_parameter<std::string>("source", "sim");
    target_id_ = declare_parameter<std::string>("target_id", "intruder_uav");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/agc/sim/target_pose");
    out_topic_ = declare_parameter<std::string>("target_topic", "/agc/targets");
    confidence_ = declare_parameter<double>("confidence", 1.0);

    pub_ = create_publisher<agc_msgs::msg::TargetTrackArray>(
      out_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    if (source_ == "sim") {
      sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {onPose(*m);});
      RCLCPP_INFO(get_logger(), "目标源 = sim 真值（%s → %s）", pose_topic_.c_str(),
        out_topic_.c_str());
    } else {
      // 感知源属 W4；此处不发布，避免与 W4 抢同一话题
      RCLCPP_INFO(get_logger(), "目标源 = %s（非 sim，本节点不发布；W4 接管）", source_.c_str());
    }
  }

private:
  void onPose(const geometry_msgs::msg::PoseStamped & pose)
  {
    agc_msgs::msg::TargetTrackArray arr;
    arr.header.stamp = now();
    arr.header.frame_id = world_frame_;

    agc_msgs::msg::TargetTrack track;
    track.header = arr.header;
    track.target_id = target_id_;
    track.pose.pose = pose.pose;
    track.confidence = static_cast<float>(confidence_);
    track.source = "sim_truth";
    arr.tracks.push_back(track);

    pub_->publish(arr);
  }

  std::string source_, target_id_, world_frame_, pose_topic_, out_topic_;
  double confidence_{1.0};
  rclcpp::Publisher<agc_msgs::msg::TargetTrackArray>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetSimSource>());
  rclcpp::shutdown();
  return 0;
}
