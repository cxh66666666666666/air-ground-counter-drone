// gt_bypass —— 世界真值旁路（W4-a，spec R1）
//
// 直读 gz `/world/<world>/dynamic_pose/info`（`gz.msgs.Pose_V`），归一为 world 系真值，
// 按实体发布 `/agc/gt/<entity>`（`geometry_msgs/PoseStamped`）。
//
// 设计要点：
//   * **不经 ros_gz_bridge**：本机桥(1.0.24) 的 `Pose_V→TFMessage` 不写 frame 名，
//     无法作为带名真值使用；故以专用节点经 gz-transport 直读（原逐实体 PosePublisher
//     桥已在 W4 退役）。
//   * **不发 TF**：真值为私有数据，不进协同层帧树（契约 #namespace 禁裸 base_link）。
//   * **限频发布**：动态真值话题按仿真步进（~250 Hz）产生，须限频（默认 30 Hz）以避免
//     灌满链路（旧树实测：真值链路 250 Hz 会拖垮同机 PX4 EKF）。
//
// gz 回调在 transport 线程执行，与 ROS 定时器线程并发：共享缓冲以互斥量保护。

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class GtBypass : public rclcpp::Node
{
public:
  GtBypass()
  : Node("gt_bypass")
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    world_name_ = declare_parameter<std::string>("world_name", "agc_world");
    gz_topic_ = declare_parameter<std::string>(
      "gz_topic", "/world/" + world_name_ + "/dynamic_pose/info");
    out_prefix_ = declare_parameter<std::string>("out_topic_prefix", "/agc/gt/");
    entities_ = declare_parameter<std::vector<std::string>>(
      "entities", std::vector<std::string>{"agc_car", "intruder_uav"});
    rate_hz_ = declare_parameter<double>("rate_hz", 30.0);

    for (const auto & e : entities_) {
      pubs_[e] = create_publisher<geometry_msgs::msg::PoseStamped>(
        out_prefix_ + e, rclcpp::QoS(10));
    }

    subscribed_ = gz_node_.Subscribe(
      gz_topic_, &GtBypass::onPoseV, this);
    if (!subscribed_) {
      RCLCPP_ERROR(
        get_logger(), "gz 真值订阅失败：%s（gz-transport）", gz_topic_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "gt_bypass：gz %s → %zu 实体（%s**），%.1f Hz",
        gz_topic_.c_str(), entities_.size(), out_prefix_.c_str(), rate_hz_);
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1e-3));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GtBypass::tick, this));
  }

private:
  void onPoseV(const gz::msgs::Pose_V & msg)
  {
    const rclcpp::Time stamp = gzStampToRos(msg.header());
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto & p : msg.pose()) {
      auto it = pubs_.find(p.name());
      if (it == pubs_.end()) {continue;}  // 只缓存被订阅的实体
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = stamp;
      ps.header.frame_id = world_frame_;
      ps.pose.position.x = p.position().x();
      ps.pose.position.y = p.position().y();
      ps.pose.position.z = p.position().z();
      ps.pose.orientation.x = p.orientation().x();
      ps.pose.orientation.y = p.orientation().y();
      ps.pose.orientation.z = p.orientation().z();
      ps.pose.orientation.w = p.orientation().w();
      latest_[p.name()] = ps;
    }
  }

  rclcpp::Time gzStampToRos(const gz::msgs::Header & h) const
  {
    const int64_t sec = h.stamp().sec();
    const uint32_t nsec = h.stamp().nsec();
    if (sec == 0 && nsec == 0) {return now();}
    return rclcpp::Time(sec, nsec, get_clock()->get_clock_type());
  }

  void tick()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto & kv : latest_) {
      auto pub_it = pubs_.find(kv.first);
      if (pub_it != pubs_.end()) {pub_it->second->publish(kv.second);}
    }
  }

  std::string world_frame_, world_name_, gz_topic_, out_prefix_;
  std::vector<std::string> entities_;
  double rate_hz_{30.0};
  bool subscribed_{false};

  gz::transport::Node gz_node_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
  pubs_;
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> latest_;
  std::mutex mtx_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GtBypass>());
  rclcpp::shutdown();
  return 0;
}