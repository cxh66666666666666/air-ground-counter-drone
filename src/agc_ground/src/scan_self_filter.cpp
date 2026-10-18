// scan_self_filter —— R-007 自体过滤（2026-10-15，评审风险 R-007）
//
// 激光（`/agc/ground/scan`，`sensor_msgs/LaserScan`，360 束/360°、range [0.2, 20]，车顶 z=0.48）
// 在水平扫描平面上会返回**车体自身**（如车头/外伸），使包围式旋转 / 后向 zone 误触发。
// 本节点逐束判断：把落在**车体矩形**（`|x| <= half_len+margin` 且 `|y| <= half_wid+margin`）内的
// 返回置为 `+inf`（自滤），其余（含真实障碍、非有限值）原样保留。
//
// 坐标系：激光无 yaw 偏移（`car/lidar_link` @ z=0.48，无旋转），传感器 xy 面与 `car/base_link`
//   xy 面重合、轴对齐 → 逐束 `point = (r·cosθ, r·sinθ)` 判断，**无需 TF**。
//
// 输入 `/agc/ground/scan`（桥出）→ 输出 `/agc/ground/scan_filtered`（同类型、同 header 时序/frame），
// 安全层（collision_monitor）与 costmap obstacle_layer 消费自滤后的扫描（契约 #params 单一注入）。
//
// 语言：C++（与 agc_ground 其余节点一致，ADR-0011）。节点**不写文件**（spec R13）。
//
// 参数（由 config/ground.yaml 单一注入，节点不得覆写）：
//   in_topic / out_topic、footprint_half_length_m(0.5037)、footprint_half_width_m(0.28545)、
//   self_filter_margin_m(0.10)。

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanSelfFilter : public rclcpp::Node
{
public:
  ScanSelfFilter()
  : Node("scan_self_filter")
  {
    in_topic_ = declare_parameter<std::string>("in_topic", "/agc/ground/scan");
    out_topic_ = declare_parameter<std::string>("out_topic", "/agc/ground/scan_filtered");
    const double half_len = declare_parameter<double>("footprint_half_length_m", 0.5037);
    const double half_wid = declare_parameter<double>("footprint_half_width_m", 0.28545);
    const double margin = declare_parameter<double>("self_filter_margin_m", 0.10);
    half_len_eff_ = half_len + margin;
    half_wid_eff_ = half_wid + margin;

    // 传感器数据：SENSOR_DATA（best-effort）QoS，对齐 Nav2 scan 源与 ros_gz 桥（ADR-0008 吸收点）
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(out_topic_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      in_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanSelfFilter::onScan, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "scan_self_filter：%s → %s（自滤矩形 |x|<=%.4f m, |y|<=%.4f m）",
      in_topic_.c_str(), out_topic_.c_str(), half_len_eff_, half_wid_eff_);
  }

private:
  void onScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    sensor_msgs::msg::LaserScan out = *msg;  // 保留 header 时序/frame 与全部字段
    const double angle_min = out.angle_min;
    const double angle_inc = out.angle_increment;
    for (size_t i = 0; i < out.ranges.size(); ++i) {
      const float r = out.ranges[i];
      if (!std::isfinite(r)) {continue;}
      const double th = angle_min + static_cast<double>(i) * angle_inc;
      const double x = static_cast<double>(r) * std::cos(th);
      const double y = static_cast<double>(r) * std::sin(th);
      if (std::fabs(x) <= half_len_eff_ && std::fabs(y) <= half_wid_eff_) {
        out.ranges[i] = std::numeric_limits<float>::infinity();
      }
    }
    pub_->publish(out);
  }

  std::string in_topic_, out_topic_;
  double half_len_eff_{0.0}, half_wid_eff_{0.0};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanSelfFilter>());
  rclcpp::shutdown();
  return 0;
}
