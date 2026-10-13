// map_adapter —— 场景地图适配器（W4-c，spec R7）
//
// 发布 /agc/map（nav_msgs/OccupancyGrid，world 系，latched）：由地图几何（map.yaml）
// + 障碍 OBB 栅格化得到（0=free / 100=occupied，2D SAT 判据）。
// 另发布 /agc/map/version_hash（std_msgs/String，latched）与 /agc/map/valid（Bool，latched）。
//
// 分层语义（ADR-0008 W4 落点）：本节点承载 costmap 的 **static / obstacle** 语义（数据契约）；
//   **inflation / voxel** 层与 costmap 组装归 **W5**。W4 **不引入** nav2_map_server /
//   nav2_costmap_2d 运行时进程（仅对齐其数据/格式约定）。
//
// 障碍来源由 /agc/obstacles/source 选择：air_fusion（消费 /agc/obstacles）或 config
//   （用 obstacles.yaml 静态几何）。相同障碍去重，避免融合噪声引发重规划风暴。

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <agc_msgs/msg/obstacle_track_array.hpp>

#include "agc_perception/math_utils.hpp"

namespace
{

struct Obstacle
{
  std::string id;
  double x{0}, y{0}, z{0}, yaw{0}, sx{0}, sy{0}, sz{0};
};

// 轴对齐方格 [center ± half] 与 yaw 对齐 OBB 是否相交（2D 分离轴）
bool rectObbOverlap(
  double cx, double cy, double half, double ox, double oy, double hx, double hy, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double ux = c, uy = s;
  const double vx = -s, vy = c;
  const double dx = cx - ox, dy = cy - oy;
  const std::array<std::array<double, 2>, 4> axes{{{{1.0, 0.0}}, {{0.0, 1.0}}, {{ux, uy}}, {{vx, vy}}}};
  for (const auto & ax : axes) {
    const double r_aabb = half * (std::fabs(ax[0]) + std::fabs(ax[1]));
    const double r_obb = hx * std::fabs(ax[0] * ux + ax[1] * uy) + hy * std::fabs(ax[0] * vx + ax[1] * vy);
    if (std::fabs(dx * ax[0] + dy * ax[1]) > r_aabb + r_obb) {return false;}
  }
  return true;
}

// FNV-1a 64 位内容哈希（对 data 字节），输出 16 进制串
std::string hashBytes(const std::vector<int8_t> & data)
{
  uint64_t h = 1469598103934665603ULL;
  for (int8_t v : data) {
    h ^= static_cast<uint8_t>(v);
    h *= 1099511628211ULL;
  }
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
  return std::string(buf);
}

// 障碍集合是否实质相同：位置/尺寸按**栅格分辨率分箱**比较（栅格图上亚格噪声不应引发重建），
// 从而避免融合噪声引发的重规划抖动（spec R7）。
bool sameObstacles(const std::vector<Obstacle> & a, const std::vector<Obstacle> & b, double res)
{
  if (a.size() != b.size()) {return false;}
  const double r = (res > 1e-9) ? res : 1.0;
  auto bin = [r](double v) {return std::llround(v / r);};
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].id != b[i].id ||
      bin(a[i].x) != bin(b[i].x) || bin(a[i].y) != bin(b[i].y) ||
      bin(a[i].sx) != bin(b[i].sx) || bin(a[i].sy) != bin(b[i].sy) ||
      std::fabs(a[i].yaw - b[i].yaw) > 0.1) {return false;}
  }
  return true;
}

}  // namespace

class MapAdapter : public rclcpp::Node
{
public:
  MapAdapter()
  : Node("map_adapter")
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    map_file_ = declare_parameter<std::string>("map_file", "");
    obstacles_file_ = declare_parameter<std::string>("obstacles_file", "");
    obstacle_source_ = declare_parameter<std::string>("obstacles_source", "config");
    rate_hz_ = declare_parameter<double>("rate_hz", 2.0);
    map_topic_ = declare_parameter<std::string>("map_topic", "/agc/map");

    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, latched);
    hash_pub_ = create_publisher<std_msgs::msg::String>("/agc/map/version_hash", latched);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/agc/map/valid", latched);

    loadMapGeometry();
    if (obstacle_source_ == "air_fusion") {
      sub_obstacles_ = create_subscription<agc_msgs::msg::ObstacleTrackArray>(
        "/agc/obstacles", rclcpp::QoS(1),
        std::bind(&MapAdapter::onObstacles, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "map_adapter：obstacles/source=air_fusion，障碍格由 /agc/obstacles 提供");
    } else {
      loadStaticObstacles();
    }
    rebuild();

    const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1e-3));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MapAdapter::tick, this));
  }

private:
  void loadMapGeometry()
  {
    if (map_file_.empty()) {RCLCPP_ERROR(get_logger(), "map_file 未设置"); return;}
    YAML::Node r;
    try {
      r = YAML::LoadFile(map_file_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "读取 %s 失败: %s", map_file_.c_str(), e.what());
      return;
    }
    if (r["resolution"]) {res_ = r["resolution"].as<double>();}
    if (r["size_x"]) {sx_ = r["size_x"].as<double>();}
    if (r["size_y"]) {sy_ = r["size_y"].as<double>();}
    if (r["origin_x"]) {ox_ = r["origin_x"].as<double>();}
    if (r["origin_y"]) {oy_ = r["origin_y"].as<double>();}
  }

  void loadStaticObstacles()
  {
    if (obstacles_file_.empty()) {RCLCPP_WARN(get_logger(), "obstacles_file 未设置：地图仅含静态几何"); return;}
    YAML::Node r;
    try {
      r = YAML::LoadFile(obstacles_file_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "读取 %s 失败: %s", obstacles_file_.c_str(), e.what());
      return;
    }
    const YAML::Node list = r["obstacles"];
    if (!list || !list.IsSequence()) {return;}
    for (const auto & n : list) {
      Obstacle o;
      if (n["id"]) {o.id = n["id"].as<std::string>();}
      if (o.id.empty()) {continue;}
      if (n["x"]) {o.x = n["x"].as<double>();}
      if (n["y"]) {o.y = n["y"].as<double>();}
      if (n["z"]) {o.z = n["z"].as<double>();}
      if (n["yaw"]) {o.yaw = n["yaw"].as<double>();}
      if (n["size"] && n["size"].IsSequence() && n["size"].size() >= 3) {
        o.sx = n["size"][0].as<double>();
        o.sy = n["size"][1].as<double>();
        o.sz = n["size"][2].as<double>();
      }
      obstacles_.push_back(o);
    }
  }

  void onObstacles(const agc_msgs::msg::ObstacleTrackArray & msg)
  {
    std::vector<Obstacle> obs;
    for (const auto & t : msg.tracks) {
      Obstacle o;
      o.id = t.obstacle_id;
      o.x = t.pose.position.x; o.y = t.pose.position.y; o.z = t.pose.position.z;
      o.yaw = agc::yawFromQuat(t.pose.orientation);
      o.sx = t.size.x; o.sy = t.size.y; o.sz = t.size.z;
      obs.push_back(o);
    }
    if (sameObstacles(obs, obstacles_, res_)) {return;}  // 未实质变化（同格）：不重建
    obstacles_ = std::move(obs);
    rebuild();
  }

  void rebuild()
  {
    const int width = static_cast<int>(std::lround(sx_ / res_));
    const int height = static_cast<int>(std::lround(sy_ / res_));
    if (res_ <= 0.0 || width <= 0 || height <= 0) {
      RCLCPP_ERROR(get_logger(), "地图几何无效：res=%g size=(%g,%g)", res_, sx_, sy_);
      publishValid(false);
      return;
    }
    std::vector<int8_t> data(static_cast<size_t>(width) * height, 0);
    const double half = res_ * 0.5;
    for (const auto & o : obstacles_) {
      const double radius = std::hypot(o.sx * 0.5, o.sy * 0.5);
      const int ix0 = static_cast<int>(std::floor((o.x - radius - ox_) / res_));
      const int ix1 = static_cast<int>(std::ceil((o.x + radius - ox_) / res_));
      const int iy0 = static_cast<int>(std::floor((o.y - radius - oy_) / res_));
      const int iy1 = static_cast<int>(std::ceil((o.y + radius - oy_) / res_));
      for (int iy = std::max(0, iy0); iy < std::min(height, iy1 + 1); ++iy) {
        const double cy = oy_ + (iy + 0.5) * res_;
        for (int ix = std::max(0, ix0); ix < std::min(width, ix1 + 1); ++ix) {
          const double cx = ox_ + (ix + 0.5) * res_;
          if (rectObbOverlap(cx, cy, half, o.x, o.y, o.sx * 0.5, o.sy * 0.5, o.yaw)) {
            data[static_cast<size_t>(iy) * width + ix] = 100;
          }
        }
      }
    }
    grid_.resolution = res_;
    grid_.width = width;
    grid_.height = height;
    grid_.origin_x = ox_;
    grid_.origin_y = oy_;
    grid_.data = std::move(data);
    grid_.occupied = std::count(grid_.data.begin(), grid_.data.end(), static_cast<int8_t>(100));
    grid_.hash = hashBytes(grid_.data);
    grid_.valid = true;
    RCLCPP_INFO(get_logger(), "map 重建：%dx%d @ %.3f m, occupied=%ld, hash=%s（obstacles=%zu source=%s）",
      grid_.width, grid_.height, grid_.resolution, grid_.occupied, grid_.hash.c_str(),
      obstacles_.size(), obstacle_source_.c_str());
    publishMap();
  }

  void publishMap()
  {
    if (!grid_.valid) {publishValid(false); return;}
    const rclcpp::Time stamp = now();
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = world_frame_;
    msg.info.resolution = grid_.resolution;
    msg.info.width = static_cast<uint32_t>(grid_.width);
    msg.info.height = static_cast<uint32_t>(grid_.height);
    msg.info.origin.position.x = grid_.origin_x;
    msg.info.origin.position.y = grid_.origin_y;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data = grid_.data;
    pub_->publish(msg);
    if (grid_.hash != last_hash_) {
      last_hash_ = grid_.hash;
      std_msgs::msg::String h;
      h.data = grid_.hash;
      hash_pub_->publish(h);
    }
    publishValid(true);
  }

  void publishValid(bool ok)
  {
    std_msgs::msg::Bool m;
    m.data = ok;
    valid_pub_->publish(m);
  }

  void tick() {publishMap();}

  struct Grid
  {
    bool valid{false};
    std::vector<int8_t> data;
    int width{0}, height{0};
    double resolution{0.0}, origin_x{0.0}, origin_y{0.0};
    long occupied{0};
    std::string hash;
  };

  std::string world_frame_, map_file_, obstacles_file_, obstacle_source_, map_topic_;
  double rate_hz_{2.0};
  double res_{0.5}, sx_{0.0}, sy_{0.0}, ox_{0.0}, oy_{0.0};
  std::vector<Obstacle> obstacles_;
  Grid grid_;
  std::string last_hash_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hash_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Subscription<agc_msgs::msg::ObstacleTrackArray>::SharedPtr sub_obstacles_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapAdapter>());
  rclcpp::shutdown();
  return 0;
}
