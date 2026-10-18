// map_adapter —— 场景地图适配器（W4-c，spec R7；R-006 修订 2026-10-15）
//
// 发布 /agc/map（nav_msgs/OccupancyGrid，world 系，latched）：三态栅格
//   0 = free / 100 = occupied / -1 = unknown（对齐 Nav2 占据栅格约定）。
// 另发布 /agc/map/version_hash（std_msgs/String，latched）、/agc/map/update_seq（std_msgs/UInt64，latched）
//   与 /agc/map/valid（Bool，latched）。
//
// R-040 地图内容身份与单调更新序号（评审风险 R-040；见 ADR-0055）：
//   * **内容身份 hash**（/agc/map/version_hash 的内容）= FNV-1a(frame_id, 障碍源版本, 场景版本,
//     resolution, width, height, origin_x, origin_y, data) —— **不只** data 字节串：相同 data 在
//     不同几何 / 源版本下身份**不同**（消除「同 data 可代表不同地图」）。
//   * **单调更新序号** update_seq_（/agc/map/update_seq）：初始 0，**仅**在实质重建（栅格内容或身份
//     改变）时 +1；与内容 hash **分别**可查询（诊断，非 ABI）。
//   * **等价判定**按**真实 raster**：把候选有效障碍集合**实际栅格化**，与当前栅格**逐字节比较**；
//     相等即内容等价（不重建、不发布、序号不变），不同即采用 + 序号递增。真实 raster 相等是
//     **精确等价**、天然**不欠包**（边缘跨格必被捕获），**弃用**原 `sameObstacles` 的分辨率分箱 +
//     固定 0.1 yaw 容差（不保证边缘跨格后栅格相同）。
//
// R-006 三分语义（评审风险 R-006 地图知识保留）：
//   * 静态先验 static_prior_：**无论** obstacles_source 取何值，均从 obstacles.yaml 加载，
//     作为地图的 static 层（场景作者给定的静止障碍，与 agc_world.sdf 一致、真值派生 ADR-0020，
//     即 **非**实时传感器；假设一次 run 内场景静止家具已知且不变）。空/丢观测数组 **SHALL NOT**
//     清除它——立柱可见→遮挡→恢复时静态立柱永不被清除。
//   * 动态观测 live_：仅 obstacles_source==air_fusion 时由 /agc/obstacles 更新；每条 track 记
//     last_seen_s = Header.stamp（最后真实观测时刻）。动态轨按「最后观测年龄 > dynamic_max_age_s_」
//     过期 -> 其覆盖格标记 unknown（而非自由），并从 live_ 移除（不即时遗忘、也不永久驻留）。
//   * 未知 unknown_idx_：已过期动态轨曾覆盖、且当前无来源支撑的栅格单元（值 -1）。
//   可见（新鲜）时用实测几何，遮挡/过期时静态先验回退。
//
// /agc/map/valid = geometry_ok_ && knowledge_current()：
//   config 模式 -> 静态先验即来源，知识恒新鲜；air_fusion 模式 -> 收到过 /agc/obstacles 且
//   源静默不超过 source_max_age_s_。**SHALL NOT** 仅因几何可栅格化即 true。知识不新鲜时仍
//   照常发布静态层地图，只是 valid=false。
//
// 分层语义（ADR-0008 W4 落点）：本节点承载 costmap 的 **static / obstacle** 语义（数据契约）；
//   **inflation / voxel** 层与 costmap 组装归 **W5**。W4 **不引入** nav2_map_server /
//   nav2_costmap_2d 运行时进程（仅对齐其数据/格式约定）。相同障碍去重，避免融合噪声引发重规划风暴。

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>

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

// FNV-1a 64 位字节哈希（对 int8_t 字节序列），输出 16 进制串。
// R-040：保留为**底层字节哈希工具**——内容身份 hash 由它 + 几何/源版本元数据组装后喂入（见
//   contentIdentityHash），故本函数仍被复用。
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
    // R-040：场景 / 源版本（折入内容身份 hash）；单调更新序号诊断话题
    scenario_version_ = declare_parameter<std::string>("scenario_version", "");
    update_seq_topic_ = declare_parameter<std::string>("update_seq_topic", "/agc/map/update_seq");
    // R-006：动态轨最后观测年龄期限（s）与 /agc/obstacles 源静默期限（s）
    dynamic_max_age_s_ = declare_parameter<double>("dynamic_max_age_s", 1.5);
    source_max_age_s_ = declare_parameter<double>("source_max_age_s", 1.0);

    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, latched);
    hash_pub_ = create_publisher<std_msgs::msg::String>("/agc/map/version_hash", latched);
    seq_pub_ = create_publisher<std_msgs::msg::UInt64>(update_seq_topic_, latched);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/agc/map/valid", latched);

    loadMapGeometry();
    // 静态先验（static 层）：无论 obstacles_source 取何值都加载为已知静态事实（R-006）
    loadStaticObstacles();
    // R-038（见 ADR-0053）：地图几何 / 障碍几何 / 角度**启动期合法性校验**，非法即**拒绝启动**
    //   （**不**带非法配置运行；与既有 rebuild() 运行期守卫并存）。
    validateStartupOrThrow();
    if (obstacle_source_ == "air_fusion") {
      sub_obstacles_ = create_subscription<agc_msgs::msg::ObstacleTrackArray>(
        "/agc/obstacles", rclcpp::QoS(1),
        std::bind(&MapAdapter::onObstacles, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "map_adapter：obstacles/source=air_fusion，动态轨由 /agc/obstacles 提供，静态先验来自 %s",
        obstacles_file_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "map_adapter：obstacles/source=config，静态先验来自 %s",
        obstacles_file_.c_str());
    }
    rebuild();
    publishMap();

    const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1e-3));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MapAdapter::tick, this));
  }

private:
  struct LiveEntry
  {
    Obstacle geo;
    double last_seen_s{0.0};   // 最后真实观测时刻（track Header.stamp）
  };

  // R-038（见 ADR-0053）：启动期几何 / 角度合法性校验，非法**明确拒绝**（抛错使节点 configure 失败；
  //   **不**静默以默认值运行）。既有 rebuild() 的 `res_<=0 || width<=0 || height<=0` 运行期守卫保留。
  void validateStartupOrThrow() const
  {
    std::ostringstream bad;
    if (map_file_.empty()) {bad << "map_file 未设置 ";}
    if (!std::isfinite(res_) || res_ <= 0.0) {bad << "resolution=" << res_ << " ";}
    if (!std::isfinite(sx_) || sx_ <= 0.0) {bad << "size_x=" << sx_ << " ";}
    if (!std::isfinite(sy_) || sy_ <= 0.0) {bad << "size_y=" << sy_ << " ";}
    if (!std::isfinite(ox_)) {bad << "origin_x=" << ox_ << " ";}
    if (!std::isfinite(oy_)) {bad << "origin_y=" << oy_ << " ";}
    for (const auto & o : static_prior_) {
      if (!std::isfinite(o.x) || !std::isfinite(o.y)) {
        bad << "obstacle[" << o.id << "].x/y=" << o.x << "," << o.y << " ";
      }
      if (!std::isfinite(o.sx) || o.sx <= 0.0 || !std::isfinite(o.sy) || o.sy <= 0.0) {
        bad << "obstacle[" << o.id << "].size=" << o.sx << "," << o.sy << " ";
      }
      if (!std::isfinite(o.yaw)) {bad << "obstacle[" << o.id << "].yaw=" << o.yaw << " ";}
    }
    if (!bad.str().empty()) {
      throw std::runtime_error(
        "map_adapter 非法配置（拒绝启动，非静默以默认值运行）：" + bad.str() +
        "；要求 resolution/size_x/size_y > 0 且有限，origin 有限，障碍 x/y/size(>0)/yaw 有限。");
    }
  }

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

  // 静态先验（场景静止障碍，static 层）——与 air_fusion/config 模式无关，始终加载
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
      static_prior_.push_back(o);
      static_ids_.insert(o.id);
    }
  }

  // 动态观测：整集合不再整体替换；逐 track 记录最后观测时刻，静态 id 也记（可见时用实测几何）
  void onObstacles(const agc_msgs::msg::ObstacleTrackArray & msg)
  {
    last_msg_s_ = now().seconds();
    got_msg_ = true;
    for (const auto & t : msg.tracks) {
      Obstacle o;
      o.id = t.obstacle_id;
      o.x = t.pose.position.x; o.y = t.pose.position.y; o.z = t.pose.position.z;
      o.yaw = agc::yawFromQuat(t.pose.orientation);
      o.sx = t.size.x; o.sy = t.size.y; o.sz = t.size.z;
      LiveEntry e;
      e.geo = o;
      e.last_seen_s = rclcpp::Time(t.header.stamp).seconds();
      live_[o.id] = e;
      // 重新观测 -> 该 id 曾覆盖的未知单元恢复（不再是 unknown）
      if (static_ids_.count(o.id) == 0) {
        std::vector<size_t> cells;
        if (collectCells(o, cells)) {
          for (size_t idx : cells) {unknown_idx_.erase(idx);}
        }
      }
    }
    if (rebuild()) {publishMap();}
  }

  // 动态轨过期：non-static 且最后观测年龄 > dynamic_max_age_s_ -> 覆盖格标记 unknown 并移除
  void expireDynamic()
  {
    const double now_s = now().seconds();
    for (auto it = live_.begin(); it != live_.end(); ) {
      if (static_ids_.count(it->first) != 0) {++it; continue;}   // static 条目永不过期
      if (now_s - it->second.last_seen_s > dynamic_max_age_s_) {
        std::vector<size_t> cells;
        if (collectCells(it->second.geo, cells)) {
          for (size_t idx : cells) {unknown_idx_.insert(idx);}
        }
        it = live_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // 某障碍 OBB 覆盖的栅格单元索引（iy*width+ix）
  bool collectCells(const Obstacle & o, std::vector<size_t> & out) const
  {
    const int width = static_cast<int>(std::lround(sx_ / res_));
    const int height = static_cast<int>(std::lround(sy_ / res_));
    if (res_ <= 0.0 || width <= 0 || height <= 0) {return false;}
    const double half = res_ * 0.5;
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
          out.push_back(static_cast<size_t>(iy) * width + ix);
        }
      }
    }
    return true;
  }

  // 知识时效：源是否仍支撑当前障碍内容
  bool knowledge_current()
  {
    if (obstacle_source_ == "air_fusion") {
      return got_msg_ && (now().seconds() - last_msg_s_) <= source_max_age_s_;
    }
    return true;   // config：静态先验即来源
  }

  bool rebuild()
  {
    const int width = static_cast<int>(std::lround(sx_ / res_));
    const int height = static_cast<int>(std::lround(sy_ / res_));
    if (res_ <= 0.0 || width <= 0 || height <= 0) {
      geometry_ok_ = false;
      grid_.valid = false;
      RCLCPP_ERROR(get_logger(), "地图几何无效：res=%g size=(%g,%g)", res_, sx_, sy_);
      publishValid(false);
      return false;
    }
    geometry_ok_ = true;

    const double now_s = now().seconds();
    std::vector<Obstacle> eff;
    eff.reserve(static_prior_.size() + live_.size());
    // static 层：新鲜活观测覆盖先验几何，否则回退先验（遮挡/过期永不清除立柱）
    for (const auto & s : static_prior_) {
      auto it = live_.find(s.id);
      if (it != live_.end() && (now_s - it->second.last_seen_s) <= dynamic_max_age_s_) {
        eff.push_back(it->second.geo);
      } else {
        eff.push_back(s);
      }
    }
    // 动态层：新鲜且 id 不属于静态先验的条目
    for (const auto & kv : live_) {
      if (static_ids_.count(kv.first) != 0) {continue;}
      if ((now_s - kv.second.last_seen_s) <= dynamic_max_age_s_) {eff.push_back(kv.second.geo);}
    }

    // R-040：等价判定按**真实 raster**——把候选有效障碍集合**实际栅格化**，与当前栅格**逐字节比较**。
    //   相同 -> 内容（空间）精确等价，不重建、不发布、序号不变；不同 -> 采用 + 序号递增。
    //   真实 raster 相等天然**不欠包**（边缘跨格必被捕获），弃用原分箱 + 固定 yaw 容差判定。
    std::vector<int8_t> data(static_cast<size_t>(width) * height, 0);
    for (size_t idx : unknown_idx_) {
      if (idx < data.size()) {data[idx] = -1;}   // unknown
    }
    for (const auto & o : eff) {
      std::vector<size_t> cells;
      if (collectCells(o, cells)) {
        for (size_t idx : cells) {data[idx] = 100;}   // occupied 覆盖 unknown
      }
    }
    if (built_ && data == grid_.data) {return false;}   // 真实 raster 精确等价 -> 不重建

    grid_.resolution = res_;
    grid_.width = width;
    grid_.height = height;
    grid_.origin_x = ox_;
    grid_.origin_y = oy_;
    grid_.data = std::move(data);
    grid_.occupied = std::count(grid_.data.begin(), grid_.data.end(), static_cast<int8_t>(100));
    grid_.hash = contentIdentityHash(grid_.data, width, height);
    grid_.valid = true;
    built_ = true;
    ++update_seq_;   // R-040：实质重建 -> 单调更新序号 +1
    RCLCPP_INFO(get_logger(),
      "map 重建：%dx%d @ %.3f m, occupied=%ld, unknown=%zu, seq=%llu, hash=%s（effective=%zu static=%zu live=%zu source=%s）",
      grid_.width, grid_.height, grid_.resolution, grid_.occupied, unknown_idx_.size(),
      static_cast<unsigned long long>(update_seq_), grid_.hash.c_str(), eff.size(),
      static_prior_.size(), live_.size(), obstacle_source_.c_str());
    return true;
  }

  // R-040 内容身份 hash：把**几何元数据与源版本**与 data 字节一并折叠（**不只** data 字节串）。
  //   组装顺序（须与核查脚本一致，见 ADR-0055）：
  //     frame_id/NUL, obstacles_source/NUL, scenario_version/NUL,
  //     resolution(8B), width(8B), height(8B), origin_x(8B), origin_y(8B), data...
  //   double/uint64 按**本机 IEEE-754 binary64 / little-endian 内存字节**直接喂入（本项目全链同机
  //   同架构，字节序稳定；避免文本格式化的精度 / 舍入歧义）。
  std::string contentIdentityHash(
    const std::vector<int8_t> & data, int width, int height) const
  {
    std::vector<int8_t> blob;
    blob.reserve(data.size() + 128);
    auto appendStr = [&blob](const std::string & s) {
        for (char c : s) {blob.push_back(static_cast<int8_t>(c));}
        blob.push_back(0);
      };
    auto appendRaw = [&blob](const void * p, size_t n) {
        const uint8_t * b = static_cast<const uint8_t *>(p);
        for (size_t i = 0; i < n; ++i) {blob.push_back(static_cast<int8_t>(b[i]));}
      };
    auto appendDouble = [&appendRaw](double v) {
        uint8_t b[8];
        std::memcpy(b, &v, sizeof(b));
        appendRaw(b, sizeof(b));
      };
    appendStr(world_frame_);       // frame_id
    appendStr(obstacle_source_);   // 障碍源版本（config / air_fusion）
    appendStr(scenario_version_);  // 场景版本（run config）
    appendDouble(res_);            // resolution
    const uint64_t w = static_cast<uint64_t>(width), h = static_cast<uint64_t>(height);
    appendRaw(&w, sizeof(w));
    appendRaw(&h, sizeof(h));
    appendDouble(ox_);             // origin_x
    appendDouble(oy_);             // origin_y
    for (int8_t v : data) {blob.push_back(v);}   // data 字节
    return hashBytes(blob);        // 复用底层字节哈希工具
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
    // R-040：单调更新序号（latched 诊断；与内容 hash 分别可查询）
    std_msgs::msg::UInt64 seq;
    seq.data = update_seq_;
    seq_pub_->publish(seq);
    publishValid(geometry_ok_ && knowledge_current());   // R-006：valid = 几何 && 知识时效
  }

  void publishValid(bool ok)
  {
    std_msgs::msg::Bool m;
    m.data = ok;
    valid_pub_->publish(m);
  }

  // 每拍：先做过期判定（即便无新消息，动态轨也随时间过期 -> 触发重建），再发布
  void tick()
  {
    expireDynamic();
    rebuild();
    publishMap();
  }

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
  std::string scenario_version_, update_seq_topic_;   // R-040：场景/源版本 与 单调更新序号话题
  double rate_hz_{2.0};
  double dynamic_max_age_s_{1.5}, source_max_age_s_{1.0};
  double res_{0.5}, sx_{0.0}, sy_{0.0}, ox_{0.0}, oy_{0.0};

  std::vector<Obstacle> static_prior_;             // 静态先验（static 层）
  std::set<std::string> static_ids_;
  std::map<std::string, LiveEntry> live_;          // 动态观测（含静态 id 的实测）
  std::set<size_t> unknown_idx_;                   // 未知单元（过期动态轨覆盖格）
  bool built_{false};
  bool geometry_ok_{false};
  bool got_msg_{false};
  double last_msg_s_{-1e9};
  uint64_t update_seq_{0};                         // R-040：单调更新序号（实质重建 +1）
  Grid grid_;
  std::string last_hash_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hash_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr seq_pub_;
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
