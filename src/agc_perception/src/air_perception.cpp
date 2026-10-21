// air_perception —— 真值派生的感知前端（W4-b，spec R4）
//
// 把世界真值（目标 /agc/gt/<target> + 己方无人机位姿 /agc/air/odom + 静态障碍真值）
// 按多个「传感器通道」转成带来源/坐标系/时间戳/置信度的观测，发布 /agc/air/obs
// （agc_msgs/AirObservationArray）。
//
// 诚实口径：这是**真值派生的感知仿真**，不是真实视觉/雷达识别（ADR-0020）。
//
// 关键口径（对齐旧树行为）：
// 1. 每通道独立 RNG 噪声 + 独立延迟/丢包/偏置；不共享随机流。
// 2. **三个时刻**（spec R-004）：
//      * 源采样时刻 = 入站消息 Header.stamp（目标真值 / 无人机 odom）；
//      * 通道采样时刻 = tick（本通道加噪定格的时刻）；
//      * 交付（发布）时刻 = latency_s 释放后的发布时刻。
//    TargetObservation.header.stamp 承载**源采样时刻**；延迟只推迟交付、不改源采样时刻。
// 3. **源寿命（spec R-004）**：目标真值与无人机 odom 分别按入站 Header.stamp 记源采样时刻；
//    年龄（仿真 now() − 源采样时刻）> source_timeout_s 即视为该源失效——目标通道本拍
//    SHALL NOT 用缓存位置重打「当前」时间戳（不发 TargetObservation）；odom 失效时
//    gatePoint 不再以陈旧位姿放行（目标/障碍观测都不产生）。
// 4. confidence 由噪声模型解析给出（Rayleigh 径向覆盖，见 confidence.hpp）。
// 5. 本节点**不写文件**（spec R10；run 级日志归 W6 agc_run_logger）。

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <agc_msgs/msg/air_observation_array.hpp>
#include <agc_msgs/msg/obstacle_observation.hpp>
#include <agc_msgs/msg/sensor_observation.hpp>
#include <agc_msgs/msg/target_observation.hpp>

#include "agc_perception/confidence.hpp"
#include "agc_perception/math_utils.hpp"

namespace
{

// 单通道配置（来自 sensors_file）
struct SensorCfg
{
  std::string id;
  double sigma_xy{0.10}, sigma_z{0.12};
  double latency_s{0.0}, drop_prob{0.0};
  double confidence{1.0};
  std::array<double, 3> boresight{{1.0, 0.0, 0.0}};
  double hfov_rad{M_PI / 2.0}, vfov_rad{M_PI / 3.0};
  double range_min{0.30}, range_max{12.0};
  double corr_xy{0.0};
  double noise_tau_s{0.0};
  std::array<double, 3> bias_m{{0.0, 0.0, 0.0}};
  double drop_burst_prob{0.0}, drop_burst_mean_s{0.5}, drop_burst_mean_gap_s{5.0};
};

struct Obstacle
{
  std::string id;
  double x{0}, y{0}, z{0}, yaw{0}, sx{0}, sy{0}, sz{0};
};

// 一次采样（已加噪），延迟释放后转成观测消息
struct ObsSample
{
  double t_obs{0.0};          // 通道采样时刻（tick）
  double t_target_src{0.0};   // 目标源采样时刻（入站 Header.stamp；spec R-004）
  bool has_target{false};
  double tx{0}, ty{0}, tz{0}, tyaw{0};
  uint32_t tseq{0};
  struct Ob
  {
    std::string id;
    double x{0}, y{0}, z{0}, yaw{0}, sx{0}, sy{0}, sz{0};
    uint32_t seq{0};
  };
  std::vector<Ob> obs;
};

struct Pending
{
  double release_t{0.0};
  ObsSample s;
};

// 协方差：对角 x/y/z + x-y 相关项（rho=0 时退化为纯对角）
void cov3(double sxy, double sz, double rho, std::array<double, 36> & c)
{
  c.fill(0.0);
  const double v = sxy * sxy;
  c[0] = c[7] = v;
  c[1] = c[6] = rho * v;
  c[14] = sz * sz;
}

// 线段 p0→p1 与障碍 OBB（按 pad 外扩）是否相交（局部 slab/AABB 测试）
bool segHitsObb(
  const std::array<double, 3> & p0, const std::array<double, 3> & p1,
  const Obstacle & o, double pad)
{
  const double cy = std::cos(-o.yaw), sy = std::sin(-o.yaw);
  auto toLocal = [&](const std::array<double, 3> & p) -> std::array<double, 3> {
      const double dx = p[0] - o.x, dy = p[1] - o.y, dz = p[2] - o.z;
      return {{cy * dx - sy * dy, sy * dx + cy * dy, dz}};
    };
  const std::array<double, 3> a = toLocal(p0), b = toLocal(p1);
  const std::array<double, 3> h{{o.sx * 0.5 + pad, o.sy * 0.5 + pad, o.sz * 0.5 + pad}};
  double tmin = 0.0, tmax = 1.0;
  for (int i = 0; i < 3; ++i) {
    const double d = b[i] - a[i];
    if (std::fabs(d) < 1e-9) {
      if (a[i] < -h[i] || a[i] > h[i]) {return false;}
    } else {
      double t1 = (-h[i] - a[i]) / d, t2 = (h[i] - a[i]) / d;
      if (t1 > t2) {std::swap(t1, t2);}
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);
      if (tmin > tmax) {return false;}
    }
  }
  return tmin <= tmax;
}

}  // namespace

class AirPerception : public rclcpp::Node
{
public:
  AirPerception()
  : Node("air_perception")
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    target_id_ = declare_parameter<std::string>("target_id", "intruder_uav");
    target_gt_topic_ = declare_parameter<std::string>("target_gt_topic", "/agc/gt/intruder_uav");
    drone_odom_topic_ = declare_parameter<std::string>("drone_odom_topic", "/agc/air/odom");
    obs_topic_ = declare_parameter<std::string>("obs_topic", "/agc/air/obs");
    sensors_file_ = declare_parameter<std::string>("sensors_file", "");
    obstacles_file_ = declare_parameter<std::string>("obstacles_file", "");
    obs_hz_ = declare_parameter<double>("obs_hz", 20.0);
    tol_m_ = declare_parameter<double>("confidence_tolerance_m", 0.25);
    seed_ = declare_parameter<std::string>("seed", "unseeded");
    // 感知源寿命（spec R-004）：仅 declare，由 launch 单一注入，节点不得覆写（契约 [参数入口]）
    source_timeout_s_ = declare_parameter<double>("source_timeout_s", 0.3);

    loadSensors();
    loadObstacles();

    pub_ = create_publisher<agc_msgs::msg::AirObservationArray>(
      obs_topic_, rclcpp::QoS(10));
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_gt_topic_, 10, std::bind(&AirPerception::onTargetGt, this, std::placeholders::_1));
    drone_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      drone_odom_topic_, 10, std::bind(&AirPerception::onDroneOdom, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(obs_hz_, 1e-3));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AirPerception::tick, this));

    RCLCPP_INFO(
      get_logger(), "air_perception 就绪：通道=%zu 障碍=%zu obs_hz=%.1f tol=%.3f m 源寿命=%.3f s 遮挡=%s（真值派生感知仿真）",
      sensors_.size(), obstacles_.size(), obs_hz_, tol_m_, source_timeout_s_,
      occlusion_enabled_ ? "on" : "off");
  }

private:
  void loadSensors()
  {
    if (sensors_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "sensors_file 未设置，无法读取通道配置");
      return;
    }
    YAML::Node root;
    try {
      root = YAML::LoadFile(sensors_file_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "读取 %s 失败: %s", sensors_file_.c_str(), e.what());
      return;
    }
    const YAML::Node list = root["sensors"];
    if (!list || !list.IsSequence()) {
      RCLCPP_ERROR(get_logger(), "%s 缺 sensors 序列", sensors_file_.c_str());
      return;
    }
    // R-038（见 ADR-0053）：seed 启动期合法性校验 —— 显式 `unseeded` 保留默认基种子；
    //   其余必须为**整数字面量**。**删除**旧的静默 `strtol` 兜底（非数值输入曾静默退回 base_seed=1）。
    long base_seed = 1;
    if (!seed_.empty() && seed_ != "unseeded") {
      char * endp = nullptr;
      const long v = std::strtol(seed_.c_str(), &endp, 10);
      if (endp == seed_.c_str() || *endp != '\0') {
        throw std::runtime_error(
          "air_perception 非法 seed（拒绝启动）：seed=\"" + seed_ +
          "\"（要求整数字面量或 \"unseeded\"；不静默退回默认基种子）。");
      }
      base_seed = v;
    }
    size_t idx = 0;
    for (const auto & n : list) {
      SensorCfg c;
      if (n["id"]) {c.id = n["id"].as<std::string>();}
      if (c.id.empty()) {++idx; continue;}
      if (n["sigma_xy_m"]) {c.sigma_xy = n["sigma_xy_m"].as<double>();}
      if (n["sigma_z_m"]) {c.sigma_z = n["sigma_z_m"].as<double>();}
      if (n["latency_s"]) {c.latency_s = n["latency_s"].as<double>();}
      if (n["drop_prob"]) {c.drop_prob = n["drop_prob"].as<double>();}
      if (n["boresight_body"] && n["boresight_body"].IsSequence() && n["boresight_body"].size() >= 3) {
        c.boresight = {{n["boresight_body"][0].as<double>(), n["boresight_body"][1].as<double>(),
          n["boresight_body"][2].as<double>()}};
      }
      if (n["hfov_deg"]) {c.hfov_rad = n["hfov_deg"].as<double>() * M_PI / 180.0;}
      if (n["vfov_deg"]) {c.vfov_rad = n["vfov_deg"].as<double>() * M_PI / 180.0;}
      if (n["range_min_m"]) {c.range_min = n["range_min_m"].as<double>();}
      if (n["range_max_m"]) {c.range_max = n["range_max_m"].as<double>();}
      if (n["corr_xy"]) {c.corr_xy = n["corr_xy"].as<double>();}
      if (n["noise_tau_s"]) {c.noise_tau_s = n["noise_tau_s"].as<double>();}
      if (n["bias_m"] && n["bias_m"].IsSequence() && n["bias_m"].size() >= 3) {
        c.bias_m = {{n["bias_m"][0].as<double>(), n["bias_m"][1].as<double>(),
          n["bias_m"][2].as<double>()}};
      }
      if (n["drop_burst_prob"]) {c.drop_burst_prob = n["drop_burst_prob"].as<double>();}
      if (n["drop_burst_mean_s"]) {c.drop_burst_mean_s = n["drop_burst_mean_s"].as<double>();}
      if (n["drop_burst_mean_gap_s"]) {c.drop_burst_mean_gap_s = n["drop_burst_mean_gap_s"].as<double>();}
      c.confidence = agc::coverageConfidence(c.sigma_xy, tol_m_);
      rng_.emplace_back(static_cast<unsigned>(base_seed + 101 * (idx + 1)));
      burst_state_.push_back(false);
      has_prev_t_.push_back(false);
      last_t_.push_back(0.0);
      a_prev_target_.push_back({{0.0, 0.0, 0.0}});
      a_prev_obs_.push_back({{0.0, 0.0, 0.0}});
      sensors_.push_back(c);
      pending_.emplace_back();
      tseq_.push_back(0);
      oseq_.push_back(0);
      ++idx;
    }
    bool oc_en = false;
    double oc_pad = 0.0;
    const YAML::Node pm = root["perception_model"];
    if (pm && pm["occlusion"]) {
      const YAML::Node oc = pm["occlusion"];
      if (oc["enabled"]) {oc_en = oc["enabled"].as<bool>();}
      if (oc["pad_m"]) {oc_pad = oc["pad_m"].as<double>();}
    }
    occlusion_enabled_ = oc_en;
    occlusion_pad_m_ = oc_pad;
  }

  void loadObstacles()
  {
    if (obstacles_file_.empty()) {
      RCLCPP_WARN(get_logger(), "obstacles_file 未设置：不生成障碍观测");
      return;
    }
    YAML::Node root;
    try {
      root = YAML::LoadFile(obstacles_file_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "读取 %s 失败: %s", obstacles_file_.c_str(), e.what());
      return;
    }
    const YAML::Node list = root["obstacles"];
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

  // W7-b 取证：入站 `Header.stamp` 须为**有效非负**仿真时刻。`sec < 0` 时
  // `rclcpp::Time(stamp)` 构造**本身即抛** "cannot store a negative time point" → 整节点崩溃，
  // 故**先查原始 sec/nanosec**（在构造之前），无效即**拒绝该样本**并如实一次性告警（含话题名）。
  bool stampValidOnce(const builtin_interfaces::msg::Time & st, const char * topic, bool * warned)
  {
    if (st.sec >= 0) {return true;}
    if (warned && !*warned) {
      *warned = true;
      RCLCPP_WARN(get_logger(),
        "入站 %s 的 Header.stamp **无效**（sec=%d nsec=%u < 0）→ 拒绝该样本（源不置就绪）；"
        "R-004 源采样时刻须为有效非负仿真时刻", topic, st.sec, st.nanosec);
    }
    return false;
  }

  void onTargetGt(const geometry_msgs::msg::PoseStamped & m)
  {
    // 源采样时刻 = 入站消息 Header.stamp（非接受/发布时刻）；旧包晚到不得刷新增龄（spec R-004）
    if (!stampValidOnce(m.header.stamp, "目标真值 /agc/gt/*", &warned_bad_target_stamp_)) {return;}
    const double s = rclcpp::Time(m.header.stamp).seconds();
    if (!target_seen_ || s >= target_src_s_) {
      truth_ = {m.pose.position.x, m.pose.position.y, m.pose.position.z};
      truth_yaw_ = agc::yawFromQuat(m.pose.orientation);
      target_src_s_ = s;
    }
    target_seen_ = true;
  }

  void onDroneOdom(const nav_msgs::msg::Odometry & m)
  {
    // 源采样时刻 = 入站消息 Header.stamp（非接受/发布时刻）；spec R-004
    if (!stampValidOnce(m.header.stamp, "无人机 odom /agc/air/odom", &warned_bad_drone_stamp_)) {return;}
    const double s = rclcpp::Time(m.header.stamp).seconds();
    if (!drone_seen_ || s >= drone_src_s_) {
      drone_pos_ = {m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z};
      drone_quat_ = {m.pose.pose.orientation.x, m.pose.pose.orientation.y,
        m.pose.pose.orientation.z, m.pose.pose.orientation.w};
      drone_src_s_ = s;
    }
    drone_seen_ = true;
  }

  // 源寿命：源已见 且 源采样时刻**有效（≥0）** 且 年龄(now − 源采样时刻) ≤ source_timeout_s
  // （≤0 视为不限时，兼容旧行为）。W7-b 取证：无效（<0）采样时刻**一律**视为源未就绪。
  bool sourceFresh(bool seen, double src_s, double now_s) const
  {
    if (!seen || src_s < 0.0) {return false;}
    if (source_timeout_s_ <= 0.0) {return true;}
    return (now_s - src_s) <= source_timeout_s_;
  }

  // B1 诊断（2026-10-21）：目标门控拒绝原因计数——只观测行为，不放宽任何门。
  enum class GateReason
  {
    kOk, kNoPose, kBehind, kRangeMin, kRangeMax, kFovAz, kFovEl, kCount
  };

  // 视场/距离门控：传感器原点 = 无人机位置，朝向 = 无人机姿态 × 机体系 boresight
  bool gatePoint(
    const SensorCfg & c, const std::array<double, 3> & pt, double * range, double * az_deg,
    double * el_deg, GateReason * why = nullptr) const
  {
    if (why) {*why = GateReason::kOk;}
    if (!drone_pose_ok_) {
      if (why) {*why = GateReason::kNoPose;}
      return false;
    }
    const std::array<double, 3> d{
      {pt[0] - drone_pos_[0], pt[1] - drone_pos_[1], pt[2] - drone_pos_[2]}};
    const double r = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    *range = r;
    if (r < c.range_min) {
      if (why) {*why = GateReason::kRangeMin;}
      return false;
    }
    if (r > c.range_max) {
      if (why) {*why = GateReason::kRangeMax;}
      return false;
    }
    const std::array<double, 3> u{{d[0] / r, d[1] / r, d[2] / r}};
    const std::array<double, 3> bore = rotateVecByQuat(drone_quat_, c.boresight);
    const std::array<double, 3> up{{0.0, 0.0, 1.0}};
    std::array<double, 3> right{{bore[1] * up[2] - bore[2] * up[1],
      bore[2] * up[0] - bore[0] * up[2], bore[0] * up[1] - bore[1] * up[0]}};
    double rn = std::sqrt(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]);
    if (rn < 1e-6) {right = {{0.0, 1.0, 0.0}}; rn = 1.0;}
    for (double & v : right) {v /= rn;}
    const std::array<double, 3> up2{{right[1] * bore[2] - right[2] * bore[1],
      right[2] * bore[0] - right[0] * bore[2], right[0] * bore[1] - right[1] * bore[0]}};
    const double fwd = u[0] * bore[0] + u[1] * bore[1] + u[2] * bore[2];
    const double rc = u[0] * right[0] + u[1] * right[1] + u[2] * right[2];
    const double uc = u[0] * up2[0] + u[1] * up2[1] + u[2] * up2[2];
    const double az = std::atan2(rc, fwd);
    const double el = std::atan2(uc, std::hypot(fwd, rc));
    *az_deg = az * 180.0 / M_PI;
    *el_deg = el * 180.0 / M_PI;
    if (fwd <= 0.0) {
      if (why) {*why = GateReason::kBehind;}
      return false;
    }
    if (std::fabs(az) > c.hfov_rad / 2.0) {
      if (why) {*why = GateReason::kFovAz;}
      return false;
    }
    if (std::fabs(el) > c.vfov_rad / 2.0) {
      if (why) {*why = GateReason::kFovEl;}
      return false;
    }
    return true;
  }

  static std::array<double, 3> rotateVecByQuat(
    const std::array<double, 4> & q, const std::array<double, 3> & v)
  {
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double tx = 2.0 * (y * v[2] - z * v[1]);
    const double ty = 2.0 * (z * v[0] - x * v[2]);
    const double tz = 2.0 * (x * v[1] - y * v[0]);
    return {{
      v[0] + w * tx + (y * tz - z * ty),
      v[1] + w * ty + (z * tx - x * tz),
      v[2] + w * tz + (x * ty - y * tx)}};
  }

  std::string firstOccluder(const std::array<double, 3> & pt) const
  {
    for (const auto & o : obstacles_) {
      if (segHitsObb(drone_pos_, pt, o, occlusion_pad_m_)) {return o.id;}
    }
    return std::string();
  }

  // AR(1) 时间相关：a = ρ_t·a_prev + sqrt(1-ρ_t²)·u
  void applyAr(double dt, const SensorCfg & c, int naxes, std::array<double, 3> & u,
    std::array<double, 3> * st)
  {
    if (c.noise_tau_s <= 0.0) {return;}
    const double rt = (dt > 0.0) ? std::exp(-dt / c.noise_tau_s) : 0.0;
    const double q = std::sqrt(std::max(0.0, 1.0 - rt * rt));
    for (int i = 0; i < naxes; ++i) {
      u[i] = rt * (*st)[i] + q * u[i];
      (*st)[i] = u[i];
    }
  }

  bool burstThisTick(size_t si, const SensorCfg & c, double dt)
  {
    if (c.drop_burst_prob <= 0.0) {return false;}
    const double eps = 1e-6;
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    if (burst_state_[si]) {
      const double p_bg = 1.0 - std::exp(-std::max(dt, 0.0) / std::max(c.drop_burst_mean_s, eps));
      if (uni(rng_[si]) < p_bg) {burst_state_[si] = false;}
    } else {
      const double base = 1.0 - std::exp(-std::max(dt, 0.0) / std::max(c.drop_burst_mean_gap_s, eps));
      const double p_gb = std::min(1.0, std::max(0.0, c.drop_burst_prob * base));
      if (uni(rng_[si]) < p_gb) {burst_state_[si] = true;}
    }
    return burst_state_[si];
  }

  bool makeSample(size_t si, double t, ObsSample * out)
  {
    const SensorCfg & c = sensors_[si];
    const double dt = has_prev_t_[si] ? (t - last_t_[si]) : 0.0;
    last_t_[si] = t;
    has_prev_t_[si] = true;
    out->t_obs = t;

    std::uniform_real_distribution<double> uni(0.0, 1.0);
    const bool indep_hit = (c.drop_prob > 0.0) && (uni(rng_[si]) < std::min(1.0, c.drop_prob));
    const bool burst = burstThisTick(si, c, dt);
    if (indep_hit || burst) {
      ++diag_dropped_;
      return false;
    }

    std::normal_distribution<double> nx(0.0, c.sigma_xy), ny(0.0, c.sigma_xy), nz(0.0, c.sigma_z);
    const bool model_on = (c.corr_xy != 0.0 || c.noise_tau_s > 0.0);

    if (target_present_) {
      double gr = 0, gaz = 0, gel = 0;
      GateReason why = GateReason::kOk;
      if (gatePoint(c, truth_, &gr, &gaz, &gel, &why)) {
        ++diag_gate_[static_cast<size_t>(GateReason::kOk)];
        const std::string occ = occlusion_enabled_ ? firstOccluder(truth_) : std::string();
        if (occ.empty()) {
          double ox, oy, oz;
          if (model_on) {
            std::array<double, 3> u{{nx(rng_[si]) / c.sigma_xy, ny(rng_[si]) / c.sigma_xy,
              nz(rng_[si]) / c.sigma_z}};
            if (c.corr_xy != 0.0) {
              const double r = c.corr_xy;
              u[1] = r * u[0] + std::sqrt(std::max(0.0, 1.0 - r * r)) * u[1];
            }
            applyAr(dt, c, 3, u, &a_prev_target_[si]);
            ox = truth_[0] + c.sigma_xy * u[0] + c.bias_m[0];
            oy = truth_[1] + c.sigma_xy * u[1] + c.bias_m[1];
            oz = truth_[2] + c.sigma_z * u[2] + c.bias_m[2];
          } else {
            ox = truth_[0] + nx(rng_[si]) + c.bias_m[0];
            oy = truth_[1] + ny(rng_[si]) + c.bias_m[1];
            oz = truth_[2] + nz(rng_[si]) + c.bias_m[2];
          }
          out->has_target = true;
          out->tx = ox; out->ty = oy; out->tz = oz;
          out->tyaw = truth_yaw_;
          out->tseq = ++tseq_[si];
          out->t_target_src = target_src_s_;   // 目标源采样时刻（供 TargetObservation.header.stamp）
        } else {
          ++diag_blocked_;   // 门控通过但被遮挡（B1 诊断）
        }
      } else {
        ++diag_gate_[static_cast<size_t>(why)];   // 门控拒绝原因（B1 诊断）
      }
    }

    for (size_t j = 0; j < obstacles_.size(); ++j) {
      const Obstacle & o = obstacles_[j];
      double gr = 0, gaz = 0, gel = 0;
      const std::array<double, 3> oc{{o.x, o.y, o.z}};
      if (!gatePoint(c, oc, &gr, &gaz, &gel)) {continue;}
      if (occlusion_enabled_) {
        bool blocked = false;
        for (size_t k = 0; k < obstacles_.size(); ++k) {
          if (k == j) {continue;}
          if (segHitsObb(drone_pos_, oc, obstacles_[k], occlusion_pad_m_)) {blocked = true; break;}
        }
        if (blocked) {continue;}
      }
      double ox, oy;
      if (model_on) {
        std::array<double, 3> u{{nx(rng_[si]) / c.sigma_xy, ny(rng_[si]) / c.sigma_xy, 0.0}};
        if (c.corr_xy != 0.0) {
          const double r = c.corr_xy;
          u[1] = r * u[0] + std::sqrt(std::max(0.0, 1.0 - r * r)) * u[1];
        }
        applyAr(dt, c, 2, u, &a_prev_obs_[si]);
        ox = o.x + c.sigma_xy * u[0] + c.bias_m[0];
        oy = o.y + c.sigma_xy * u[1] + c.bias_m[1];
      } else {
        ox = o.x + nx(rng_[si]) + c.bias_m[0];
        oy = o.y + ny(rng_[si]) + c.bias_m[1];
      }
      ObsSample::Ob ob;
      ob.id = o.id;
      ob.x = ox; ob.y = oy; ob.z = o.z;
      ob.yaw = o.yaw;
      ob.sx = o.sx; ob.sy = o.sy; ob.sz = o.sz;
      ob.seq = ++oseq_[si];
      out->obs.push_back(ob);
    }
    return true;
  }

  agc_msgs::msg::SensorObservation toMsg(size_t si, const ObsSample & s, double now_s)
  {
    const SensorCfg & c = sensors_[si];
    agc_msgs::msg::SensorObservation so;
    so.header.stamp = rclcpp::Time(static_cast<int64_t>(now_s * 1e9), get_clock()->get_clock_type());
    so.header.frame_id = world_frame_;
    so.sensor_id = c.id;
    if (s.has_target) {
      agc_msgs::msg::TargetObservation to;
      // header.stamp = **源采样时刻**（入站真值 Header.stamp），非通道 tick、非交付时刻（spec R-004）
      to.header.stamp = rclcpp::Time(
        static_cast<int64_t>(s.t_target_src * 1e9), get_clock()->get_clock_type());
      to.header.frame_id = world_frame_;
      to.target_id = target_id_;
      to.sensor_id = c.id;
      to.pose.pose.position.x = s.tx;
      to.pose.pose.position.y = s.ty;
      to.pose.pose.position.z = s.tz;
      to.pose.pose.orientation = agc::quatFromYaw(s.tyaw);
      cov3(c.sigma_xy, c.sigma_z, c.corr_xy, to.pose.covariance);
      to.confidence = static_cast<float>(c.confidence);
      to.obs_seq = s.tseq;
      so.targets.push_back(to);
    }
    for (const auto & o : s.obs) {
      agc_msgs::msg::ObstacleObservation oo;
      // header.stamp = 通道采样时刻（tick）：障碍真值静态，本拍由新鲜 odom 门控采得
      oo.header.stamp = rclcpp::Time(static_cast<int64_t>(s.t_obs * 1e9), get_clock()->get_clock_type());
      oo.header.frame_id = world_frame_;
      oo.obstacle_id = o.id;
      oo.sensor_id = c.id;
      oo.pose.pose.position.x = o.x;
      oo.pose.pose.position.y = o.y;
      oo.pose.pose.position.z = o.z;
      oo.pose.pose.orientation = agc::quatFromYaw(o.yaw);
      cov3(c.sigma_xy, c.sigma_z, c.corr_xy, oo.pose.covariance);
      oo.size.x = o.sx; oo.size.y = o.sy; oo.size.z = o.sz;
      oo.confidence = static_cast<float>(c.confidence);
      oo.obs_seq = o.seq;
      so.obstacles.push_back(oo);
    }
    return so;
  }

  void tick()
  {
    const rclcpp::Time now_t = now();
    const double t = now_t.seconds();
    if (t <= 0.0) {return;}  // /clock 未就绪

    // 由源寿命派生的瞬时有效标志（spec R-004）：非永久 true
    target_present_ = sourceFresh(target_seen_, target_src_s_, t);
    drone_pose_ok_ = sourceFresh(drone_seen_, drone_src_s_, t);

    // B1 诊断（2026-10-21）：每 5 s 汇总目标门控拒绝原因；**只观测**，不放宽任何门。
    ++diag_ticks_;
    if (target_present_) {++diag_present_ticks_;}
    if (t - diag_last_log_s_ >= 5.0) {
      const double present_ratio =
        diag_ticks_ > 0 ? static_cast<double>(diag_present_ticks_) / diag_ticks_ : 0.0;
      RCLCPP_INFO(get_logger(),
        "B1 诊断(5s)：channels=%zu target_present=%.2f gate_ok=%llu no_pose=%llu behind=%llu "
        "rmin=%llu rmax=%llu fov_az=%llu fov_el=%llu blocked=%llu dropped=%llu",
        sensors_.size(), present_ratio,
        static_cast<unsigned long long>(diag_gate_[static_cast<size_t>(GateReason::kOk)]),
        static_cast<unsigned long long>(diag_gate_[static_cast<size_t>(GateReason::kNoPose)]),
        static_cast<unsigned long long>(diag_gate_[static_cast<size_t>(GateReason::kBehind)]),
        static_cast<unsigned long long>(diag_gate_[static_cast<size_t>(GateReason::kRangeMin)]),
        static_cast<unsigned long long>(diag_gate_[static_cast<size_t>(GateReason::kRangeMax)]),
        static_cast<unsigned long long>(diag_gate_[static_cast<size_t>(GateReason::kFovAz)]),
        static_cast<unsigned long long>(diag_gate_[static_cast<size_t>(GateReason::kFovEl)]),
        static_cast<unsigned long long>(diag_blocked_),
        static_cast<unsigned long long>(diag_dropped_));
      diag_gate_.fill(0);
      diag_blocked_ = 0;
      diag_dropped_ = 0;
      diag_ticks_ = 0;
      diag_present_ticks_ = 0;
      diag_last_log_s_ = t;
    }

    agc_msgs::msg::AirObservationArray arr;
    arr.header.stamp = now_t;
    arr.header.frame_id = world_frame_;

    for (size_t si = 0; si < sensors_.size(); ++si) {
      const SensorCfg & c = sensors_[si];
      ObsSample s;
      if (makeSample(si, t, &s)) {
        pending_[si].push_back(Pending{t + std::max(0.0, c.latency_s), s});
      }
      bool have = false;
      ObsSample latest;
      while (!pending_[si].empty() && pending_[si].front().release_t <= t) {
        latest = pending_[si].front().s;
        have = true;
        pending_[si].pop_front();
      }
      if (have) {arr.sensors.push_back(toMsg(si, latest, t));}
    }
    pub_->publish(arr);
  }

  std::string world_frame_, target_id_, target_gt_topic_, drone_odom_topic_, obs_topic_;
  std::string sensors_file_, obstacles_file_, seed_;
  double obs_hz_{20.0}, tol_m_{0.25};
  bool occlusion_enabled_{false};
  double occlusion_pad_m_{0.0};

  std::vector<SensorCfg> sensors_;
  std::vector<Obstacle> obstacles_;
  std::vector<std::mt19937> rng_;
  std::vector<char> burst_state_;
  std::vector<char> has_prev_t_;
  std::vector<double> last_t_;
  std::vector<std::array<double, 3>> a_prev_target_;
  std::vector<std::array<double, 3>> a_prev_obs_;
  std::vector<std::deque<Pending>> pending_;
  std::vector<uint32_t> tseq_, oseq_;

  std::array<double, 3> truth_{{0.0, 0.0, 0.0}};
  double truth_yaw_{0.0};
  bool target_seen_{false};        // 是否曾收到目标真值（原始）
  double target_src_s_{-1.0};      // 目标源采样时刻（入站 Header.stamp）；<0 = 未就绪
  bool target_present_{false};     // 派生：源寿命内的瞬时有效（每 tick 重算）
  std::array<double, 3> drone_pos_{{0.0, 0.0, 0.0}};
  std::array<double, 4> drone_quat_{{0.0, 0.0, 0.0, 1.0}};
  bool drone_seen_{false};         // 是否曾收到无人机 odom（原始）
  double drone_src_s_{-1.0};       // 无人机 odom 源采样时刻（入站 Header.stamp）；<0 = 未就绪
  bool drone_pose_ok_{false};      // 派生：源寿命内的瞬时有效（每 tick 重算）
  // W7-b 取证：无效（<0）源采样时刻的**一次性**告警（如实记录原始 sec/nsec，便于定位上游）
  bool warned_bad_target_stamp_{false}, warned_bad_drone_stamp_{false};
  double source_timeout_s_{0.3};   // 源寿命期限（launch 单一注入）

  // B1 诊断（2026-10-21）：目标门控拒绝原因计数（每 5 s 汇总后清零；不影响观测行为）
  std::array<uint64_t, static_cast<size_t>(GateReason::kCount)> diag_gate_{};
  uint64_t diag_blocked_{0}, diag_dropped_{0}, diag_ticks_{0}, diag_present_ticks_{0};
  double diag_last_log_s_{0.0};

  rclcpp::Publisher<agc_msgs::msg::AirObservationArray>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AirPerception>());
  rclcpp::shutdown();
  return 0;
}
