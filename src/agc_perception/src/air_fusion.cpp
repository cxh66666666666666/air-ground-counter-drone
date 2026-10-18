// air_fusion —— 目标与障碍多源融合（W4-b，spec R5/R6）
//
// 输入 /agc/air/obs（air_perception 的多通道观测），输出：
//   * /agc/targets   (TargetTrackArray，source=air_fusion)  —— 被空中跟踪 / 地面规划消费
//   * /agc/obstacles (ObstacleTrackArray，source=air_fusion) —— 障碍源为 air_fusion 时被规划消费
//
// 门控（对齐旧树行为 + R-005 诚实时间口径）：新鲜度（0 ≤ age ≤ target_max_age_s，**拒绝未来戳**）、
//   置信度（≥ confidence_min）、空间一致性（与参考估计距离 > gate_m 判离群）。
// 时间口径（R-005）：多通道候选**先对齐到共同时刻** t_ref = max(候选采样时刻) —— 仅保留与 t_ref
//   相差 ≤ time_sync_tol_s 的「同代 cohort」；再把各候选按**上一次融合速度**一步常速传播到 t_ref
//   （冷启动速度为 0，等价不平移），然后做逆方差加权。输出 track 的 header.stamp = t_ref（真实采样
//   时刻），不再用最大戳冒充「最新」。
// 不确定度（R-005）：各通道同源、误差相关，协方差以 max(min_sigma_inlier, correlated_pad_m) 设下界，
//   **不按 1/N 缩小**（相关误差不随通道数变小）；confidence 由该诚实 σ 派生。
// 速度（R-005）：基于**时间一致**样本 —— 以上次融合的 t_ref 与位置对当前 t_ref 差分；重发 / 同刻
//   （epoch 未前进，dt_e < min_velocity_dt_s）保持上次速度，避免重发样本差分出归零 / 跳变；冷启动为 0。
// 障碍（R-005）：按 obstacle_id 关联候选，逐个候选 OBB 换算为**轴对齐包围盒**后取**并集**
//   （可证明包含各候选 OBB），输出轴对齐并集盒 + 保守外扩；不再仅取最大尺寸加 pad。
// 无有效观测发空数组，绝不刷新旧观测时间。
//
// 新鲜度不变量（spec R6）：本节点内部门控 target_max_age_s（默认 0.35 s）SHALL ≤
//   契约 C10 消费侧冻结阈值（0.5 s）—— 感知侧冻结不晚于消费侧；两值语义不同，不合并。
//
// 置信度口径由 agc_perception 共享头给出（spec R5）；本节点**不写文件**（spec R10）。

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <agc_msgs/msg/air_observation_array.hpp>
#include <agc_msgs/msg/obstacle_observation.hpp>
#include <agc_msgs/msg/obstacle_track.hpp>
#include <agc_msgs/msg/obstacle_track_array.hpp>
#include <agc_msgs/msg/target_observation.hpp>
#include <agc_msgs/msg/target_track.hpp>
#include <agc_msgs/msg/target_track_array.hpp>

#include "agc_perception/confidence.hpp"
#include "agc_perception/math_utils.hpp"

namespace
{
const char * const kSource = "air_fusion";

void diagCov3(double sxy, double sz, std::array<double, 36> & c)
{
  c.fill(0.0);
  c[0] = c[7] = sxy * sxy;
  c[14] = sz * sz;
}
}  // namespace

class AirFusion : public rclcpp::Node
{
public:
  AirFusion()
  : Node("air_fusion")
  {
    target_id_ = declare_parameter<std::string>("target_id", "intruder_uav");
    hz_ = declare_parameter<double>("hz", 10.0);
    target_max_age_ = declare_parameter<double>("target_max_age_s", 0.35);
    obs_max_age_ = declare_parameter<double>("obs_max_age_s", 0.35);
    gate_m_ = declare_parameter<double>("gate_m", 1.5);
    confidence_min_ = declare_parameter<double>("confidence_min", 0.5);
    min_sources_ = declare_parameter<int>("min_target_sources", 1);
    obstacle_pad_m_ = declare_parameter<double>("obstacle_pad_m", 0.05);
    correlated_pad_m_ = declare_parameter<double>("correlated_pad_m", 0.05);
    cold_start_min_agree_ = declare_parameter<int>("cold_start_min_agree", 2);
    ref_hold_s_ = declare_parameter<double>("reference_hold_s", 1.0);
    time_sync_tol_s_ = declare_parameter<double>("time_sync_tol_s", 0.15);
    min_velocity_dt_s_ = declare_parameter<double>("min_velocity_dt_s", 0.02);
    tol_m_ = declare_parameter<double>("confidence_tolerance_m", 0.25);
    target_source_ = declare_parameter<std::string>("target_source", "gt_adapter");
    obstacle_source_ = declare_parameter<std::string>("obstacles_source", "air_fusion");
    obs_topic_ = declare_parameter<std::string>("obs_topic", "/agc/air/obs");
    target_topic_ = declare_parameter<std::string>("target_topic", "/agc/targets");
    obstacle_topic_ = declare_parameter<std::string>("obstacle_topic", "/agc/obstacles");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");

    if (target_source_ == kSource) {
      pub_targets_ = create_publisher<agc_msgs::msg::TargetTrackArray>(target_topic_, rclcpp::QoS(10));
    } else {
      RCLCPP_WARN(get_logger(), "air_fusion：/agc/target/source=%s，本节点不发布 %s",
        target_source_.c_str(), target_topic_.c_str());
    }
    if (obstacle_source_ == kSource) {
      pub_obstacles_ = create_publisher<agc_msgs::msg::ObstacleTrackArray>(obstacle_topic_,
        rclcpp::QoS(10));
    } else {
      RCLCPP_WARN(get_logger(), "air_fusion：/agc/obstacles/source=%s，本节点不发布 %s",
        obstacle_source_.c_str(), obstacle_topic_.c_str());
    }

    sub_ = create_subscription<agc_msgs::msg::AirObservationArray>(
      obs_topic_, rclcpp::QoS(1), std::bind(&AirFusion::onObs, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(hz_, 1e-3));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AirFusion::tick, this));

    RCLCPP_INFO(get_logger(), "air_fusion 就绪：hz=%.1f age<=%.2f gate=%.2f conf_min=%.2f（source=%s/%s）",
      hz_, target_max_age_, gate_m_, confidence_min_, target_source_.c_str(), obstacle_source_.c_str());
  }

private:
  struct SensorState
  {
    bool has_target{false};
    agc_msgs::msg::TargetObservation target;
    std::map<std::string, agc_msgs::msg::ObstacleObservation> obstacles;
  };

  void onObs(const agc_msgs::msg::AirObservationArray & msg)
  {
    for (const auto & so : msg.sensors) {
      SensorState & st = state_[so.sensor_id];
      for (const auto & t : so.targets) {st.target = t; st.has_target = true;}
      for (const auto & o : so.obstacles) {st.obstacles[o.obstacle_id] = o;}
    }
  }

  // σ 由观测协方差给出（cov[0]=sigma_xy²），避免重复导入 sensors_file
  static double sigmaOf(const agc_msgs::msg::TargetObservation & o)
  {
    const double v = o.pose.covariance[0];
    return (v > 1e-12) ? std::sqrt(v) : 0.15;
  }
  static double sigmaOfObs(const agc_msgs::msg::ObstacleObservation & o)
  {
    const double v = o.pose.covariance[0];
    return (v > 1e-12) ? std::sqrt(v) : 0.15;
  }

  double coverage(double sigma) const {return agc::coverageConfidence(sigma, tol_m_);}

  void fuseTargets(double now_s)
  {
    struct Cand
    {
      std::string sensor;
      agc_msgs::msg::TargetObservation o;
      double sigma{0.1};
      double stamp{0.0};
    };
    std::vector<Cand> cands;
    for (const auto & kv : state_) {
      if (!kv.second.has_target) {continue;}
      const auto & o = kv.second.target;
      const double st = rclcpp::Time(o.header.stamp).seconds();
      const double age = now_s - st;
      if (age < 0.0) {continue;}              // R-005：拒绝未来戳
      if (age > target_max_age_) {continue;}  // 过期
      if (o.confidence < confidence_min_) {continue;}
      cands.push_back(Cand{kv.first, o, sigmaOf(o), st});
    }

    if (cands.empty()) {
      publishEmptyTargets(now_s);
      has_last_fused_ = false;
      return;
    }

    // R-005 共同时刻对齐：t_ref = max(候选采样时刻)；仅保留 (t_ref - t_i) <= time_sync_tol_s 的同代 cohort
    double t_ref = 0.0;
    for (const auto & c : cands) {t_ref = std::max(t_ref, c.stamp);}
    std::vector<Cand> cohort;
    for (const auto & c : cands) {
      if ((t_ref - c.stamp) <= time_sync_tol_s_) {cohort.push_back(c);}
    }
    if (static_cast<int>(cohort.size()) < std::max(1, min_sources_)) {
      publishEmptyTargets(now_s);
      has_last_fused_ = false;
      return;
    }

    double rx, ry;
    if (has_last_fused_ && (now_s - last_fused_sim_) <= ref_hold_s_) {
      rx = last_fused_x_; ry = last_fused_y_;
    } else {
      // 冷启动：按 gate_m 单链聚类取最大簇作参考（在 cohort 上）
      const int n = static_cast<int>(cohort.size());
      std::vector<int> label(n, -1);
      int nc = 0;
      for (int i = 0; i < n; ++i) {
        if (label[i] != -1) {continue;}
        label[i] = nc;
        for (int j = i + 1; j < n; ++j) {
          if (label[j] != -1) {continue;}
          const double d = std::hypot(
            cohort[i].o.pose.pose.position.x - cohort[j].o.pose.pose.position.x,
            cohort[i].o.pose.pose.position.y - cohort[j].o.pose.pose.position.y);
          if (d <= gate_m_) {label[j] = nc;}
        }
        ++nc;
      }
      int best_c = -1, best_sz = 0;
      for (int c = 0; c < nc; ++c) {
        const int sz = static_cast<int>(std::count(label.begin(), label.end(), c));
        if (sz > best_sz) {best_sz = sz; best_c = c;}
      }
      if (n >= 2 && best_sz < std::max(2, cold_start_min_agree_)) {
        publishEmptyTargets(now_s);  // 冷启动歧义：不发高置信目标
        has_last_fused_ = false;
        return;
      }
      double sx = 0, sy = 0;
      int cnt = 0;
      for (int i = 0; i < n; ++i) {
        if (label[i] != best_c) {continue;}
        sx += cohort[i].o.pose.pose.position.x;
        sy += cohort[i].o.pose.pose.position.y;
        ++cnt;
      }
      rx = sx / std::max(1, cnt);
      ry = sy / std::max(1, cnt);
    }

    // 空间一致性门控（在 cohort 上）
    std::vector<const Cand *> inliers;
    for (const auto & c : cohort) {
      const double d = std::hypot(
        c.o.pose.pose.position.x - rx, c.o.pose.pose.position.y - ry);
      if (d <= gate_m_) {inliers.push_back(&c);}
    }
    if (static_cast<int>(inliers.size()) < std::max(1, min_sources_)) {
      publishEmptyTargets(now_s);
      has_last_fused_ = false;
      return;
    }
    const Cand * bestc = inliers[0];  // 姿态取 confidence 最大的候选（现逻辑）
    for (const Cand * c : inliers) {
      if (c->o.confidence > bestc->o.confidence) {bestc = c;}
    }

    // R-005：按模型（上一次融合速度）一步常速传播到共同时刻 t_ref，再逆方差加权
    // (t_ref - t_i) ∈ [0, time_sync_tol_s]；冷启动速度为 0（不传播）
    const double vmx = has_last_fused_ ? last_fused_vx_ : 0.0;
    const double vmy = has_last_fused_ ? last_fused_vy_ : 0.0;
    const double vmz = has_last_fused_ ? last_fused_vz_ : 0.0;
    double sw = 0, swx = 0, swy = 0, swz = 0, min_sigma = 1e9;
    for (const Cand * c : inliers) {
      const double dt = t_ref - c->stamp;  // ∈ [0, time_sync_tol_s]
      const double px = c->o.pose.pose.position.x + vmx * dt;
      const double py = c->o.pose.pose.position.y + vmy * dt;
      const double pz = c->o.pose.pose.position.z + vmz * dt;
      const double w = 1.0 / std::max(1e-6, c->sigma * c->sigma);
      sw += w;
      swx += w * px;
      swy += w * py;
      swz += w * pz;
      min_sigma = std::min(min_sigma, c->sigma);
    }
    const double fx = swx / sw, fy = swy / sw, fz = swz / sw;
    // R-005 协方差诚实地板：相关误差不按 1/N 收缩（通道同源），取各 inlier σ 的最小值为下界
    const double sigma_fused = std::max(min_sigma, correlated_pad_m_);
    const double conf_fused = coverage(sigma_fused);

    // R-005 速度：基于时间一致样本；重发 / 同刻（epoch 未前进）保持上次速度
    double vx = last_fused_vx_, vy = last_fused_vy_, vz = last_fused_vz_;
    if (has_last_fused_) {
      const double dt_e = t_ref - last_fused_tref_;
      if (dt_e >= min_velocity_dt_s_) {
        vx = (fx - last_fused_x_) / dt_e;
        vy = (fy - last_fused_y_) / dt_e;
        vz = (fz - last_fused_z_) / dt_e;
      }  // else：同刻 / 重发，保持上次速度，避免差分出归零 / 跳变
    }  // 冷启动：保持 0

    agc_msgs::msg::TargetTrackArray arr;
    arr.header.stamp = rclcpp::Time(static_cast<int64_t>(now_s * 1e9), get_clock()->get_clock_type());
    arr.header.frame_id = world_frame_;
    agc_msgs::msg::TargetTrack tr;
    // R-005：track 时刻 = 对齐后的共同时刻 t_ref（最新候选的真实采样时刻），不用 max 戳冒充
    tr.header.stamp = rclcpp::Time(static_cast<int64_t>(t_ref * 1e9), get_clock()->get_clock_type());
    tr.header.frame_id = world_frame_;
    tr.target_id = target_id_;
    tr.pose.pose.position.x = fx;
    tr.pose.pose.position.y = fy;
    tr.pose.pose.position.z = fz;
    tr.pose.pose.orientation = agc::quatFromYaw(agc::yawFromQuat(bestc->o.pose.pose.orientation));
    diagCov3(sigma_fused, std::max(sigma_fused, correlated_pad_m_), tr.pose.covariance);
    tr.twist.twist.linear.x = vx;
    tr.twist.twist.linear.y = vy;
    tr.twist.twist.linear.z = vz;
    diagCov3(0.05, 0.05, tr.twist.covariance);
    tr.confidence = static_cast<float>(conf_fused);
    tr.source = kSource;
    arr.tracks.push_back(tr);
    if (pub_targets_) {pub_targets_->publish(arr);}

    last_fused_sim_ = now_s;
    last_fused_x_ = fx; last_fused_y_ = fy; last_fused_z_ = fz;
    last_fused_tref_ = t_ref;
    last_fused_vx_ = vx; last_fused_vy_ = vy; last_fused_vz_ = vz;
    has_last_fused_ = true;
  }

  void publishEmptyTargets(double now_s)
  {
    if (!pub_targets_) {return;}
    agc_msgs::msg::TargetTrackArray arr;
    arr.header.stamp = rclcpp::Time(static_cast<int64_t>(now_s * 1e9), get_clock()->get_clock_type());
    arr.header.frame_id = world_frame_;
    pub_targets_->publish(arr);  // 空数组：不刷新旧 stamp
  }

  void fuseObstacles(double now_s)
  {
    if (!pub_obstacles_) {return;}
    std::map<std::string, std::vector<const agc_msgs::msg::ObstacleObservation *>> by_id;
    for (const auto & kv : state_) {
      for (const auto & okv : kv.second.obstacles) {
        const auto & o = okv.second;
        const double age = now_s - rclcpp::Time(o.header.stamp).seconds();
        if (age < 0.0) {continue;}  // R-005：拒绝未来戳
        if (age > obs_max_age_ || o.confidence < confidence_min_) {continue;}
        by_id[okv.first].push_back(&o);
      }
    }

    agc_msgs::msg::ObstacleTrackArray arr;
    arr.header.stamp = rclcpp::Time(static_cast<int64_t>(now_s * 1e9), get_clock()->get_clock_type());
    arr.header.frame_id = world_frame_;
    for (const auto & kv : by_id) {
      if (kv.second.empty()) {continue;}
      // R-005 可证明包含各候选的保守包络：逐个候选 OBB -> 各自轴对齐包围盒 -> 求并集
      double minx = 1e18, maxx = -1e18, miny = 1e18, maxy = -1e18, minz = 1e18, maxz = -1e18;
      double best_stamp = 0.0, min_sigma = 1e9;
      for (const auto * o : kv.second) {
        const double hx = o->size.x * 0.5;
        const double hy = o->size.y * 0.5;
        const double hz = o->size.z * 0.5;
        const double th = agc::yawFromQuat(o->pose.pose.orientation);
        const double ct = std::cos(th), stt = std::sin(th);
        const double ex = std::abs(hx * ct) + std::abs(hy * stt);
        const double ey = std::abs(hx * stt) + std::abs(hy * ct);
        const double ez = hz;
        const double cx = o->pose.pose.position.x;
        const double cy = o->pose.pose.position.y;
        const double cz = o->pose.pose.position.z;
        minx = std::min(minx, cx - ex); maxx = std::max(maxx, cx + ex);
        miny = std::min(miny, cy - ey); maxy = std::max(maxy, cy + ey);
        minz = std::min(minz, cz - ez); maxz = std::max(maxz, cz + ez);
        best_stamp = std::max(best_stamp, rclcpp::Time(o->header.stamp).seconds());
        min_sigma = std::min(min_sigma, sigmaOfObs(*o));
      }
      agc_msgs::msg::ObstacleTrack t;
      t.header.stamp = rclcpp::Time(static_cast<int64_t>(best_stamp * 1e9), get_clock()->get_clock_type());
      t.header.frame_id = world_frame_;
      t.obstacle_id = kv.first;
      t.pose.position.x = 0.5 * (minx + maxx);
      t.pose.position.y = 0.5 * (miny + maxy);
      t.pose.position.z = 0.5 * (minz + maxz);
      t.pose.orientation = agc::quatFromYaw(0.0);  // 轴对齐并集盒
      t.size.x = (maxx - minx) + obstacle_pad_m_;
      t.size.y = (maxy - miny) + obstacle_pad_m_;
      t.size.z = (maxz - minz);
      // R-005 协方差诚实地板：相关误差不按 1/N 收缩；ObstacleTrack 无协方差字段，仅影响 confidence
      const double sigma_fused = std::max(min_sigma, correlated_pad_m_);
      t.confidence = static_cast<float>(coverage(sigma_fused));
      t.source = kSource;
      arr.tracks.push_back(t);
    }
    pub_obstacles_->publish(arr);
  }

  void tick()
  {
    const double now_s = now().seconds();
    if (now_s <= 0.0) {return;}
    if (pub_targets_) {fuseTargets(now_s);}
    fuseObstacles(now_s);
  }

  std::map<std::string, SensorState> state_;

  std::string target_id_, target_source_, obstacle_source_, obs_topic_, target_topic_;
  std::string obstacle_topic_, world_frame_;
  double hz_{10.0}, target_max_age_{0.35}, obs_max_age_{0.35}, gate_m_{1.5};
  double confidence_min_{0.5}, obstacle_pad_m_{0.05}, correlated_pad_m_{0.05};
  double ref_hold_s_{1.0}, tol_m_{0.25}, time_sync_tol_s_{0.15}, min_velocity_dt_s_{0.02};
  int min_sources_{1}, cold_start_min_agree_{2};

  bool has_last_fused_{false};
  double last_fused_sim_{0.0}, last_fused_x_{0.0}, last_fused_y_{0.0}, last_fused_z_{0.0};
  double last_fused_tref_{0.0}, last_fused_vx_{0.0}, last_fused_vy_{0.0}, last_fused_vz_{0.0};

  rclcpp::Publisher<agc_msgs::msg::TargetTrackArray>::SharedPtr pub_targets_;
  rclcpp::Publisher<agc_msgs::msg::ObstacleTrackArray>::SharedPtr pub_obstacles_;
  rclcpp::Subscription<agc_msgs::msg::AirObservationArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AirFusion>());
  rclcpp::shutdown();
  return 0;
}
