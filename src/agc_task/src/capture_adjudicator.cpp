// capture_adjudicator —— 发射式网捕获裁决器（W6 R9 / ADR-0022；R-020 占位包络标注 / 高度含车体 / 独立干扰包络）
//
// 物理捕获 = **发射式网**（非"目标穿越固定网框"）。现阶段**不建真实弹道**：以网发射物理模型
// （初速 v0 / 抛射角 θ / 重力 g / 阻力因子 / 展开后有效直径 d）的**可及集**做**包络几何闭式判定**。
//
// **占位口径（R-020 / ADR-0035）**：本模型为**闭式占位**（[ADR-0024]），带**模型身份与适用范围**标签；
// 目标高度超出 [applicable_min_alt_m, applicable_max_alt_m] 或不可达 → **不判命中**并**标注**，
// **不得**据此声称「移动式主动拦截」。
//   * 高度 **含车体**：`h = target.z − car.z − launcher_h`（R-020，未计车体高度的旧式会偏离物理）。
//   * **捕获作用包络**（`/agc/task/envelope_in`，仍用本网弹道模型）与**独立干扰作用包络**
//     （`/agc/task/envelope_jam`，模型 `jam_model_id`，与网弹道**不同源**）**分离**（[ADR-0024](b)）。
//
// **可插拔接口**（R9/10.2/10.3）：本节点只依赖 `CaptureAdjudicator` 抽象（`evaluate()`）；
// 现用 `EnvelopeAdjudicator`（闭式）。后续补真实弹道时替换为逐帧弹道判定器，**调用方
// （`task_manager`，订阅 `/agc/task/capture_event` 配套事件）不改**——I/O 契约稳定（见文件末说明）。
//
// I/O 契约：
//   输入：`/agc/ground/odom`（车世界位姿，含 z）、`/agc/targets`（目标世界位姿，含 z）
//         `/agc/task/state`（`AgcTaskState`，任务窗口门控，R-021）、`/agc/scenario/reset_epoch`
//         （`std_msgs/Int32`，transient_local；episode 复位，R-021）
//   输出：`/agc/task/capture_hit`（`std_msgs/String`，命中值**恰为** `CAPTURE_NET`，一次性 latched；
//          契约 #terminal-semantics：`task_manager` 精确匹配，**不得**追加标签）
//         `/agc/task/capture_event`（`std_msgs/String`，**配套事件**，R-021；值
//          `CAPTURE_NET ep=<epoch> t=<ros_sec> target=<id>`；**非**命令者，不改冻结 9 消息 ABI）
//         `/agc/task/envelope_in`（`std_msgs/Bool`，**捕获**作用包络：目标是否在网弹道可及集内）
//         `/agc/task/envelope_jam`（`std_msgs/Bool`，**独立干扰**作用包络，见下）
//         `/agc/task/capture_evidence`（`std_msgs/String`，标注：命中带 `model_id`/适用高度/h/dist；
//          越界 / 不可达标 `out_of_applicable_range` / `unreachable`，**限频**——同一状态跳变只发一次）
//   参数（launch/config 单一注入，节点不得覆写；契约 #params）：net_v0_mps / net_angle_deg / net_g /
//         net_diameter_m / net_drag / launcher_height_m / model_id / applicable_min_alt_m /
//         applicable_max_alt_m / jam_model_id / jam_envelope_min_alt_m / jam_envelope_max_alt_m /
//         jam_envelope_max_range_m + 话题名参数。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <agc_msgs/msg/agc_task_state.hpp>
#include <agc_msgs/msg/target_track_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

using AgcTaskState = agc_msgs::msg::AgcTaskState;

namespace
{
struct Pose2
{
  double x{0.0}, y{0.0}, z{0.0};
  bool valid{false};
};

struct EnvelopeParams
{
  double v0{6.5};        // 发射初速 m/s
  double angle_rad{1.309};  // 抛射角 75°
  double g{9.81};        // 重力
  double diameter{1.0};  // 展开后有效直径 m
  double drag{0.9};      // 阻力因子（对可及集的水平射程折算）
  double launcher_h{0.0};  // 发射器相对车体基准的高度 m
  double applicable_min{0.0};    // 模型适用高度下限 m（R-020）
  double applicable_max{2.009};  // 模型适用高度上限 m（R-020；见节点 declare 注释）
};

// 裁决结果（R-020）：是否命中 + 供证据标注的状态 / 观测量。
enum class CaptureStatus
{
  kHit,                    // 命中（在可及集内）
  kOutOfApplicableRange,   // h 超出 [applicable_min, applicable_max] → 不判命中并标注
  kUnreachable,            // 该高度不可达（disc = vy² − 2gh < 0）→ 不判命中并标注
  kOutOfEnvelope,          // 在适用高度内但水平距离超出射程（正常"未入包络"，不标注）
  kInvalid                 // 车 / 目标位姿缺失（不裁决、不标注）
};

struct CaptureDecision
{
  CaptureStatus status{CaptureStatus::kInvalid};
  bool hit{false};
  double dist{0.0};
  double h{0.0};
};

// 可插拔裁决器接口（R9）：现为包络闭式，后续弹道判定器仅换实现，I/O 契约不变。
class CaptureAdjudicator
{
public:
  virtual ~CaptureAdjudicator() = default;
  virtual CaptureDecision evaluate(const Pose2 & car, const Pose2 & target) const = 0;
};
}  // namespace

// 包络几何闭式判定：由网发射物理模型的**可及集**推导交战包络。
// 高度 h（目标相对发射器）处水平射程 R(h)=v0·cosθ·t_desc·drag，t_desc = (vy+sqrt(vy²-2gh))/g；
// 捕获判据：h≥0 且 h∈[applicable_min, applicable_max] 且可达（vy²≥2gh）且 水平距离 ≤ R(h) + d/2。
class EnvelopeAdjudicator : public CaptureAdjudicator
{
public:
  explicit EnvelopeAdjudicator(const EnvelopeParams & p) : p_(p) {}

  CaptureDecision evaluate(const Pose2 & car, const Pose2 & target) const override
  {
    CaptureDecision d;
    if (!car.valid || !target.valid) {return d;}   // kInvalid
    const double dx = target.x - car.x, dy = target.y - car.y;
    d.dist = std::hypot(dx, dy);
    // R-020：高度**含车体**——h = target.z − car.z − launcher_h。
    d.h = target.z - car.z - p_.launcher_h;
    if (d.h < 0.0 || d.h < p_.applicable_min || d.h > p_.applicable_max) {
      d.status = CaptureStatus::kOutOfApplicableRange;  // 超适用范围（含 h<0）→ 不命中并标注
      return d;
    }
    const double vy = p_.v0 * std::sin(p_.angle_rad);
    const double disc = vy * vy - 2.0 * p_.g * d.h;
    if (disc < 0.0) {
      d.status = CaptureStatus::kUnreachable;      // 该高度不可达 → 不命中并标注
      return d;
    }
    const double t = (vy + std::sqrt(disc)) / p_.g;  // 下降段飞行时间
    const double reach = p_.v0 * std::cos(p_.angle_rad) * t * p_.drag;
    if (d.dist <= reach + 0.5 * p_.diameter) {
      d.hit = true;
      d.status = CaptureStatus::kHit;
    } else {
      d.status = CaptureStatus::kOutOfEnvelope;
    }
    return d;
  }

private:
  EnvelopeParams p_;
};

class CaptureAdjudicatorNode : public rclcpp::Node
{
public:
  CaptureAdjudicatorNode()
  : Node("capture_adjudicator")
  {
    EnvelopeParams p;
    p.v0 = declare_parameter<double>("net_v0_mps", p.v0);
    const double angle_deg = declare_parameter<double>("net_angle_deg", 75.0);
    p.angle_rad = angle_deg * M_PI / 180.0;
    p.g = declare_parameter<double>("net_g", p.g);
    p.diameter = declare_parameter<double>("net_diameter_m", p.diameter);
    p.drag = declare_parameter<double>("net_drag", p.drag);
    p.launcher_h = declare_parameter<double>("launcher_height_m", p.launcher_h);
    // R-020：模型身份与适用范围标签。
    model_id_ = declare_parameter<std::string>("model_id", "ballistic_closed_form");
    // 默认上限 2.009 m ≈ 当前默认点弹道解析顶点 (v0·sinθ)²/(2g)：
    //   v0=6.5 m/s、θ=75°、g=9.81 → (6.5·sin75°)²/(2·9.81) ≈ 2.009 m。
    //   这是**占位**模型的可达顶点（未计 t_exp 与网几何），**不**代表真实网捕能力。
    p.applicable_min = declare_parameter<double>("applicable_min_alt_m", 0.0);
    p.applicable_max = declare_parameter<double>("applicable_max_alt_m", 2.009);
    applicableMin_ = p.applicable_min;
    applicableMax_ = p.applicable_max;
    // 独立干扰作用包络（R-020）——与网弹道模型**不同源**，仅需自身参数。
    jam_model_id_ = declare_parameter<std::string>("jam_model_id", "behavioral_range_gate");
    jam_min_ = declare_parameter<double>("jam_envelope_min_alt_m", 0.0);
    jam_max_ = declare_parameter<double>("jam_envelope_max_alt_m", 2.0);
    jam_max_range_ = declare_parameter<double>("jam_envelope_max_range_m", 1.0);

    // R-021：任务窗口 + 输入寿命 + episode 身份（launch/config 单一注入，节点不得覆写；契约 #params）
    car_max_age_s_ = declare_parameter<double>("car_max_age_s", 0.5);
    target_max_age_s_ = declare_parameter<double>("target_max_age_s", 0.5);
    confidence_min_ = declare_parameter<double>("confidence_min", 0.5);
    // 空 = 不校验身份（仅校验输入新鲜）；非空 = 仅当 target_id 匹配才允许产出命中
    required_target_id_ = declare_parameter<std::string>("required_target_id", "");

    rate_hz_ = declare_parameter<double>("rate_hz", 20.0);
    const auto odom_topic = declare_parameter<std::string>("ground_odom_topic", "/agc/ground/odom");
    const auto target_topic = declare_parameter<std::string>("target_topic", "/agc/targets");
    const auto hit_topic = declare_parameter<std::string>("hit_topic", "/agc/task/capture_hit");
    const auto env_topic = declare_parameter<std::string>("envelope_topic", "/agc/task/envelope_in");
    const auto evidence_topic =
      declare_parameter<std::string>("capture_evidence_topic", "/agc/task/capture_evidence");
    const auto jam_env_topic =
      declare_parameter<std::string>("jam_envelope_topic", "/agc/task/envelope_jam");
    // R-021：任务窗口订阅 + episode 复位 + 配套事件发布话题（新增，均为 std_msgs 状态 / 事件）
    const auto task_state_topic =
      declare_parameter<std::string>("task_state_topic", "/agc/task/state");
    const auto epoch_topic =
      declare_parameter<std::string>("epoch_topic", "/agc/scenario/reset_epoch");
    const auto event_topic =
      declare_parameter<std::string>("capture_event_topic", "/agc/task/capture_event");

    adjudicator_ = std::make_unique<EnvelopeAdjudicator>(p);
    hit_pub_ = create_publisher<std_msgs::msg::String>(
      hit_topic, rclcpp::QoS(1).transient_local());
    event_pub_ = create_publisher<std_msgs::msg::String>(
      event_topic, rclcpp::QoS(1).transient_local());
    env_pub_ = create_publisher<std_msgs::msg::Bool>(env_topic, 10);
    evidence_pub_ = create_publisher<std_msgs::msg::String>(evidence_topic, 10);
    jam_env_pub_ = create_publisher<std_msgs::msg::Bool>(jam_env_topic, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        car_.x = m->pose.pose.position.x; car_.y = m->pose.pose.position.y;
        car_.z = m->pose.pose.position.z; car_.valid = true;
        car_rx_ = now();   // R-021：记录输入到达时刻（新鲜度基准）
      });
    target_sub_ = create_subscription<agc_msgs::msg::TargetTrackArray>(
      target_topic, 10, [this](agc_msgs::msg::TargetTrackArray::SharedPtr m) {
        target_.valid = !m->tracks.empty();
        if (target_.valid) {
          const auto & t = m->tracks.front();
          const auto & p = t.pose.pose.position;
          target_.x = p.x; target_.y = p.y; target_.z = p.z;
          target_conf_ = t.confidence;
          target_id_ = t.target_id;
          target_rx_ = now();   // R-021：记录输入到达时刻
        }
      });
    // R-021：任务窗口（仅 GROUND_APPROACH 内允许产出命中）
    task_state_sub_ = create_subscription<AgcTaskState>(
      task_state_topic, 10,
      [this](AgcTaskState::SharedPtr m) {task_state_ = m->state;});
    // R-021：episode 复位（epoch 值变化即复位一次性标志；transient_local 对齐发布端 / R-015）
    epoch_sub_ = create_subscription<std_msgs::msg::Int32>(
      epoch_topic, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Int32::SharedPtr m) {onEpoch(m->data);});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1e-3)),
      [this]() {tick();});
    RCLCPP_INFO(get_logger(),
      "capture_adjudicator 就绪：model_id=%s v0=%.2f θ=%.1f° d=%.2f drag=%.2f 适用高度=[%.3f, %.3f] m；"
      "独立干扰包络模型=%s 高度=[%.3f, %.3f] m 水平≤%.3f m",
      model_id_.c_str(), p.v0, angle_deg, p.diameter, p.drag,
      p.applicable_min, p.applicable_max,
      jam_model_id_.c_str(), jam_min_, jam_max_, jam_max_range_);
  }

private:
  void tick()
  {
    const auto t = now();
    // R-021：输入寿命——源停 / 超龄 / 低 confidence → 判失效，不产出（含不发布包络）
    const bool car_fresh =
      car_.valid && (t - car_rx_).seconds() <= car_max_age_s_;
    const bool target_fresh =
      target_.valid && (t - target_rx_).seconds() <= target_max_age_s_ &&
      target_conf_ >= confidence_min_;
    if (!car_fresh || !target_fresh) {
      return;
    }
    const CaptureDecision d = adjudicator_->evaluate(car_, target_);

    // ---- 捕获作用包络（`/agc/task/envelope_in`，仍用网弹道模型）----
    std_msgs::msg::Bool eb;
    eb.data = d.hit;
    env_pub_->publish(eb);

    // ---- 独立干扰作用包络（`/agc/task/envelope_jam`，不同源模型；hj 也含车体）----
    const double hj = target_.z - car_.z;
    const double jdist = std::hypot(target_.x - car_.x, target_.y - car_.y);
    std_msgs::msg::Bool jb;
    jb.data = (hj >= jam_min_ && hj <= jam_max_ && jdist <= jam_max_range_);
    jam_env_pub_->publish(jb);

    // R-021：命中仅在**有效任务窗口**（GROUND_APPROACH）+ 身份匹配（required_target_id 非空时）
    // 且本 episode 一次性标志未置位时产出；episode 变化由 onEpoch() 复位 fired_。
    const bool window_open = (task_state_ == AgcTaskState::GROUND_APPROACH);
    const bool identity_ok =
      required_target_id_.empty() || (target_id_ == required_target_id_);

    // ---- 命中：/capture_hit 值恰为 CAPTURE_NET；配套事件携带 ep / 时刻 / target ----
    if (window_open && identity_ok && d.hit && !fired_) {
      fired_ = true;
      std_msgs::msg::String hit;
      hit.data = "CAPTURE_NET";
      hit_pub_->publish(hit);
      publishEvent(t);
      const std::string ev = evidenceString(
        "CAPTURE_HIT", "CAPTURE_NET", d);
      publishEvidence(ev);
      RCLCPP_WARN(get_logger(),
        "CAPTURE（发射式网包络命中）：model_id=%s h=%.3f dist=%.3f 适用高度=[%.3f, %.3f] m "
        "（闭式占位，非弹道 / 火控）epoch=%d",
        model_id_.c_str(), d.h, d.dist, applicableMin_, applicableMax_, current_epoch_);
    }

    // ---- 越界 / 不可达 → 不命中并标注；**限频**（同一状态跳变只发一次）----
    const std::string label = skipLabel(d.status);
    if (!label.empty() && label != last_skip_label_) {
      last_skip_label_ = label;
      const std::string ev = evidenceString("CAPTURE_SKIP", label, d);
      publishEvidence(ev);
      RCLCPP_WARN(get_logger(), "CAPTURE SKIP：%s", ev.c_str());
    } else if (label.empty()) {
      last_skip_label_.clear();   // 离开标注态 → 允许后续同态重新标注
    }
  }

  // R-021：epoch 值变化 = 新 episode → 复位一次性命中标志与标注消费态。
  void onEpoch(int e)
  {
    if (!have_epoch_ || e != current_epoch_) {
      current_epoch_ = e;
      have_epoch_ = true;
      fired_ = false;
      last_skip_label_.clear();
    }
  }

  // R-021：配套事件串 `CAPTURE_NET ep=<epoch> t=<ros_sec> target=<id>`（可解析；**非**命令者）。
  void publishEvent(const rclcpp::Time & t)
  {
    std::string text = "CAPTURE_NET ep=" + std::to_string(current_epoch_) +
                       " t=" + fmt(t.seconds()) + " target=" + target_id_;
    std_msgs::msg::String m;
    m.data = text;
    event_pub_->publish(m);
  }

  // 标注串：命中 / 越界携带 h、dist、model_id 与适用高度。
  std::string evidenceString(const char * kind, const std::string & value,
    const CaptureDecision & d) const
  {
    return kind + std::string(" ") + value +
           " model_id=" + model_id_ +
           " applicable_min_alt_m=" + fmt(applicableMin_) +
           " applicable_max_alt_m=" + fmt(applicableMax_) +
           " h=" + fmt(d.h) + " dist=" + fmt(d.dist);
  }

  void publishEvidence(const std::string & text)
  {
    std_msgs::msg::String m;
    m.data = text;
    evidence_pub_->publish(m);
  }

  std::string skipLabel(CaptureStatus s) const
  {
    switch (s) {
      case CaptureStatus::kOutOfApplicableRange: return "out_of_applicable_range";
      case CaptureStatus::kUnreachable: return "unreachable";
      default: return "";
    }
  }

  static std::string fmt(double v)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.4f", v);
    return std::string(buf);
  }

  double rate_hz_{20.0};
  bool fired_{false};
  std::string last_skip_label_;
  std::string model_id_{"ballistic_closed_form"};
  std::string jam_model_id_{"behavioral_range_gate"};
  double applicableMin_{0.0}, applicableMax_{2.009};
  double jam_min_{0.0}, jam_max_{2.0}, jam_max_range_{1.0};
  // R-021：任务窗口 / 输入寿命 / episode 身份
  double car_max_age_s_{0.5}, target_max_age_s_{0.5}, confidence_min_{0.5};
  std::string required_target_id_{};
  uint8_t task_state_{0};          // AgcTaskState（0=IDLE；仅 GROUND_APPROACH 为有效任务窗口）
  int current_epoch_{-1};
  bool have_epoch_{false};
  float target_conf_{0.0f};
  std::string target_id_{};
  Pose2 car_, target_;
  rclcpp::Time car_rx_{0, 0, RCL_ROS_TIME}, target_rx_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<CaptureAdjudicator> adjudicator_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hit_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr env_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr evidence_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr jam_env_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<agc_msgs::msg::TargetTrackArray>::SharedPtr target_sub_;
  rclcpp::Subscription<AgcTaskState>::SharedPtr task_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr epoch_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaptureAdjudicatorNode>());
  rclcpp::shutdown();
  return 0;
}
