// run_logger —— run 产物落盘（W6 加厚 / C9）
//
// 按 ADR-0007 落 experiments/<campaign>/<run_id>/{config,metrics}：
//   * config/：参数快照 + git commit hash
//   * metrics/summary.json：见下 finalize()。
// C9 两指标（契约 #hard-constraints）：
//   * shield_infeasible_rate —— **重定义**：安全层触发（collision_monitor STOP/SLOWDOWN）
//     且"触发时不可行"（前向扫描 ≤ contact_dist_m，或 TTC < ttc_min_s）的事件占比 =
//     infeasible_ticks / triggered_ticks（无触发时记 0）。与旧义（RL 安全集为空）**不可对照**。
//   * reset_failure_rate —— 本 episode 复位重试耗尽（>3）或复位服务调用失败 → 1.0，否则 0.0
//     （跨 episode 的比率由评估 runner 汇总）。
//
// R-027（ADR-0042）—— 安全不可行率**诚实口径**与**真实接触证据**：
//   本节点同时输出**两套口径**并显式标注，互不冒充：
//   ① **proxy_v1**（历史口径，保留字段不动）：`shield_infeasible_rate` / `ticks_*` ——
//      「安全层触发且前向扫描 ≤ contact_dist_m」的**触发拍代理**，**不要求**实际接触 / 制动失败。
//   ② **gz_contact_v1**（新增真实口径）：以 GZ contact sensor（车模 base_collision ↔ 其它碰撞体）
//      为「真实碰撞」的**真值**，另记 `collision_count` / `collision_events` / `trigger_events` /
//      `intervention_duration_s` / `stopping_margin_*` / `true_shield_infeasible(_rate)`。
//      真实碰撞 = 「车↔非车」接触对，且**排除世界地面**（base_collision 复位瞬间没入 ground_plane
//      属建模产物，非障碍 / 实体碰撞）。
//   除旧字段外，额外输出 `definition_version` 与 `metrics_comparability`：**proxy_v1 与 gz_contact_v1
//   均不与旧 RL 安全集指标直接对照**。零速停于阈值外 → 无 GZ 接触 → `collision_count==0`（天然满足，
//   **不**用 scan≤contact 去推导 collision）。缺数据段以 `not_applicable` 掩码如实标注，**不**以 0 冒充。
//   新增订阅 / 参数（`contact_topic` / `a_brake_mps2`）均为**诊断类**，非 ROS 命令者；**不改**冻结 9 消息 ABI。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <agc_msgs/msg/agc_task_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

namespace fs = std::filesystem;

class RunLogger : public rclcpp::Node
{
public:
  RunLogger()
  : Node("run_logger")
  {
    experiments_root_ = declare_parameter<std::string>("experiments_root", "/home/chen/AGC/experiments");
    git_root_ = declare_parameter<std::string>("git_root", "/home/chen/AGC");
    campaign_ = declare_parameter<std::string>("campaign", "m0-skeleton");
    scenario_ = declare_parameter<std::string>("scenario", "flat");
    seed_ = declare_parameter<int>("seed", 0);
    // R-026：scene_manifest（声明轴 + 承载者 + 实际生效参数）与合并后参数文件路径；
    // 由顶层 launch 生成并经参数单一注入（缺省空 = 不落，不影响既有调用方）。
    scene_manifest_file_ = declare_parameter<std::string>("scene_manifest_file", "");
    params_file_in_ = declare_parameter<std::string>("params_file", "");
    const auto odom_valid_topic =
      declare_parameter<std::string>("odom_valid_topic", "/agc/ground/odom_valid");
    const auto cmd_topic =
      declare_parameter<std::string>("cmd_topic", "/agc/ground/cmd_vel");
    const auto state_topic =
      declare_parameter<std::string>("state_topic", "/agc/task/state");
    const auto odom_topic =
      declare_parameter<std::string>("ground_odom_topic", "/agc/ground/odom");
    const auto scan_topic =
      declare_parameter<std::string>("scan_topic", "/agc/ground/scan");
    const auto cm_state_topic =
      declare_parameter<std::string>("collision_monitor_state_topic", "/collision_monitor_state");
    const auto reset_retries_topic =
      declare_parameter<std::string>("reset_retries_topic", "/agc/task/reset_retries");
    const auto reset_failed_topic =
      declare_parameter<std::string>("reset_failed_topic", "/agc/task/reset_failed");
    // C9 口径阈值（可标定；与 C2 碰撞档 2.0 m 对齐）
    contact_dist_m_ = declare_parameter<double>("contact_dist_m", 2.0);
    ttc_min_s_ = declare_parameter<double>("ttc_min_s", 1.0);
    forward_cone_rad_ = declare_parameter<double>("forward_cone_rad", 0.9);
    // R-027（ADR-0042）：接触真值话题（GZ contact sensor → ROS，ros_gz_interfaces/Contacts）；
    // **空字符串 = 接触真值不可用**（not_applicable.collision_truth=true）。由 launch 单一注入。
    contact_topic_ = declare_parameter<std::string>("contact_topic", "/agc/ground/contact");
    // R-027：锁轮制动减速度（m/s^2）——取自 ADR-0021(C) 实测锁轮制动；供剩余停车裕度
    // brake_d = v^2/(2a)、margin = scan_min_forward - brake_d（可标定量，非保证）。
    a_brake_mps2_ = declare_parameter<double>("a_brake_mps2", 0.9);
    // R-028（ADR-0043）：有效样本掩码的采样新鲜度上限（s）——odom 拍时刻与最近 scan / collision
    // action 快照的年龄超过此上限即视为该拍无效样本（陈旧 scan 不按新 odom 重复计数）。由 launch
    // 单一注入（默认值仅作缺省；节点不得覆写）。
    scan_max_age_s_ = declare_parameter<double>("scan_max_age_s", 0.5);
    action_max_age_s_ = declare_parameter<double>("action_max_age_s", 0.5);
    // R-029（ADR-0044）：episode 身份话题（std_msgs/Int32，transient_local；task_manager start 成功后发布）。
    const auto episode_topic = declare_parameter<std::string>("episode_topic", "/agc/task/episode");
    // R-029：世界模型文件（sha256 记入 config/toolchain_world.json；空 = 不记）。
    world_file_ = declare_parameter<std::string>("world_file", "");
    // R-029：可选 PX4 根目录（记其 git HEAD 作工具链身份；空 = 不记）。
    px4_root_ = declare_parameter<std::string>("px4_root", "");
    // R-029：确定性验证 / 审计用 run_id 覆盖（空 = 用 makeRunId() 生成唯一 id）。
    // 仅改**名字**；独占创建仍拒绝已有目录 → 不削弱「唯一不可覆盖」。
    const auto run_id_override = declare_parameter<std::string>("run_id_override", "");

    run_id_ = run_id_override.empty() ? makeRunId() : run_id_override;
    run_dir_ = fs::path(experiments_root_) / campaign_ / run_id_;
    // R-029：独占创建——先建父目录（campaign），再对 run 目录独占创建；已存在 → 拒绝写入。
    std::error_code ec;
    fs::create_directories(fs::path(experiments_root_) / campaign_, ec);
    if (ec || !fs::create_directory(run_dir_, ec) || ec) {
      RCLCPP_ERROR(get_logger(),
        "拒绝写入已有 run 目录（R-029 唯一 run_id 不可覆盖）：%s", run_dir_.c_str());
      dir_ok_ = false;
      return;
    }
    fs::create_directories(run_dir_ / "config");
    fs::create_directories(run_dir_ / "metrics");

    git_commit_ = gitCommit();
    writeConfig();

    odom_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      odom_valid_topic, 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {++odom_valid_total_; if (m->data) {++odom_valid_true_;}});
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_topic, 50, [this](geometry_msgs::msg::TwistStamped::SharedPtr) {++cmd_count_;});
    state_sub_ = create_subscription<agc_msgs::msg::AgcTaskState>(
      state_topic, 10,
      [this](agc_msgs::msg::AgcTaskState::SharedPtr m) {onState(*m);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 50, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        vx_body_ = m->twist.twist.linear.x;
        have_odom_ = true;
        evaluateTick();
      });
    // 传感器数据用 best-effort QoS（与 ros_gz_bridge SENSOR_DATA 一致，否则不投递）
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) {onScan(*m);});
    cm_sub_ = create_subscription<nav2_msgs::msg::CollisionMonitorState>(
      cm_state_topic, 20, [this](nav2_msgs::msg::CollisionMonitorState::SharedPtr m) {
        cm_action_ = m->action_type;
        have_cm_state_ = true;   // R-028：action 快照到达（有效样本掩码用）
        cm_rx_s_ = now().seconds();
      });
    // 复位记账（task_manager 以 transient_local 发布，晚订阅可获最后值）
    reset_retries_sub_ = create_subscription<std_msgs::msg::Int32>(
      reset_retries_topic, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Int32::SharedPtr m) {reset_retries_ = m->data;});
    reset_failed_sub_ = create_subscription<std_msgs::msg::Bool>(
      reset_failed_topic, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr m) {reset_failed_ = m->data;});
    // R-029（ADR-0044）：episode 身份贯穿日志 / config / summary（非命令者，诊断类）。
    episode_sub_ = create_subscription<std_msgs::msg::Int32>(
      episode_topic, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Int32::SharedPtr m) {
        have_episode_ = true;
        episode_id_ = m->data;
        writeText(run_dir_ / "config" / "episode_id.txt", std::to_string(episode_id_) + "\n");
        RCLCPP_INFO(get_logger(), "episode_id = %d", episode_id_);
      });
    // R-027（ADR-0042）：真实接触真值订阅（GZ contact sensor → ROS）。空话题 = 不可用（不订阅）。
    if (!contact_topic_.empty()) {
      contact_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
        contact_topic_, 10,
        [this](ros_gz_interfaces::msg::Contacts::SharedPtr m) {onContacts(*m);});
    }

    start_wall_ = std::chrono::steady_clock::now();
    wall_start_unix_s_ = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    RCLCPP_INFO(get_logger(), "run dir: %s (git %s)", run_dir_.c_str(), git_commit_.substr(0, 8).c_str());
  }

  bool dirOk() const {return dir_ok_;}

  void finalize()
  {
    const double dur = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_wall_).count();
    wall_end_unix_s_ = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    if (now().seconds() > 0.0) {sim_end_s_ = now().seconds(); have_sim_ = true;}
    const double sim_delta = have_sim_ ? (sim_end_s_ - sim_start_s_) : 0.0;
    const double ratio = odom_valid_total_ > 0 ?
      static_cast<double>(odom_valid_true_) / odom_valid_total_ : 0.0;
    const double rate = dur > 0.0 ? cmd_count_ / dur : 0.0;
    const double shield_rate = triggered_ticks_ > 0 ?
      static_cast<double>(infeasible_ticks_) / triggered_ticks_ : 0.0;
    const double reset_rate = reset_failed_ ? 1.0 : 0.0;
    // R-027（ADR-0042）：诚实口径量 + 有效样本掩码
    const bool scan_derived_na = (total_ticks_ == 0) || (scan_samples_ == 0);
    const bool trigger_derived_na = (trigger_events_ == 0);
    const bool collision_truth_na = contact_topic_.empty();
    // R-027：若 episode 在**仍处于触发期**时结束且该触发期内已发生真实接触，则未发生下降沿——
    // 此处补记一次「真实安全不可行」，避免漏报（honest 优先）。
    const long infeasible_events_total = infeasible_trigger_events_ +
      ((in_trigger_ && current_trigger_has_contact_) ? 1 : 0);
    const bool true_shield_infeasible = infeasible_events_total > 0;

    // R-028（ADR-0043）：finalize 时若仍有开窗 → 关闭它，closed=false（显式标注未闭合）。
    if (current_window_ >= 0) {
      EpisodeWindow & w = windows_[current_window_];
      w.leave_s = have_sim_ ? sim_end_s_ : last_tick_sim_s_;
      w.closed = false;
      w.exit_state = last_state_name_;
      w.exit_reason = reason_code_;
      current_window_ = -1;
    }
    // R-028：终态锁存输出（首个终态；无则回落到最后 state_name；再空则回退 IDLE）。
    const std::string terminal_state_out = !terminal_state_.empty() ? terminal_state_ :
      (!last_state_name_.empty() ? last_state_name_ : std::string("IDLE"));
    const std::string terminal_state_source = terminal_latched_ ? "first_terminal_latched" :
      (!last_state_name_.empty() ? "last_state_no_terminal" : "none");
    // R-028：episode 窗口聚合 + 拒绝可用评价（无任务 / 无 scan / 无 odom 拍 → 不以 0 冒充）。
    const bool has_window = !windows_.empty();
    long agg_ticks_total = 0, agg_ticks_valid = 0;
    long agg_ticks_invalid_scan = 0, agg_ticks_invalid_action = 0;
    long agg_triggered_ticks = 0, agg_infeasible_ticks = 0, agg_scan_samples = 0;
    for (const auto & w : windows_) {
      agg_ticks_total += w.ticks_total;
      agg_ticks_valid += w.ticks_valid;
      agg_ticks_invalid_scan += w.ticks_invalid_scan;
      agg_ticks_invalid_action += w.ticks_invalid_action;
      agg_triggered_ticks += w.triggered_ticks;
      agg_infeasible_ticks += w.infeasible_ticks;
      agg_scan_samples += w.scan_samples;
    }
    std::vector<std::string> na_reasons;
    if (!has_window) {
      na_reasons.push_back("no_episode_window");
    } else {
      if (agg_ticks_total == 0) {na_reasons.push_back("no_odom_ticks_in_window");}
      if (agg_scan_samples == 0) {na_reasons.push_back("no_scan_in_window");}
    }
    const bool evaluation_applicable = na_reasons.empty();
    const bool episode_window_na = !has_window;
    const bool window_scan_na = has_window && (agg_scan_samples == 0);
    const bool window_action_na = has_window && (agg_ticks_total > 0) && (agg_ticks_valid == 0);
    const std::string ratio_scan = agg_ticks_total > 0 ?
      std::to_string(static_cast<double>(agg_ticks_invalid_scan) / agg_ticks_total) : std::string("null");
    const std::string ratio_action = agg_ticks_total > 0 ?
      std::to_string(static_cast<double>(agg_ticks_invalid_action) / agg_ticks_total) : std::string("null");
    const std::string window_rate = (evaluation_applicable && agg_triggered_ticks > 0) ?
      std::to_string(static_cast<double>(agg_infeasible_ticks) / agg_triggered_ticks) : std::string("null");

    std::ofstream out(run_dir_ / "metrics" / "summary.json");
    out << "{\n"
        << "  \"run_id\": \"" << run_id_ << "\",\n"
        << "  \"episode_id\": " << (have_episode_ ? std::to_string(episode_id_) : std::string("null")) << ",\n"
        << "  \"git_commit\": \"" << git_commit_ << "\",\n"
        << "  \"seed\": " << seed_ << ",\n"
        << "  \"scenario\": \"" << scenario_ << "\",\n"
        << "  \"terminal_state\": \"" << terminal_state_out << "\",\n"
        << "  \"terminal_state_latched\": " << (terminal_latched_ ? "true" : "false") << ",\n"
        << "  \"terminal_state_source\": \"" << terminal_state_source << "\",\n"
        << "  \"terminal_reason_code\": \"" << terminal_reason_code_ << "\",\n"
        << "  \"reason_code\": \"" << reason_code_ << "\",\n"
        << "  \"duration_s\": " << dur << ",\n"
        << "  \"odom_valid_ratio\": " << ratio << ",\n"
        << "  \"cmd_vel_rate_hz\": " << rate << ",\n"
        << "  \"reset_retries\": " << reset_retries_ << ",\n"
        << "  \"reset_failed\": " << (reset_failed_ ? "true" : "false") << ",\n"
        // === 历史口径 proxy_v1（**保留不动**，仅加定义版本标注）===
        << "  \"shield_infeasible_rate\": " << shield_rate << ",\n"
        << "  \"reset_failure_rate\": " << reset_rate << ",\n"
        << "  \"ticks_total\": " << total_ticks_ << ",\n"
        << "  \"ticks_triggered\": " << triggered_ticks_ << ",\n"
        << "  \"ticks_infeasible\": " << infeasible_ticks_ << ",\n"
        // R-024 时间口径标注 + 起止事件 + RTF（新增键；duration_s 语义不变）
        << "  \"duration_kind\": \"process_wall_incl_warmup_shutdown\",\n"
        << "  \"wall_start_unix_s\": " << wall_start_unix_s_ << ",\n"
        << "  \"wall_end_unix_s\": " << wall_end_unix_s_ << ",\n"
        << "  \"sim_start_s\": " << (have_sim_ ? std::to_string(sim_start_s_) : std::string("null")) << ",\n"
        << "  \"sim_end_s\": " << (have_sim_ ? std::to_string(sim_end_s_) : std::string("null")) << ",\n"
        << "  \"rtf\": " << (have_sim_ && dur > 0.0 ? std::to_string(sim_delta / dur) : std::string("null")) << ",\n"
        // === R-027 新增：口径版本 + 真实接触证据（gz_contact_v1）===
        << "  \"definition_version\": {\"shield_infeasible_rate\": \"proxy_v1\", "
           "\"collision_truth\": \"gz_contact_v1\", \"true_shield_infeasible\": \"gz_contact_v1\", "
           "\"shield_infeasible_rate_window\": \"window_valid_v1\", "
           "\"terminal_state\": \"first_terminal_latch_v1\"},\n"
        << "  \"metrics_comparability\": \"proxy_v1（进程全时长）与 window_valid_v1（episode 窗口）"
           "及 gz_contact_v1 三口径互不冒充，均不与旧 RL 安全集指标直接对照（口径/分母不同）\",\n"
        << "  \"collision_count\": " << collision_count_ << ",\n"
        << "  \"collision_events\": [";
    for (size_t i = 0; i < collision_events_.size(); ++i) {
      const auto & ev = collision_events_[i];
      if (i > 0) {out << ", ";}
      out << "{\"t_sim\": " << ev.t_sim_s << ", \"other\": \"" << ev.other << "\", \"depth\": "
          << (ev.have_depth ? std::to_string(ev.depth) : std::string("null")) << ", \"position\": ";
      if (ev.have_pos) {
        out << "{\"x\": " << ev.pos_x << ", \"y\": " << ev.pos_y << ", \"z\": " << ev.pos_z << "}";
      } else {
        out << "null";
      }
      out << "}";
    }
    out << "],\n"
        << "  \"contact_msgs_received\": " << contact_msgs_received_ << ",\n"
        << "  \"trigger_events\": " << trigger_events_ << ",\n"
        << "  \"intervention_duration_s\": " << intervention_duration_s_ << ",\n"
        << "  \"stopping_margin_min_m\": "
        << (have_stopping_margin_ ? std::to_string(stopping_margin_min_m_) : std::string("null")) << ",\n"
        << "  \"stopping_margin_negative_ticks\": " << stopping_margin_negative_ticks_ << ",\n"
        << "  \"ticks_triggered_no_scan\": " << ticks_triggered_no_scan_ << ",\n"
        << "  \"true_shield_infeasible\": " << (true_shield_infeasible ? "true" : "false") << ",\n"
        << "  \"true_shield_infeasible_rate\": "
        << (trigger_events_ > 0 ?
            std::to_string(static_cast<double>(infeasible_events_total) / trigger_events_) :
            std::string("null")) << ",\n"
        << "  \"not_applicable\": {\"scan_derived\": " << (scan_derived_na ? "true" : "false")
        << ", \"trigger_derived\": " << (trigger_derived_na ? "true" : "false")
        << ", \"collision_truth\": " << (collision_truth_na ? "true" : "false")
        << ", \"episode_window\": " << (episode_window_na ? "true" : "false")
        << ", \"window_scan\": " << (window_scan_na ? "true" : "false")
        << ", \"window_action\": " << (window_action_na ? "true" : "false") << "},\n"
        // === R-028 新增：episode 窗口统计 / 有效样本掩码 / 拒绝可用评价 ===
        << "  \"evaluation_applicable\": " << (evaluation_applicable ? "true" : "false") << ",\n"
        << "  \"evaluation_na_reasons\": [";
    for (size_t i = 0; i < na_reasons.size(); ++i) {
      if (i > 0) {out << ", ";}
      out << "\"" << na_reasons[i] << "\"";
    }
    out << "],\n"
        << "  \"episode_window_metrics\": {\"window_count\": " << windows_.size()
        << ", \"ticks_total\": " << agg_ticks_total
        << ", \"ticks_valid\": " << agg_ticks_valid
        << ", \"ticks_invalid_scan\": " << agg_ticks_invalid_scan
        << ", \"ticks_invalid_action\": " << agg_ticks_invalid_action
        << ", \"ticks_missing_ratio_scan\": " << ratio_scan
        << ", \"ticks_missing_ratio_action\": " << ratio_action
        << ", \"triggered_ticks\": " << agg_triggered_ticks
        << ", \"infeasible_ticks\": " << agg_infeasible_ticks
        << ", \"scan_samples\": " << agg_scan_samples
        << ", \"shield_infeasible_rate\": " << window_rate << "},\n"
        << "  \"episode_windows\": [";
    for (size_t i = 0; i < windows_.size(); ++i) {
      const auto & w = windows_[i];
      if (i > 0) {out << ", ";}
      out << "{\"index\": " << w.index
          << ", \"enter_s\": " << w.enter_s
          << ", \"leave_s\": " << w.leave_s
          << ", \"closed\": " << (w.closed ? "true" : "false")
          << ", \"exit_state\": \"" << w.exit_state << "\""
          << ", \"exit_reason\": \"" << w.exit_reason << "\""
          << ", \"ticks_total\": " << w.ticks_total
          << ", \"ticks_valid\": " << w.ticks_valid
          << ", \"ticks_invalid_scan\": " << w.ticks_invalid_scan
          << ", \"ticks_invalid_action\": " << w.ticks_invalid_action
          << ", \"triggered_ticks\": " << w.triggered_ticks
          << ", \"infeasible_ticks\": " << w.infeasible_ticks
          << ", \"scan_samples\": " << w.scan_samples
          << ", \"shield_infeasible_rate\": "
          << (w.triggered_ticks > 0 ?
              std::to_string(static_cast<double>(w.infeasible_ticks) / w.triggered_ticks) :
              std::string("null"))
          << "}";
    }
    out << "],\n"
        << "  \"process_metrics_note\": \"duration_s / odom_valid_ratio / cmd_vel_rate_hz / ticks_* / "
           "shield_infeasible_rate 为**进程全时长**口径（含预热 / 关闭），与 episode_window_metrics 的 "
           "episode 窗口口径互不冒充。\"\n"
        << "}\n";
    RCLCPP_INFO(get_logger(), "summary written: %s", (run_dir_ / "metrics" / "summary.json").c_str());
  }

private:
  // R-027：一次**真实碰撞事件**（GZ contact 真值；独立于 proxy_v1「触发拍代理」计数）。
  struct CollisionEvent
  {
    double t_sim_s{0.0};   // now().seconds()（可能为 0 —— 仿真时间未起步；如实记录）
    std::string other;     // 非车侧碰撞体名（含 pole / 其它实体）
    bool have_depth{false};
    double depth{0.0};     // 该对接触的最小穿透深度（无有效 depth → null）
    bool have_pos{false};
    double pos_x{0.0}, pos_y{0.0}, pos_z{0.0};  // 该对接触的首个接触点（world 系；无 → null）
  };

  // R-028（ADR-0043）：一次 episode 事件窗口（task 进入 active 开窗、离开 active（终态 / IDLE）关窗）。
  // 窗口内以**同时间快照 + 有效样本掩码**逐拍计数（陈旧 scan 一律进 ticks_invalid_scan）。
  struct EpisodeWindow
  {
    int index{0};
    double enter_s{0.0}, leave_s{0.0};   // 开 / 关窗时刻（now().seconds()；可能为 0，仿真未起步）
    bool closed{false};                  // false = finalize 时仍在开窗（显式标注未闭合）
    std::string exit_state, exit_reason; // 关窗时的 state_name / reason_code
    long ticks_total{0}, ticks_valid{0}, ticks_invalid_scan{0}, ticks_invalid_action{0};
    long triggered_ticks{0}, infeasible_ticks{0}, scan_samples{0};
  };

  // R-028：active / 终态判据（与 task_manager 对齐；active = AIR_SEARCH|TARGET_ACQUIRED|GROUND_APPROACH）。
  static bool isActiveState(uint8_t s)
  {
    return s == agc_msgs::msg::AgcTaskState::AIR_SEARCH ||
           s == agc_msgs::msg::AgcTaskState::TARGET_ACQUIRED ||
           s == agc_msgs::msg::AgcTaskState::GROUND_APPROACH;
  }

  static bool isTerminalState(uint8_t s)
  {
    return s == agc_msgs::msg::AgcTaskState::TRACKING_SUCCESS ||
           s == agc_msgs::msg::AgcTaskState::INTERCEPT_CONFIRMED ||
           s == agc_msgs::msg::AgcTaskState::MISSION_FAILED ||
           s == agc_msgs::msg::AgcTaskState::ABORTED ||
           s == agc_msgs::msg::AgcTaskState::INFRA_FAILURE;
  }

  void onScan(const sensor_msgs::msg::LaserScan & m)
  {
    ++scan_samples_;  // R-027：scan 帧计数（供 not_applicable.scan_derived 掩码）
    scan_rx_s_ = now().seconds();  // R-028：同时间快照——最近 scan 到达时刻
    if (current_window_ >= 0) {++windows_[current_window_].scan_samples;}  // R-028：窗口内 scan 帧
    double min_r = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < m.ranges.size(); ++i) {
      const double a = m.angle_min + static_cast<double>(i) * m.angle_increment;
      if (std::abs(a) > forward_cone_rad_) {continue;}
      const double r = m.ranges[i];
      if (std::isfinite(r) && r > 0.0) {min_r = std::min(min_r, r);}
    }
    scan_min_forward_ = min_r;
    have_scan_ = std::isfinite(min_r);
  }

  // R-028：task 状态回调——① 终态锁存（首个终态，之后不得覆写）；② episode 事件窗口分段
  // （进入 active 开窗、离开 active（终态 / IDLE）关窗）。active / 终态定义见
  // [#glossary] …… 与 task_manager 对齐：active = AIR_SEARCH|TARGET_ACQUIRED|GROUND_APPROACH。
  void onState(const agc_msgs::msg::AgcTaskState & m)
  {
    last_state_name_ = m.state_name;
    reason_code_ = m.reason_code;  // 语义不变：最后一次 state 的 reason_code
    const uint8_t s = m.state;
    // 终态锁存：首个终态写入一次；之后任何 state 消息都不得覆写。
    if (isTerminalState(s) && !terminal_latched_) {
      terminal_latched_ = true;
      terminal_state_ = m.state_name;
      terminal_reason_code_ = m.reason_code;
    }
    // episode 事件窗口：active 且无开窗 → 开窗；有开窗且（终态 | IDLE）→ 关窗。
    if (isActiveState(s) && current_window_ < 0) {
      EpisodeWindow w;
      w.index = static_cast<int>(windows_.size());
      w.enter_s = now().seconds();
      windows_.push_back(w);
      current_window_ = w.index;
    } else if (current_window_ >= 0 && (isTerminalState(s) || s == agc_msgs::msg::AgcTaskState::IDLE)) {
      EpisodeWindow & w = windows_[current_window_];
      w.leave_s = now().seconds();
      w.closed = true;
      w.exit_state = (s == agc_msgs::msg::AgcTaskState::IDLE) ?
        std::string("IDLE") : m.state_name;
      w.exit_reason = m.reason_code;
      current_window_ = -1;
    }
  }

  // R-027（ADR-0042）：GZ contact 真值回调——逐帧判定**车↔非车**接触对，按**上升沿去重**计真实碰撞事件。
  // 车侧：名字含 `agc_car`；另一侧：名字非空且不含 `agc_car`（车↔车 / 非车↔非车 / 名缺失不计）。
  void onContacts(const ros_gz_interfaces::msg::Contacts & m)
  {
    ++contact_msgs_received_;
    std::unordered_set<std::string> frame_pairs;          // 本帧处于接触的对（键=非车侧名）
    std::unordered_map<std::string, double> frame_depth;  // 各对最小 depth
    std::unordered_map<std::string, std::array<double, 3>> frame_pos;  // 各对首个接触点
    for (const auto & c : m.contacts) {
      const std::string & n1 = c.collision1.name;
      const std::string & n2 = c.collision2.name;
      const bool car1 = n1.find("agc_car") != std::string::npos;
      const bool car2 = n2.find("agc_car") != std::string::npos;
      std::string other;
      if (car1 && !car2 && !n2.empty()) {
        other = n2;
      } else if (car2 && !car1 && !n1.empty()) {
        other = n1;
      } else {
        continue;  // 非「车↔非车」对：非本口径的真实碰撞
      }
      // R-027：排除世界地面（base_collision 在复位瞬间轻微没入 ground_plane 属建模产物，
      // 非**障碍 / 实体**碰撞；真实碰撞只对场景实体）。车体正常由车轮触地，非 base_collision。
      if (other.find("ground_plane") != std::string::npos) {continue;}
      frame_pairs.insert(other);
      double d = std::numeric_limits<double>::infinity();
      for (double p : c.depths) {
        if (std::isfinite(p)) {d = std::min(d, p);}
      }
      auto it = frame_depth.find(other);
      if (it == frame_depth.end() || d < it->second) {frame_depth[other] = d;}
      // R-027：记该对**首个接触点**（world 系；供「接触事件含几何」证据）
      if (frame_pos.find(other) == frame_pos.end() && !c.positions.empty() &&
        std::isfinite(c.positions.front().x) && std::isfinite(c.positions.front().y) &&
        std::isfinite(c.positions.front().z))
      {
        frame_pos[other] = {c.positions.front().x, c.positions.front().y, c.positions.front().z};
      }
    }
    // 上升沿：上一帧无此对、本帧有 → 记一次真实碰撞事件
    for (const auto & key : frame_pairs) {
      if (contact_pairs_prev_.count(key) == 0) {
        CollisionEvent ev;
        ev.t_sim_s = now().seconds();  // 可能为 0（仿真时间未起步）；如实记录
        ev.other = key;
        const auto it = frame_depth.find(key);
        if (it != frame_depth.end() && std::isfinite(it->second)) {
          ev.have_depth = true;
          ev.depth = it->second;
        }
        const auto pit = frame_pos.find(key);
        if (pit != frame_pos.end()) {
          ev.have_pos = true;
          ev.pos_x = pit->second[0]; ev.pos_y = pit->second[1]; ev.pos_z = pit->second[2];
        }
        collision_events_.push_back(ev);
        ++collision_count_;
        if (in_trigger_) {current_trigger_has_contact_ = true;}
      }
    }
    contact_pairs_prev_ = std::move(frame_pairs);
  }

  // R-024：在 odom 回调时间轴上采到首个非零仿真时间（run 证据 sim_start_s）
  void noteSim()
  {
    if (!have_sim_ && now().seconds() > 0.0) {
      sim_start_s_ = now().seconds();
      have_sim_ = true;
    }
  }

  // C9 shield_infeasible_rate【proxy_v1，保留历史口径】 + R-027【gz_contact_v1 真实口径】。
  void evaluateTick()
  {
    noteSim();
    ++total_ticks_;
    const bool triggered = (cm_action_ == 1 /*STOP*/ || cm_action_ == 2 /*SLOWDOWN*/);
    const double t_now = now().seconds();
    // R-028：同时间快照 + 有效样本掩码——该拍此刻的 scan / collision action 新鲜度。
    const bool scan_fresh = have_scan_ && (t_now - scan_rx_s_) <= scan_max_age_s_;
    const bool action_fresh = have_cm_state_ && (t_now - cm_rx_s_) <= action_max_age_s_;
    const bool tick_valid = scan_fresh && action_fresh;

    // R-027：触发事件计数（上升沿 triggered && !prev_triggered_）；进入触发期时重置「本触发期是否有真实接触」。
    if (triggered && !prev_triggered_) {
      ++trigger_events_;
      current_trigger_has_contact_ = false;
    }
    // R-027：干预时长——触发期间按仿真时钟增量累计（dt>0 才累加）。
    if (triggered) {
      const double dt = t_now - last_tick_sim_s_;
      if (dt > 0.0) {intervention_duration_s_ += dt;}
    }
    last_tick_sim_s_ = t_now;
    // R-027：触发结束（下降沿 !triggered && prev_triggered_）——若本触发期发生过真实碰撞 → 真实安全不可行。
    if (!triggered && prev_triggered_ && current_trigger_has_contact_) {
      ++infeasible_trigger_events_;
    }
    prev_triggered_ = triggered;
    in_trigger_ = triggered;

    // === proxy_v1（保留不动）：触发拍统计 + 「前向扫描 ≤ contact 或 TTC<阈值」→ infeasible_ticks_ ===
    if (triggered) {
      ++triggered_ticks_;
      if (!have_scan_) {
        ++ticks_triggered_no_scan_;  // R-027：无 scan 的触发拍（not_applicable.scan_derived 的证据）
      } else {
        // R-027：剩余停车裕度 brake_d = v^2/(2a)、margin = scan_min_forward - brake_d。
        const double v = std::abs(vx_body_);
        const double brake_d = (a_brake_mps2_ > 1e-6) ? (v * v) / (2.0 * a_brake_mps2_) : 0.0;
        const double margin = scan_min_forward_ - brake_d;
        if (!have_stopping_margin_ || margin < stopping_margin_min_m_) {stopping_margin_min_m_ = margin;}
        have_stopping_margin_ = true;
        if (margin < 0.0) {++stopping_margin_negative_ticks_;}

        const double ttc = v > 1e-3 ? scan_min_forward_ / v : std::numeric_limits<double>::infinity();
        const bool contact = scan_min_forward_ <= contact_dist_m_;
        if (contact || ttc < ttc_min_s_) {++infeasible_ticks_;}
      }
    }

    // === R-028：episode 窗口逐拍计数（仅当有开窗）===
    // 关键语义：陈旧 scan 段落一律进 ticks_invalid_scan，**不**计入窗口 valid / triggered /
    // infeasible —— 即「陈旧 scan 不按新 odom 重复计数」。
    if (current_window_ >= 0) {
      EpisodeWindow & w = windows_[current_window_];
      ++w.ticks_total;
      if (tick_valid) {
        ++w.ticks_valid;
        if (triggered) {
          ++w.triggered_ticks;
          const double v = std::abs(vx_body_);
          const double ttc = v > 1e-3 ? scan_min_forward_ / v :
            std::numeric_limits<double>::infinity();
          if (scan_min_forward_ <= contact_dist_m_ || ttc < ttc_min_s_) {++w.infeasible_ticks;}
        }
      } else if (!scan_fresh) {
        ++w.ticks_invalid_scan;
      } else {
        ++w.ticks_invalid_action;
      }
    }
  }

  std::string makeRunId() const
  {
    // R-029：唯一且保留 `-<scenario>-<seed>` 子串（既有 glob 兼容）+ 时刻 + 随机后缀。
    const auto t = std::time(nullptr);
    char day[16], hms[16];
    std::strftime(day, sizeof(day), "%Y%m%d", std::localtime(&t));
    std::strftime(hms, sizeof(hms), "%H%M%S", std::localtime(&t));
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(0, 9999);
    char rnd[8];
    std::snprintf(rnd, sizeof(rnd), "%04d", dist(gen));
    return std::string(day) + "-" + scenario_ + "-" + std::to_string(seed_) + "-" +
           std::string(hms) + "-" + std::string(rnd);
  }

  std::string runShellRaw(const std::string & cmd) const
  {
    std::string result;
    if (FILE * p = popen(cmd.c_str(), "r")) {
      char buf[4096];
      while (fgets(buf, sizeof(buf), p)) {result += buf;}
      pclose(p);
    }
    return result;
  }

  std::string runShell(const std::string & cmd) const
  {
    std::string result = runShellRaw(cmd);
    while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {result.pop_back();}
    return result;
  }

  void writeText(const fs::path & p, const std::string & s) const
  {
    std::ofstream o(p, std::ios::binary);
    o << s;
  }

  std::string sha256OfFile(const std::string & path) const
  {
    const std::string out = runShell("sha256sum \"" + path + "\" 2>/dev/null");
    return out.size() >= 64 ? out.substr(0, 64) : std::string("null");
  }

  std::string sha256OfText(const std::string & text) const
  {
    const fs::path tmp = run_dir_ / "config" / ".fp_tmp";
    writeText(tmp, text);
    const std::string h = sha256OfFile(tmp.string());
    std::error_code ec;
    fs::remove(tmp, ec);
    return h;
  }

  std::string envOr(const char * key, const char * fallback) const
  {
    const char * v = std::getenv(key);
    return (v && *v) ? std::string(v) : std::string(fallback);
  }

  static size_t countLines(const std::string & s)
  {
    size_t n = 0;
    for (char c : s) {if (c == '\n') {++n;}}
    return n;
  }

  std::string gitCommit() const
  {
    const std::string h = runShell("git -C \"" + git_root_ + "\" rev-parse HEAD 2>/dev/null");
    return h.empty() ? "unknown" : h;
  }

  void writeConfig()
  {
    std::ofstream params(run_dir_ / "config" / "params.yaml");
    params << "campaign: " << campaign_ << "\n"
           << "scenario: " << scenario_ << "\n"
           << "seed: " << seed_ << "\n"
           << "experiments_root: " << experiments_root_ << "\n"
           << "contact_dist_m: " << contact_dist_m_ << "\n"
           << "ttc_min_s: " << ttc_min_s_ << "\n"
           << "forward_cone_rad: " << forward_cone_rad_ << "\n";
    std::ofstream commit(run_dir_ / "config" / "git_commit.txt");
    commit << git_commit_ << "\n";
    // R-026：把本题的 scene_manifest（声明轴 / 承载者 / 实际生效参数）与合并后参数文件
    // 拷入 run 的 config/（ADR-0007 唯一落点），使「用的是哪套栈 / 声明轴 / 实际参数」可复核。
    copyIntoConfig(scene_manifest_file_, "scene_manifest.json");
    copyIntoConfig(params_file_in_, "params_applied.yaml");
    writeEpisodePlaceholder();
    writeIdentity();
  }

  // R-029：episode_id 初值（-1 = 未收到）；收到 /agc/task/episode 后覆写。
  void writeEpisodePlaceholder() const
  {
    writeText(run_dir_ / "config" / "episode_id.txt", "-1\n");
  }

  // R-029（ADR-0044）：代码身份 / dirty diff / 未跟踪清单 / 内容指纹 + 工具链与世界模型哈希。
  // 只落 run 的 config/（ADR-0007）；dirty 工作区非禁跑，但 run 可据此重建。
  void writeIdentity() const
  {
    const std::string head = git_commit_;
    const std::string status = runShellRaw("git -C \"" + git_root_ + "\" status --porcelain 2>/dev/null");
    const std::string diff = runShellRaw("git -C \"" + git_root_ + "\" diff 2>/dev/null");
    const std::string untracked = runShellRaw(
      "git -C \"" + git_root_ + "\" ls-files --others --exclude-standard 2>/dev/null");
    writeText(run_dir_ / "config" / "git_head.txt", head + "\n");
    writeText(run_dir_ / "config" / "git_status_porcelain.txt", status);
    writeText(run_dir_ / "config" / "git_diff.patch", diff);
    writeText(run_dir_ / "config" / "git_untracked.txt", untracked);
    const std::string fp_src = head + "\n--status--\n" + status + "\n--diff--\n" + diff;
    std::string world_hash = "null";
    if (!world_file_.empty()) {
      std::error_code ec;
      if (fs::exists(world_file_, ec)) {world_hash = sha256OfFile(world_file_);}
    }
    const std::string px4_head = px4_root_.empty() ? std::string("") :
      runShell("git -C \"" + px4_root_ + "\" rev-parse HEAD 2>/dev/null");
    {
      std::ofstream o(run_dir_ / "config" / "workspace_fingerprint.json");
      o << "{\n"
        << "  \"git_head\": \"" << head << "\",\n"
        << "  \"dirty\": " << (status.empty() ? "false" : "true") << ",\n"
        << "  \"status_entries\": " << countLines(status) << ",\n"
        << "  \"diff_sha256\": \"" << sha256OfText(diff) << "\",\n"
        << "  \"fingerprint_sha256\": \"" << sha256OfText(fp_src) << "\",\n"
        << "  \"note\": \"内容指纹 = sha256(git HEAD + status --porcelain + diff)；dirty 非禁跑，可据此重建\"\n"
        << "}\n";
    }
    {
      std::ofstream o(run_dir_ / "config" / "toolchain_world.json");
      o << "{\n"
        << "  \"ros_distro\": \"" << envOr("ROS_DISTRO", "unknown") << "\",\n"
        << "  \"gz_sim_versions\": \"" << runShell("gz sim --versions 2>/dev/null") << "\",\n"
        << "  \"code_root\": \"" << git_root_ << "\",\n"
        << "  \"git_head\": \"" << head << "\",\n"
        << "  \"px4_root\": \"" << px4_root_ << "\",\n"
        << "  \"px4_head\": \"" << px4_head << "\",\n"
        << "  \"world_file\": \"" << world_file_ << "\",\n"
        << "  \"world_sha256\": " << (world_hash == "null" ? std::string("null") : "\"" + world_hash + "\"") << "\n"
        << "}\n";
    }
  }

  void copyIntoConfig(const std::string & src, const std::string & dst_name)
  {
    if (src.empty()) {return;}
    std::error_code ec;
    if (!fs::exists(src, ec)) {
      RCLCPP_WARN(get_logger(), "R-026：源文件不存在，跳过拷贝 %s", src.c_str());
      return;
    }
    fs::copy_file(src, run_dir_ / "config" / dst_name, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "R-026：拷贝 %s → config/%s 失败：%s",
        src.c_str(), dst_name.c_str(), ec.message().c_str());
    }
  }

  std::string experiments_root_, git_root_, campaign_, scenario_, run_id_, git_commit_;
  std::string scene_manifest_file_, params_file_in_;
  std::string world_file_, px4_root_;
  int episode_id_{-1};
  bool have_episode_{false};
  bool dir_ok_{true};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr episode_sub_;
  int seed_{0};
  // R-028（ADR-0043）：终态锁存——terminal_state_ = **首个**终态 state_name（空=未观测）；
  // last_state_name_ = 最后观测到的 state_name；terminal_latched_ 标注是否已锁存。
  std::string terminal_state_, last_state_name_, terminal_reason_code_, reason_code_;
  bool terminal_latched_{false};
  size_t odom_valid_total_{0}, odom_valid_true_{0}, cmd_count_{0};
  double contact_dist_m_{2.0}, ttc_min_s_{1.0}, forward_cone_rad_{0.9};
  // R-027（ADR-0042）：接触真值话题（空 = 不可用）+ 锁轮制动减速度
  std::string contact_topic_;
  double a_brake_mps2_{0.9};

  // C9
  int reset_retries_{0};
  bool reset_failed_{false};
  uint8_t cm_action_{0};
  double vx_body_{0.0}, scan_min_forward_{0.0};
  bool have_odom_{false}, have_scan_{false};
  size_t total_ticks_{0}, triggered_ticks_{0}, infeasible_ticks_{0};

  // R-027（gz_contact_v1）：真实接触事件 + 触发事件 / 干预时长 / 剩余停车裕度 / 有效样本掩码
  uint64_t collision_count_{0}, contact_msgs_received_{0}, scan_samples_{0};
  long trigger_events_{0}, infeasible_trigger_events_{0};
  long ticks_triggered_no_scan_{0}, stopping_margin_negative_ticks_{0};
  double intervention_duration_s_{0.0}, stopping_margin_min_m_{0.0};
  bool have_stopping_margin_{false};
  bool prev_triggered_{false}, in_trigger_{false}, current_trigger_has_contact_{false};
  double last_tick_sim_s_{0.0};
  std::vector<CollisionEvent> collision_events_;
  std::unordered_set<std::string> contact_pairs_prev_;

  // R-028（ADR-0043）：有效样本掩码新鲜度阈值 + scan / action 快照到达时刻 + episode 事件窗口。
  double scan_max_age_s_{0.5}, action_max_age_s_{0.5};
  double scan_rx_s_{0.0}, cm_rx_s_{0.0};
  bool have_cm_state_{false};
  std::vector<EpisodeWindow> windows_;
  int current_window_{-1};

  std::chrono::steady_clock::time_point start_wall_;
  // R-024：起止事件 + RTF（sim 缺省 -1.0；have_sim_ 标注是否采到仿真时间）
  double sim_start_s_{-1.0}, sim_end_s_{-1.0};
  bool have_sim_{false};
  double wall_start_unix_s_{0.0}, wall_end_unix_s_{0.0};
  fs::path run_dir_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odom_valid_sub_, reset_failed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<agc_msgs::msg::AgcTaskState>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav2_msgs::msg::CollisionMonitorState>::SharedPtr cm_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reset_retries_sub_;
  // R-027：接触真值订阅（contact_topic 为空时不创建）
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RunLogger>();
  if (!node->dirOk()) {
    RCLCPP_ERROR(node->get_logger(), "run 目录已存在 → 拒绝写入（退出，不重写已有 summary）");
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::spin(node);
  node->finalize();
  rclcpp::shutdown();
  return 0;
}
