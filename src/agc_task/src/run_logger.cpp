// run_logger —— M0 最小 run 产物落盘（W6 加厚）
//
// 按 ADR-0007 落 experiments/<campaign>/<run_id>/{config,metrics}：
//   * config/：参数快照 + git commit hash
//   * metrics/summary.json：{run_id, git_commit, seed, scenario, terminal_state,
//                            duration_s, odom_valid_ratio, cmd_vel_rate_hz}
// run_id = <YYYYMMDD>-<scenario>-<seed>。C9 指标（shield_infeasible_rate 等）属 W6。

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <agc_msgs/msg/agc_task_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

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
    const auto odom_valid_topic =
      declare_parameter<std::string>("odom_valid_topic", "/agc/ground/odom_valid");
    const auto cmd_topic =
      declare_parameter<std::string>("cmd_topic", "/agc/ground/cmd_vel");
    const auto state_topic =
      declare_parameter<std::string>("state_topic", "/agc/task/state");

    run_id_ = makeRunId();
    run_dir_ = fs::path(experiments_root_) / campaign_ / run_id_;
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
      [this](agc_msgs::msg::AgcTaskState::SharedPtr m) {terminal_state_ = m->state_name;});

    start_wall_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "run dir: %s (git %s)", run_dir_.c_str(), git_commit_.substr(0, 8).c_str());
  }

  void finalize()
  {
    const double dur = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_wall_).count();
    const double ratio = odom_valid_total_ > 0 ?
      static_cast<double>(odom_valid_true_) / odom_valid_total_ : 0.0;
    const double rate = dur > 0.0 ? cmd_count_ / dur : 0.0;

    std::ofstream out(run_dir_ / "metrics" / "summary.json");
    out << "{\n"
        << "  \"run_id\": \"" << run_id_ << "\",\n"
        << "  \"git_commit\": \"" << git_commit_ << "\",\n"
        << "  \"seed\": " << seed_ << ",\n"
        << "  \"scenario\": \"" << scenario_ << "\",\n"
        << "  \"terminal_state\": \"" << (terminal_state_.empty() ? "IDLE" : terminal_state_) << "\",\n"
        << "  \"duration_s\": " << dur << ",\n"
        << "  \"odom_valid_ratio\": " << ratio << ",\n"
        << "  \"cmd_vel_rate_hz\": " << rate << "\n"
        << "}\n";
    RCLCPP_INFO(get_logger(), "summary written: %s", (run_dir_ / "metrics" / "summary.json").c_str());
  }

private:
  std::string makeRunId() const
  {
    const auto t = std::time(nullptr);
    char buf[16];
    std::strftime(buf, sizeof(buf), "%Y%m%d", std::localtime(&t));
    return std::string(buf) + "-" + scenario_ + "-" + std::to_string(seed_);
  }

  std::string gitCommit() const
  {
    const std::string cmd = "git -C " + git_root_ + " rev-parse HEAD 2>/dev/null";
    std::string result;
    if (FILE * p = popen(cmd.c_str(), "r")) {
      char buf[128];
      if (fgets(buf, sizeof(buf), p)) {result = buf;}
      pclose(p);
    }
    while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
      result.pop_back();
    }
    return result.empty() ? "unknown" : result;
  }

  void writeConfig()
  {
    std::ofstream params(run_dir_ / "config" / "params.yaml");
    params << "campaign: " << campaign_ << "\n"
           << "scenario: " << scenario_ << "\n"
           << "seed: " << seed_ << "\n"
           << "experiments_root: " << experiments_root_ << "\n";
    std::ofstream commit(run_dir_ / "config" / "git_commit.txt");
    commit << git_commit_ << "\n";
  }

  std::string experiments_root_, git_root_, campaign_, scenario_, run_id_, git_commit_;
  int seed_{0};
  std::string terminal_state_{"IDLE"};
  size_t odom_valid_total_{0}, odom_valid_true_{0}, cmd_count_{0};
  std::chrono::steady_clock::time_point start_wall_;
  fs::path run_dir_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odom_valid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<agc_msgs::msg::AgcTaskState>::SharedPtr state_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RunLogger>();
  rclcpp::spin(node);
  node->finalize();
  rclcpp::shutdown();
  return 0;
}