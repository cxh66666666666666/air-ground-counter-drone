// ground_baseline —— W5-c 离线对照基线（R8）
//
// 依 ADR-0004 / ADR-0018：地面主控维持 RL（agc_rl），Nav2 planner/controller 退为**可切换的离线基线**。
// 本节点复用 Nav2 的 **plugin 抽象**（ADR-0008 吸收点）：
//   * planner  = pluginlib 加载 Nav2 `navfn_planner` 插件（真复用 Nav2 planner，非自研）；
//   * controller = pluginlib 加载 `agc_control` 的 LQR 控制器插件（共用实现，未复制，ADR-0018）。
// 自持内建 costmap（`nav2_costmap_2d::Costmap2DROS`，节点名 `costmap`；参数取 `config/costmap.yaml`
// 的 `costmap:` 段），与 planner 共用一个 costmap（对齐 Nav2 planner_server 的宿主模式）。
//
// 输出 `TwistStamped`（车体系）至 `/agc/ground/cmd_vel_raw`，**SHALL NOT** 直接写唯一执行口。
// 与 RL 的**互斥**由 ground_bringup 的 `command_source` 单一选择（同一时刻仅一路下发）。
//
// R-041：目标失效与重规划抖动（ADR-0056）——**移除** goal 回调「任意浮点变化（>1e-6）即强制重规划」，
//   重规划仅由 needReplan 的位移阈值（replan_displacement_m）/ 地图变化 / 时限 / 周期触发（对齐 Nav2
//   GoalUpdated 离散事件语义）；仅**首次**获得 goal 时立即规划。消费伴随诊断 /agc/ground/goal_info 的
//   `valid=`：目标失效（valid=false）→ 安全停车（fail-safe stop，先于路径跟随）。与 C10 空中 freeze 分别定义。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/node_thread.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_ros/buffer.h>

#include <agc_msgs/msg/agc_task_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

#include "agc_control/controller.hpp"
#include "agc_control/param_reader.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
// 重规划限流：刚规划完的短窗口内不重复重规划（避免同 tick 抖动；非地图/目标变化触发）
constexpr double kReplanGuardS = 0.2;

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrapPi(double a)
{
  while (a > kPi) {a -= 2.0 * kPi;}
  while (a < -kPi) {a += 2.0 * kPi;}
  return a;
}

// R-041：从伴随诊断 `valid=<0|1>;...` 解析目标有效性。含 `valid=0` → false；否则 true（默认有效，
//   兼容无此话题 / 消息的旧运行，保持既有行为）。
bool parseGoalInfoValid(const std::string & s)
{
  return s.find("valid=0") == std::string::npos;
}
}  // namespace

class GroundBaseline : public nav2_util::LifecycleNode
{
public:
  explicit GroundBaseline(const rclcpp::NodeOptions & options)
  : nav2_util::LifecycleNode("ground_baseline", "", options),
    planner_loader_("nav2_core", "nav2_core::GlobalPlanner"),
    ctrl_loader_("agc_control", "agc_control::Controller")
  {
    base_frame_ = declare_parameter<std::string>("base_frame_id", "car/base_link");
    global_frame_ = declare_parameter<std::string>("global_frame_id", "world");
    rate_hz_ = declare_parameter<double>("rate_hz", 20.0);
    planner_plugin_ = declare_parameter<std::string>(
      "planner_plugin", "nav2_navfn_planner::NavfnPlanner");
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    ctrl_plugin_ = declare_parameter<std::string>(
      "controller_plugin", "agc_control/LqrController");
    replan_period_s_ = declare_parameter<double>("replan_period_s", 2.0);
    replan_disp_m_ = declare_parameter<double>("replan_displacement_m", 1.0);
    max_path_age_s_ = declare_parameter<double>("max_path_age_s", 5.0);
    // stop_on_invalid：路径失效（规划失败 / 绑定破坏）时是否失效停车（默认 fail-stop）
    stop_on_invalid_ = declare_parameter<bool>("stop_on_invalid", true);
    goal_tol_m_ = declare_parameter<double>("goal_reached_tol_m", 0.3);
    lookahead_m_ = declare_parameter<double>("lookahead_m", 1.0);
    k_yaw_ = declare_parameter<double>("k_yaw", 1.5);
    v_ref_ = declare_parameter<double>("reference_speed", 1.0);
    v_max_ = declare_parameter<double>("max_linear_velocity", 1.5);
    w_max_ = declare_parameter<double>("max_angular_velocity", 2.0);

    odom_topic_ = declare_parameter<std::string>("odom_topic", "/agc/ground/odom");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/agc/ground/goal");
    state_topic_ = declare_parameter<std::string>("state_topic", "/agc/task/state");
    // R-041：目标有效性 / 身份 / 时刻伴随诊断话题（std_msgs/String，transient_local，非命令者）。
    //   消费其中的 valid=；valid=false → 失效安全停车（fail-safe stop，先于路径跟随）。
    goal_info_topic_ = declare_parameter<std::string>(
      "goal_info_topic", "/agc/ground/goal_info");
    cmd_raw_topic_ = declare_parameter<std::string>(
      "cmd_raw_topic", "/agc/ground/cmd_vel_raw");
    path_topic_ = declare_parameter<std::string>("path_topic", "/agc/ground/path");

    // 内建 costmap：节点名 `costmap`，参数取 `config/costmap.yaml` 的 `costmap:` 段（同一参数源）
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "costmap", std::string(get_namespace()), "",
      get_parameter("use_sim_time").as_bool());

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_raw_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 1);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        x_ = m->pose.pose.position.x; y_ = m->pose.pose.position.y;
        yaw_ = yawFromQuat(m->pose.pose.orientation);
        // odom twist 表达在 child（车体）系 → 旋转到 world（R(yaw)·v_body）
        const auto & vb = m->twist.twist.linear;
        vx_ = std::cos(yaw_) * vb.x - std::sin(yaw_) * vb.y;
        vy_ = std::sin(yaw_) * vb.x + std::cos(yaw_) * vb.y;
        have_odom_ = true;
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 20,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
        // R-041：**移除**「任意浮点变化（>1e-6）即强制重规划」——重规划改由 needReplan 的
        //   位移阈值（replan_disp_m_）/ 地图变化 / 时限 / 周期触发（对齐 Nav2 GoalUpdated 离散事件，
        //   非逐消息浮点抖动）。仅在**首次**获得 goal（合法实质绑定）时立即规划。
        goal_x_ = m->pose.position.x;
        goal_y_ = m->pose.position.y;
        if (!have_goal_) {have_goal_ = true; force_replan_ = true;}
      });
    // R-041：消费目标有效性伴随诊断——valid=false → 目标失效，安全停车（见 tick）。
    goal_info_sub_ = create_subscription<std_msgs::msg::String>(
      goal_info_topic_, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::SharedPtr m) {
        const bool v = parseGoalInfoValid(m->data);
        if (v != goal_valid_) {
          goal_valid_ = v;
          RCLCPP_WARN(
            get_logger(), "R-041：目标有效性 goal_valid=%s（%s）",
            v ? "true" : "false", m->data.c_str());
        }
      });
    state_sub_ = create_subscription<agc_msgs::msg::AgcTaskState>(
      state_topic_, 20,
      [this](agc_msgs::msg::AgcTaskState::SharedPtr m) {state_ = m->state;});

    const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1e-3));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GroundBaseline::tick, this));
  }

  ~GroundBaseline() override
  {
    planner_.reset();
    costmap_thread_.reset();
  }

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configuring（内建 costmap + Nav2 planner + agc_control LQR）");
    costmap_ros_->configure();
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
    tf_ = costmap_ros_->getTfBuffer();
    try {
      planner_ = planner_loader_.createUniqueInstance(planner_plugin_);
      planner_->configure(shared_from_this(), planner_id_, tf_, costmap_ros_);
      agc_control::NodeParamReader ctrl_params(get_node_parameters_interface(), "lqr.");
      controller_ = ctrl_loader_.createSharedInstance(ctrl_plugin_);
      controller_->configure(ctrl_params);
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(get_logger(), "插件加载失败：%s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "planner=%s controller=%s",
      planner_plugin_.c_str(), controller_->name().c_str());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    const auto cm_state = costmap_ros_->activate();
    if (cm_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return nav2_util::CallbackReturn::FAILURE;
    }
    planner_->activate();
    controller_->reset();
    active_ = true;
    createBond();
    RCLCPP_INFO(get_logger(), "Active（基线成为命令源，写 %s）", cmd_raw_topic_.c_str());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    publish(0.0, 0.0);
    planner_->deactivate();
    costmap_ros_->deactivate();
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    if (planner_) {planner_->cleanup(); planner_.reset();}
    tf_.reset();
    costmap_ros_->cleanup();
    costmap_thread_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  // ---------------------------------------------------------------- 控制循环
  // costmap 内容签名（尺寸 / 原点 / **硬障碍栅格**），用于检测地图变化（R-012）。
  // 复用 Nav2 组件 `Costmap2DROS::getCostmap()` + `Costmap2D::getCharMap()`（本机 Jazzy 头文件已核验）。
  // 只计入**硬障碍**（cost ≥ INSCRIBED_INFLATED_OBSTACLE=253，即 lethal / inscribed），
  // 以屏蔽 obstacle/voxel 层逐帧 raytrace clearing（FREE/UNKNOWN 翻转）与 inflation 传播的噪声：
  // 实测（r012 GUI）全景内容哈希在行驶中会瞬态抖动，而硬障碍集在行驶中稳定、增障时即偏离。
  uint64_t mapSignature() const
  {
    auto * cm = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
    if (cm == nullptr) {return 0;}
    const unsigned char * data = cm->getCharMap();
    const unsigned int sx = cm->getSizeInCellsX();
    const unsigned int sy = cm->getSizeInCellsY();
    if (data == nullptr || sx == 0 || sy == 0) {return 0;}
    uint64_t h = 1469598103934665603ULL;  // FNV-1a 64 位偏移基
    const auto mix = [&h](uint64_t v) {
        for (int i = 0; i < 8; ++i) {h ^= (v >> (8 * i)) & 0xFFULL; h *= 1099511628211ULL;}
      };
    mix(sx); mix(sy);
    double ox = cm->getOriginX(), oy = cm->getOriginY();
    uint64_t oxb = 0, oyb = 0;
    std::memcpy(&oxb, &ox, sizeof(oxb)); std::memcpy(&oyb, &oy, sizeof(oyb));
    mix(oxb); mix(oyb);
    const unsigned int n = sx * sy;
    for (unsigned int i = 0; i < n; ++i) {
      if (data[i] >= 253) {mix(i);}   // 仅硬障碍栅格参与签名
    }
    return h;
  }

  // 地图自上次规划以来是否变化：costmap 栅格内容签名（尺寸 / 原点 / 硬障碍栅格）与规划时刻不一致。
  // 实测（r012 GUI）：路径跟踪中签名稳定（无增障时不抖动）；增障后签名即偏离 → 触发重规划。
  bool mapChangedSincePlan()
  {
    return mapSignature() != last_plan_map_sig_;
  }

  bool needReplan(double now_s)
  {
    if (force_replan_) {force_replan_ = false; return true;}
    const double age = now_s - last_plan_s_;
    if (age < kReplanGuardS) {return false;}                 // 限流：刚规划完不立即重规划
    if (!path_valid_) {return age >= replan_period_s_;}      // 规划失败：按周期重试（期间失效停车）
    if (path_.poses.size() < 2) {return true;}
    // 绑定检查：目标身份 / 地图签名 / 有效时限，任一变化 → 立即重规划（R-012）
    if (std::hypot(goal_x_ - last_plan_goal_.first, goal_y_ - last_plan_goal_.second) > replan_disp_m_) {
      RCLCPP_INFO(get_logger(), "重规划触发：目标身份变");
      return true;
    }
    if (mapChangedSincePlan()) {
      RCLCPP_INFO(get_logger(), "重规划触发：地图变化（costmap 硬障碍签名偏离）");
      return true;
    }
    if (age > max_path_age_s_) {
      RCLCPP_INFO(get_logger(), "重规划触发：路径超有效时限 %.1fs > %.1fs", age, max_path_age_s_);
      return true;
    }
    return age >= replan_period_s_;                          // 周期性刷新
  }

  void plan()
  {
    geometry_msgs::msg::PoseStamped start, goal;
    start.header.frame_id = global_frame_;
    start.header.stamp = now();
    start.pose.position.x = x_; start.pose.position.y = y_; start.pose.position.z = 0.0;
    start.pose.orientation.z = std::sin(yaw_ * 0.5);
    start.pose.orientation.w = std::cos(yaw_ * 0.5);
    goal.header.frame_id = global_frame_;
    goal.header.stamp = now();
    goal.pose.position.x = goal_x_; goal.pose.position.y = goal_y_; goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;

    last_plan_s_ = now().seconds();
    try {
      nav_msgs::msg::Path path = planner_->createPlan(start, goal, []() {return false;});
      if (path.poses.size() >= 2) {
        path_ = path;
        // 绑定：规划目标身份 + 地图签名（规划时刻）；仅当三者随路径一致时路径才可继续使用
        last_plan_goal_ = {goal_x_, goal_y_};
        last_plan_map_sig_ = mapSignature();
        path_valid_ = true;
        path_pub_->publish(path_);
        RCLCPP_INFO(get_logger(), "规划成功：%zu 点", path_.poses.size());
      } else {
        // 空路径 → 当前路径失效（fail-stop；不再沿旧路径 lookaheadRef）
        path_valid_ = false;
        RCLCPP_WARN(get_logger(), "规划失败：路径为空 → 路径失效");
      }
    } catch (const std::exception & ex) {
      // 规划异常 → 当前路径失效（fail-stop）
      path_valid_ = false;
      RCLCPP_WARN(get_logger(), "规划异常：%s → 路径失效", ex.what());
    }
  }

  // 取路径上距车最近投影点后 lookahead 处的参考点与切线
  bool lookaheadRef(double * rx, double * ry, double * tx, double * ty)
  {
    const auto & poses = path_.poses;
    if (poses.size() < 2) {return false;}
    // 最近点（只向前看，避免倒车）
    size_t best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < poses.size(); ++i) {
      const double d = std::hypot(poses[i].pose.position.x - x_, poses[i].pose.position.y - y_);
      if (d < best_d) {best_d = d; best = i;}
    }
    double acc = 0.0;
    size_t i = best;
    for (; i + 1 < poses.size(); ++i) {
      const double segx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
      const double segy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
      const double seg = std::hypot(segx, segy);
      if (acc + seg >= lookahead_m_) {
        const double t = seg <= 1e-9 ? 0.0 : (lookahead_m_ - acc) / seg;
        *rx = poses[i].pose.position.x + t * segx;
        *ry = poses[i].pose.position.y + t * segy;
        const double n = seg <= 1e-9 ? 1.0 : seg;
        *tx = segx / n; *ty = segy / n;
        return true;
      }
      acc += seg;
    }
    // 路径末端
    const size_t last = poses.size() - 1;
    *rx = poses[last].pose.position.x; *ry = poses[last].pose.position.y;
    const double segx = poses[last].pose.position.x - poses[last - 1].pose.position.x;
    const double segy = poses[last].pose.position.y - poses[last - 1].pose.position.y;
    const double n = std::max(std::hypot(segx, segy), 1e-9);
    *tx = segx / n; *ty = segy / n;
    return true;
  }

  void publish(double v, double w)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = base_frame_;
    cmd.twist.linear.x = v;
    cmd.twist.angular.z = w;
    cmd_pub_->publish(cmd);
  }

  void tick()
  {
    if (!active_ || !controller_ || !planner_) {return;}
    // R-041：目标失效（valid=false）→ 安全停车（fail-safe stop），先于路径跟随 / 重规划。
    //   地面失效动作与 C10 空中 `freeze`（冻结位姿 / 悬停）**分别定义**：地面=有期限保持后安全停车。
    if (!goal_valid_) {
      publish(0.0, 0.0);
      return;
    }
    if (state_ != agc_msgs::msg::AgcTaskState::GROUND_APPROACH || !have_odom_ || !have_goal_) {
      publish(0.0, 0.0);
      return;
    }
    if (std::hypot(goal_x_ - x_, goal_y_ - y_) <= goal_tol_m_) {
      publish(0.0, 0.0);
      return;
    }
    if (needReplan(now().seconds())) {plan();}
    // 路径失效（规划失败 / 绑定破坏且重规划未成功）：默认 fail-stop，不沿失效旧路径继续 lookaheadRef。
    // `stop_on_invalid` 为显式 opt-out（默认 True；置 False 才回落到旧路径，见 ADR-0027）。
    if (!path_valid_ && stop_on_invalid_) {
      publish(0.0, 0.0);
      return;
    }
    double rx, ry, tx, ty;
    if (!lookaheadRef(&rx, &ry, &tx, &ty)) {
      publish(0.0, 0.0);
      return;
    }

    // LQR：把车拉向 lookahead（参考速度沿路径切向）
    agc_control::State st;
    st.pos = Eigen::Vector3d(x_, y_, 0.0);
    st.vel = Eigen::Vector3d(vx_, vy_, 0.0);
    agc_control::Reference ref;
    ref.pos = Eigen::Vector3d(rx, ry, 0.0);
    ref.vel = Eigen::Vector3d(v_ref_ * tx, v_ref_ * ty, 0.0);
    ref.acc = Eigen::Vector3d::Zero();
    const Eigen::Vector3d acc = controller_->computeAcceleration(st, ref);

    // ⚠ R-038（见 ADR-0053）**适用性标注 —— 非完整差速动力学映射**（本批**不**重写控制律）：
    //   共享 LQR 输出为**世界系加速度**（3 自由度，holonomic；与 air 同一模型），此处把世界系
    //   期望速度积分后**仅**取车体系前进分量 `bx`（`lin=clamp(bx,0,v_max)`）与航向 `atan2(by,bx)`。
    //   由此**丢弃 / 截断**以下自由度（**不**代表 air 与 ground 同等闭环）：
    //     · `clamp(bx, 0, v_max)` **丢弃倒车**（bx<0 一律置 0）与超速；
    //     · 侧向 `by` **只**转成偏航速率（`k_yaw·atan2(by, max(bx,0.1))`），**丢弃纯横向自由度**
    //       （差速车无法直接产生车体横向速度 vy_body）；`max(bx,0.1)` 在 bx≤0 时对 by 放大 → 低速 / 原地大 yaw；
    //     · 无加速度 / 轮速动力学约束（LQR 的 acc_max 不能直接映射为轮加速度）。
    //   故此为**参考滤波 + 运动学近似**，其可达速率集 ⊊ 世界系 holonomic 参考集；须**单独**做可达性 /
    //   跟踪对拍（验收脚本 analytic 对拍），**不得**据 air（无障碍、holonomic 参考）结论外推 ground 同等闭环。
    const double dt = 1.0 / std::max(rate_hz_, 1e-3);
    double vx_des = vx_ + acc.x() * dt;
    double vy_des = vy_ + acc.y() * dt;
    const double speed = std::hypot(vx_des, vy_des);
    if (speed > v_max_) {vx_des *= v_max_ / speed; vy_des *= v_max_ / speed;}

    // world → 车体系
    const double bx = std::cos(yaw_) * vx_des + std::sin(yaw_) * vy_des;
    const double by = -std::sin(yaw_) * vx_des + std::cos(yaw_) * vy_des;

    const double lin = std::clamp(bx, 0.0, v_max_);
    const double ang = std::clamp(k_yaw_ * std::atan2(by, std::max(bx, 0.1)), -w_max_, w_max_);
    publish(lin, ang);
  }

  // ---- 参数 / 输入 ----
  std::string base_frame_, global_frame_;
  std::string planner_plugin_, planner_id_, ctrl_plugin_;
  std::string odom_topic_, goal_topic_, state_topic_, cmd_raw_topic_, path_topic_;
  std::string goal_info_topic_;   // R-041：目标有效性伴随诊断话题
  double rate_hz_{20.0}, replan_period_s_{2.0}, replan_disp_m_{1.0}, goal_tol_m_{0.3};
  double max_path_age_s_{5.0};
  bool stop_on_invalid_{true};
  double lookahead_m_{1.0}, k_yaw_{1.5}, v_max_{1.5}, w_max_{2.0};
  double v_ref_{1.0};

  bool active_{false}, have_odom_{false}, have_goal_{false}, force_replan_{false};
  // R-041：目标有效性（由伴随诊断驱动；默认 true 保持既有行为，收到 valid=0 即安全停车）
  bool goal_valid_{true};
  // 路径有效性与绑定：path_valid_ = 最近一次规划成功；绑定 = last_plan_goal_ / last_plan_map_sig_ / last_plan_s_
  bool path_valid_{false};
  uint64_t last_plan_map_sig_{0};
  double x_{0.0}, y_{0.0}, yaw_{0.0}, vx_{0.0}, vy_{0.0};
  double goal_x_{0.0}, goal_y_{0.0};
  uint8_t state_{agc_msgs::msg::AgcTaskState::IDLE};
  double last_plan_s_{-1.0e9};  // 使首帧即可触发首次规划（不受限流影响）
  std::pair<double, double> last_plan_goal_{0.0, 0.0};

  // ---- 组件 ----
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> planner_loader_;
  pluginlib::ClassLoader<agc_control::Controller> ctrl_loader_;
  nav2_core::GlobalPlanner::Ptr planner_;
  agc_control::Controller::Ptr controller_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  nav_msgs::msg::Path path_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_info_sub_;   // R-041：目标有效性伴随诊断
  rclcpp::Subscription<agc_msgs::msg::AgcTaskState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundBaseline>(rclcpp::NodeOptions());
  // lifecycle 节点：用 executor 自旋（内部 costmap 由 NodeThread 自旋）
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
