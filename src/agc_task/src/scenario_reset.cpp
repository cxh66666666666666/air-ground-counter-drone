// scenario_reset —— M0 最小场景复位（W6 加厚）
//
// * 服务 /agc/scenario/reset（std_srvs/Trigger）：经 GZ set_pose 把车 / 目标置回 spawn；
//   受运行期门控（仅**非 active** 态可复位——IDLE + 全终态，含 INTERCEPT_CONFIRMED；契约 #lessons）。
// * **R-015（ADR-0030）复位受控事务**：
//   - 门控即「撤销运动许可」；随后做**有界等待**观察车静止（订阅 /agc/ground/odom；**不发布 cmd_vel**）。
//   - 读回前置：订阅真值旁路 gt（车 / 目标），要求写入前缓存**新鲜**（年龄 ≤ readback_max_age_s）。
//   - 事务：写车 → 读回（XY 欧氏距 ≤ 容差，有界等待）→ 写目标 → 读回；任一步失败 → **回滚**已写实体到
//     「写入前」位姿；回滚再失败 → 持久标记 env_needs_rebuild_ 且 reason 后缀 env_rebuild_needed。
//     **绝不留「车已移动、目标未复位」中间态。**
//   - car_start（/agc/scenario/car_start）**仅在完整成功后**发布。
//   - 每次成功复位 reset_epoch_++ 并发布 /agc/scenario/reset_epoch（Int32，transient_local）；应答含 epoch=N。
// * 启动初始复位**走同一门控**（不 bypass），保留 4 s 延迟等 gz / 桥就绪。
//
// 统一执行口（契约 #execution-port）：本节点**禁止**发布任何 cmd_vel；「停止执行」以门控 + 车静止证据实现。

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <agc_msgs/msg/agc_task_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

using Trigger = std_srvs::srv::Trigger;
using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class ScenarioReset : public rclcpp::Node
{
public:
  ScenarioReset()
  : Node("scenario_reset")
  {
    world_name_ = declare_parameter<std::string>("world_name", "agc_world");
    car_x_ = declare_parameter<double>("car_x", -4.0);
    car_y_ = declare_parameter<double>("car_y", -3.0);
    car_z_ = declare_parameter<double>("car_z", 0.010);
    car_yaw_ = declare_parameter<double>("car_yaw", 0.0);
    target_x_ = declare_parameter<double>("target_x", 6.5);
    target_y_ = declare_parameter<double>("target_y", 2.5);
    target_z_ = declare_parameter<double>("target_z", 2.0);
    target_yaw_ = declare_parameter<double>("target_yaw", 0.0);
    // C7 出生净空（ADR-0023）：按新 C2 安全档 + 实体外伸推导的可标定下限（暂定 ≈6 m，
    // 以"车-目标 spawn 中心距"为代理）；旧值 2.4 m 作废。R-014 起为车↔目标对下限（标定值）。
    clearance_min_m_ = declare_parameter<double>("spawn_clearance_min_m", 6.0);
    // R-014 出生净空统一候选校验（ADR-0029）：实体几何外伸 + 实体对下限 + 作业区边界。
    clearance_radius_car_m_ = declare_parameter<double>("clearance_radius_car_m", 0.5037);
    clearance_radius_uav_m_ = declare_parameter<double>("clearance_radius_uav_m", 0.25);
    clearance_radius_obstacle_m_ = declare_parameter<double>("clearance_radius_obstacle_m", 0.15);
    clearance_min_car_own_uav_m_ = declare_parameter<double>("clearance_min_car_own_uav_m", -1.0);
    clearance_min_target_own_uav_m_ =
      declare_parameter<double>("clearance_min_target_own_uav_m", -1.0);
    clearance_min_car_obstacle_m_ = declare_parameter<double>("clearance_min_car_obstacle_m", -1.0);
    clearance_min_target_obstacle_m_ =
      declare_parameter<double>("clearance_min_target_obstacle_m", -1.0);
    // R-014 出生净空统一候选校验（ADR-0029）：实体几何外伸 + 实体对下限 + 作业区边界。
    // S-06（ADR-0062）：作业区边界（map_*）与静态障碍清单（obstacle_*）的**几何真源**为
    //   `agc_sim/config/world_geometry.yaml`，由 w6_composition 派生注入；此处默认值
    //   **仅**为 standalone 回退，不构成第二份几何真源（修改几何只改真源文件）。
    map_x_min_ = declare_parameter<double>("map_x_min", -40.0);
    map_x_max_ = declare_parameter<double>("map_x_max", 40.0);
    map_y_min_ = declare_parameter<double>("map_y_min", -40.0);
    map_y_max_ = declare_parameter<double>("map_y_max", 40.0);
    // 显式运行模式（同 R-001 air_present 原则：不得由实体缺席推定）
    own_uav_present_ = declare_parameter<bool>("own_uav_present", false);
    own_uav_x_ = declare_parameter<double>("own_uav_x", 0.0);
    own_uav_y_ = declare_parameter<double>("own_uav_y", 0.0);
    obstacle_names_ = declare_parameter<std::vector<std::string>>(
      "obstacle_names", std::vector<std::string>{"pole_1", "pole_2"});
    obstacle_x_ = declare_parameter<std::vector<double>>(
      "obstacle_x", std::vector<double>{0.0, 6.0});
    obstacle_y_ = declare_parameter<std::vector<double>>(
      "obstacle_y", std::vector<double>{-6.0, 6.0});
    spawn_randomize_ = declare_parameter<bool>("spawn_randomize", false);
    spawn_seed_ = declare_parameter<int>("spawn_seed", 0);
    spawn_search_radius_m_ = declare_parameter<double>("spawn_search_radius_m", 4.0);
    spawn_internal_retry_ = declare_parameter<int>("spawn_internal_retry", 8);
    // R-015（ADR-0030）：受控事务参数（config 单一注入，节点不得覆写）
    car_entity_name_ = declare_parameter<std::string>("car_entity_name", "agc_car");
    target_entity_name_ = declare_parameter<std::string>("target_entity_name", "intruder_uav");
    car_gt_topic_ = declare_parameter<std::string>("car_gt_topic", "/agc/gt/agc_car");
    target_gt_topic_ = declare_parameter<std::string>("target_gt_topic", "/agc/gt/intruder_uav");
    reset_epoch_topic_ =
      declare_parameter<std::string>("reset_epoch_topic", "/agc/scenario/reset_epoch");
    readback_tolerance_m_ = declare_parameter<double>("readback_tolerance_m", 0.25);
    readback_timeout_s_ = declare_parameter<double>("readback_timeout_s", 1.0);
    readback_max_age_s_ = declare_parameter<double>("readback_max_age_s", 0.5);
    stop_exec_max_s_ = declare_parameter<double>("stop_exec_max_s", 0.5);
    quiescence_eps_ = declare_parameter<double>("quiescence_eps", 0.05);

    const auto state_topic =
      declare_parameter<std::string>("state_topic", "/agc/task/state");
    const auto car_start_topic =
      declare_parameter<std::string>("car_start_topic", "/agc/scenario/car_start");
    const auto ground_odom_topic =
      declare_parameter<std::string>("ground_odom_topic", "/agc/ground/odom");

    state_sub_ = create_subscription<agc_msgs::msg::AgcTaskState>(
      state_topic, 10, [this](agc_msgs::msg::AgcTaskState::SharedPtr m) {state_ = m->state;});
    car_gt_sub_ = create_subscription<PoseStamped>(
      car_gt_topic_, 10,
      [this](PoseStamped::SharedPtr m) {
        std::lock_guard<std::mutex> lk(gt_mtx_);
        car_gt_ = *m;
        car_gt_rx_ = now();
        have_car_gt_ = true;
      });
    target_gt_sub_ = create_subscription<PoseStamped>(
      target_gt_topic_, 10,
      [this](PoseStamped::SharedPtr m) {
        std::lock_guard<std::mutex> lk(gt_mtx_);
        target_gt_ = *m;
        target_gt_rx_ = now();
        have_target_gt_ = true;
      });
    // R-015：车静止观察（**只读**，不发布 cmd_vel——唯一执行口 constraints #execution-port）
    ground_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ground_odom_topic, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        const auto & t = m->twist.twist.linear;
        std::lock_guard<std::mutex> lk(odom_mtx_);
        car_speed_ = std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
      });

    car_start_pub_ = create_publisher<PoseStamped>(car_start_topic, 10);
    reset_epoch_pub_ =
      create_publisher<std_msgs::msg::Int32>(reset_epoch_topic_, rclcpp::QoS(1).transient_local());

    client_node_ = std::make_shared<rclcpp::Node>("scenario_reset_client");
    set_pose_client_ = client_node_->create_client<SetEntityPose>(
      "/world/" + world_name_ + "/set_pose");

    // R-015：复位服务 + 启动定时器置于**独立互斥回调组**。事务回调内会**有界阻塞**（静止观察 /
    // 读回等待 / 回滚），订阅须留在默认组由多线程执行器在其它线程持续更新（对齐 R-013 执行器范式）。
    txn_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    reset_srv_ = create_service<Trigger>(
      "/agc/scenario/reset",
      std::bind(&ScenarioReset::onReset, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), txn_cb_group_);

    // 启动初始复位（延迟等待 gz / 桥就绪）；**经同一门控**（R-015 / tasks 3.4：不 bypass）。
    startup_timer_ = create_wall_timer(
      std::chrono::seconds(4), [this]() {
        startup_timer_->cancel();
        RCLCPP_INFO(get_logger(), "initial reset (gated): %s", doReset() ? "ok" : "FAILED");
      },
      txn_cb_group_);
  }

private:
  // 可复位态集合 = 全部**非 active** 态：IDLE + 终态（含 INTERCEPT_CONFIRMED，R-015 / tasks 3.4）。
  // active（AIR_SEARCH / TARGET_ACQUIRED / GROUND_APPROACH）一律拒（reason not_resettable）。
  bool isResettable() const
  {
    return state_ == agc_msgs::msg::AgcTaskState::IDLE ||
           state_ == agc_msgs::msg::AgcTaskState::TRACKING_SUCCESS ||
           state_ == agc_msgs::msg::AgcTaskState::INTERCEPT_CONFIRMED ||
           state_ == agc_msgs::msg::AgcTaskState::MISSION_FAILED ||
           state_ == agc_msgs::msg::AgcTaskState::ABORTED ||
           state_ == agc_msgs::msg::AgcTaskState::INFRA_FAILURE;
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // R-014：实体对下限——若有显式覆盖（≥0）则用之，否则按两实体几何外伸之和推导。
  double pairMin(double r_a, double r_b, double override_min) const
  {
    return override_min >= 0.0 ? override_min : (r_a + r_b);
  }

  // R-014：统一候选校验（ADR-0029）。按序检查，任一不满足即返回 false 并设 reason。
  bool validateCandidate(double cx, double cy, double tx, double ty,
    std::string & reason) const
  {
    if (obstacle_names_.size() != obstacle_x_.size() ||
      obstacle_names_.size() != obstacle_y_.size())
    {
      reason = "obstacle_config_invalid";
      return false;
    }
    const double rc = clearance_radius_car_m_;
    const double ru = clearance_radius_uav_m_;
    const double ro = clearance_radius_obstacle_m_;
    if (cx < map_x_min_ + rc || cx > map_x_max_ - rc ||
      cy < map_y_min_ + rc || cy > map_y_max_ - rc)
    {
      reason = "boundary_out_car";
      return false;
    }
    if (tx < map_x_min_ + ru || tx > map_x_max_ - ru ||
      ty < map_y_min_ + ru || ty > map_y_max_ - ru)
    {
      reason = "boundary_out_target";
      return false;
    }
    if (std::hypot(cx - tx, cy - ty) < clearance_min_m_) {
      reason = "car_target_clearance";
      return false;
    }
    if (own_uav_present_) {
      if (std::hypot(cx - own_uav_x_, cy - own_uav_y_) <
        pairMin(rc, ru, clearance_min_car_own_uav_m_))
      {
        reason = "car_own_uav_clearance";
        return false;
      }
      if (std::hypot(tx - own_uav_x_, ty - own_uav_y_) <
        pairMin(ru, ru, clearance_min_target_own_uav_m_))
      {
        reason = "target_own_uav_clearance";
        return false;
      }
    }
    for (size_t i = 0; i < obstacle_names_.size(); ++i) {
      const double ox = obstacle_x_[i];
      const double oy = obstacle_y_[i];
      if (std::hypot(cx - ox, cy - oy) < pairMin(rc, ro, clearance_min_car_obstacle_m_)) {
        reason = "car_obstacle_clearance:" + obstacle_names_[i];
        return false;
      }
      if (std::hypot(tx - ox, ty - oy) < pairMin(ru, ro, clearance_min_target_obstacle_m_)) {
        reason = "target_obstacle_clearance:" + obstacle_names_[i];
        return false;
      }
    }
    return true;
  }

  // R-014：候选生成（按 scenario + seed）并对**每一轮**候选做统一校验。
  bool selectCandidate(double & cx, double & cy, double & tx, double & ty,
    std::string & reason) const
  {
    cx = car_x_; cy = car_y_; tx = target_x_; ty = target_y_;
    std::mt19937 rng(static_cast<unsigned>(spawn_seed_));
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    const int tries = spawn_randomize_ ? (spawn_internal_retry_ > 1 ? spawn_internal_retry_ : 1) : 1;
    reason = "no_candidate";
    for (int i = 0; i < tries; ++i) {
      if (spawn_randomize_) {
        cx = car_x_ + uni(rng) * spawn_search_radius_m_;
        cy = car_y_ + uni(rng) * spawn_search_radius_m_;
        tx = target_x_ + uni(rng) * spawn_search_radius_m_;
        ty = target_y_ + uni(rng) * spawn_search_radius_m_;
      }
      if (validateCandidate(cx, cy, tx, ty, reason)) {return true;}
    }
    return false;
  }

  bool setPose(const std::string & name, double x, double y, double z, double yaw)
  {
    if (!set_pose_client_->wait_for_service(std::chrono::seconds(2))) {
      return false;
    }
    auto req = std::make_shared<SetEntityPose::Request>();
    req->entity.name = name;
    req->entity.type = 2;  // MODEL
    req->pose.position.x = x;
    req->pose.position.y = y;
    req->pose.position.z = z;
    req->pose.orientation.z = std::sin(yaw * 0.5);
    req->pose.orientation.w = std::cos(yaw * 0.5);
    auto future = set_pose_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }
    return future.get()->success;
  }

  // R-015（tasks 3.2）：撤销运动许可（门控已保证非 active、无授权命令者）后**有界等待**观察车静止。
  // 残余滑行为滑行（复位 teleport 会中和），**非致命**，仅记录证据（速度 / 等待时长）。**不发布 cmd_vel**。
  void waitForQuiescence()
  {
    const auto t0 = std::chrono::steady_clock::now();
    const auto deadline = t0 + std::chrono::duration<double>(stop_exec_max_s_);
    double speed = -1.0;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lk(odom_mtx_);
        speed = car_speed_;
      }
      if (speed >= 0.0 && speed < quiescence_eps_) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const double waited =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (speed >= 0.0 && speed < quiescence_eps_) {
      RCLCPP_INFO(
        get_logger(), "car quiescent: |v|=%.3f < %.3f (waited=%.3f s)", speed, quiescence_eps_,
        waited);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "car_not_quiescent: |v|=%.3f eps=%.3f waited=%.3f s (residual coast; gate ensures "
        "non-active, no authorized commander)", speed, quiescence_eps_, waited);
    }
  }

  // R-015（tasks 3.3）：写后读回校验——等待 gt 缓存**更新到写入之后**且 XY 欧氏距 ≤ 容差，有界等待。
  bool awaitReadback(bool is_car, double x, double y, const rclcpp::Time & write_time)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(readback_timeout_s_);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lk(gt_mtx_);
        const PoseStamped & p = is_car ? car_gt_ : target_gt_;
        const bool have = is_car ? have_car_gt_ : have_target_gt_;
        const rclcpp::Time rx = is_car ? car_gt_rx_ : target_gt_rx_;
        if (have && rx > write_time &&
          std::hypot(p.pose.position.x - x, p.pose.position.y - y) <= readback_tolerance_m_)
        {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  // R-015（tasks 3.3）：回滚已写入实体到「写入前」位姿；任一失败 → 返回 false（→ env_needs_rebuild_）。
  bool rollback(const PoseStamped & pre_car, const PoseStamped & pre_tgt,
    bool wrote_car, bool wrote_tgt)
  {
    bool ok = true;
    if (wrote_car) {
      const bool r = setPose(
        car_entity_name_, pre_car.pose.position.x, pre_car.pose.position.y,
        pre_car.pose.position.z, yawFromQuat(pre_car.pose.orientation));
      ok = r && ok;
    }
    if (wrote_tgt) {
      const bool r = setPose(
        target_entity_name_, pre_tgt.pose.position.x, pre_tgt.pose.position.y,
        pre_tgt.pose.position.z, yawFromQuat(pre_tgt.pose.orientation));
      ok = r && ok;
    }
    return ok;
  }

  bool doReset()
  {
    // (1) 门控：全部非 active 态可复位；active 拒（含启动初始复位——不 bypass）。
    if (!isResettable()) {
      last_reason_ = "not_resettable";
      RCLCPP_WARN(get_logger(), "reset denied: not_resettable (state=%u)", state_);
      return false;
    }

    // (2) 撤销运动许可 + 停止执行（有界观察车静止，非致命）。
    waitForQuiescence();

    // (3) 候选生成 + 统一净空校验（R-014 / ADR-0029）。
    double cx, cy, tx, ty;
    std::string reason;
    if (!selectCandidate(cx, cy, tx, ty, reason)) {
      last_reason_ = reason;
      RCLCPP_WARN(get_logger(), "reset rejected: %s", reason.c_str());
      return false;
    }

    // (4) 读回前置：写入前 gt 缓存须新鲜（作为「写入前状态」与回滚目标）。
    PoseStamped pre_car, pre_tgt;
    {
      std::lock_guard<std::mutex> lk(gt_mtx_);
      const double car_age = have_car_gt_ ? (now() - car_gt_rx_).seconds() : -1.0;
      const double tgt_age = have_target_gt_ ? (now() - target_gt_rx_).seconds() : -1.0;
      if (!have_car_gt_ || car_age > readback_max_age_s_ ||
        !have_target_gt_ || tgt_age > readback_max_age_s_)
      {
        last_reason_ = "gt_stale";
        RCLCPP_WARN(
          get_logger(), "reset rejected: gt_stale (car_age=%.3f target_age=%.3f)", car_age, tgt_age);
        return false;
      }
      pre_car = car_gt_;
      pre_tgt = target_gt_;
    }

    // (5) 受控事务：写车 → 读回 → 写目标 → 读回；任一步失败 → 回滚。
    bool wrote_car = false, wrote_tgt = false;
    std::string fail_reason;
    bool ok = true;

    if (!setPose(car_entity_name_, cx, cy, car_z_, car_yaw_)) {
      fail_reason = "car_set_pose_failed";
      ok = false;
    } else {
      wrote_car = true;
      const rclcpp::Time t_car = now();
      if (!awaitReadback(true, cx, cy, t_car)) {
        fail_reason = "readback_mismatch_car";
        ok = false;
      }
    }

    if (ok) {
      if (!setPose(target_entity_name_, tx, ty, target_z_, target_yaw_)) {
        fail_reason = "target_set_pose_failed";
        ok = false;
      } else {
        wrote_tgt = true;
        const rclcpp::Time t_tgt = now();
        if (!awaitReadback(false, tx, ty, t_tgt)) {
          fail_reason = "readback_mismatch_target";
          ok = false;
        }
      }
    }

    if (ok) {
      // (6) 完整成功：epoch++ → 发布 epoch → **仅此时**发布 car_start。
      reset_epoch_++;
      publishEpoch();
      publishCarStart(cx, cy);
      last_reason_ = "ok";
      RCLCPP_INFO(
        get_logger(), "reset ok epoch=%d car=(%.3f,%.3f) target=(%.3f,%.3f)", reset_epoch_, cx, cy,
        tx, ty);
      return true;
    }

    // (7) 失败：回滚已写实体；回滚再失败 → 持久标记「环境需重建」。
    if (!rollback(pre_car, pre_tgt, wrote_car, wrote_tgt)) {
      env_needs_rebuild_ = true;
      last_reason_ = "rollback_failed env_rebuild_needed";
      RCLCPP_ERROR(
        get_logger(), "reset failed (%s) AND rollback failed → env_needs_rebuild", fail_reason.c_str());
    } else {
      last_reason_ = fail_reason;
      RCLCPP_WARN(get_logger(), "reset rejected: %s (rolled back)", fail_reason.c_str());
    }
    return false;
  }

  void publishEpoch()
  {
    std_msgs::msg::Int32 msg;
    msg.data = reset_epoch_;
    reset_epoch_pub_->publish(msg);
  }

  void publishCarStart(double x, double y)
  {
    PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = car_z_;
    msg.pose.orientation.z = std::sin(car_yaw_ * 0.5);
    msg.pose.orientation.w = std::cos(car_yaw_ * 0.5);
    car_start_pub_->publish(msg);
  }

  void onReset(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res)
  {
    const bool ok = doReset();
    res->success = ok;
    res->message = ok ? ("reset ok epoch=" + std::to_string(reset_epoch_)) :
      ("reset rejected: " + last_reason_);
  }

  std::string world_name_;
  double car_x_, car_y_, car_z_, car_yaw_;
  double target_x_, target_y_, target_z_, target_yaw_;
  double clearance_min_m_{6.0}, spawn_search_radius_m_{4.0};
  // R-014（ADR-0029）：实体几何外伸 / 实体对下限覆盖 / 作业区边界 / 己方 UAV / 静态障碍
  double clearance_radius_car_m_{0.5037}, clearance_radius_uav_m_{0.25};
  double clearance_radius_obstacle_m_{0.15};
  double clearance_min_car_own_uav_m_{-1.0}, clearance_min_target_own_uav_m_{-1.0};
  double clearance_min_car_obstacle_m_{-1.0}, clearance_min_target_obstacle_m_{-1.0};
  double map_x_min_{-40.0}, map_x_max_{40.0}, map_y_min_{-40.0}, map_y_max_{40.0};
  bool own_uav_present_{false};
  double own_uav_x_{0.0}, own_uav_y_{0.0};
  std::vector<std::string> obstacle_names_;
  std::vector<double> obstacle_x_, obstacle_y_;
  std::string last_reason_;
  bool spawn_randomize_{false};
  int spawn_seed_{0}, spawn_internal_retry_{8};
  // R-015（ADR-0030）：受控事务
  std::string car_entity_name_, target_entity_name_, car_gt_topic_, target_gt_topic_;
  std::string reset_epoch_topic_;
  double readback_tolerance_m_{0.25}, readback_timeout_s_{1.0}, readback_max_age_s_{0.5};
  double stop_exec_max_s_{0.5}, quiescence_eps_{0.05};
  int reset_epoch_{0};
  bool env_needs_rebuild_{false};
  uint8_t state_{agc_msgs::msg::AgcTaskState::IDLE};

  PoseStamped car_gt_, target_gt_;
  rclcpp::Time car_gt_rx_{0, 0, RCL_ROS_TIME}, target_gt_rx_{0, 0, RCL_ROS_TIME};
  bool have_car_gt_{false}, have_target_gt_{false};
  double car_speed_{-1.0};
  std::mutex gt_mtx_, odom_mtx_;

  rclcpp::Subscription<agc_msgs::msg::AgcTaskState>::SharedPtr state_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr car_gt_sub_, target_gt_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_odom_sub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr car_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr reset_epoch_pub_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<SetEntityPose>::SharedPtr set_pose_client_;
  rclcpp::Service<Trigger>::SharedPtr reset_srv_;
  rclcpp::CallbackGroup::SharedPtr txn_cb_group_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // R-015：多线程执行器。复位服务 + 启动定时器在独立互斥回调组；订阅在默认组由其它线程持续更新，
  // 使事务回调内的有界阻塞（静止观察 / 读回等待）期间 gt / odom 缓存仍得到刷新。
  auto node = std::make_shared<ScenarioReset>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
