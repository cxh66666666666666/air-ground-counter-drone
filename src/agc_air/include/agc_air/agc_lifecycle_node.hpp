// agc_air/agc_lifecycle_node.hpp —— 生命周期节点薄基类（Nav2 吸收点：lifecycle managed node）
//
// 对齐 Nav2 `nav2_util::LifecycleNode` 的用法（ADR-0008 吸收矩阵；口径见 ADR-0049）：
//   - 四段语义：`on_configure / on_activate / on_deactivate / on_cleanup`，骨架统一转发；
//   - **可重复资源状态机**：configure **幂等**（声明参数前先 `has_parameter`），cleanup **释放并注销**
//     （`undeclare_parameter`）→ `configure→activate→deactivate→cleanup→configure` 可重复；
//   - **转发** `on_shutdown` / `on_error` 到新的虚函数（默认复用 cleanup 语义）做**完整清理**；
//   - **bond 失联检测**：对齐 Nav2 bond（topic=`bond`、id=节点名、`bond_heartbeat_period`），
//     被 `lifecycle_manager` 心跳检测；节点侧看门狗在 **bond 已建立后断开**（manager 死亡）时
//     触发**失效停机**（真实 `deactivate→cleanup` 转移，停止产出控制数据）。
// 约定：**资源建在 configure**（订阅 / 参数），**发布器激活在 activate**，deactivate 停发，cleanup 释放。

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <bond/msg/constants.hpp>
#include <bondcpp/bond.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace agc_air
{

class AgcLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit AgcLifecycleNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name)
  {
    // bond 心跳周期（0 = 禁用 bond）；由 launch 单一设置（契约 [参数入口]）。
    // **不**置 `bond_disable_heartbeat_timeout`（留给 bondcpp 默认 false）→ 节点侧亦检测
    // manager 失联（与 nav2_util 默认「server 侧不超时」不同，为空中失效停机语义，见 ADR-0049）。
    if (!has_parameter("bond_heartbeat_period")) {
      declare_parameter("bond_heartbeat_period", 0.1);
    }
    bond_heartbeat_period_ = get_parameter("bond_heartbeat_period").as_double();
  }

  ~AgcLifecycleNode() override {destroyBond();}

  /// **幂等**参数声明：已声明则直接读取（对齐 Nav2 `declare_parameter_if_not_declared`；
  /// 使 `cleanup→configure` 二轮**不**抛重复声明异常）。
  /// 注：rclcpp 的**静态类型参数**（以默认值声明）**不可 undeclare**（会抛
  /// `cannot undeclare a statically typed parameter`），故重复配置的幂等性由 `has_parameter`
  /// 守卫承载（Nav2 惯例），cleanup 以**释放资源**（订阅 / 发布器 / 定时器 / 服务）为「注销」。
  template<typename T>
  T declare_param_idempotent(const std::string & name, const T & default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

protected:
  /// configure：声明 / 读取参数，创建订阅与（未激活的）发布器
  virtual CallbackReturn on_configure_impl() {return CallbackReturn::SUCCESS;}
  /// activate：建定时器、激活发布器、开始生产数据
  virtual CallbackReturn on_activate_impl() {return CallbackReturn::SUCCESS;}
  /// deactivate：停定时器、停发（**失效停机**）
  virtual CallbackReturn on_deactivate_impl() {return CallbackReturn::SUCCESS;}
  /// cleanup：释放全部资源
  virtual CallbackReturn on_cleanup_impl() {return CallbackReturn::SUCCESS;}
  /// shutdown：**完整清理**（默认复用 cleanup 语义；被管节点可覆写）
  virtual CallbackReturn on_shutdown_impl() {return on_cleanup_impl();}
  /// error：**完整清理**（默认复用 cleanup 语义；被管节点可覆写）
  virtual CallbackReturn on_error_impl() {return on_cleanup_impl();}
  /// bond 断开（manager 失联）→ **失效停机**：默认走真实 lifecycle 转移（deactivate→cleanup）。
  virtual void on_bond_broken()
  {
    RCLCPP_ERROR(get_logger(),
      "与 lifecycle_manager 的 bond 断开 → 失效停机（停止产出控制数据，完整清理）");
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  }

  /// 创建到 lifecycle_manager 的 bond（对齐 `nav2_util::LifecycleNode::createBond`）。
  void createBond()
  {
    if (bond_heartbeat_period_ <= 0.0 || bond_) {return;}
    bond_formed_ = false;
    bond_ = std::make_shared<bond::Bond>(
      std::string("bond"), get_name(), shared_from_this(),
      [this]() {on_bond_broken();},
      [this]() {
        bond_formed_ = true;
        RCLCPP_INFO(get_logger(), "已与 lifecycle_manager 建立 bond（bond formed）");
      });
    bond_->setHeartbeatPeriod(bond_heartbeat_period_);
    bond_->setHeartbeatTimeout(4.0);
    bond_->start();
  }

  /// 销毁 bond（幂等）
  void destroyBond()
  {
    if (bond_) {
      bond_.reset();
    }
    bond_formed_ = false;
  }

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) final
  {
    // ADR-0080：bond **不**在 configure 创建——Nav2 manager 只在 activate 时才建立其侧 bond；
    //   在 configure 开始计时会在"配置→激活"间隔 > 4 s 时误触发 broken → 失效停机。
    return on_configure_impl();
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) final
  {
    const CallbackReturn r = on_activate_impl();
    if (r == CallbackReturn::SUCCESS) {
      createBond();   // 与 manager 的 activate 侧 bond 同拍建立
      startBondWatchdog();
    }
    return r;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) final
  {
    stopBondWatchdog();
    return on_deactivate_impl();
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) final
  {
    stopBondWatchdog();
    const CallbackReturn r = on_cleanup_impl();
    destroyBond();
    return r;
  }
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) final
  {
    RCLCPP_WARN(get_logger(), "shutdown：完整清理（失效停机，停止产出控制数据）");
    stopBondWatchdog();
    const CallbackReturn r = on_shutdown_impl();
    destroyBond();
    return r;
  }
  CallbackReturn on_error(const rclcpp_lifecycle::State &) final
  {
    RCLCPP_ERROR(get_logger(), "error：完整清理（失效停机，停止产出控制数据）");
    stopBondWatchdog();
    const CallbackReturn r = on_error_impl();
    destroyBond();
    return r;
  }

  void startBondWatchdog()
  {
    if (bond_heartbeat_period_ <= 0.0 || !bond_ || bond_watchdog_timer_) {return;}
    bond_watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), [this]() {checkBondHealth();});
  }
  void stopBondWatchdog()
  {
    if (bond_watchdog_timer_) {
      bond_watchdog_timer_->cancel();
      bond_watchdog_timer_.reset();
    }
  }
  /// 看门狗：**曾建立过**的 bond 断开（manager 死亡）→ 失效停机。
  /// （未建立过即 bond_formed_ 恒 false → 无 manager 的独立运行**不**被误停。）
  void checkBondHealth()
  {
    if (!bond_) {return;}
    if (bond_formed_ && bond_->isBroken()) {
      stopBondWatchdog();
      on_bond_broken();
    }
  }
  double bond_heartbeat_period_{0.1};
  std::shared_ptr<bond::Bond> bond_;
  bool bond_formed_{false};
  rclcpp::TimerBase::SharedPtr bond_watchdog_timer_;
};

}  // namespace agc_air
