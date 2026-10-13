// agc_air/agc_lifecycle_node.hpp —— 生命周期节点薄基类（Nav2 吸收点：lifecycle managed node）
//
// 对齐 Nav2 `nav2_util::LifecycleNode` 的用法（ADR-0008）：节点只实现
// `on_configure / on_activate / on_deactivate / on_cleanup` 四段语义，骨架统一转发。
// 约定：**资源建在 configure**（订阅 / 参数），**发布器激活在 activate**，deactivate 停发，cleanup 释放。

#pragma once

#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace agc_air
{

class AgcLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit AgcLifecycleNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name)
  {
  }

protected:
  /// configure：声明 / 读取参数，创建订阅与（未激活的）发布器
  virtual CallbackReturn on_configure_impl() {return CallbackReturn::SUCCESS;}
  /// activate：建定时器、激活发布器、开始生产数据
  virtual CallbackReturn on_activate_impl() {return CallbackReturn::SUCCESS;}
  /// deactivate：停定时器、停发
  virtual CallbackReturn on_deactivate_impl() {return CallbackReturn::SUCCESS;}
  /// cleanup：释放全部资源
  virtual CallbackReturn on_cleanup_impl() {return CallbackReturn::SUCCESS;}

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) final
  {
    return on_configure_impl();
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) final
  {
    return on_activate_impl();
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) final
  {
    return on_deactivate_impl();
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) final
  {
    return on_cleanup_impl();
  }
};

}  // namespace agc_air
