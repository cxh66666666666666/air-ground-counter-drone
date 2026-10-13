// agc_control/param_reader.hpp —— 插件读取参数的**最小接口**
//
// 插件不绑定具体节点类型（`rclcpp::Node` 与 `rclcpp_lifecycle::LifecycleNode` 在 Jazzy
// 互不继承），只依赖本接口；由消费者提供实现。参数一律由 launch 单一设置（契约 #params），
// 故 `getX(name, default)` 在缺省时**声明**该参数并返回默认值。

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

namespace agc_control
{

class ParamReader
{
public:
  virtual ~ParamReader() = default;
  virtual double getDouble(const std::string & name, double default_value) = 0;
  virtual int getInt(const std::string & name, int default_value) = 0;
  virtual bool getBool(const std::string & name, bool default_value) = 0;
  virtual std::string getString(const std::string & name, const std::string & default_value) = 0;
  /// 数组参数（无默认值；未声明时为声明空数组）
  virtual std::vector<std::string> getStringArray(const std::string & name) = 0;
  virtual std::vector<double> getDoubleArray(const std::string & name) = 0;
};

/// 基于 `NodeParametersInterface` 的实现：`prefix` 为插件的参数命名空间（如 "traj."）。
class NodeParamReader : public ParamReader
{
public:
  NodeParamReader(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params,
    std::string prefix)
  : params_(std::move(params)), prefix_(std::move(prefix))
  {
    if (!prefix_.empty() && prefix_.back() != '.') {prefix_ += '.';}
  }

  double getDouble(const std::string & name, double default_value) override
  {
    return value(name, rclcpp::ParameterValue(default_value)).get<double>();
  }
  int getInt(const std::string & name, int default_value) override
  {
    return static_cast<int>(
      value(name, rclcpp::ParameterValue(static_cast<std::int64_t>(default_value))).get<std::int64_t>());
  }
  bool getBool(const std::string & name, bool default_value) override
  {
    return value(name, rclcpp::ParameterValue(default_value)).get<bool>();
  }
  std::string getString(const std::string & name, const std::string & default_value) override
  {
    return value(name, rclcpp::ParameterValue(default_value)).get<std::string>();
  }
  std::vector<std::string> getStringArray(const std::string & name) override
  {
    return value(name,
      rclcpp::ParameterValue(std::vector<std::string>{})).get<std::vector<std::string>>();
  }
  std::vector<double> getDoubleArray(const std::string & name) override
  {
    return value(name, rclcpp::ParameterValue(std::vector<double>{})).get<std::vector<double>>();
  }

private:
  rclcpp::ParameterValue value(const std::string & name, const rclcpp::ParameterValue & def)
  {
    const std::string key = prefix_ + name;
    if (!params_->has_parameter(key)) {
      params_->declare_parameter(key, def);
    }
    return params_->get_parameter(key).get_parameter_value();
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_;
  std::string prefix_;
};

}  // namespace agc_control
