// agc_air/reference_msg.hpp —— 参考轨迹的对外消息形态（/agc/air/ref、/agc/air/tracking_ref）
//
// 契约未定义专用参考消息，故复用标准 `trajectory_msgs/MultiDOFJointTrajectory`
// （含 Header，可置 frame_id=world），以**单点**承载 位置 / 速度 / 加速度（spec R8、R6）。

#pragma once

#include <string>

#include <std_msgs/msg/header.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include "agc_control/types.hpp"

namespace agc_air
{

inline trajectory_msgs::msg::MultiDOFJointTrajectory toMsg(const agc_control::Reference & ref,
  const std::string & frame_id, const rclcpp::Time & stamp)
{
  trajectory_msgs::msg::MultiDOFJointTrajectory msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;

  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint p;
  geometry_msgs::msg::Transform tf;
  tf.translation.x = ref.pos.x();
  tf.translation.y = ref.pos.y();
  tf.translation.z = ref.pos.z();
  tf.rotation.w = 1.0;
  p.transforms.push_back(tf);

  geometry_msgs::msg::Twist vel;
  vel.linear.x = ref.vel.x();
  vel.linear.y = ref.vel.y();
  vel.linear.z = ref.vel.z();
  p.velocities.push_back(vel);

  geometry_msgs::msg::Twist acc;
  acc.linear.x = ref.acc.x();
  acc.linear.y = ref.acc.y();
  acc.linear.z = ref.acc.z();
  p.accelerations.push_back(acc);

  p.time_from_start = rclcpp::Duration::from_seconds(0.0);
  msg.points.push_back(p);
  return msg;
}

inline agc_control::Reference fromMsg(const trajectory_msgs::msg::MultiDOFJointTrajectory & msg)
{
  agc_control::Reference ref;
  if (msg.points.empty()) {return ref;}
  const auto & p = msg.points.front();
  if (!p.transforms.empty()) {
    ref.pos = agc_control::Vector3(p.transforms.front().translation.x,
      p.transforms.front().translation.y, p.transforms.front().translation.z);
  }
  if (!p.velocities.empty()) {
    ref.vel = agc_control::Vector3(p.velocities.front().linear.x,
      p.velocities.front().linear.y, p.velocities.front().linear.z);
  }
  if (!p.accelerations.empty()) {
    ref.acc = agc_control::Vector3(p.accelerations.front().linear.x,
      p.accelerations.front().linear.y, p.accelerations.front().linear.z);
  }
  return ref;
}

}  // namespace agc_air
