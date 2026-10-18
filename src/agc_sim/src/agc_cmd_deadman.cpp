// agc_cmd_deadman —— GZ 侧末端失效停车守门人（R-009，ADR-0025）
//
// 契约 #execution-port：唯一 ROS 命令者是地面执行中继 ground_drive_relay；本插件**不是** ROS 命令者。
//
// 职责：作为 GZ DiffDrive 命令口（DiffDrive 默认订阅 gz 话题 /model/<model>/cmd_vel）的
// **唯一写入者**，把「意图」与「执行」分离：
//   * 意图(ROS→GZ)：ros_gz_bridge 把 ROS 侧 /model/agc_car/cmd_vel（中继发布）桥到 **gz** 话题
//     /agc/sim/cmd_intent（见 src/agc_sim/config/bridge.yaml 的 gz_topic_name；**ROS 话题名不变**，
//     故中继与验收脚本不受影响）。
//   * 本插件订阅该意图话题；PreUpdate 中若距上次收到意图 < timeout → 转发缓存值；否则 → 发**零速**。
// 于是插件只能「转发 ROS 已下达的值」或「归零」，**永不自创运动**、**不产生任何 ROS 话题**，
// 故不构成第二 ROS 命令者（边界声明见 ADR-0025）。
//
// 失效场景：上游（中继）被 SIGKILL → 意图流停止 → 超时后归零 → 真值停车。因插件位于 GZ 服务器
// 进程内（与物理引擎同生命周期），即使 ROS 侧全部死亡亦生效——这是唯一可靠落点。
//
// 超时用 std::chrono::steady_clock（与仿真暂停 / ROS 时钟冻结解耦）。timeout 由 SDF <timeout> 注入，
// 其值源自 src/agc_sim/config/car.yaml → world_bringup 的 xacro arg（参数单一来源，契约 #params）。
//
// gz 订阅回调在 transport 线程执行，PreUpdate 在仿真线程执行 → 共享状态以互斥量保护。

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace agc_sim
{
/// \brief 意图→执行 守门人：fresh 转发，stale 归零。
class CmdDeadman
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &/*_entity*/,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &/*_ecm*/,
      gz::sim::EventManager &/*_eventMgr*/) override
  {
    std::string inTopic = "/agc/sim/cmd_intent";
    std::string outTopic = "/model/agc_car/cmd_vel";
    double timeoutS = 0.3;
    if (_sdf)
    {
      if (_sdf->HasElement("in_topic")) {inTopic = _sdf->Get<std::string>("in_topic");}
      if (_sdf->HasElement("out_topic")) {outTopic = _sdf->Get<std::string>("out_topic");}
      if (_sdf->HasElement("timeout")) {timeoutS = _sdf->Get<double>("timeout");}
    }
    this->timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeoutS));
    this->lastRx = std::chrono::steady_clock::now();

    this->pub = this->node.Advertise<gz::msgs::Twist>(outTopic);
    if (!this->node.Subscribe(inTopic, &CmdDeadman::OnIntent, this))
    {
      std::cerr << "[agc_cmd_deadman] 订阅意图话题失败：" << inTopic << std::endl;
    }
    std::cout << "[agc_cmd_deadman] intent='" << inTopic << "' -> cmd_vel='" << outTopic
              << "', timeout=" << timeoutS << " s" << std::endl;
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &/*_ecm*/) override
  {
    if (_info.paused) {return;}   // 暂停时不推进看门狗、不发命令
    gz::msgs::Twist out;          // 默认零速（linear/angular 全 0）
    {
      std::lock_guard<std::mutex> lk(this->mtx);
      const auto now = std::chrono::steady_clock::now();
      if (this->have && (now - this->lastRx) < this->timeout)
      {
        out = this->last;         // 只转发 ROS 已下达的值
      }
      // 否则：零速（out 保持默认）
    }
    this->pub.Publish(out);
  }

  private: void OnIntent(const gz::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lk(this->mtx);
    this->last = _msg;
    this->have = true;
    this->lastRx = std::chrono::steady_clock::now();
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher pub;
  private: gz::msgs::Twist last;
  private: bool have{false};
  private: std::mutex mtx;
  private: std::chrono::steady_clock::time_point lastRx;
  private: std::chrono::steady_clock::duration timeout;
};
}  // namespace agc_sim

GZ_ADD_PLUGIN(agc_sim::CmdDeadman,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(agc_sim::CmdDeadman, "agc_sim::CmdDeadman", "agc_cmd_deadman")
