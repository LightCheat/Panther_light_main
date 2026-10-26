#include "6Dof_dyna.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace dynamics_application
{

class ZeroGravityModeNode : public rclcpp::Node
{
public:
  ZeroGravityModeNode()
  : Node("zero_gravity_mode")
  {
    // ROS 2 修改：串口和电机由 ros2_control 硬件接口统一管理，
    // 本节点只读取关节状态，并向力矩控制器发布六个关节的力矩指令。
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/joint_states");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/panther_effort_controller/commands");
    publish_rate_ = declare_parameter<double>("publish_rate", 100.0);
    state_timeout_ = declare_parameter<double>("state_timeout", 0.1);

    if (publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be greater than zero");
    }
    if (state_timeout_ <= 0.0) {
      throw std::invalid_argument("state_timeout must be greater than zero");
    }

    effort_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      command_topic_, rclcpp::QoS(10));

    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ZeroGravityModeNode::jointStateCallback, this, std::placeholders::_1));

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));
    command_timer_ = create_wall_timer(
      timer_period, std::bind(&ZeroGravityModeNode::publishCompensationTorque, this));

    RCLCPP_INFO(
      get_logger(), "Zero-gravity node publishes to %s at %.1f Hz",
      command_topic_.c_str(), publish_rate_);
  }

private:
  static constexpr std::size_t kJointCount = 6;
  static constexpr std::array<const char *, kJointCount> kJointNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    // ROS 2 修改：根据关节名称匹配数据，不依赖 /joint_states 中的关节排列顺序。
    std::unordered_map<std::string, std::size_t> message_indices;
    message_indices.reserve(message->name.size());
    for (std::size_t i = 0; i < message->name.size(); ++i) {
      message_indices.emplace(message->name[i], i);
    }

    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const auto index_it = message_indices.find(kJointNames[joint]);
      if (index_it == message_indices.end()) {
        continue;
      }

      const std::size_t message_index = index_it->second;
      if (message_index >= message->position.size() ||
        message_index >= message->velocity.size())
      {
        continue;
      }

      positions_[joint] = message->position[message_index];
      velocities_[joint] = message->velocity[message_index];
      joint_received_[joint] = true;
    }

    last_state_time_ = now();
  }

  void publishCompensationTorque()
  {
    const bool all_joints_received = std::all_of(
      joint_received_.begin(), joint_received_.end(), [](bool received) {return received;});

    // 安全修改：未收到完整状态或状态超时时只发布零力矩，
    // 避免将过期的重力补偿结果持续作用在电机上。
    if (!all_joints_received ||
      (now() - last_state_time_).seconds() > state_timeout_)
    {
      publishZeroTorque();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for complete and fresh joint states; publishing zero torque");
      return;
    }

    Eigen::VectorXd torque = compute_gravity_compensation(positions_.data());
    const auto friction = computeFrictionCompensation();

    std_msgs::msg::Float64MultiArray command;
    command.data.resize(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      command.data[i] = torque[static_cast<Eigen::Index>(i)] + friction[i];
    }
    effort_publisher_->publish(command);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Gravity torque: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      command.data[0], command.data[1], command.data[2],
      command.data[3], command.data[4], command.data[5]);
  }

  std::array<double, kJointCount> computeFrictionCompensation() const
  {
    // 算法保留：沿用原 ROS 1 节点的摩擦力补偿参数。
    // 修正原代码在速度进入死区后仍保留上一次补偿值的问题。
    std::array<double, kJointCount> friction{};

    if (std::abs(velocities_[0]) > 0.006) {
      friction[0] = 0.06 * velocities_[0];
    }
    if (velocities_[1] < -0.006) {
      friction[1] = -0.1 + 0.05 * velocities_[1];
    } else if (velocities_[1] > 0.006) {
      friction[1] = 0.1 + 0.05 * velocities_[1];
    }
    if (velocities_[2] < -0.006) {
      friction[2] = -0.2 + 0.1 * velocities_[2];
    } else if (velocities_[2] > 0.006) {
      friction[2] = 0.2 + 0.1 * velocities_[2];
    }
    if (velocities_[3] < -0.006) {
      friction[3] = -0.30 + 0.2 * velocities_[3];
    } else if (velocities_[3] > 0.006) {
      friction[3] = 0.30 + 0.2 * velocities_[3];
    }
    if (velocities_[4] < -0.01) {
      friction[4] = 0.03 - 0.05 * velocities_[4];
    } else if (velocities_[4] > 0.01) {
      friction[4] = -0.03 - 0.05 * velocities_[4];
    }

    return friction;
  }

  void publishZeroTorque()
  {
    std_msgs::msg::Float64MultiArray command;
    command.data.assign(kJointCount, 0.0);
    effort_publisher_->publish(command);
  }

  std::string joint_state_topic_;
  std::string command_topic_;
  double publish_rate_ = 100.0;
  double state_timeout_ = 0.1;

  std::array<double, kJointCount> positions_{};
  std::array<double, kJointCount> velocities_{};
  std::array<bool, kJointCount> joint_received_{};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};

}  // namespace dynamics_application

int main(int argc, char * argv[])
{
  // ROS 2 修改：使用 rclcpp 初始化、单线程执行器和 shutdown 生命周期。
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dynamics_application::ZeroGravityModeNode>());
  rclcpp::shutdown();
  return 0;
}
