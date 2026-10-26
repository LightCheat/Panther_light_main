#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "kinematics_application/ik_solver_dls.hpp"

class IkDlsDemo : public rclcpp::Node
{
public:
  IkDlsDemo()
  : Node("ik_dls_demo")
  {
    const std::string default_urdf =
      ament_index_cpp::get_package_share_directory("panther_description")
      + "/urdf/panther_description.urdf";

    const auto urdf_path = declare_parameter<std::string>("urdf_path", default_urdf);
    const auto end_effector_frame =
      declare_parameter<std::string>("end_effector_frame", "link6");
    const bool dry_run = declare_parameter<bool>("dry_run", false);

    try {
      solver_ = std::make_unique<IkSolverDLS>(urdf_path, end_effector_frame);

      const Eigen::VectorXd q_home = Eigen::VectorXd::Zero(6);
      const pinocchio::SE3 home_pose = solver_->forwardKinematics(q_home);

      Eigen::Vector3d target_position = home_pose.translation();
      target_position.z() += 0.05;

      const auto result = solver_->solve(
        q_home, target_position, home_pose.rotation());

      if (!result.success) {
        throw std::runtime_error(
          "DLS IK 未能求出沿 world Z 轴平移 5 cm 的目标关节位置");
      }

      command_.data.assign(result.q.data(), result.q.data() + result.q.size());

      RCLCPP_INFO(
        get_logger(),
        "IK 求解成功，home 末端位置=[%.4f, %.4f, %.4f] m，目标位置=[%.4f, %.4f, %.4f] m",
        home_pose.translation().x(), home_pose.translation().y(), home_pose.translation().z(),
        target_position.x(), target_position.y(), target_position.z());

      RCLCPP_INFO(
        get_logger(),
        "将直接输出目标关节位置=[%.5f, %.5f, %.5f, %.5f, %.5f, %.5f] rad",
        command_.data[0], command_.data[1], command_.data[2],
        command_.data[3], command_.data[4], command_.data[5]);

      if (dry_run) {
        RCLCPP_INFO(get_logger(), "dry_run=true，仅验证 IK，不输出电机位置命令");
        dry_run_complete_ = true;
        return;
      }

      command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/panther_position_controller/commands", 10);

      command_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&IkDlsDemo::publishPositionCommand, this));
    } catch (const std::exception & exception) {
      RCLCPP_FATAL(get_logger(), "初始化 IK 测试失败: %s", exception.what());
      initialization_failed_ = true;
    }
  }

  bool initializationFailed() const
  {
    return initialization_failed_;
  }

  bool dryRunComplete() const
  {
    return dry_run_complete_;
  }

private:
  void publishPositionCommand()
  {
    if (command_publisher_->get_subscription_count() == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "等待 panther_position_controller；请先激活该控制器");
      return;
    }

    command_publisher_->publish(command_);

    if (!command_announced_) {
      RCLCPP_INFO(
        get_logger(),
        "已开始向 panther_position_controller 直接发布目标关节位置");
      command_announced_ = true;
    }
  }

  std::unique_ptr<IkSolverDLS> solver_;
  std_msgs::msg::Float64MultiArray command_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  bool initialization_failed_ = false;
  bool dry_run_complete_ = false;
  bool command_announced_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IkDlsDemo>();

  if (node->initializationFailed()) {
    rclcpp::shutdown();
    return 1;
  }

  if (node->dryRunComplete()) {
    rclcpp::shutdown();
    return 0;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
