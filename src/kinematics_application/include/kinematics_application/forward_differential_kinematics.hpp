#ifndef FORWARD_DIFFERENTIAL_KINEMATICS_H_
#define FORWARD_DIFFERENTIAL_KINEMATICS_H_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <eigen3/Eigen/Dense>

using namespace Eigen;

class ForwardDifferentialKinematics {
public:
    // 构造函数，传入ROS2节点指针，用于初始化tf2
    ForwardDifferentialKinematics(const rclcpp::Node::SharedPtr& node);
    // 前向运动学
    Eigen::Matrix4d forwardKinematics(const std::string & target_frame, const std::string & source_frame,const Eigen::VectorXd& q);
    // 速度雅可比矩阵构建
    Eigen::MatrixXd jacobian();
    // 前向微分运动学
    Eigen::Matrix<double, 6, 1> forwardDifferentialKinematics(const Eigen::VectorXd& q_dot);
    
    // 辅助函数：提取位置
    Eigen::Vector3d getTranslation(const geometry_msgs::msg::TransformStamped& tf)
    {
        return Eigen::Vector3d(
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z);
    }

    // 辅助函数：提取旋转矩阵
    Eigen::Matrix3d getRotationMatrix(const geometry_msgs::msg::TransformStamped& tf)
    {
        const auto& q_msg = tf.transform.rotation;

        Eigen::Quaterniond q(
            q_msg.w,
            q_msg.x,
            q_msg.y,
            q_msg.z);

        return q.normalized().toRotationMatrix();
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// 以下变量可以根据不同的机械臂进行修改
    // 基座和末端执行器的frame名称
    std::string base_frame_ = "base_link";
    std::string tcp_frame_ = "link6";

    // 对应ros2中每个关节的frame名称
    std::vector<std::string> joint_frames_ = {
        "link1",
        "link2",
        "link3",
        "link4",
        "link5",
        "link6"
    };

    // 对应笔记中的关节轴向量，假设所有关节都是绕z轴旋转
    std::vector<Eigen::Vector3d>joint_axes_ = {
        Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(0, 0, 1)
    };
};



#endif