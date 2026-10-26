#include "kinematics_application/forward_differential_kinematics.hpp"

ForwardDifferentialKinematics::ForwardDifferentialKinematics(const rclcpp::Node::SharedPtr& node): node_(node)
{
    tf_buffer_ =std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    tf_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

Eigen::Matrix4d ForwardDifferentialKinematics::forwardKinematics(const std::string & base, const std::string & tool, const Eigen::VectorXd& q)
{
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped = tf_buffer_->lookupTransform(
        base,  // target_frame
        tool,  // source_frame
        tf2::TimePointZero
    );

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    auto &trans = transform_stamped.transform.translation;
    auto &rot   = transform_stamped.transform.rotation;

    // 平移
    T(0, 3) = trans.x;
    T(1, 3) = trans.y;
    T(2, 3) = trans.z;

    // 四元数
    Eigen::Quaterniond quaternion(
        rot.w,
        rot.x,
        rot.y,
        rot.z
    );

    // 四元数 -> 旋转矩阵
    T.block<3, 3>(0, 0) = quaternion.toRotationMatrix();

    return T;
}

Eigen::MatrixXd ForwardDifferentialKinematics::jacobian()
{
    // 获取关节数量
    int n = joint_frames_.size();

    // 雅克比为6*n，6为位置和姿态，n为关节数量
    Eigen::MatrixXd J(6, n);
    J.setZero();

    // 获取tcp的变换信息
    auto tcp_tf = tf_buffer_->lookupTransform(
                    base_frame_,
                    tcp_frame_,
                    tf2::TimePointZero);

    // 获取tcp的位置
    Eigen::Vector3d p_tcp = getTranslation(tcp_tf);

    for (int i = 0; i < n; ++i)
        {
            // 获取关节i的变换信息
            auto joint_tf = tf_buffer_->lookupTransform(
                                base_frame_,
                                joint_frames_[i],
                                tf2::TimePointZero);

            // 获取关节i的位置
            Eigen::Vector3d p_joint = getTranslation(joint_tf);

            // 获取关节i的旋转矩阵
            Eigen::Matrix3d R_base_joint = getRotationMatrix(joint_tf);

            // URDF里的局部关节轴
            Eigen::Vector3d axis_local = joint_axes_[i].normalized();

            // 转换到base坐标系
            Eigen::Vector3d axis_base = R_base_joint * axis_local;

            // Jacobian线速度部分 对应笔记zi-1 x(pe - pi-1)
            Eigen::Vector3d Jv = axis_base.cross(p_tcp - p_joint);

            // Jacobian角速度部分
            Eigen::Vector3d Jw = axis_base;

            J.block<3, 1>(0, i) = Jv;
            J.block<3, 1>(3, i) = Jw;
        }
    return J;
}

Eigen::Matrix<double, 6, 1> ForwardDifferentialKinematics::forwardDifferentialKinematics(const Eigen::VectorXd& q_dot)
{
    // 获取雅可比矩阵
    Eigen::MatrixXd J = jacobian();

    // 计算末端执行器的速度
    Eigen::Matrix<double, 6, 1> Jv = J * q_dot;

    return Jv;
}
