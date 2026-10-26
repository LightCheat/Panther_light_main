#ifndef IK_SOLVER_DLS_H_
#define IK_SOLVER_DLS_H_

#include <memory>
#include <string>
#include <stdexcept>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <eigen3/Eigen/Dense>
#include <Eigen/Core>
#include <stdexcept>


class IkSolverDLS {
public:
    // 构造函数
    IkSolverDLS(const std::string& urdf_path, const std::string& end_effector_frame);

    // 基于pinocchio的前向运动学
    pinocchio::SE3 forwardKinematics(const Eigen::VectorXd& q);

    // ik_solve返回结构体
    struct IKResult{
        Eigen::VectorXd q;
        bool success = false;
    };

    // 求解器
    IKResult solve(
        const Eigen::VectorXd& q_init,
        const Eigen::Vector3d& target_position,
        const Eigen::Matrix3d& target_rotation,
        int max_iterations = 100,
        double position_tolerance = 1e-4,
        double orientation_tolerance = 1e-3,
        double damping = 1e-3
    );


private:
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex end_effector_frame_id;

    // 创建一个6x1数组，包含位置和姿态的权重，后面逐项会和各个项的误差相乘，权重越大，相当于求解器越重视对应的误差
    Eigen::Matrix<double, 6, 1> _task_weights;
    double max_joint_step = 0.1;  // 关节更新步长上限
    // 位置与姿态权重
    float position_weight = 1.0;
    float orientation_weight = 0.3;

    // 关节位置和速度的维度，用于检查
    int nq;
    int nv;

    // 诊断信息结构体
    struct Diagnostics {
        bool success = false;
        int iterations = 0;
        double position_error = 0.0;
        double rotation_error = 0.0;
    };

    // 类的成员变量
    Diagnostics last_diagnostics;


};



#endif // IK_SOLVER_H_