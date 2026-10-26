#include "kinematics_application/ik_solver_dls.hpp"

IkSolverDLS::IkSolverDLS(const std::string& urdf_path, const std::string& end_effector_frame)
{
    // 向pinocchio中加载urdf模型
    pinocchio::urdf::buildModel(urdf_path, model_);

    // 创建算法计算所需的数据
    data_ = std::make_unique<pinocchio::Data>(model_);

    nq = model_.nq;
    nv = model_.nv;

    if (nq != 6 || nv != 6) {
        throw std::invalid_argument(
            "当前 IKSolver 预期加载 6 自由度固定基座 UR5e。"
        );
    }

    // 末端坐标系输入检验
    end_effector_frame_id = model_.getFrameId(end_effector_frame);
        if (end_effector_frame_id == model_.frames.size()) {
        throw std::invalid_argument(
            "URDF 中未找到末端坐标系：" + end_effector_frame
        );
    }

    // 权重赋值以及关节更新最大歩长赋值
    _task_weights <<
        position_weight, position_weight, position_weight,
        orientation_weight, orientation_weight, orientation_weight;

}

pinocchio::SE3 IkSolverDLS::forwardKinematics(const Eigen::VectorXd& q)
{
    // 检查关节向量的长度
    if (q.size() != model_.nq) {
        throw std::invalid_argument(
            "q 的长度必须等于 model.nq"
        );
    }

    // 计算所有关节的正运动学
    pinocchio::forwardKinematics(
        model_,
        *data_,
        q
    );

    // 更新指定末端坐标系的位姿
    pinocchio::updateFramePlacement(
        model_,
        *data_,
        end_effector_frame_id
    );

    // 返回末端坐标系相对于世界坐标系的位姿副本
    return data_->oMf[end_effector_frame_id];

}

IkSolverDLS::IKResult IkSolverDLS::solve(
    const Eigen::VectorXd& q_init,
    const Eigen::Vector3d& target_position,
    const Eigen::Matrix3d& target_rotation,
    int max_iterations,
    double position_tolerance,
    double orientation_tolerance,
    double damping
)
{
    // 定位ik返回结果，包含标志位以及位置
    IkSolverDLS::IKResult ikResult;

    if (position_tolerance <= 0) {
        throw std::invalid_argument(
            "容差和 damping 必须为正数"
        );
    }

    ikResult.q = q_init;

    // 构建目标位姿矩阵
    pinocchio::SE3 target_pose(
        target_rotation,
        target_position
    );

    for(int iteration = 0; iteration < max_iterations; iteration++)
    {
        // 当前位姿矩阵，由前向运动学模型获得
        pinocchio::SE3 current_pose = forwardKinematics(ikResult.q);
        // 当前位姿与期望位姿的误差位姿矩阵 = 当前位姿矩阵的逆 x 期望位姿矩阵
        pinocchio::SE3 error_transform = current_pose.actInv(target_pose);

        // log6 将 SE(3) 位姿误差变为 [平移误差，旋转误差] 的 6 维向量。这里是6x1
        pinocchio::Motion motion_error =
            pinocchio::log6(error_transform);

        // 计算位置误差与旋转误差 .norm是计算欧几里得范数，即计算向量长度
        double position_error = motion_error.linear().norm();

        double rotation_error = motion_error.angular().norm();
    
        // 满足位置和姿态误差条件后，返回关节位置和成功标志位
        if ((position_error <= position_tolerance) && (rotation_error <= orientation_tolerance))
        {
            ikResult.success = true;
            return ikResult;
        }

        // 创建末端坐标系的雅克比矩阵对象
        Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model_.nv);
        // 将雅克比矩阵计算结果输入进 frame_jacobian
        pinocchio::computeFrameJacobian(model_, *data_, ikResult.q, end_effector_frame_id, pinocchio::ReferenceFrame::LOCAL, frame_jacobian);
    
        // log6将SE(3) 位姿误差映射成六维运动误差，其中SE（3）的成员变量包含了3x3的旋转矩阵和3x1的平移向量。
        Eigen::Matrix<double, 6, 6> log_jacobian =
            pinocchio::Jlog6(error_transform.inverse());

        // 由于最终目标是向期望位置移动，所以这里是负号，表示误差的趋向是减小的
        Eigen::Matrix<double, 6, Eigen::Dynamic> error_jacobian =
            -log_jacobian * frame_jacobian;

        // 将motion_error转成六维向量
        const Eigen::Matrix<double, 6, 1> error =
            motion_error.toVector();

        // cwiseProduct表示矩阵逐项相乘，即_task_weights和error逐项相乘
        Eigen::Matrix<double, 6, 1> weighted_error =
            _task_weights.cwiseProduct(error);

        // asDiagonal表示将向量转换成对角矩阵
        Eigen::Matrix<double, 6, Eigen::Dynamic> weighted_jacobian =
            _task_weights.asDiagonal() * error_jacobian;

        // 阻尼最小二乘
        Eigen::Matrix<double, 6, 6> system_matrix =
            weighted_jacobian * weighted_jacobian.transpose()
            + damping * damping
                * Eigen::Matrix<double, 6, 6>::Identity();

        // 计算delta_q,公式见笔记
        Eigen::VectorXd delta_q =
            -weighted_jacobian.transpose()
            * system_matrix.ldlt().solve(weighted_error);

        double delta_norm = delta_q.norm();

        // 限制关节变化幅度，以免发生震荡
        if (delta_norm > max_joint_step)
            delta_q *= max_joint_step/delta_norm;

        // 通过integrate来计算返回的q
        ikResult.q = pinocchio::integrate(model_, ikResult.q, delta_q);

        // 从模型中读出位置限幅，并限制q
        ikResult.q = ikResult.q.cwiseMax(model_.lowerPositionLimit).cwiseMin(model_.upperPositionLimit);
    }

    // ik迭代到最大次数后会到这里，代表失败了，记录最终误差
    // 最终 q 已经更新过，重新计算最终位姿误差。
    Eigen::Matrix<double, 6, 1> final_error =
        pinocchio::log6(
            forwardKinematics(ikResult.q).actInv(target_pose)
        ).toVector();

    double final_position_error =
        final_error.head<3>().norm();

    double final_rotation_error =
        final_error.tail<3>().norm();

    return IKResult{ikResult.q, false};

}