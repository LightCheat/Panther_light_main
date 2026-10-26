#include "6Dof_dyna.h"

VectorXd compute_gravity_compensation(const double q[6])
{
    // 初始化运动学链
    Matrix3d R01, R12, R23, R34, R45, R56; // R_i^{i-1}
    Matrix3d R10, R21, R32, R43, R54, R65; // R_i^{i-1}
    Vector3d v11d, v22d, v33d, v44d, v55d, v66d;
    Vector3d vc11d, vc22d, vc33d, vc44d, vc55d, vc66d;
    Vector3d F11, F22, F33, F44, F55, F66;
    Vector3d f11, f22, f33, f44, f55, f66;
    Vector3d n11, n22, n33, n44, n55, n66;

    // 初始 theta 偏置
    double q_offset[6];
    q_offset[0] = q[0];
    q_offset[1] = q[1] + PI;
    q_offset[2] = q[2] - 162.43 * PI / 180.0;
    q_offset[3] = q[3] + 17.57 * PI / 180.0;
    q_offset[4] = q[4] + PI / 2;
    q_offset[5] = q[5];

    R01 << cos(q_offset[0]), -sin(q_offset[0]), 0.0,
        sin(q_offset[0]), cos(q_offset[0]), 0.0,
        0.0, 0.0, 1;

    R12 << cos(q_offset[1]), -sin(q_offset[1]), 0.0,
        0.0, 0.0, 1,
        -sin(q_offset[1]), -cos(q_offset[1]), 0.0;

    R23 << cos(q_offset[2]), -sin(q_offset[2]), 0.0,
        -sin(q_offset[2]), -cos(q_offset[2]), 0.0,
        0.0, 0.0, -1;

    R34 << cos(q_offset[3]), -sin(q_offset[3]), 0.0,
        -sin(q_offset[3]), -cos(q_offset[3]), 0.0,
        0.0, 0.0, -1;

    R45 << cos(q_offset[4]), -sin(q_offset[4]), 0.0,
        0.0, 0.0, 1,
        -sin(q_offset[4]), -cos(q_offset[4]), 0.0;

    R56 << cos(q_offset[5]), -sin(q_offset[5]), 0.0,
        0.0, 0.0, -1,
        sin(q_offset[5]), cos(q_offset[5]), 0.0;

    R10 = R01.transpose();
    R21 = R12.transpose();
    R32 = R23.transpose();
    R43 = R34.transpose();
    R54 = R45.transpose();
    R65 = R56.transpose();

    // 外推初始化（基座）
    Vector3d v00d = GRAVITY; // 基座线加速度（包含重力）

    v11d = R10 * v00d;
    v22d = R21 * v11d;
    v33d = R32 * v22d;
    v44d = R43 * v33d;
    v55d = R54 * v44d;
    v66d = R65 * v55d;

    vc11d = v11d;
    vc22d = v22d;
    vc33d = v33d;
    vc44d = v44d;
    vc55d = v55d;
    vc66d = v66d;

    F11 = MASS[0] * vc11d;
    F22 = MASS[1] * vc22d;
    F33 = MASS[2] * vc33d;
    F44 = MASS[3] * vc44d;
    F55 = MASS[4] * vc55d;
    F66 = MASS[5] * vc66d;

    VectorXd tau(6);

    f66 = F66;
    f55 = R56 * f66 + F55;
    f44 = R45 * f55 + F44;
    f33 = R34 * f44 + F33;
    f22 = R23 * f33 + F22;
    f11 = R12 * f22 + F11;

    n66 = pc[6 - 1].cross(F66);
    n55 = (R56 * n66) + pc[5 - 1].cross(F55) + p[6 - 1].cross(R56 * f66);
    n44 = (R45 * n55) + pc[4 - 1].cross(F44) + p[5 - 1].cross(R45 * f55);
    n33 = (R34 * n44) + pc[3 - 1].cross(F33) + p[4 - 1].cross(R34 * f44);
    n22 = (R23 * n33) + pc[2 - 1].cross(F22) + p[3 - 1].cross(R23 * f33);
    n11 = (R12 * n22) + pc[1 - 1].cross(F11) + p[2 - 1].cross(R12 * f22);

    tau[5] = n66.dot(Z_AXIS);
    tau[4] = n55.dot(Z_AXIS);
    tau[3] = n44.dot(Z_AXIS);
    tau[2] = n33.dot(Z_AXIS);
    tau[1] = n22.dot(Z_AXIS);
    tau[0] = n11.dot(Z_AXIS);

    return tau;
}
