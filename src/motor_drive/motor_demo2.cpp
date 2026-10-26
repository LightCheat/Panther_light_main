#include "motor_drive/dm_drive.h"
#include "motor_drive/serialPort.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

struct MitGain {
    float kp;
    float kd;
};

struct TestResult {
    float max_error;
    float avg_error;
    float final_error;
};

constexpr float kMinPosition = -1.57f;
constexpr float kMaxPosition = 1.57f;
constexpr float kZeroVelocity = 0.0f;
constexpr float kZeroTorque = 0.0f;

float clamp_position(float position)
{
    return std::clamp(position, kMinPosition, kMaxPosition);
}

TestResult run_target_test(motor_drive::DmDrive& drive,
                           motor_drive::Motor& motor,
                           const MitGain& gain,
                           float target_position)
{
    target_position = clamp_position(target_position);

    float max_error = 0.0f;
    float sum_error = 0.0f;
    float final_error = 0.0f;
    constexpr int kSamples = 250;

    for (int i = 0; i < kSamples; ++i)
    {
        drive.control_mit(motor, gain.kp, gain.kd, target_position, kZeroVelocity, kZeroTorque);

        const float error = std::fabs(target_position - motor.Get_Position());
        max_error = std::max(max_error, error);
        sum_error += error;
        final_error = error;

        usleep(4000);  // 约 250Hz 下发 MIT 控制帧。
    }

    return {max_error, sum_error / kSamples, final_error};
}

}  // namespace

int main(int argc, char* argv[])
{
    auto serial = std::make_shared<motor_drive::SerialPort>("/dev/ttyACM0", B921600);
    motor_drive::DmDrive drive(serial);

    motor_drive::Motor M1(motor_drive::DM4340, 0x01, 0x11);
    drive.addMotor(&M1);

    if (!drive.switchControlMode(M1, motor_drive::MIT_MODE))
    {
        std::cerr << "switch M1 to MIT_MODE failed" << std::endl;
        return 1;
    }

    drive.enable(M1);
    usleep(100000);

    // 从保守参数开始逐步提高 kp，kd 用来抑制振荡；如出现明显抖动或异响，应立即停止程序。
    const std::vector<MitGain> gains = {
        {10.0f, 0.20f},
        {20.0f, 0.35f},
        {35.0f, 0.55f},
        {50.0f, 0.80f},
        {70.0f, 1.10f},
    };

    // 目标点限制在 M1 可动范围 [-1.57, 1.57] 内，先用小幅阶跃观察误差和稳定性。
    const std::vector<float> targets = {
        0.0f,
        0.4f,
        -0.4f,
        0.8f,
        -0.8f,
        0.0f,
    };

    for (const auto& gain : gains)
    {
        std::cout << "Testing MIT gain: kp=" << gain.kp << ", kd=" << gain.kd << std::endl;

        for (float target : targets)
        {
            const auto result = run_target_test(drive, M1, gain, target);
            std::cout << "  target=" << target
                      << " current=" << M1.Get_Position()
                      << " final_error=" << result.final_error
                      << " avg_error=" << result.avg_error
                      << " max_error=" << result.max_error
                      << std::endl;
        }

        std::cout << std::endl;
    }

    // 测试结束后回到零位并失能，避免电机长时间保持力矩。
    for (int i = 0; i < 250; ++i)
    {
        drive.control_mit(M1, 20.0f, 0.4f, 0.0f, kZeroVelocity, kZeroTorque);
        usleep(4000);
    }

    drive.disable(M1);
    return 0;
}
