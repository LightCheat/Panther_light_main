#ifndef PANTHER_HARDWARE_INTERFACE__PANTHER_HARDWARE_INTERFACE_HPP_
#define PANTHER_HARDWARE_INTERFACE__PANTHER_HARDWARE_INTERFACE_HPP_

// c++ 标准库
#include <memory>
#include <string>
#include <vector>

// ROS2 hardware interface library 硬件接口库
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// ROS2 control library 控制库
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// 电机驱动库
#include "motor_drive/dm_drive.h"

namespace panther_hardware_interface
{
// PantherHard硬件接口类
class PantherHardwareInterface : public hardware_interface::SystemInterface
{
    public:
        // 初始化硬件接口
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;

        // 注册硬件接口，建立真实硬件通信
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        // 激活硬件接口，可以把使能放在这里
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        // 停用硬件接口，可以把失能放在这里
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;  

        // 读取状态接口
        hardware_interface::return_type read(
            const rclcpp::Time & time,
            const rclcpp::Duration & period) override;

        // 写入状态接口
        hardware_interface::return_type write(
            const rclcpp::Time & time,
            const rclcpp::Duration & period) override;

        // 切换命令模式接口，主要是切换位置模式和速度模式
        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces) override;

        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces) override;

        // 导出状态接口
        std::vector<hardware_interface::StateInterface>export_state_interfaces() override;

        // 导出命令接口
        std::vector<hardware_interface::CommandInterface>export_command_interfaces() override;

        private:

            // 创建串口、电机和驱动对象
            motor_drive::SerialPort::SharedPtr serial_;
            std::vector<motor_drive::Motor> motors_;
            std::unique_ptr<motor_drive::DmDrive> drive_;

            // 加载硬件参数方法
            bool loadHardwareParameters();
            // 重置电机状态函数
            void resizeStateBuffers();
            // 将 panther_description.ros2_control.xacro 中字符串类型的电机类型转换为 motor_drive::DM_Motor_Type 的枚举类型
            motor_drive::DM_Motor_Type parseMotorType(const std::string& motor_type) const;
            // 将 panther_description.ros2_control.xacro 中的波特率参数转换为 speed_t 类型
            speed_t parseBaudrate(int baudrate) const;
            
        // panther_description.ros2_control.xacro 相关参数
            std::string serial_port_;
            int baudrate_ = 921600;

            // 位速模式的kp和kd以及速度限制参数
            std::vector<double> kp_;
            std::vector<double> kd_;
            std::vector<double> pos_vel_velocities_;

            // 电机的从机ID、主机ID、电机类型、最小位置和最大位置参数
            std::vector<uint32_t> slave_ids_;
            std::vector<uint32_t> master_ids_;
            std::vector<motor_drive::DM_Motor_Type> motor_types_;
            std::vector<double> min_positions_;
            std::vector<double> max_positions_;

        // hardware状态的写入和读取变量
            std::vector<double> hw_positions_;
            std::vector<double> hw_velocity_;

            std::vector<double> hw_position_commands_;
            std::vector<double> hw_velocity_commands_;

        // 模式切换相关变量
            enum class CommandMode { kNone, kPosition, kVelocity, kEffort };
            // 记录当前请求的命令模式和当前激活的命令模式的标志位
            CommandMode requested_command_mode_ = CommandMode::kNone;
            CommandMode active_command_mode_ = CommandMode::kNone;

            // 限制命令值在最小和最大位置之间,内部调用了std的clamp限幅函数
            double clampCommand(std::size_t index, double command) const;



            
};
}

#endif