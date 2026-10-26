// 程序启动时，ROS 2 Control 会解析 panther_description.ros2_control.xacro文件中的参数
#include "panther_hardware_interface/panther_hardware_interface.hpp"

// 注册插件类
#include <pluginlib/class_list_macros.hpp>
// hardware 硬件类型
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace panther_hardware_interface
{
  // panther_description.ros2_control.xacro 的参数都存在 info_ 中，on_init函数会被ROS2 Control调用
  // 该函数主要是初始化电机参数，不包括电机使能部分
  hardware_interface::CallbackReturn PantherHardwareInterface::on_init(
      const hardware_interface::HardwareInfo& info)
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 加载参数
    if (!loadHardwareParameters()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 初始化电机状态
    resizeStateBuffers();

    return hardware_interface::CallbackReturn::SUCCESS;

  }

  // 注册函数，主要是创建电机和电机驱动对象，然后更改驱动模式以及初始化电机真实状态
  hardware_interface::CallbackReturn PantherHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& previous_state)
  {
    // 未使用该参数，消除编译器警告
    (void)previous_state;
    
    serial_ = std::make_shared<motor_drive::SerialPort>(serial_port_, parseBaudrate(baudrate_));
    drive_ = std::make_unique<motor_drive::DmDrive>(serial_);

    // 传入motors_的相关参数，并添加电机驱动对象，切换模式，获取当前状态并更新至hw_中
    motors_.clear();
    motors_.reserve(info_.joints.size());
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      motors_.emplace_back(motor_types_[i], slave_ids_[i], master_ids_[i]);
      drive_->addMotor(&motors_.back());
      drive_->switchControlMode(motors_.back(), motor_drive::Control_Mode::POS_VEL_MODE);
      drive_->refresh_motor_status(motors_.back());

      // 更新电机状态以及将当前状态作为命令
      hw_positions_[i] = motors_[i].Get_Position();

      hw_position_commands_[i] = hw_positions_[i];
      hw_velocity_commands_[i] = 0.0;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 这里只放了使能
  hardware_interface::CallbackReturn PantherHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& previous_state)
  {
    // 未使用该参数，消除编译器警告
    (void)previous_state;

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      // 使能电机
      drive_->enable(motors_[i]);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 使能稳定后重新读取真实位置
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        drive_->refresh_motor_status(motors_[i]);
        hw_positions_[i] = motors_[i].Get_Position();
        hw_velocity_[i] = motors_[i].Get_Velocity();
    }

    // 将电机当前位置状态赋值到位置命令数组中，防止使能后电机位置跳变
    // 注：这里使用copy和fill是因为hw_position_commands_ = hw_positions_ 会改变底层地址的原地
    std::copy(
    hw_positions_.begin(),
    hw_positions_.end(),
    hw_position_commands_.begin());

    std::fill(
        hw_velocity_commands_.begin(),
        hw_velocity_commands_.end(),
        0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
    
  }

  // 这里只放了失能
  hardware_interface::CallbackReturn PantherHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
  {
    // 未使用该参数，消除编译器警告
    (void)previous_state;

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      // 失能电机
      drive_->disable(motors_[i]);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 导出状态接口，从hw_positions_中获取值赋值到state_interfaces
  std::vector<hardware_interface::StateInterface> PantherHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(info_.joints.size() * 3);

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      state_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]);

      state_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_[i]);
    }
    return state_interfaces;
  }

  // 导出命令接口，从hw_position_commands_中获取值赋值到command_interfaces
  std::vector<hardware_interface::CommandInterface> PantherHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(info_.joints.size() * 3);

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      command_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_position_commands_[i]);
    }

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      command_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_commands_[i]);
    }

    return command_interfaces;
  }

  // 读取状态接口，将电机的当前位置和速度更新到hw_positions_和hw_velocity_中
  hardware_interface::return_type PantherHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    (void)time;
    (void)period;
      
    for (std::size_t i = 0; i < motors_.size(); ++i) {
      drive_->refresh_motor_status(motors_[i]);
      hw_positions_[i] = motors_[i].Get_Position();
      hw_velocity_[i] = motors_[i].Get_Velocity();
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type PantherHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    (void)time;
    (void)period;
    
    for (std::size_t i = 0; i < motors_.size(); ++i) {
      const auto limits = motors_[i].get_limit_param();
      switch (active_command_mode_) {
        case CommandMode::kPosition:
          drive_->control_pos_vel(motors_[i], static_cast<float>(clampCommand(i, hw_position_commands_[i])), static_cast<float>(pos_vel_velocities_[i]));
          break;
        case CommandMode::kVelocity:
          drive_->control_vel(motors_[i], static_cast<float>(std::clamp(hw_velocity_commands_[i], -static_cast<double>(limits.DQ_MAX), static_cast<double>(limits.DQ_MAX))));
          // RCLCPP_INFO(rclcpp::get_logger("PantherHardwareInterface"), "Velocity command: %.4f", hw_velocity_commands_[0]);
          break;
        case CommandMode::kEffort:
          break;
        case CommandMode::kNone:
          break;
      }
      // drive_->control_pos_vel(motors_[i], hw_position_commands_[i], static_cast<float>(pos_vel_velocities_[i]));
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type PantherHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
  {
      bool start_position = false;
      bool start_velocity = false;
      bool stop_position = false;
      bool stop_velocity = false;

      const auto ends_with =
          [](const std::string& value, const std::string& suffix) {
              return value.size() >= suffix.size() &&
                    value.compare(
                        value.size() - suffix.size(),
                        suffix.size(),
                        suffix) == 0;
          };

      for (const auto& interface : start_interfaces) {
          start_position |= ends_with(interface, "/position");
          start_velocity |= ends_with(interface, "/velocity");
      }

      for (const auto& interface : stop_interfaces) {
          stop_position |= ends_with(interface, "/position");
          stop_velocity |= ends_with(interface, "/velocity");
      }

      if (start_position && start_velocity) {
          RCLCPP_ERROR(
              rclcpp::get_logger("PantherHardwareInterface"),
              "Cannot activate position and velocity simultaneously");
          return hardware_interface::return_type::ERROR;
      }

      if (start_position) {
          requested_command_mode_ = CommandMode::kPosition;
      } else if (start_velocity) {
          requested_command_mode_ = CommandMode::kVelocity;
      } else if (stop_position || stop_velocity) {
          requested_command_mode_ = CommandMode::kNone;
      }

      RCLCPP_INFO(
          rclcpp::get_logger("PantherHardwareInterface"),
          "Requested command mode: %d",
          static_cast<int>(requested_command_mode_));

      return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type PantherHardwareInterface::perform_command_mode_switch(const std::vector<std::string>&, const std::vector<std::string>&)
  {
      if (requested_command_mode_ == CommandMode::kVelocity) {
          std::fill(
              hw_velocity_commands_.begin(),
              hw_velocity_commands_.end(),
              0.0);

          for (auto& motor : motors_) {
              if (!drive_->switchControlMode(
                      motor, motor_drive::Control_Mode::VEL_MODE)) {
                  return hardware_interface::return_type::ERROR;
              }
          }
      } else if (requested_command_mode_ == CommandMode::kPosition) {
          std::copy(
              hw_positions_.begin(),
              hw_positions_.end(),
              hw_position_commands_.begin());

          for (auto& motor : motors_) {
              if (!drive_->switchControlMode(
                      motor, motor_drive::Control_Mode::POS_VEL_MODE)) {
                  return hardware_interface::return_type::ERROR;
              }
          }
      }

      active_command_mode_ = requested_command_mode_;
      return hardware_interface::return_type::OK;
  }

  bool PantherHardwareInterface::loadHardwareParameters()
  {
    // 从 info_ 查找串口参数信息，如果迭代后没找到，则使用/dev/ttyACM0
    auto serial_it = info_.hardware_parameters.find("serial_port");
    if (serial_it == info_.hardware_parameters.end()) {
        serial_port_ = "/dev/ttyACM0";
    } else {
        serial_port_ = serial_it->second;
    }

    // 从 info_ 查找波特率参数信息，如果没有则使用默认值921600
    auto baudrate_it = info_.hardware_parameters.find("baudrate");
    if (baudrate_it != info_.hardware_parameters.end()) {
        baudrate_ = std::stoi(baudrate_it->second);
    }

    // 从 info_ 中每个关节电机的kp和kd初始化赋值
    kp_.assign(info_.joints.size(), 50.0);
    kd_.assign(info_.joints.size(), 0.8);
    pos_vel_velocities_.assign(info_.joints.size(), 1.0);

    // 从 info_ 中对每个关节电机的发送id和接收id以及电机类型，最小位置和最大位置初始化赋值
    slave_ids_.assign(info_.joints.size(), 0);
    master_ids_.assign(info_.joints.size(), 0);
    motor_types_.assign(info_.joints.size(), motor_drive::DM4340);
    min_positions_.assign(info_.joints.size(), -std::numeric_limits<double>::infinity());
    max_positions_.assign(info_.joints.size(), std::numeric_limits<double>::infinity());

    // 遍历每个关节的参数，提取kp、kd、pos_vel_velocity、motor_type、slave_id、master_id、min_position和max_position
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      const auto& params = info_.joints[i].parameters;

      auto kp_it = params.find("kp");
      if (kp_it != params.end()) {
        kp_[i] = std::stod(kp_it->second);
      }

      auto kd_it = params.find("kd");
      if (kd_it != params.end()) {
        kd_[i] = std::stod(kd_it->second);
      }

      auto pos_vel_velocity_it = params.find("pos_vel_velocity");
      if (pos_vel_velocity_it != params.end()) {
        pos_vel_velocities_[i] = std::abs(std::stod(pos_vel_velocity_it->second));
      }

      auto motor_type_it = params.find("motor_type");
      if (motor_type_it != params.end()) {
        motor_types_[i] = parseMotorType(motor_type_it->second);
      }

      auto slave_id_it = params.find("slave_id");
      if (slave_id_it == params.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("PantherSystemInterface"),
                    "Joint %s missing required parameter slave_id",
                    info_.joints[i].name.c_str());
        return false;
      }
      slave_ids_[i] = static_cast<uint32_t>(std::stoul(slave_id_it->second, nullptr, 0));

      auto master_id_it = params.find("master_id");
      if (master_id_it != params.end()) {
        master_ids_[i] = static_cast<uint32_t>(std::stoul(master_id_it->second, nullptr, 0));
      }

      auto min_it = params.find("min_position");
      if (min_it != params.end()) {
        min_positions_[i] = std::stod(min_it->second);
      }

      auto max_it = params.find("max_position");
      if (max_it != params.end()) {
        max_positions_[i] = std::stod(max_it->second);
      }
    }

    return true;
  }

  // 将 panther_description.ros2_control.xacro 中字符串类型的电机类型转换为 motor_drive::DM_Motor_Type 的枚举类型
  motor_drive::DM_Motor_Type PantherHardwareInterface::parseMotorType(
      const std::string& motor_type) const
  {
    static const std::unordered_map<std::string, motor_drive::DM_Motor_Type> kMotorTypes = {
        {"DM4310", motor_drive::DM4310},
        {"DM4310_48V", motor_drive::DM4310_48V},
        {"DM4340", motor_drive::DM4340},
        {"DM4340_48V", motor_drive::DM4340_48V},
        {"DM6006", motor_drive::DM6006},
        {"DM8006", motor_drive::DM8006},
        {"DM8009", motor_drive::DM8009},
        {"DM10010L", motor_drive::DM10010L},
        {"DM10010", motor_drive::DM10010},
        {"DMH3510", motor_drive::DMH3510},
        {"DMH6215", motor_drive::DMH6215},
        {"DMG6220", motor_drive::DMG6220},
    };

    auto it = kMotorTypes.find(motor_type);
    if (it == kMotorTypes.end()) {
      throw std::invalid_argument("Unsupported motor_type: " + motor_type);
    }
    return it->second;
  }

  // 重置电机状态函数
  void PantherHardwareInterface::resizeStateBuffers()
  {
    const auto joint_count = info_.joints.size();
    const double quiet_nan = std::numeric_limits<double>::quiet_NaN();

    hw_positions_.assign(joint_count, 0.0);
    hw_velocity_.assign(joint_count, 0.0);
    hw_position_commands_.assign(joint_count, quiet_nan);
    hw_velocity_commands_.assign(joint_count, quiet_nan);

  }

  // 这里波特率的前面要加一个B才能识别读取
  speed_t PantherHardwareInterface::parseBaudrate(int baudrate) const
  {
    switch (baudrate) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 921600:
        return B921600;
      default:
        throw std::invalid_argument("Unsupported serial baudrate: " + std::to_string(baudrate));
    }
  }

  double PantherHardwareInterface::clampCommand(std::size_t index, double command) const
  {
    if (!std::isfinite(command)) {
      return hw_positions_[index];
    }
    return std::clamp(command, min_positions_[index], max_positions_[index]);
  }
}




// 注册插件，使得 ROS 2 Control 可以识别 PantherHardwareInterface 类
PLUGINLIB_EXPORT_CLASS(
    panther_hardware_interface::PantherHardwareInterface,
    hardware_interface::SystemInterface)

