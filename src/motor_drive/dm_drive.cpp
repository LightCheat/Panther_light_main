#include "motor_drive/dm_drive.h"

namespace motor_drive {

    /*
    * @brief 定义电机控制对象
    * @param serial 串口对象
    * 如果未指定串口，则默认串口为/dev/ttyACM0
    */
    DmDrive::DmDrive(SerialPort::SharedPtr serial): serial_(std::move(serial)){
        if (serial_ == nullptr) {
            //default serial port
            serial_ = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
        }
    }

    DmDrive::~DmDrive() = default;

    /*
    * @brief enable the motor 使能电机
    * @param motor 电机对象
    */
    void DmDrive::enable(const Motor& motor)
    {
        control_cmd(motor.GetSlaveId(), 0xFC);
        usleep(100000);//100ms
        this->receive();
    }

    /*
    * @brief enable motor which is old version 使能达妙旧款电机固件 使用旧版本固件建议尽快升级成新版本
    * @param motor object 电机对象
    * @param mode 控制模式  damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
    */
    void DmDrive::enable_old(const Motor& motor, Control_Mode mode)
    {
        uint32_t id = ((mode -1) << 2) + motor.GetSlaveId();
        control_cmd(id, 0xFC);
        usleep(100000);
        this->receive();
    }

    /*
    * @brief refresh motor status 刷新电机状态
    * @param motor object 电机对象
    */
    void DmDrive::refresh_motor_status(const Motor& motor)
    {
        uint32_t id = 0x7FF;
        uint8_t can_low = motor.GetSlaveId() & 0xff; // id low 8 bit
        uint8_t can_high = (motor.GetSlaveId() >> 8) & 0xff; //id high 8 bit
        std::array<uint8_t, 8> data_buf = {can_low,can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_data.modify(id, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        this->receive();
    }
    /*
    * @brief  disable the motor 失能电机
    * @param  motor object 电机对象
    */
    void DmDrive::disable(const Motor& motor) {
        control_cmd(motor.GetSlaveId(), 0xFD);
        usleep(100000);
        this->receive();
    }

    /*
    * @brief set the now position as zero point 保存当前位置为电机零点
    * @param motor object 电机对象
    */
    void DmDrive::set_zero_position(const Motor& motor)
    {
        control_cmd(motor.GetSlaveId(), 0xFE);
        usleep(100000);
        this->receive();
    }

    /* @description: MIT Control Mode MIT控制模式 具体描述请参考达妙手册
        *@param kp: 比例系数
        *@param kd: 微分系数
        *@param q: position 位置
        *@param dq: velocity 速度
        *@param tau: torque 扭矩
    */
    void DmDrive::control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
    {
        // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
        static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
            float span = xmax - xmin;
            float data_norm = (x - xmin) / span;
            uint16_t data_uint = data_norm * ((1 << bits) - 1);
            return data_uint;
        };
        Motor_id id = DM_Motor.GetSlaveId();
        if(motors.find(id) == motors.end())
        {
            throw std::runtime_error("Motor_Control id not found");
        }
        auto& m = motors[id];
        uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
        uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
        Limit_param limit_param_cmd = m->get_limit_param();
        uint16_t q_uint = float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
        uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX,limit_param_cmd.DQ_MAX, 12);
        uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

        std::array<uint8_t, 8> data_buf{};
        data_buf[0] = (q_uint >> 8) & 0xff;
        data_buf[1] = q_uint & 0xff;
        data_buf[2] = dq_uint >> 4;
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
        data_buf[4] = kp_uint & 0xff;
        data_buf[5] = kd_uint >> 4;
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
        data_buf[7] = tau_uint & 0xff;

        send_data.modify(id, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        this->receive();
    }

    /*
     * @description: Position Control Mode with velocity  位置速度控制模式
     * @param pos: position 位置
     * @param vel: velocity 速度
     * @param DM_Motor: Motor object 电机对象
     */
    void DmDrive::control_pos_vel(Motor &DM_Motor,float pos,float vel)
    {
        // 从motor_id类中得到id，并进行比照，如果id不存在，则报错
        Motor_id id = DM_Motor.GetSlaveId();
        if(motors.find(id) == motors.end())
        {
            throw std::runtime_error("POS_VEL ERROR : Motor_Control id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        // memcpy内存拷贝函数，将pos和vel地址下的内存数据拷贝到data_buf.data()内存中
        std::memcpy(data_buf.data(), &pos, sizeof(float));
        std::memcpy(data_buf.data() + 4, &vel, sizeof(float));
        id += POS_MODE;
        send_data.modify(id, data_buf.data());
        serial_->send(reinterpret_cast<uint8_t*>(&send_data), sizeof(can_send_frame));
        this->receive();
    }

    /*
     * @description: velocity control mode 速度控制模式
     * @param DM_Motor: motor object 电机对象
     * @param vel: velocity 速度（rad/s）
     */
    void DmDrive::control_vel(Motor &DM_Motor,float vel)
    {
        Motor_id id = DM_Motor.GetSlaveId();
        if(motors.find(id) == motors.end())
        {
            throw std::runtime_error("VEL ERROR : Motor_Control id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        std::memcpy(data_buf.data(), &vel, sizeof(float));
        id += SPEED_MODE;
        send_data.modify(id, data_buf.data());
        serial_->send(reinterpret_cast<uint8_t*>(&send_data), sizeof(can_send_frame));
        this->receive();
    }

    /*
     * @description 函数库内部调用,用于解算电机can线返回的电机参数.
     * 从串口/CAN 转接设备里接收一帧电机反馈数据，然后解析出电机的位置、速度、力矩，并更新到对应的电机对象里。
     */
    void DmDrive::receive()
    {
        // CAN_Receive_Frame 已经通过 #pragma pack(push, 1) 强制 1 字节对齐，
        // 因此这里可以直接按结构体大小接收完整 USB2CAN 返回帧。
        serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CAN_Receive_Frame));

        // CMD == 0x11：可能表示“接收到 CAN 数据，frameEnd == 0x55：帧尾正确
        if(receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
        {
            // 定义一个无符号整数转浮点的转换函数，下面用于接收q dq tau
            static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
                float span = xmax - xmin;
                float data_norm = float(x) / ((1 << bits) - 1);
                float data = data_norm * span + xmin;
                return data;
            };

            // data地址接收can数据地址
            auto & data = receive_data.canData;

            // 从can数据中拆出q,dq,tau相应数据
            uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
            uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
            uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];

            // 根据电机 ID 找到对应电机对象
            if(receive_data.canId != 0x00)   //make sure the motor id is not 0x00 电机id不为0时进入该函数
            {
                // 通过find函数和end函数进行id比对,id不存在则直接返回
                if(motors.find(receive_data.canId) == motors.end())
                {
                    return;
                }

                // auto关键字是根据右边表达式自动推断变量类型
                auto m = motors[receive_data.canId];

                // 通过限制参数类拿到不同对象的限制参数
                Limit_param limit_param_receive = m->get_limit_param();
                float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
                float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
                float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
                // 接收返回数据q,dq,tau
                m->receive_data(receive_q, receive_dq, receive_tau);
            }
            else //why the user set the masterid as 0x00 ??? 这段主要是用户为什么设置为0x00的猜测情况，实际上应该很难用到。
            {
                // 如果 receive_data.canId == 0x00，代码会从 data[0] 的低 4 位里取出真正的从机 ID：
                uint32_t slaveID = data[0] & 0x0f;
                if(motors.find(slaveID) == motors.end())
                {
                    return;
                }
                auto m = motors[slaveID];
                Limit_param limit_param_receive = m->get_limit_param();
                float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
                float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
                float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
                m->receive_data(receive_q, receive_dq, receive_tau);
            }
            return;
        }
        else if (receive_data.CMD == 0x01) // receive fail
        {
            /* code */
        }
        else if (receive_data.CMD == 0x02) // send fail
        {
            /* code */
        }
        else if (receive_data.CMD == 0x03) // send success
        {
            /* code */
        }
        else if (receive_data.CMD == 0xEE) // communication error
        {
            /* code */
        }
    }

    void DmDrive::receive_param()
    {
        // CAN_Receive_Frame 使用 1 字节对齐，结构体大小与 USB2CAN 返回帧长度一致。
        serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CAN_Receive_Frame));

        if(receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
        {
            auto & data = receive_data.canData;
            if(data[2]==0x33 or data[2]==0x55)
            {
                uint32_t slaveID = (uint32_t(data[1]) << 8) | data[0];
                uint8_t RID = data[3];
                if (motors.find(slaveID) == motors.end())
                {
                    //can not found motor id
                    return;
                }
                if(is_in_ranges(RID))
                {
                    uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) | (uint32_t(data[5]) << 8) | data[4];
                    motors[slaveID]->set_param(RID, data_uint32);
                }
                else
                {
                    float data_float = uint8_to_float(data + 4);
                    motors[slaveID]->set_param(RID, data_float);
                }
            }
            return ;
        }
    }

    /**
     * @brief add motor to class 添加电机
     * @param DM_Motor : motor object 电机对象
     */
    void DmDrive::addMotor(Motor *DM_Motor)
    {
        motors.insert({DM_Motor->GetSlaveId(), DM_Motor});
        if (DM_Motor->GetMasterId() != 0)
        {
            motors.insert({DM_Motor->GetMasterId(), DM_Motor});
        }
    }

    /*
    * @description: switch control mode 切换电机控制模式
    * @param DM_Motor: motor object 电机对象
    * @param mode: control mode 控制模式 like:damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
    */
    bool DmDrive::switchControlMode(Motor &DM_Motor,Control_Mode mode)
    {
        if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
        {
            return false;
        }

        const uint8_t write_data[4] = {
            static_cast<uint8_t>(mode), 0x00, 0x00, 0x00};
        constexpr uint8_t RID = 10;
        auto * registered_motor = motors[DM_Motor.GetSlaveId()];

        for(uint8_t i = 0; i < max_retries; i++)
        {
            // 每次重试前同时清除串口队列和 RID 10 旧缓存。
            // 运行期的状态查询会积累普通反馈帧，若不清理，本次 RID 回复可能一直堵在队尾。
            serial_->clear_input();
            registered_motor->clear_param(RID);
            write_motor_param(DM_Motor, RID, write_data);
            usleep(retry_interval);
            receive_param();
            if (registered_motor->is_have_param(RID) &&
                registered_motor->get_param_as_uint32(RID) ==
                    static_cast<uint32_t>(mode))
            {
                return true;
            }
        }
        return false;
    }

    void DmDrive::control_cmd(Motor_id id , uint8_t cmd)
    {
        std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
        send_data.modify(id, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
    }


    void DmDrive::write_motor_param(Motor &DM_Motor,uint8_t RID,const uint8_t data[4])
    {
        uint32_t id = DM_Motor.GetSlaveId();
        uint8_t can_low = id & 0xff;
        uint8_t can_high = (id >> 8) & 0xff;
        std::array<uint8_t, 8> data_buf{can_low, can_high, 0x55, RID, 0x00, 0x00, 0x00, 0x00};
        data_buf[4] = data[0];
        data_buf[5] = data[1];
        data_buf[6] = data[2];
        data_buf[7] = data[3];
        send_data.modify(0x7FF, data_buf.data());
        serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
    }


}  // namespace motor_drive
