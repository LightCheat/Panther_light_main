#ifndef DM_DRIVE_H_
#define DM_DRIVE_H_

#include "motor_drive/serialPort.h"
#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <stdexcept>
#include <unordered_map>


// 不同模式的id前缀，电机说明书有写
#define POS_MODE 0x100
#define SPEED_MODE 0x200
#define POSI_MODE 0x300
#define max_retries 20
#define retry_interval 50000


namespace motor_drive {

#define Motor_id uint32_t  

    // enum 是 C/C++ 里的枚举类型，作用是给一组相关的整数常量起名字。不赋值的话默认从0开始，例如 DM4310 = 0, DM4340_48V = 3
    /*!
    * @brief Motor Type 电机类型
    */
    enum DM_Motor_Type
    {
        DM4310,
        DM4310_48V,
        DM4340,
        DM4340_48V,
        DM6006,
        DM8006,
        DM8009,
        DM10010L,
        DM10010,
        DMH3510,
        DMH6215,
        DMG6220,
        Num_Of_Motor
    };

    /*
     * @brief 电机控制模式
     */
    enum Control_Mode
    {
        MIT_MODE=1,
        POS_VEL_MODE=2,
        VEL_MODE=3,
        POS_FORCE_MODE=4,
    };

    // USB2CAN 协议帧必须按字节紧凑排列。
    // 这里强制 1 字节对齐，避免 uint32_t 等成员触发编译器自动插入 padding，
    // 否则 sizeof(can_send_frame) 会大于协议规定的 30 字节，导致 USB2CAN 解析失败。
#pragma pack(push, 1)

    // 两种结构体的写法，第一种是匿名的，第二种给了一个名称。有一些区别，可查阅。
    struct CAN_Receive_Frame
    {
        uint8_t FrameHeader;
        uint8_t CMD;// 命令 0x00: 心跳
        //     0x01: receive fail 0x11: receive success
        //     0x02: send fail 0x12: send success
        //     0x03: set baudrate fail 0x13: set baudrate success
        //     0xEE: communication error 此时格式段为错误码
        //     8: 超压 9: 欠压 A: 过流 B: MOS过温 C: 电机线圈过温 D: 通讯丢失 E: 过载
        uint8_t canDataLen: 6; // 数据长度
        uint8_t canIde: 1; // 0: 标准帧 1: 扩展帧
        uint8_t canRtr: 1; // 0: 数据帧 1: 远程帧
        uint32_t canId; // 电机反馈的ID
        uint8_t canData[8];
        uint8_t frameEnd; // 帧尾
    };

    typedef struct can_send_frame
    {
        uint8_t FrameHeader[2] = {0x55, 0xAA}; // 帧头
        uint8_t FrameLen = 0x1e; // 帧长
        uint8_t CMD = 0x03; // 命令 1：转发CAN数据帧 2：PC与设备握手，设备反馈OK 3: 非反馈CAN转发，不反馈发送状态
        uint32_t sendTimes = 1; // 发送次数
        uint32_t timeInterval = 10; // 时间间隔
        uint8_t IDType = 0; // ID类型 0：标准帧 1：扩展帧
        uint32_t canId=0x01; // CAN ID 使用电机ID作为CAN ID
        uint8_t frameType = 0; // 帧类型 0： 数据帧 1：远程帧
        uint8_t len = 0x08; // len
        uint8_t idAcc=0;
        uint8_t dataAcc=0;
        uint8_t data[8]={0};
        uint8_t crc=0; // 未解析，任意值

        void modify(const Motor_id id, const uint8_t* send_data)
        {
            canId = id;
            std::copy(send_data, send_data+8, data);
        }

    } can_send_frame;

#pragma pack(pop)

    typedef struct
    {
        float Q_MAX;
        float DQ_MAX;
        float TAU_MAX;
    }Limit_param;

    //电机PMAX DQMAX TAUMAX参数
    inline constexpr Limit_param limit_param[Num_Of_Motor]=
    {
        {12.5, 30, 10 }, // DM4310
        {12.5, 50, 10 }, // DM4310_48V
        {12.5, 8, 28 },  // DM4340
        {12.5, 10, 28 }, // DM4340_48V
        {12.5, 45, 20 }, // DM6006
        {12.5, 45, 40 }, // DM8006
        {12.5, 45, 54 }, // DM8009
        {12.5,25,  200}, // DM10010L
        {12.5,20, 200},  // DM10010
        {12.5,280,1},    // DMH3510
        {12.5,45,10},    // DMH6215
        {12.5,45,10}     // DMG6220
    };

    
    class Motor{
    private:
        /* data */
        Motor_id Master_id;
        Motor_id Slave_id;
        float state_q=0;
        float state_dq=0;
        float state_tau=0;
        Limit_param limit_param_{};
        DM_Motor_Type Motor_Type;

        union ValueUnion {
            float floatValue;
            uint32_t uint32Value;
        };

        struct ValueType {
            ValueUnion value;
            bool isFloat;
        };

        std::unordered_map<uint32_t , ValueType> param_map;

    public:
        /**
         * @brief Construct a new Motor object
         *
         * @param Motor_Type 电机类型
         * @param Slave_id canId 从机ID即电机ID
         * @param Master_id 主机ID建议主机ID不要都设为0x00
         *
         */
        Motor(DM_Motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id)
                : Master_id(Master_id), Slave_id(Slave_id), Motor_Type(Motor_Type) {
            this->limit_param_ = motor_drive::limit_param[Motor_Type];
        }

        Motor() : Master_id(0x01), Slave_id(0x11), Motor_Type(DM4310) {
            this->limit_param_ = motor_drive::limit_param[DM4310];
        }

        ~Motor() = default;

        void receive_data(float q, float dq, float tau)
        {
            this->state_q = q;
            this->state_dq = dq;
            this->state_tau = tau;
        }

        DM_Motor_Type GetMotorType() const { return this->Motor_Type; }

        /*
         * @brief get master id 获取主机ID
         * @return MasterID
         */
        Motor_id GetMasterId() const { return this->Master_id; }

        /*
         * @brief get motor slave id(can id)  获取电机CAN ID
         * @return SlaveID
         */
        Motor_id GetSlaveId() const { return this->Slave_id; }

        /*
         * @brief get motor position 获取电机位置
         * @return motor position 电机位置
         */
        float Get_Position() const { return this->state_q; }

        /*
         * @brief get motor velocity 获取电机速度
         * @return motor velocity 电机速度
         */
        float Get_Velocity() const { return this->state_dq; }

        /*
         * @brief get torque of the motor  获取电机实际输出扭矩
         * @return motor torque 电机实际输出扭矩
         */
        float Get_tau() const { return this->state_tau; }

        /*
         * @brief get limit param 获取电机限制参数
         * @return limit_param 电机限制参数
         */
        Limit_param get_limit_param() { return this->limit_param_; }


        void set_param(int key, float value)
        {
            ValueType v{};
            v.value.floatValue = value;
            v.isFloat = true;
            param_map[key] = v;
        }

        void set_param(int key, uint32_t value)
        {
            ValueType v{};
            v.value.uint32Value = value;
            v.isFloat = false;
            param_map[key] = v;
        }

        float get_param_as_float(int key) const
        {
            auto it = param_map.find(key);
            if (it != param_map.end())
            {
                if (it->second.isFloat)
                {
                    return it->second.value.floatValue;
                }
                else
                {
                    return 0;
                }
            }
            return 0;
        }

        uint32_t get_param_as_uint32(int key) const {
            auto it = param_map.find(key);
            if (it != param_map.end()) {
                if (!it->second.isFloat) {
                    return it->second.value.uint32Value;
                }
                else
                {
                    return 0;
                }
            }
            return 0;
        }

        bool is_have_param(int key) const
        {
            return param_map.find(key) != param_map.end();
        }

        // 切换控制模式前清除旧参数，避免将上一次的 RID 反馈当成本次结果。
        void clear_param(int key)
        {
            param_map.erase(key);
        }
    };

    class DmDrive {

    public:
        DmDrive(SerialPort::SharedPtr serial = nullptr);
        ~DmDrive();

        void enable(const Motor& motor);
        void enable_old(const Motor& motor, Control_Mode mode);
        void refresh_motor_status(const Motor& motor);
        void disable(const Motor& motor);
        void set_zero_position(const Motor& motor);
        void control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau);
        void control_pos_vel(Motor &DM_Motor,float pos,float vel);
        void control_vel(Motor &DM_Motor,float vel);


        void receive();
        void receive_param();
        void addMotor(Motor *DM_Motor);
        bool switchControlMode(Motor &DM_Motor,Control_Mode mode);

    private:
        void control_cmd(Motor_id id , uint8_t cmd);
        void write_motor_param(Motor &DM_Motor,uint8_t RID,const uint8_t data[4]);

        static bool is_in_ranges(int number) {
            return (7 <= number && number <= 10) ||
                    (13 <= number && number <= 16) ||
                    (35 <= number && number <= 36);
        }

        static uint32_t float_to_uint32(float value) {
            return static_cast<uint32_t>(value);
        }

        static float uint32_to_float(uint32_t value) {
            return static_cast<float>(value);
        }

        static float uint8_to_float(const uint8_t data[4]) {
            uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) |
                                (static_cast<uint32_t>(data[2]) << 16) |
                                (static_cast<uint32_t>(data[1]) << 8)  |
                                static_cast<uint32_t>(data[0]);
            float result;
            memcpy(&result, &combined, sizeof(result));
            return result;
        }
        
        // 定义了一个名叫 motors 的哈希表，用来保存“电机 ID 到电机对象指针”的映射关系
        std::unordered_map<Motor_id, Motor*> motors;
        SerialPort::SharedPtr serial_;  //serial port
        can_send_frame send_data; //send data frame
        CAN_Receive_Frame receive_data{};//receive data frame

    };

}  // namespace motor_drive

#endif  // DM_DRIVE_H_
