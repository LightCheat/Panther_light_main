#include "motor_drive/dm_drive.h"
#include "motor_drive/serialPort.h"

int main(int argc, char *argv[])
{
    auto serial = std::make_shared<motor_drive::SerialPort>("/dev/ttyACM0", B921600);
    motor_drive::DmDrive dm1(serial);

    motor_drive::Motor M1(motor_drive::DM4310, 0x01, 0x11);
    motor_drive::Motor M2(motor_drive::DM4340, 0x02, 0x12);
    motor_drive::Motor M3(motor_drive::DM4340, 0x03, 0x13);

    dm1.addMotor(&M1);
    dm1.addMotor(&M2);
    dm1.addMotor(&M3);

    dm1.switchControlMode(M1, motor_drive::POS_VEL_MODE);
    
    dm1.enable(M1);
    dm1.enable(M3);

    
    dm1.control_pos_vel(M1, 1, 0.5);

    return 0;
}
