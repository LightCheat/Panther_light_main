#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <sys/select.h>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <queue>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace motor_drive {

    void print_data(const uint8_t* data, uint8_t len);

    class SerialPort
    {
    public:
        using SharedPtr = std::shared_ptr<SerialPort>;

        SerialPort(std::string port, speed_t baudrate, int timeout_ms = 2);

        ~SerialPort();

        ssize_t send(const uint8_t* data, size_t len);

        ssize_t recv(uint8_t* data, size_t len);

        void recv(uint8_t* data, uint8_t head, ssize_t len);

        void set_timeout(int timeout_ms);

        // 清空内核输入缓冲和内部组帧队列，用于开始独占的参数通信事务。
        void clear_input();

    private:
        void Init(std::string port, speed_t baudrate);

        int fd_;
        fd_set rSet_;
        timeval timeout_;

        std::queue<uint8_t> recv_queue;
        std::array<uint8_t, 1024> recv_buf;
    };

}

#endif // SERIAL_PORT_H
