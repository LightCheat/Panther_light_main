#include "motor_drive/serialPort.h"

namespace motor_drive {

    /*
    * @description: 打印数据
    * @param data: 数据
    * @param len: 数据长度
    */
    void print_data(const uint8_t* data, uint8_t len)
    {
        for (int i = 0; i < len; i++)
        {
        printf("%02x ", data[i]);
        }
        printf("\n");
    }

    /*
    * @description: 构造函数 创建 SerialPort 对象时，会先设置接收超时时间，然后初始化串口
    * @param port: 端口号
    * @param baudrate: 波特率
    * @param timeout_ms: 超时时间
    */
    SerialPort::SerialPort(std::string port, speed_t baudrate, int timeout_ms)
    {
        set_timeout(timeout_ms);
        Init(port, baudrate);
    }

    /*
    * @description: 析构函数 对象销毁时关闭串口文件描述符。fd_ 就是 Linux 打开串口后得到的“句柄”。
    */
    SerialPort::~SerialPort()
    {
        close(fd_);
    }

    /*
    * @description: 发送函数 调用 Linux 的 write() 往串口写数据
    * @param data: 数据
    * @param len: 数据长度
    */
    ssize_t SerialPort::send(const uint8_t* data, size_t len)
    {
        // tcflush(fd_, TCIFLUSH);
        ssize_t ret = ::write(fd_, data, len);
        // tcdrain(fd_);
        return ret;
    }

    /*
    * @description: 接收函数 从串口读最多 len 个字节到 data。
    * @param data: 数据
    * @param len: 数据长度
    */
    ssize_t SerialPort::recv(uint8_t* data, size_t len)
    {
        FD_ZERO(&rSet_);
        FD_SET(fd_, &rSet_);
        ssize_t recv_len = 0;

        switch (select(fd_ + 1, &rSet_, NULL, NULL, &timeout_))
        {
        case -1: // error
            // std::cout << "communication error" << std::endl;
            break;
        case 0: // timeout
            // std::cout << "timeout" << std::endl;
            break;
        default:
            recv_len = ::read(fd_, data, len);
            break;
        }

        return recv_len;
    }

    /*
    * @description: 按帧接收函数 从串口读最多 len 个字节到 data。
    * @param data: 数据
    * @param head: 帧头
    * @param len: 数据长度
    */
    void SerialPort::recv(uint8_t* data, uint8_t head, ssize_t len)
    {
        if (len <= 0) {
            return;
        }
        const auto frame_len = static_cast<std::size_t>(len);

        // 存入队列
        ssize_t recv_len = this->recv(recv_buf.data(), frame_len);
        for (int i = 0; i < recv_len; i++)
        {
            recv_queue.push(recv_buf[i]);
        }

        // 查找帧头
        while (recv_queue.size() >= frame_len)
        {
            if(recv_queue.front() != head)
            {
            recv_queue.pop();
            continue;
            }
            break;
        }

        if(recv_queue.size() < frame_len) return;

        // 读取数据
        for(std::size_t i = 0; i < frame_len; i++)
        {
            data[i] = recv_queue.front();
            recv_queue.pop();
        }
    }

    /*
    * @description: 设置超时
    * @param timeout_ms: 超时时间
    */
    void SerialPort::set_timeout(int timeout_ms)
    {
        timeout_.tv_sec = timeout_ms / 1000;
        timeout_.tv_usec = (timeout_ms % 1000) * 1000;
    }

    void SerialPort::clear_input()
    {
        // 参数读写前丢弃普通状态查询留下的字节，避免 RID 回复被堵在队尾。
        tcflush(fd_, TCIFLUSH);
        std::queue<uint8_t> empty_queue;
        recv_queue.swap(empty_queue);
    }

    /*
    * @description: 初始化串口配置
    * @param port: 端口号
    * @param baudrate: 波特率
    */
    void SerialPort::Init(std::string port, speed_t baudrate)
    {
        // Open serial port
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0)
        {
            printf("Open serial port %s failed\n", port.c_str());
            exit(-1);
        }

        // Set attributes
        struct termios option;
        memset(&option, 0, sizeof(option));
        tcgetattr(fd_, &option);

        option.c_oflag = 0;
        option.c_lflag = 0;
        option.c_iflag = 0;

        cfsetispeed(&option, baudrate);
        cfsetospeed(&option, baudrate);

        option.c_cflag &= ~CSIZE;
        option.c_cflag |= CS8; // 8
        option.c_cflag &= ~PARENB; // no parity
        option.c_iflag &= ~INPCK; // no parity
        option.c_cflag &= ~CSTOPB; // 1 stop bit

        option.c_cc[VTIME] = 0;
        option.c_cc[VMIN] = 0;
        option.c_lflag |= CBAUDEX;

        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSANOW, &option);
    }

}
