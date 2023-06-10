#ifndef __WMJ_CAN_PORT_HPP__
#define __WMJ_CAN_PORT_HPP__

#include <iostream>
#include <cstring>
#include <ctime>
#include <cerrno>
#include <mutex>
#include <thread>
#include <memory>
#include <queue>
#include <functional>
#include <exception>
#include <unordered_map>

#include <linux/can/raw.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <net/if.h>

#include <WMJProtocol.h>
#include <Package.hpp>

#define ERROR_PLACE std::string(__FILE__) + " : " + std::to_string(__LINE__) + " in function " + std::string(__FUNCTION__)

class CanPort
{
private:
    std::unordered_map<int, std::shared_ptr<BasePackage>> m_id_map; // 包id到类成员的映射
    bool canUseThisPort;                                            //接口可用

    int m_sock;
    sockaddr_can m_addr{};
    ifreq m_ifr{};

    canfd_frame m_send_frame{};
    canfd_frame m_read_frame{};

    std::thread m_readThread;
    std::thread m_writeThread;

    std::mutex m_can_mutex;
    std::mutex m_write_buffer_mutex;
    std::mutex m_read_buffer_mutex;

    BufferWithIDQueue m_write_buffer;

public:
    /**
     * @brief 写进程
     *
     */
    void writeThread();
    /**
     * @brief 读进程
     *
     */
    void readTread();
    /**
     * @brief Buffer转换成can包
     *
     * @param data 输入buffer
     * @param frame 输出can
     */
    int Buffer2Can(BufferWithID *data, canfd_frame *frame);
    /**
     * @brief can包转成buffer
     *
     * @param data 输出Buffer
     * @param frame 输入can
     */
    void Can2Buffer(canfd_frame *frame, Buffer *data);
    /**
     * @brief 注册can包
     *
     * @param package
     * @return int
     */
    int registerPackage(std::shared_ptr<BasePackage> package);

    /**
     * @brief 接受上层传递过来的buffer并放到缓冲区
     *
     */
    void recvBuffer(Buffer buffer, int id);

    CanPort(std::string port_name);
    ~CanPort();
};

class CanPortException : public std::exception
{
public:
    CanPortException(std::string message){this->message = message;};
    ~CanPortException(){};
    std::string message;
    const char* what(){ if(message.empty()) std::cout << "empty" << std::endl; return message.c_str();}
};

#endif //__WMJ_CAN_PORT_HPP__