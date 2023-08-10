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

#include <Port.hpp>
#include <Utility.hpp>
#include <Package.hpp>
#include <PackageManager.hpp>

class CanPort : public Port
{
public:
    using SharedPtr = std::shared_ptr<CanPort>;
private:

    int m_sock;
    sockaddr_can m_addr{};
    ifreq m_ifr{};

    canfd_frame m_send_frame{};
    canfd_frame m_read_frame{};

    std::thread m_readThread;
    std::thread m_writeThread;

    std::mutex m_can_mutex;

    IENUM USLEEP_LENGTH = 10;

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
     * @brief 写一次
     * 
     * @param failed_cnt 
     */
    void writeOnce(int &failed_cnt);
    /**
     * @brief 读一次
     * 
     * @param failed_cnt 
     */
    void readOnce(int &failed_cnt);
    
    CanPort(std::string port_name);
    ~CanPort();
};

#endif //__WMJ_CAN_PORT_HPP__