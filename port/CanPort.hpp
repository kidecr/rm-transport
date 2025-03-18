#ifndef __WMJ_CAN_PORT_HPP__
#define __WMJ_CAN_PORT_HPP__

#ifdef ENABLE_UNIX_CAN_PORT

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

#include "impls/Port.hpp"
#include "utils/mask.hpp"
#include "utils/Utility.hpp"
#include "impls/BasePackage.hpp"
#include "PackageManager.hpp"
#include "impls/logger.hpp"

namespace transport
{

class CanPort : public Port
{
public:
    using SharedPtr = std::shared_ptr<CanPort>;
    constexpr static double TIMEOUT = 2;

private:
    int m_sock;
    sockaddr_can m_addr{};
    ifreq m_ifr{};

    canfd_frame m_send_frame{};
    canfd_frame m_read_frame{};

    std::jthread m_readThread;
    std::jthread m_writeThread;

    std::mutex m_can_mutex;

    int write_usleep_length;
    int read_usleep_length;
    IENUM WRITE_USLEEP_LENGTH = 200;     // 写线程正常usleep时间
    IENUM READ_USLEEP_LENGTH = 200;     // 读线程正常usleep时间
    IENUM STANDBY_USLEEP_LENGTH = 1e6; // 进程崩溃后待机时间

public:
    CanPort(std::string port_name, uint32_t group_id = 0, uint32_t port_id = 0, std::string passwd = "a") : Port(port_name, group_id, port_id, passwd)
    {
        //可以使用can设备的标志位
        this->m_port_scheduler_available = false;
        this->m_port_name = port_name;
        this->m_port_is_available = initCanDevice(port_name);
        if (m_port_is_available)
        {
            LOGINFO("%s Open", port_name.c_str());
            m_readThread = std::jthread(&CanPort::readTread, this);
            m_writeThread = std::jthread(&CanPort::writeThread, this);
            if (m_port_scheduler_available) {
                m_port_status->status = PortStatus::Available;
            }
        }
        else
        {
            LOGERROR("[CANERROR] create port failed! port name: %s! U2CAN插好了吗?代码拷过了吗?sudo ip link了吗?", port_name.c_str());
            throw PORT_EXCEPTION("create port failed! port name: " + port_name);
        }
    }

    /**
     * @brief 析构函数，停止循环，port置为不可访问
     *
     */
    ~CanPort()
    {
        m_port_is_available = false;
        close(m_sock);
        transport::shutdown();
    }

    bool reinit() override {
        LOGINFO("CanPort reinit");

        // 关闭can socket
        close(m_sock);
        // 系统命令重置can设备
        char cmd[256];
        sprintf(cmd,
                "echo \"%s\" | sudo -S ip link set %s down && sudo ip link set %s type can bitrate 1000000 && sudo ip link set %s up",
                m_passwd.c_str(), m_port_name.c_str(), m_port_name.c_str(), m_port_name.c_str());
        int ret = 1;
        for(int i = 0; i < 5 && ret; ++i) { // 重复5次尝试执行命令
            ret = std::system(cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if(ret){
            LOGERROR("reinit cannot open can device %s", m_port_name.c_str())
            return false;
        }

        std::lock_guard<std::mutex> lock(m_can_mutex);
        m_port_is_available = initCanDevice(m_port_name);

        if(m_port_is_available) {
            if (m_port_scheduler_available) {
                m_port_status->status = PortStatus::Available;
            }
            return true;
        }
        return false;
    }

private:
    bool initCanDevice(std::string port_name)
    {
        // create a socketfd
        if ((m_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            LOGERROR("[CANERROR] Cannot create socket for device %s , error code %d: %s", port_name.c_str(), errno, strerror(errno));
            return false;
        }

        // socket和设备进行绑定
        strcpy(m_ifr.ifr_name, port_name.c_str());
        m_ifr.ifr_ifindex = if_nametoindex(m_ifr.ifr_name);

        if (ioctl(m_sock, SIOCGIFINDEX, &m_ifr) < 0)
        {
            LOGERROR("[CANERROR] Cannot find device %s, error code %d: %s", port_name.c_str(), errno, strerror(errno));
            return false;
        }

        // bind socket
        m_addr.can_family = AF_CAN;
        m_addr.can_ifindex = m_ifr.ifr_ifindex;
        if (bind(m_sock, (sockaddr *)&m_addr, sizeof(m_addr)) < 0)
        {
            LOGERROR("[CANERROR] Cannot bind device %s, error code %d: %s", port_name.c_str(), errno, strerror(errno));
            return false;
        }

        // 设置为非阻塞模式
        int fdflags = fcntl(m_sock, F_GETFL, 0);
        if (fcntl(m_sock, F_SETFL, fdflags | O_NONBLOCK) < 0)
        {
            LOGERROR("[CANERROR] Connot set device %s Non-blocking, error code %d: %s", port_name.c_str(), errno, strerror(errno));
            return false;
        }
        return true;
    }

    /**
     * @brief 写进程
     *
     */
    void writeThread()
    {
        set_cpu_affinity(0);
        std::string port_name = m_port_name; // 防止类被释放后无法访问成员

        LOGINFO("port %s : write thread start!", port_name.c_str());

        int failed_cnt = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        while (transport::ok())
        {
            writeOnce(failed_cnt);
            write_usleep_length = m_port_is_available ? WRITE_USLEEP_LENGTH : STANDBY_USLEEP_LENGTH;
            std::this_thread::sleep_for(std::chrono::microseconds(write_usleep_length));
        }
        LOGINFO("port %s : write thread exit", port_name.c_str());
    }

    /**
     * @brief 读进程
     *
     */
    void readTread()
    {   
        set_cpu_affinity(0);
        std::string port_name = m_port_name; // 防止类被释放后无法访问成员

        LOGINFO("port %s : read thread start!", port_name.c_str());

        int failed_cnt = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        while (transport::ok())
        {
            readOnce(failed_cnt);
            read_usleep_length = m_port_is_available ? READ_USLEEP_LENGTH : STANDBY_USLEEP_LENGTH;
            std::this_thread::sleep_for(std::chrono::microseconds(read_usleep_length));
        }
        LOGINFO("port %s : read thread exit", port_name.c_str());
    }

    /**
     * @brief Buffer转换成can包
     *
     * @param data 输入buffer
     * @param frame 输出can
     * @return int 返回输出的数据部分长度
     */
    int Buffer2Can(BufferWithID *data, canfd_frame *frame)
    {
        int len = data->buffer.size();
        clear(frame);
        if (len > CAN_MTU)
            len = CAN_MTU;

        frame->can_id = unmask(data->id);   // 获取can id
        for (int i = 0; i < len; ++i)
        {
            frame->data[i] = data->buffer[i];
        }
        frame->len = len;
        return len;
    }

    /**
     * @brief can包转成buffer
     *
     * @param data 输出Buffer
     * @param frame 输入can
     */
    void Can2Buffer(canfd_frame *frame, Buffer *data)
    {
        data->clear();
        data->copy(frame->data, frame->len);
    }

    /**
     * @brief 写一次
     *
     * @param failed_cnt
     */
    void writeOnce(int &failed_cnt)
    {
        // 1.从输出队列中取一个包
        BufferWithID buffer_with_id;
        if (!popOneBuffer(buffer_with_id))
            return;
        int required_mtu = Buffer2Can(&buffer_with_id, &m_send_frame);
        // 2.尝试发送
        std::unique_lock can_lock(m_can_mutex);
        int nbytes = send(m_sock, &m_send_frame, sizeof(m_send_frame), MSG_DONTWAIT);
        can_lock.unlock();
        // 3.异常处理
        if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            LOGERROR("Write error! errno code %d : %s.", errno, strerror(errno));
            ++failed_cnt;
            if (failed_cnt > 10)
            {
                m_port_is_available = false;
                if (m_port_scheduler_available)
                {
                    m_port_status->status = PortStatus::Unavailable;
                }
                LOGERROR("port %s 's write thread failed count > 10, port status had been set to unavailable. can线插好了吗?can口是不是插错了?电控代码是不是停了?", m_port_name.c_str());
            }
            if (failed_cnt & 0x01) // 为奇数时
            {
                char cmd[256] = {0};
                sprintf(cmd,
                        "echo \"%s\" | sudo -S ip link set %s down && sudo ip link set %s type can bitrate 1000000 && sudo ip link set %s up",
                        m_passwd.c_str(), m_port_name.c_str(), m_port_name.c_str(), m_port_name.c_str());
                int ret = system(cmd);
                LOGWARN("exec cmd: '%s', return %d", cmd, ret);
            }
        }
        else // 发送成功 // 正常成功将会返回发送包长度
        {
            failed_cnt = 0;
            if (m_port_scheduler_available)
            {
                m_port_status->workload.write.update();
            }
        }
    }

    /**
     * @brief 读一次
     *
     * @param failed_cnt
     */
    void readOnce(int &failed_cnt)
    {
        auto tv_begin = gettimeval();
        static auto tv_begin_2 = gettimeval();
        // (void)clock_begin_2;
        // 1.读一个包
        std::unique_lock can_lock(m_can_mutex);
        int nbytes = recv(m_sock, &(m_read_frame), sizeof(m_read_frame), MSG_DONTWAIT);
        can_lock.unlock();
        // 2.解码
        if (nbytes == CAN_MTU) //接收正常
        {
            // 2.1.正常解码
            failed_cnt = 0;
            Buffer buffer;
            Can2Buffer(&m_read_frame, &buffer);

            // 查找此canport是否有这个包
            ID id = mask(PORT_TYPE::CAN, m_read_frame.can_id, m_group_id, m_port_id);  // 编码can id
            recvOnePackage(id, buffer);
            if (m_port_scheduler_available)
            {
                m_port_status->workload.read.update();
            }
        }
        else // 出现异常，接收失败
        {
            // 2.2.异常处理
            if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) //接收异常，可能U转can寄了
            {
                ++failed_cnt;
                if (failed_cnt > 10)
                {
                    m_port_is_available = false;
                    if (m_port_scheduler_available)
                    {
                        m_port_status->status = PortStatus::Unavailable;
                    }
                    LOGERROR("port %s's read thread failed count > 10, port status had been set to unavailable. can线插好了吗?can口是不是插错了?电控代码是不是停了?", m_port_name.c_str());
                }
                if (failed_cnt & 0x01) // 为奇数时
                {
                    char cmd[256];
                    sprintf(cmd,
                            "echo \"%s\" | sudo -S ip link set %s down && sudo ip link set %s type can bitrate 1000000 && sudo ip link set %s up",
                            m_passwd.c_str(), m_port_name.c_str(), m_port_name.c_str(), m_port_name.c_str());
                    int ret = system(cmd);
                    LOGWARN("exec cmd: '%s', return %d", cmd, ret);
                }
            }
            else if(nbytes > 0) //没有收到完整的包
            {
                LOGWARN("Incomplete data packs, package length %d package id %x", (int)m_read_frame.len, (int)m_read_frame.can_id);
            }
        }
        // 计算了一下是否超时
        if ((gettimeval() - tv_begin).tv_sec > TIMEOUT)
        {
            LOGWARN("port %s recv package time out!", m_port_name.c_str());
        }
    }
};

} // namespace transport

#endif // ENABLE_CAN_PORT

#endif //__WMJ_CAN_PORT_HPP__