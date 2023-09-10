#include "port/CanPort.hpp"
#include "Utility.hpp"

#ifdef __USE_FAKE__
#include "fakePort.hpp"
#endif // __USE_FAKE__

constexpr double TIMEOUT = 2;

CanPort::CanPort(std::string port_name) : Port(port_name)
{
    //可以使用can设备的标志位
    this->m_port_is_available = true;
    this->m_port_scheduler_available = false;
    this->loop_condition = false;
    this->m_port_name = port_name;

    // create a socketfd
#ifndef __USE_FAKE__
    if ((m_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        std::cout << "[CANERROR] Cannot create socket for device " << port_name << std::endl;
        std::cerr << "error code " << errno << " : " << strerror(errno) << std::endl;
        m_port_is_available = false;
    }

    // socket和设备进行绑定
    strcpy(m_ifr.ifr_name, port_name.c_str());
    m_ifr.ifr_ifindex = if_nametoindex(m_ifr.ifr_name);

    if (ioctl(m_sock, SIOCGIFINDEX, &m_ifr) < 0)
    {
        std::cout << "[CANERROR] Cannot find device " << port_name << std::endl;
        std::cerr << "error code " << errno << " : " << strerror(errno) << std::endl;
        m_port_is_available = false;
    }

    // bind socket
    m_addr.can_family = AF_CAN;
    m_addr.can_ifindex = m_ifr.ifr_ifindex;
    if (bind(m_sock, (sockaddr *)&m_addr, sizeof(m_addr)) < 0)
    {
        std::cout << "[CANERROR] Cannot bind device " << port_name << std::endl;
        std::cerr << "error code " << errno << " : " << strerror(errno) << std::endl;
        m_port_is_available = false;
    }

    //
    int fdflags = fcntl(m_sock, F_GETFL, 0);
    if (fcntl(m_sock, F_SETFL, fdflags | O_NONBLOCK) < 0)
    {
        std::cout << "[CANERROR] Connot set device " << port_name << "Non-blocking" << std::endl;
        std::cerr << "error code " << errno << " : " << strerror(errno) << std::endl;
        m_port_is_available = false;
    }
#endif // __USE_FAKE__
    if (m_port_is_available)
    {
        std::cout << port_name << " Open" << std::endl;
        loop_condition = true;
        m_readThread = std::thread(&CanPort::readTread, this);
        m_writeThread = std::thread(&CanPort::writeThread, this);
        m_readThread.detach();
        m_writeThread.detach();
    }
    else
    {
        throw PORT_EXCEPTION("create port failed! port name: " + port_name);
    }
}

void CanPort::writeThread()
{
    set_cpu_affinity(0);

    std::cout << "write thread start!" << std::endl;
    
    int failed_cnt = 0;
    usleep(1e6);
    while (loop_condition)
    {
        writeOnce(failed_cnt);
        usleep(write_usleep_length);
        write_usleep_length = m_port_is_available ? WRITE_USLEEP_LENGTH : STANDBY_USLEEP_LENGTH;
    }
}

void CanPort::readTread()
{
    set_cpu_affinity(0);

    std::cout << "read thread start!" << std::endl;

    int failed_cnt = 0;
    usleep(2e6);
    while (loop_condition)
    {
        readOnce(failed_cnt);
        usleep(read_usleep_length);
        read_usleep_length = m_port_is_available ? READ_USLEEP_LENGTH : STANDBY_USLEEP_LENGTH;
    }
}

void CanPort::writeOnce(int &failed_cnt)
{
    // 1.从输出队列中取一个包
    BufferWithID buffer_with_id;
    if(!popOneBuffer(buffer_with_id)) return;
    int required_mtu = Buffer2Can(&buffer_with_id, &m_send_frame);
    // 2.尝试发送
    std::unique_lock can_lock(m_can_mutex);
#ifndef __USE_FAKE__
    int nbytes = send(m_sock, &m_send_frame, required_mtu, MSG_DONTWAIT);
#else
    int nbytes = fake::send(m_sock, &m_send_frame, required_mtu, MSG_DONTWAIT);
#endif // __USE_FAKE__
    can_lock.unlock();
    // 3.异常处理
    if (nbytes == CAN_MTU)
    {
        if (m_port_scheduler_available)
        {
            m_port_status->workload.write.update();
        }
        failed_cnt = 0;
    }
    else
    {
        std::cout << "Write error! errno code " << errno << " : " << strerror(errno) << std::endl;
        ++failed_cnt;
        if (failed_cnt > 10)
        {
            std::cout << "port " << m_port_name << "'s write thread failed count > 10, port status will be set to unavailable" << std::endl;
            m_port_is_available = false;
            if (m_port_scheduler_available)
            {
                m_port_status->status = PortStatus::Unavailable;
            }
        }
        if (failed_cnt & 0x01) // 为奇数时
        {
            char cmd[256];
            sprintf(cmd, 
                "echo \"w\" | sudo -S ip link set %s down && sudo ip link set %s type can bitrate 1000000 && sudo ip link set %s up",
                m_port_name.c_str(), m_port_name.c_str(), m_port_name.c_str());
            int ret = system(cmd);
            (void)ret;
        }
    }
}

void CanPort::readOnce(int &failed_cnt)
{
    auto tv_begin = gettimeval();
    static auto tv_begin_2 = gettimeval();
    // (void)clock_begin_2;
    // 1.读一个包
    std::unique_lock can_lock(m_can_mutex);
#ifndef __USE_FAKE__
    int nbytes = recv(m_sock, &(m_read_frame), sizeof(m_read_frame), MSG_DONTWAIT);
#else
    int nbytes = fake::recv(m_sock, &(m_read_frame), sizeof(m_read_frame), MSG_DONTWAIT);
    if(m_port_name == "can1" && (gettimeval() - tv_begin_2).tv_sec > 10) nbytes = -1;
    if(m_port_name == "can0" && (gettimeval() - tv_begin_2).tv_sec > 20) nbytes = -1;
#endif // __USE_FAKE__
    can_lock.unlock();
    // 2.解码
    if (nbytes == CAN_MTU) //发送正常
    {
        // 2.1.正常解码
        failed_cnt = 0;
        Buffer buffer;
        Can2Buffer(&m_read_frame, &buffer);

        // 查找此canport是否有这个包
        auto package_it = m_id_map.find(m_read_frame.can_id);
        if (package_it == m_id_map.end())
            return;

        BufferWithTime buffer_with_time;
        timeval tv;

        gettimeofday(&tv, NULL);
        buffer_with_time.buffer = buffer;
        buffer_with_time.tv = tv;
        // 复制buffer到对应包里
        package_it->second->recvBuffer(buffer_with_time);
        if (m_port_scheduler_available)
        {
            m_port_status->workload.read.update();
        }
    }
    else // 出现异常，发送失败
    {
        // 2.2.异常处理
        if (nbytes < 0) //接收异常，可能U转can寄了
        {
            ++failed_cnt;
            std::cout << "Read error! errno code " << errno << " : " << strerror(errno) << std::endl;
            if (failed_cnt > 10)
            {
                std::cout << "port " << m_port_name << "'s read thread failed count > 10, port status will be set to unavailable" << std::endl;
                m_port_is_available = false;
                if (m_port_scheduler_available)
                {
                    m_port_status->status = PortStatus::Unavailable;
                }
            }
            if (failed_cnt & 0x01) // 为奇数时
            {
                char cmd[256];
                sprintf(cmd, 
                    "echo \"w\" | sudo -S ip link set %s down && sudo ip link set %s type can bitrate 1000000 && sudo ip link set %s up",
                    m_port_name.c_str(), m_port_name.c_str(), m_port_name.c_str());
                int ret = system(cmd);
                (void)ret;
            }
        }
        else //没有收到完整的包
        {
            std::cout << "Incomplete data packs, package length " << m_read_frame.len << " package id " << m_read_frame.can_id << std::endl;
        }
    }
    // 计算了一下是否超时
    if ((gettimeval() - tv_begin).tv_sec > TIMEOUT)
    {
        std::cout << "recv package time out! " << std::endl;
    }
}

void CanPort::Can2Buffer(canfd_frame *frame, Buffer *data)
{
    data->clear();
    data->copy(frame->data, frame->len);
}

int CanPort::Buffer2Can(BufferWithID *data, canfd_frame *frame)
{
    int len = data->buffer.size();
    clear(frame);
    // if (len > CAN_MTU)
    //     len = CAN_MTU;

    frame->can_id = data->id;
    for (int i = 0; i < len; ++i)
    {
        frame->data[i] = data->buffer[i];
    }
    frame->len = len;
    return len;
}

CanPort::~CanPort()
{
    m_port_is_available = false;
    loop_condition = false;
}
