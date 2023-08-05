#include <CanPort.hpp>
#include "Utility.hpp"

#ifdef USE_FAKE
#include "fakePort.hpp"
#endif // USE_FAKE

#define LOOP_CONDITION (m_port_is_available)
constexpr double TIMEOUT = 2;

CanPort::CanPort(std::string port_name)
{
    //可以使用can设备的标志位
    this->m_port_is_available = true;
    this->m_port_controller_available = false;
    this->m_port_name = port_name;

    // create a socketfd
#ifndef USE_FAKE
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
#endif // USE_FAKE
    if (m_port_is_available)
    {
        std::cout << port_name << " Open" << std::endl;
        m_readThread = std::thread(&CanPort::readTread, this);
        m_writeThread = std::thread(&CanPort::writeThread, this);
        m_readThread.detach();
        m_writeThread.detach();
    }
}

void CanPort::writeThread()
{
    set_cpu_affinity(0);

    std::cout << "write thread start!" << std::endl;
    
    int failed_cnt = 0;
    usleep(1e6);
    while (LOOP_CONDITION)
    {
        writeOnce(failed_cnt);
        usleep(10);
    }
}

void CanPort::readTread()
{
    set_cpu_affinity(0);

    std::cout << "read thread start!" << std::endl;

    int failed_cnt = 0;
    usleep(2e6);
    while (LOOP_CONDITION)
    {
        readOnce(failed_cnt);
        usleep(10);
    }
}

void CanPort::writeOnce(int &failed_cnt)
{
    // 1.从输出队列中取一个包
    m_write_buffer_mutex.lock();
    if (m_write_buffer.empty())
    {
        m_write_buffer_mutex.unlock();
        return;
    }
    int required_mtu = Buffer2Can(&m_write_buffer.front(), &m_send_frame);
    m_write_buffer.pop();
    m_write_buffer_mutex.unlock();

    // 2.尝试发送
    m_can_mutex.lock();
#ifndef USE_FAKE
    int nbytes = send(m_sock, &m_send_frame, required_mtu, MSG_DONTWAIT);
#else
    int nbytes = fake::send(m_sock, &m_send_frame, required_mtu, MSG_DONTWAIT);
#endif // USE_FAKE
    m_can_mutex.unlock();
    // 3.异常处理
    if (nbytes == CAN_MTU)
    {
        if (m_port_controller_available)
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
            m_port_is_available = false;
            if (m_port_controller_available)
            {
                m_port_status->status = PortStatus::Unavailable;
            }
        }
        if (failed_cnt & 0x01) // 为奇数时
        {
            system("echo \"w\" | sudo -S ip link set can0 down && sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up");
        }
    }
}

void CanPort::readOnce(int &failed_cnt)
{
    auto clock_begin = clock();
    auto clock_begin_2 = clock();
    // 1.读一个包
    m_can_mutex.lock();
#ifndef USE_FAKE
    int nbytes = recv(m_sock, &(m_read_frame), sizeof(m_read_frame), MSG_DONTWAIT);
#else
    int nbytes = fake::recv(m_sock, &(m_read_frame), sizeof(m_read_frame), MSG_DONTWAIT);
    // if(m_port_name == "can1" && (clock() - clock_begin_2) / CLOCKS_PER_SEC > 10) nbytes = -1;
    // if(m_port_name == "can0" && (clock() - clock_begin_2) / CLOCKS_PER_SEC > 20) nbytes = -1;
#endif // USE_FAKE
    m_can_mutex.unlock();
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
        buffer_with_time.first = buffer;
        buffer_with_time.second = tv;
        // 复制buffer到对应包里
        package_it->second->recvBuffer(buffer_with_time);
        if (m_port_controller_available)
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
                m_port_is_available = false;
                if (m_port_controller_available)
                {
                    m_port_status->status = PortStatus::Unavailable;
                }
            }
            if (failed_cnt & 0x01) // 为奇数时
            {
                system("echo \"w\" | sudo -S ip link set can0 down && sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up");
            }
        }
        else //没有收到完整的包
        {
            std::cout << "Incomplete data packs" << std::endl;
        }
    }
    // 计算了一下是否超时
    if ((clock() - clock_begin) / CLOCKS_PER_SEC > TIMEOUT)
    {
        std::cout << "recv package time out! " << std::endl;
    }
}

void CanPort::Can2Buffer(canfd_frame *frame, Buffer *data)
{
    data->clear();
    for (auto c : frame->data)
    {
        data->push_back(c);
    }
}

int CanPort::Buffer2Can(BufferWithID *data, canfd_frame *frame)
{
    int len = data->first.size();
    memset(frame, 0, sizeof(*frame));
    // if (len > CAN_MTU)
    //     len = CAN_MTU;

    frame->can_id = data->second;
    for (int i = 0; i < len; ++i)
    {
        frame->data[i] = data->first[i];
    }
    frame->len = len;
    return len;
}

CanPort::~CanPort()
{
    m_port_is_available = false;
}
