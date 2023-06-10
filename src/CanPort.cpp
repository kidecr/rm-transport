#include <CanPort.hpp>

#define LOOP_CONDITION (canUseThisPort)
constexpr double TIMEOUT = 2;

CanPort::CanPort(std::string port_name)
{
    //可以使用can设备的标志位
    this->canUseThisPort = true;

    // create a socketfd
    if ((m_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        std::cout << "[CANERROR] Cannot create socket for device " << port_name << std::endl;
        std::cerr << "error code " << errno << " : " << strerror(errno) << std::endl;
        canUseThisPort = false;
    }

    // socket和设备进行绑定
    strcpy(m_ifr.ifr_name, port_name.c_str());
    m_ifr.ifr_ifindex = if_nametoindex(m_ifr.ifr_name);

    if (ioctl(m_sock, SIOCGIFINDEX, &m_ifr) < 0)
    {
        std::cout << "[CANERROR] Cannot find device " << port_name << std::endl;
        std::cerr << "error code " << errno << " : " << strerror(errno) << std::endl;
        canUseThisPort = false;
    }

    // bind socket
    m_addr.can_family = AF_CAN;
    m_addr.can_ifindex = m_ifr.ifr_ifindex;
    if (bind(m_sock, (sockaddr *)&m_addr, sizeof(m_addr)) < 0)
    {
        std::cout << "[CANERROR] Cannot bind device " << port_name << std::endl;
        std::cerr << "error code " << errno << " : " << strerror(errno) << std::endl;
        canUseThisPort = false;
    }

    //
    int fdflags = fcntl(m_sock, F_GETFL, 0);
    if (fcntl(m_sock, F_SETFL, fdflags | O_NONBLOCK) < 0)
    {
        std::cout << "[CANERROR] Connot set device " << port_name << "Non-blocking" << std::endl;
        std::cerr << "error code " << errno << " : " << strerror(errno) << std::endl;
        canUseThisPort = false;
    }
    if (canUseThisPort)
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
    std::cout << "write thread start!" << std::endl;

    int required_mtu = CAN_MTU;
    int failed_cnt = 0;

    while (LOOP_CONDITION)
    {
        m_write_buffer_mutex.lock();
        if (m_write_buffer.empty())
        {
            m_write_buffer_mutex.unlock();
            return;
        }

        required_mtu = Buffer2Can(&m_write_buffer.front(), &m_send_frame);
        m_write_buffer.pop();
        m_write_buffer_mutex.unlock();

        m_can_mutex.lock();
        if (send(m_sock, &m_send_frame, required_mtu, MSG_DONTWAIT) != required_mtu)
        {
            m_can_mutex.unlock();
            std::cout << "Write error! errno code " << errno << " : " << strerror(errno) << std::endl;
            ++failed_cnt;
            if (failed_cnt > 10)
            {
                throw CanPortException(ERROR_PLACE + " send can frame failed! error code " + std::to_string(errno));
                canUseThisPort = false;
                //! TODO
                // 不知道这么写能不能让子进程崩掉从而重启systemd脚本
                // exit(-1);
            }
        }
        else
        {
            m_can_mutex.unlock();
            failed_cnt = 0;
        }

        usleep(200);
    }
}

void CanPort::readTread()
{
    std::cout << "read thread start!" << std::endl;

    clock_t clock_begin = clock();
    int failed_cnt = 0;

    while (LOOP_CONDITION)
    {
        m_can_mutex.lock();

        int nbytes = recv(m_sock, &(m_read_frame), sizeof(m_read_frame), MSG_DONTWAIT);
        m_can_mutex.unlock();

        if (nbytes == CAN_MTU) //发送正常
        {
            failed_cnt = 0;
            Buffer buffer;
            Can2Buffer(&m_read_frame, &buffer);

            // auto package = m_id_map[m_read_frame.can_id];
            auto package_ptr = m_id_map.find(m_read_frame.can_id);
            if (package_ptr == m_id_map.end())
                continue; //没有这个包

            BufferWithTime buffer_with_time;
            timeval tv;

            gettimeofday(&tv, NULL);
            buffer_with_time.first = buffer;
            buffer_with_time.second = tv;

            package_ptr->second->recvBuffer(buffer_with_time);
        }
        else // 出现异常，发送失败
        {
            if (nbytes < 0) //接收异常，可能U转can寄了
            {
                ++failed_cnt;
                std::cout << "Read error! errno code " << errno << " : " << strerror(errno) << std::endl;
                if (failed_cnt > 50)
                {
                    throw CanPortException(ERROR_PLACE + " read can frame failed! error code " + std::to_string(errno));
                    canUseThisPort = false;
                    //! TODO
                    // 不知道这么写能不能让子进程崩掉从而重启systemd脚本
                    // exit(-1);
                }
            }
            else //没有收到完整的包
            {
                std::cout << "Incomplete data packs" << std::endl;
            }
        }

        if ((clock() - clock_begin) / CLOCKS_PER_SEC > TIMEOUT)
        {
            std::cout << "Time out! " << std::endl;
        }

        usleep(200);
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
    if (len < 2)
        return 0;

    frame->can_id = data->second;
    for (int i = 0; i < len; ++i)
    {
        frame->data[i] = data->first[i];
    }
    frame->len = len;
    return CAN_MTU;
}

void CanPort::recvBuffer(Buffer buffer, int id)
{
    BufferWithID buffer_with_id;
    buffer_with_id.first = buffer;
    buffer_with_id.second = id;

    m_write_buffer_mutex.lock();
    m_write_buffer.push(buffer_with_id);
    m_write_buffer_mutex.unlock();
}

int CanPort::registerPackage(std::shared_ptr<BasePackage> package)
{
    if (package == nullptr)
    {
        throw CanPortException(ERROR_PLACE + ": package ptr is null");
        return -1;
    }
    if (package->m_can_id == 0)
    {
        throw CanPortException(ERROR_PLACE + ": can id is null");
        return -2;
    }
    int can_id = package->m_can_id;
    m_id_map[can_id] = package;

    package->sendBuffer = std::bind(&CanPort::recvBuffer, this, std::placeholders::_1, std::placeholders::_2);
    return 0;
}

CanPort::~CanPort()
{
    canUseThisPort = false;
}
