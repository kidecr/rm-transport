#ifndef __SERIAL_PORT_HPP__
#define __SERIAL_PORT_HPP__

#ifdef __USE_SERIAL_PORT__

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

#include <sys/time.h>
#include <unistd.h>


#include "impls/Port.hpp"
#include "utils/Utility.hpp"
// #include "impls/BasePackage.hpp"
#include "PackageManager.hpp"
#include "impls/logger.hpp"
#include "port/CRC.hpp"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>

#ifdef __USE_FAKE__
#include "fakePort.hpp"
#endif // __USE_FAKE__


#define ROBOT_DATA_CMD_ID_MIN       	(0x0200)
#define ROBOT_DATA_CMD_ID_MAX       	(0x02FF)
#define JUDGE_DATA_MAX_SIZE        	    (128)
#define RX_BUFFER_MAX_SIZE              (1024)
#define TX_BUFFER_MAX_SIZE              (1024)
#define INTERACTIVE_DATA_MAX_SIZE   	(113)
#define JUDGE_FRAME_HEADER_LENGTH   	(5u)

namespace transport
{

#pragma pack(1)

struct SerialPortFrame
{
    struct SerialPortFrameHead
    {
        uint8_t SOF;
        uint16_t data_length;
        uint8_t seq;
        uint8_t CRC8;
    } frame_header;
    uint16_t cmd_id;
    uint8_t data[JUDGE_DATA_MAX_SIZE + 2]; // 同时包含尾部crc校验
};

#pragma pack()

class SerialPort : public Port
{
public:
    using SharedPtr = std::shared_ptr<SerialPort>;

private:
    int m_baud_read;
    int m_pack_seq;
    boost::shared_ptr<boost::asio::io_service>  m_io_service;
    boost::shared_ptr<boost::asio::serial_port> m_serial_port;

    uint8_t m_read_buffer[RX_BUFFER_MAX_SIZE];
    uint8_t m_write_buffer[TX_BUFFER_MAX_SIZE];
    boost::mutex m_read_buffer_mutex;
    boost::mutex m_write_buffer_mutex;
    int m_write_failed_cnt;
    int m_read_failed_cnt;

    std::thread m_io_service_thread;

    boost::mutex m_serial_mutex;

    bool m_quit;

    int write_usleep_length;
    int read_usleep_length;
    IENUM WRITE_USLEEP_LENGTH = 0;     // 写线程正常usleep时间
    IENUM READ_USLEEP_LENGTH = 10;     // 读线程正常usleep时间
    IENUM STANDBY_USLEEP_LENGTH = 1e6; // 进程崩溃后待机时间

public:

    /**
     * @brief 创建串口
     * 
     * @param port_name 串口名
     * @param baud_read 波特率
     */
    SerialPort(std::string port_name, int baud_read) : Port(port_name)
    {
        checkPortExist(port_name);
        //可以使用can设备的标志位
        this->m_port_is_available = false;
        this->m_port_scheduler_available = false;
        this->m_port_name = port_name;

        this->m_quit = true;
        this->m_baud_read = baud_read;
        this->m_pack_seq = 0;

        try{
            LOGINFO("set serial param.");
            m_io_service = boost::make_shared<boost::asio::io_service>();
            m_serial_port  = boost::make_shared<boost::asio::serial_port>(*m_io_service, port_name);
        
            m_serial_port->set_option(boost::asio::serial_port::baud_rate(m_baud_read));                                           // 特率
            m_serial_port->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));  // 流量控制
            m_serial_port->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));              // 奇偶校验
            m_serial_port->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));         // 停止位
            m_serial_port->set_option(boost::asio::serial_port::character_size(8));                                           // 数据位
            m_port_is_available = true;
        }
        catch(boost::system::system_error &e)
        {
            boost::system::error_code ec = e.code(); 
            LOGERROR("init SerialPort error, error code %d : %s", ec.value(), ec.message().c_str());
        }

        if(m_port_is_available)
        {
            m_io_service_thread = std::thread([&](){
                readOnce(0);
                writeOnce(0);
                boost::asio::io_service::work work(*m_io_service);
                m_io_service->run();
            });
            m_io_service_thread.detach();
            LOGINFO("Serial Port started.");
        }
    }

    /**
     * @brief 析构函数，停止循环，port置为不可访问
     *
     */
    ~SerialPort()
    {
        m_port_is_available = false;
        m_quit = true;
        transport::shutdown();
        if(m_serial_port){
            m_serial_port->close();
        }
        if(m_io_service){
            m_io_service->stop();
        }
    }

private:
    /**
     * @brief Buffer转换成串口包
     *
     * @param data 输入buffer
     * @param frame 输出frame
     */
    int Buffer2Frame(BufferWithID *data, Buffer *frame)
    {
        uint16_t data_length = data->buffer.size();
        uint16_t cmd_id = unmask(data->id);

        SerialPortFrame serial_port_frame;
        // 填充帧头
        serial_port_frame.frame_header.SOF = 0xA5;
        serial_port_frame.frame_header.data_length = data_length;
        serial_port_frame.frame_header.seq = m_pack_seq++;

        // crc8 校验码
        Append_CRC8_Check_Sum((uint8_t *)&serial_port_frame, JUDGE_FRAME_HEADER_LENGTH);
        // cmd id
        serial_port_frame.cmd_id = cmd_id;
        // 拷贝数据
        data->buffer.copyTo(serial_port_frame.data, data_length);
        // crc16校验
        Append_CRC16_Check_Sum((uint8_t *)&serial_port_frame, 9 + data_length);
        // 转成Buffer
        frame->resize(9 + data_length);
        frame->copy((uint8_t *)&serial_port_frame, 9 + data_length);
        return 9 + data_length;
    }

    /**
     * @brief 查找帧头
     * 
     * @param buffer 带检查数据
     * @param offset 从offset开始找
     * @param length buffer有效长度
     * @return int <0 需要更多数据; 0 没找到; >0 帧头起始位置;
     */
    int findFrameHead(uint8_t* buffer, int offset, int length)
    {
        int i = offset;
        while(i < length)
        {
            if(buffer[i] != 0xA5) // 不是帧头
            {
                ++i;
                continue;
            }
            else 
            {
                // 帧头超缓冲区了，可能不完整，还需要接着读
                if(i + JUDGE_FRAME_HEADER_LENGTH >= length) { 
                    return -1 * i;
                }
                // crc8校验帧头
                if(Verify_CRC8_Check_Sum(buffer + i, JUDGE_FRAME_HEADER_LENGTH))
                {
                    return i;   // crc8校验成功，返回帧头所在位置
                }
                else // crc8校验失败
                {
                    ++i;
                    continue;
                }
            }
        }
        return 0;
    }
    
    /**
     * @brief 串口包包转成buffer with id
     * 
     * @param frame 输入串口数据
     * @param frame_length 输入数据长度
     * @param data_with_id 输出buffer和id
     * @return int >0 串口帧总长度; <0 需要更多数据; 0 校验失败，数据损坏，包无效
     */
    int Frame2Buffer(uint8_t *frame, int frame_length, BufferWithID *data_with_id)
    {
        uint16_t data_length = ((uint16_t)frame[2]) << 8 | (uint16_t)frame[1];
        
        if(frame_length < data_length + 9) 
            return -1 * ((int)data_length); // 数据不足
        
        // 正常读数据
        SerialPortFrame serial_port_frame;
        // 拷贝整个包
        memcpy((uint8_t *)&serial_port_frame, frame, data_length + 9);
        // crc16校验
        if(Verify_CRC16_Check_Sum((uint8_t *)&serial_port_frame, 9 + data_length))
        {
            // 校验通过，是正常包
            data_with_id->id = mask((SERIAL_ID)serial_port_frame.cmd_id);
            data_with_id->buffer.copy(serial_port_frame.data, data_length);
            return 9 + data_length;
        }
        else // crc16校验失败
        {
            return 0;
        }
    }

    /**
     * @brief 异步发送回调函数
     * 
     * @param ec 异常
     * @param bytes_transferred 成功发送出的数据量
     * @param length 目标发送数据量
     */
    void asyncWriteCallback(const boost::system::error_code &ec, std::size_t bytes_transferred, int length)
    {
        if(ec) // 出现异常
        {
            LOGERROR("init SerialPort error, error code %d : %s", ec.value(), ec.message().c_str());
            ++m_write_failed_cnt;
            if (m_write_failed_cnt > 10)
            {
                m_port_is_available = false;
                if (m_port_scheduler_available)
                {
                    m_port_status->status = PortStatus::Unavailable;
                }
                LOGERROR("port %s 's write thread failed count > 10, port status had been set to unavailable. can线插好了吗?can口是不是插错了?电控代码是不是停了?", m_port_name.c_str());
            }
        }
        if(bytes_transferred < length) // 没发完
        {
            boost::mutex::scoped_lock serial_lock(m_write_buffer_mutex);
            // 从m_write_buffer拿走bytes数据;
            memmove(m_write_buffer, m_write_buffer + bytes_transferred, length - bytes_transferred);
            // 重新调用，并少发bytes数据,把这个包发完
            writeOnce(length - bytes_transferred);
        }
        // 发完了，发下一个包
        // 1.从输出队列中取一个包
        BufferWithID buffer_with_id;
        if (!popOneBuffer(buffer_with_id)){
            writeOnce(0);
        }
        else{
            Buffer frame;
            int len = Buffer2Frame(&buffer_with_id, &frame);
            frame.copyTo(m_write_buffer, len);
            writeOnce(len);
        }
    }

    /**
     * @brief 异步读取回调函数
     * 
     * @param ec 异常
     * @param bytes_transferred 成功读到的数据量
     * @param tail 读缓冲区末尾的下标
     */
    void asyncReadCallback(const boost::system::error_code &ec, std::size_t bytes_transferred, int tail)
    {
        if(ec) // 出现异常
        {
            LOGERROR("read SerialPort error, error code %d : %s", ec.value(), ec.message().c_str());
            // 2.2.异常处理
            ++m_read_failed_cnt;
            if (m_read_failed_cnt > 10)
            {
                m_port_is_available = false;
                if (m_port_scheduler_available)
                {
                    m_port_status->status = PortStatus::Unavailable;
                }
                LOGERROR("port %s's read thread failed count > 10, port status had been set to unavailable. can线插好了吗?can口是不是插错了?电控代码是不是停了?", m_port_name.c_str());
            }
        }
        
        
        BufferWithID data_with_id;  // 用于解包
        int offset = 0;     // 用于重新开始找帧头

        boost::mutex::scoped_lock lock(m_read_buffer_mutex);
        while(transport::ok())
        {
            int frame_head_index = findFrameHead(m_read_buffer, offset, tail + bytes_transferred);

            if (frame_head_index == 0) // 没找到帧头
            { 
                //  继续读，如果超过缓冲区了，清空缓冲区
                readOnce(0);
                break;
            }
            else if(frame_head_index < 0) // 数据不够，接着读
            {
                if(-1 * frame_head_index > RX_BUFFER_MAX_SIZE / 2)
                {
                    frame_head_index *= -1;
                    memmove(m_read_buffer, m_read_buffer + frame_head_index, tail + bytes_transferred - frame_head_index);
                    readOnce(tail + bytes_transferred - frame_head_index);
                    break;
                }
                else{
                    readOnce(tail + bytes_transferred);
                    break;
                }
            }
            else // 找到帧头了
            {
                // 从帧头位置开始解析包
                int frame_length = Frame2Buffer(m_read_buffer + frame_head_index, 
                                                tail + bytes_transferred - frame_head_index, &data_with_id);
                if(frame_length < 0) // 需要更多数据
                {
                    if(frame_head_index + (-1 * frame_length) + 9 > RX_BUFFER_MAX_SIZE) // 缓冲区不够了
                    {
                        // 清空无用缓冲区，接着读
                        memmove(m_read_buffer, m_read_buffer + frame_head_index, tail + bytes_transferred - frame_head_index);
                        readOnce(tail + bytes_transferred - frame_head_index);
                        break;
                    }
                    else
                    {
                        readOnce(tail + bytes_transferred);
                        break;
                    }
                }
                else if(frame_length == 0) // 数据顺坏，此包无效
                {
                    // 接着找下一个包头
                    offset = frame_head_index + 1;
                    continue;
                }
                else
                {
                    // 清空前面的缓冲区
                    memmove(m_read_buffer, m_read_buffer + frame_head_index + frame_length, 
                            tail + bytes_transferred - frame_head_index - frame_length);
                    readOnce(tail + bytes_transferred - frame_head_index - frame_length);
                    // 将包发出去
                    // 查找此SerialPort是否有这个包
                    auto package_it = m_id_map.find(data_with_id.id);
                    if (package_it == m_id_map.end())
                        break;

                    BufferWithTime buffer_with_time;

                    buffer_with_time.buffer = data_with_id.buffer;
                    buffer_with_time.tv = gettimeval();
                    // 复制buffer到对应包里
                    package_it->second->recvBuffer(buffer_with_time);
                    
            #ifdef __DEBUG__
                    if (package_it->second->m_debug_flag & DEBUG_PRINT_ID_IF_RECEIVED)
                    {
                        LOGDEBUG("[Debug Print]: port %s received package id 0x%x", m_port_name.c_str(), unmask(data_with_id.id));
                    }
            #endif // __DEBUG__
                    break;
                }
            }
        }
    }

    /**
     * @brief 发送一次数据，并注册回调函数
     * 
     * @param length 目标发送数据量
     */
    void writeOnce(int length)
    {
        if(m_quit)
            return;
        // 1.开始异步发送
        boost::mutex::scoped_lock serial_lock(m_serial_mutex);
        m_serial_port->async_write_some(boost::asio::buffer(m_write_buffer, length),
                                    boost::bind(&transport::SerialPort::asyncWriteCallback, this, boost::placeholders::_1, boost::placeholders::_2, length));
        // 2.更新一下负载
        if (m_port_scheduler_available)
        {
            m_port_status->workload.write.update();
        }
    }

    /**
     * @brief 读一次
     * 
     * @param head 从缓冲区的head位置开始写入
     */
    void readOnce(int head)
    {
        if(m_quit)
            return;
        int read_length = JUDGE_DATA_MAX_SIZE + head > RX_BUFFER_MAX_SIZE ? 
                            RX_BUFFER_MAX_SIZE - (JUDGE_DATA_MAX_SIZE + head) : JUDGE_DATA_MAX_SIZE;
        // 1.读一个包
        boost::mutex::scoped_lock serial_lock(m_serial_mutex);
        m_serial_port->async_read_some(boost::asio::buffer(m_read_buffer + head, read_length),
                                        boost::bind(&transport::SerialPort::asyncReadCallback, this, boost::placeholders::_1, boost::placeholders::_2, head));
        // 2.更新一下负载
        if (m_port_scheduler_available)
        {
            m_port_status->workload.read.update();
        }
    }

    void checkPortExist(std::string port_name)
    {
        int fd   = open(port_name.c_str(), O_EXCL, NULL);
        bool ret = false;
        if(fd < 0)
        {
            LOGWARN("找不到串口 %s", port_name.c_str());
            ret = true;
        }
        close(fd);
        if(ret)
        {
            LOGERROR("[SERIALERROR] create port failed! port name: %s! tty2USB插好了吗?", port_name.c_str());
            LOGINFO("当前存在的串口:");
            int ret = std::system("ls -l /dev/ | grep ttyUSB");
            if(ret) LOGWARN("执行命令 'ls -l /dev/ | grep ttyUSB' 报错：%d", ret);
            throw PORT_EXCEPTION("create port failed! port name: " + port_name);
        }
        LOGINFO("读串口: %s", port_name.c_str());
    }
};

} // namespace transport

#endif // __USE_SERIAL_PORT__
#endif //__SERIAL_PORT_HPP__