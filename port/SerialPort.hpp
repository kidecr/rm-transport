#ifndef __SERIAL_PORT_HPP__
#define __SERIAL_PORT_HPP__

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
#include <fcntl.h>


#include "impls/Port.hpp"
#include "utils/Buffer.hpp"
#include "utils/Utility.hpp"
#include "PackageManager.hpp"
#include "impls/logger.hpp"
#include "port/CRC.hpp"

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
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
#define JUDGE_FRAME_HEAD              	(5u)
#define JUDGE_FRAME_HEAD_AND_LENGTH   	(9u)

namespace transport
{

struct HeadIndex{
    enum HEAD_INDEX_TYPE {
        HEAD_INDEX_FOUND, 
        HEAD_INDEX_NOT_FOUND, 
        HEAD_INDEX_NEED_MORE_DATA
    };
    int32_t index;
    HEAD_INDEX_TYPE type;
};

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

template<size_t buffer_size = 1024, size_t max_reserve_size = JUDGE_DATA_MAX_SIZE + 1>
class SerialBuffer
{
private:
    uint8_t m_buffer[buffer_size]; // 2倍大小
    size_t m_head;  // 头，数据包括该位置
    size_t m_tail;  // 尾，数据不包括该位置
    std::shared_mutex m_buffer_mutex;
public:
    SerialBuffer(): m_head(0), m_tail(0){}

    size_t headIndex()
    {
        std::shared_lock<std::shared_mutex> lock(m_buffer_mutex);
        return m_head;
    }

    size_t tailIndex()
    {
        std::shared_lock<std::shared_mutex> lock(m_buffer_mutex);
        return m_tail;
    }

    // 向尾部添加了length这么长的数据
    size_t append(size_t length)
    {
        std::lock_guard<std::shared_mutex> lock(m_buffer_mutex);
        if (m_tail + length > buffer_size - max_reserve_size)
        {
            LOGWARN("length %ld exceeds available buffer space %ld. Flushing buffer to accommodate new data.", length, buffer_size);
            m_head = 0;
            m_tail = 0;
        }
        m_tail = m_tail + length;
        if (m_tail > buffer_size - max_reserve_size && m_head != 0)
        {
            memmove(m_buffer, m_buffer + m_head, m_tail - m_head);
            m_tail = m_tail - m_head;
            m_head = 0;
        }
        return m_tail;
    }

    size_t append(const uint8_t* buffer, size_t length)
    {
        std::lock_guard<std::shared_mutex> lock(m_buffer_mutex);
        if (m_tail + length > buffer_size - max_reserve_size)
        {
            LOGWARN("length %ld exceeds available buffer space %ld. Flushing buffer to accommodate new data.", length, buffer_size);
            m_head = 0;
            m_tail = 0;
        }
        memcpy(m_buffer + m_head, buffer, length);
        m_tail = m_tail + length;
        if (m_tail > buffer_size - max_reserve_size && m_head != 0)
        {
            memmove(m_buffer, m_buffer + m_head, m_tail - m_head);
            m_tail = m_tail - m_head;
            m_head = 0;
        }
        return m_tail;
    }

    const uint8_t* head()
    {
        std::shared_lock<std::shared_mutex> lock(m_buffer_mutex);
        return m_buffer + m_head;
    }

    const uint8_t* tail()
    {
        std::lock_guard<std::shared_mutex> lock(m_buffer_mutex);
        if(m_tail > buffer_size - max_reserve_size && m_head != 0){   // 尾部要超了，头部前面还有空间
            memmove(m_buffer, m_buffer + m_head, m_tail - m_head);
            m_tail = m_tail - m_head;
            m_head = 0;
        }
        return m_buffer + m_tail;
    }

    uint8_t operator[] (size_t index){
        std::shared_lock<std::shared_mutex> lock(m_buffer_mutex);
        return m_buffer[m_head + index];
    }

    size_t clear()
    {
        std::lock_guard<std::shared_mutex> lock(m_buffer_mutex);
        m_head = m_tail;
        return 0;
    }

    size_t flush() //refresh()
    {
        std::lock_guard<std::shared_mutex> lock(m_buffer_mutex);
        m_head = 0;
        m_tail = 0;
        return 0;
    }

    size_t length()
    {
        std::shared_lock<std::shared_mutex> lock(m_buffer_mutex);
        return m_tail - m_head;
    }

    bool empty()
    {
        std::shared_lock<std::shared_mutex> lock(m_buffer_mutex);
        return m_head == m_tail;
    }

    /**
     * @brief 在原head基础上改变index个位置
     * 
     * @param index 变化位置
     * @return size_t 变化后的head位置
     */
    size_t setHead(size_t index)
    {
        std::lock_guard<std::shared_mutex> lock(m_buffer_mutex);
        m_head = m_head + index;
        if(m_head < 0) m_head = 0;
        if(m_head > buffer_size - 1) m_head = buffer_size - 1;
        if(m_head > m_tail) m_head = m_tail;
        return m_head;
    }
};

class SerialPort : public Port
{
public:
    using SharedPtr = std::shared_ptr<SerialPort>;

private:
    int m_baud_read;
    int m_pack_seq;
    boost::shared_ptr<boost::asio::io_service>  m_io_service;
    boost::shared_ptr<boost::asio::serial_port> m_serial_port;

    // uint8_t m_read_buffer[RX_BUFFER_MAX_SIZE];
    SerialBuffer<RX_BUFFER_MAX_SIZE> m_read_buffer;
    SerialBuffer<TX_BUFFER_MAX_SIZE> m_write_buffer;
    boost::mutex m_read_buffer_mutex;
    boost::mutex m_write_buffer_mutex;
    int m_write_failed_cnt;
    int m_read_failed_cnt;

    std::jthread m_io_service_thread;

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
    SerialPort(std::string port_name, int baud_read, uint32_t group_id = 0, uint32_t port_id = 0) : Port(port_name, group_id, port_id)
    {
        if(!checkPortExist(port_name))
            throw PORT_EXCEPTION("create port failed! port : " + port_name + " is not exist!");
        //可以使用can设备的标志位
        this->m_port_is_available = false;
        this->m_port_scheduler_available = false;
        this->m_port_name = port_name;

        this->m_quit = false;
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
            m_io_service_thread = std::jthread([this](){
                try{
                    readOnce();
                    writeOnce();
                    boost::asio::io_service::work work(*m_io_service);
                    m_io_service->run();

                }catch(PortException& e) {
                    LOGERROR("SerialPort Thread PortException: %s", e.what());
                    // 异常退出后设置port状态
                    m_port_is_available = false;
                    if (m_port_scheduler_available){
                        m_port_status->status == PortStatus::Unavailable;
                    }
                }catch (boost::system::system_error& e) {
                    LOGERROR("SerialPort Thread  Boost.Asio System Error: %s", e.what());
                    // 异常退出后设置port状态
                    m_port_is_available = false;
                    if (m_port_scheduler_available){
                        m_port_status->status == PortStatus::Unavailable;
                    }
                }catch (...) {
                    LOGERROR("SerialPort Thread Unknown Exception");
                    // 异常退出后设置port状态
                    m_port_is_available = false;
                    if (m_port_scheduler_available){
                        m_port_status->status == PortStatus::Unavailable;
                    }
                }
                
            });
            // m_io_service_thread.detach();
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
        // 顺序重要：先关service,等thread退出后,再关port
        if(m_io_service){
            m_io_service->stop();
        }
        if(m_io_service_thread.joinable())
            m_io_service_thread.join();
        if(m_serial_port){
            m_serial_port->close();
        }
    }

    /**
     * @brief 重置接口，当port不可用时，调用该函数尝试重连
     * 
     * @return true 重连成功
     * @return false 重连失败
     */
    bool reinit()
    {
        if (m_port_scheduler_available)
            if (m_port_status->status == PortStatus::Available)
                return true;
        
        // 1.0 close all device
        m_serial_port->close();
        m_io_service->stop();
        // 1.1 check if port is exist
        if(!checkPortExist(m_port_name))
        {
            // 1.2 if port not exist, find avaliable device
#ifdef ONLY_ONE_SERIAL_PORT_DEVICE
            /**
             * @brief 以下代码用于当设备名变化时，重新搜索可用设备名
             *        但由于当存在多个设备同时工作时，当前架构不支持
             *        过滤掉其他正在工作的设备，因此以下代码不可在多
             *        设备情况下使用
             */
            auto device_list = execCommand("ls -d /dev/* | grep ttyUSB");
            if(device_list.empty())
                return false;
            for(auto device : device_list) {
                if(checkPortExist(device))
                {
                    m_port_name = device;
                    break;
                }
            }
#else
            return false;
#endif // ONLY_ONE_SERIAL_PORT_DEVICE
        }
        
        // 1.3 port is available, reopen serial port
        try{
            m_serial_port->open(m_port_name);
            m_serial_port->set_option(boost::asio::serial_port::baud_rate(m_baud_read));                                           // 特率
            m_serial_port->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));  // 流量控制
            m_serial_port->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));              // 奇偶校验
            m_serial_port->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));         // 停止位
            m_serial_port->set_option(boost::asio::serial_port::character_size(8));
            m_port_is_available = true;
        }
        catch(boost::system::system_error &e)
        {
            m_port_is_available = false;
            boost::system::error_code ec = e.code(); 
            LOGERROR("reinit SerialPort error, error code %d : %s", ec.value(), ec.message().c_str());
            return false;
        }
        // 1.4 restart
        if(m_port_is_available)
        {
            m_read_failed_cnt = 0;
            m_write_failed_cnt = 0;
            if (m_port_scheduler_available)
            {
                m_port_status->status = PortStatus::Available;
            }
            m_io_service_thread = std::jthread([this](){
                try{
                    readOnce();
                    writeOnce();
                    boost::asio::io_service::work work(*m_io_service);
                    m_io_service->run();
                }catch(PortException& e) {
                    LOGERROR("SerialPort Thread PortException: %s", e.what());
                    m_port_is_available = false;
                    if (m_port_scheduler_available){
                        m_port_status->status == PortStatus::Unavailable;
                    }
                }catch (boost::system::system_error& e) {
                    LOGERROR("SerialPort Thread  Boost.Asio System Error: %s", e.what());
                    m_port_is_available = false;
                    if (m_port_scheduler_available){
                        m_port_status->status == PortStatus::Unavailable;
                    }
                }catch (...) {
                    LOGERROR("SerialPort Thread Unknown Exception");
                    m_port_is_available = false;
                    if (m_port_scheduler_available){
                        m_port_status->status == PortStatus::Unavailable;
                    }
                }
            });
            LOGINFO("Serial Port started.");
            return true;
        }
        return false;
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
        Append_CRC8_Check_Sum((uint8_t *)&serial_port_frame, JUDGE_FRAME_HEAD);
        // cmd id
        serial_port_frame.cmd_id = cmd_id;
        // 拷贝数据
        data->buffer.copyTo(serial_port_frame.data, data_length);
        // crc16校验
        Append_CRC16_Check_Sum((uint8_t *)&serial_port_frame, JUDGE_FRAME_HEAD_AND_LENGTH + data_length);
        // 转成Buffer
        frame->resize(JUDGE_FRAME_HEAD_AND_LENGTH + data_length);
        frame->copy((uint8_t *)&serial_port_frame, JUDGE_FRAME_HEAD_AND_LENGTH + data_length);
        return JUDGE_FRAME_HEAD_AND_LENGTH + data_length;
    }

    /**
     * @brief 查找帧头
     * 
     * @param buffer 带检查数据
     * @param length buffer有效长度
     * @return int <0 需要更多数据; 0 没找到; >0 帧头起始位置;
     */
    HeadIndex findFrameHead(uint8_t* buffer, int length)
    {
        int i = 0;
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
                if(i + JUDGE_FRAME_HEAD >= length) { 
                    return {i, HeadIndex::HEAD_INDEX_TYPE::HEAD_INDEX_NEED_MORE_DATA};
                }
                // crc8校验帧头
                if(Verify_CRC8_Check_Sum(buffer + i, JUDGE_FRAME_HEAD))
                {
                    return {i, HeadIndex::HEAD_INDEX_TYPE::HEAD_INDEX_FOUND};   // crc8校验成功，返回帧头所在位置
                }
                else // crc8校验失败
                {
                    ++i;
                    continue;
                }
            }
        }
        return {0, HeadIndex::HEAD_INDEX_TYPE::HEAD_INDEX_NOT_FOUND};
    }
    
    /**
     * @brief 串口包包转成buffer with id
     * 
     * @param frame 输入串口数据
     * @param frame_length 输入数据长度
     * @param data_with_id 输出buffer和id
     * @return int >0 串口帧总长度; <0 需要更多数据; 0 校验失败，数据损坏，包无效
     */
    int Frame2Buffer(const uint8_t *frame, int frame_length, BufferWithID *data_with_id)
    {
        uint16_t data_length = ((uint16_t)frame[2]) << 8 | (uint16_t)frame[1];
        
        if(frame_length < data_length + JUDGE_FRAME_HEAD_AND_LENGTH) 
            return -1 * ((int)data_length); // 数据不足
        
        // 正常读数据
        SerialPortFrame serial_port_frame;
        // 拷贝整个包
        memcpy((uint8_t *)&serial_port_frame, frame, data_length + JUDGE_FRAME_HEAD_AND_LENGTH);
        // crc16校验
        if(Verify_CRC16_Check_Sum((uint8_t *)&serial_port_frame, JUDGE_FRAME_HEAD_AND_LENGTH + data_length))
        {
            // 校验通过，是正常包
            data_with_id->id = mask(PORT_TYPE::SERIAL, serial_port_frame.cmd_id, m_group_id, m_port_id);
            data_with_id->buffer.copy(serial_port_frame.data, data_length);
            return JUDGE_FRAME_HEAD_AND_LENGTH + data_length;
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
    void asyncWriteCallback(const boost::system::error_code &ec, std::size_t bytes_transferred)
    {
        if(m_quit) return;

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
                writeOnce();
                return;
            }
        }
        else{
            m_write_failed_cnt = m_write_failed_cnt > 0 ? m_write_failed_cnt - 1 : 0;
        }

        m_write_buffer.setHead(bytes_transferred);
        // 1.从输出队列中取一个包
        BufferWithID buffer_with_id;
        if (popOneBuffer(buffer_with_id)){
            Buffer frame;
            int len = Buffer2Frame(&buffer_with_id, &frame);
            m_write_buffer.append(frame.data, len);
#ifdef __DEBUG__
            LOGDEBUG("send frame, id is 0x%lx.", buffer_with_id.id);
        }
        else
        {
            static int cnt = 0;
            if((++cnt) % 20 == 0)
                LOGDEBUG("write queue is empty");
#endif // __DEBUG__
        }

        writeOnce();

        if(m_port_scheduler_available){
            m_port_status->workload.write.update();
        }
    }

    /**
     * @brief 异步读取回调函数
     * 
     * @param ec 异常
     * @param bytes_transferred 成功读到的数据量
     * @param tail 读缓冲区末尾的下标
     */
    void asyncReadCallback(const boost::system::error_code &ec, std::size_t bytes_transferred)
    {
        if(m_quit) return;

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
        else {
            m_read_failed_cnt = m_read_failed_cnt > 0 ? m_read_failed_cnt - 1 : 0;
        }
        // 向队列中加入数据
        m_read_buffer.append(bytes_transferred);
        readOnce();
        BufferWithID data_with_id;  // 用于解包

        boost::mutex::scoped_lock lock(m_read_buffer_mutex);
        while(!m_quit)
        {
            HeadIndex frame_head_index = findFrameHead((uint8_t*)m_read_buffer.head(), m_read_buffer.length());

            if (frame_head_index.type == HeadIndex::HEAD_INDEX_TYPE::HEAD_INDEX_NOT_FOUND) // 没找到帧头
            { 
                //  继续读，清空之前的缓冲区
                m_read_buffer.clear();
                LOGDEBUG("frame head not found!");
                break;
            }
            else if(frame_head_index.type == HeadIndex::HEAD_INDEX_TYPE::HEAD_INDEX_NEED_MORE_DATA) // 数据不够，接着读
            {
                m_read_buffer.setHead(frame_head_index.index);
                LOGDEBUG("found frame head, more data is needed!");
                break;
            }
            else // 找到帧头了
            {
                LOGDEBUG("found frame head successfully!");
                m_read_buffer.setHead(frame_head_index.index);
                // 从帧头位置开始解析包
                int frame_length = Frame2Buffer(m_read_buffer.head(), m_read_buffer.length(), &data_with_id);
                // 没找到帧尾，继续读
                if(frame_length < 0) // 需要更多数据
                {
                    LOGDEBUG("cannot found frame EOF, more data is needed!");
                    break;
                }
                else if(frame_length == 0) // 数据顺坏，此包无效
                {
                    LOGDEBUG("frame broken, find next frame!");
                    // 给head增加1,接着找下一个包头
                    m_read_buffer.setHead(1); 
                    continue;
                }
                else
                {
                    LOGDEBUG("read frame successfully!");
                    // 注册新的读取任务
                    m_read_buffer.setHead(frame_head_index.index + frame_length);
                    // 尝试向Serial中添加这个包
                    recvOnePackage(data_with_id.id, data_with_id.buffer);
                    
                    // break;
                }
                // break;
            }
        }
        if (m_port_scheduler_available)
        {
            m_port_status->workload.read.update();
        }
    }

    /**
     * @brief 发送一次数据，并注册回调函数
     * 
     * @param length 目标发送数据量
     */
    void writeOnce()
    {
        if(m_quit) return;

        // 1.开始异步发送
        boost::mutex::scoped_lock serial_lock(m_serial_mutex);
        m_serial_port->async_write_some(boost::asio::buffer(m_write_buffer.head(), m_write_buffer.length()),
                                    boost::bind(&transport::SerialPort::asyncWriteCallback, this, boost::placeholders::_1, boost::placeholders::_2));
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
    void readOnce()
    {
        if(m_quit)
            return;
        // 1.读一个包
        boost::mutex::scoped_lock serial_lock(m_serial_mutex);
        m_serial_port->async_read_some(boost::asio::buffer((void*)m_read_buffer.tail(), (std::size_t)JUDGE_DATA_MAX_SIZE),
                                        boost::bind(&transport::SerialPort::asyncReadCallback, this, boost::placeholders::_1, boost::placeholders::_2));
        // 2.更新一下负载
        if (m_port_scheduler_available)
        {
            m_port_status->workload.read.update();
        }
    }

    bool checkPortExist(std::string port_name)
    {
        // 1.1 check if port is exist
        if(access(port_name.c_str(), F_OK) < 0)
        {
            LOGERROR("找不到串口 %s, access error code: %d", port_name.c_str(), errno);
            LOGINFO("当前存在的串口:");
            int ret = std::system("ls -l /dev/ | grep ttyUSB");
            if(ret) LOGWARN("执行命令 'ls -l /dev/ | grep ttyUSB' 报错：%d", ret);
            //throw PORT_EXCEPTION("create port failed! port : " + port_name + " is not exist!");
            return false;
        }
        // 1.2 check if port is readable and writeable
        if(access(port_name.c_str(), R_OK | W_OK) < 0)
        {
            std::string cmd = "echo 'a' | sudo -S chmod 777 " + port_name;
            LOGERROR("串口 %s 无读写权限, access error code: %d, 将执行命令 %s", port_name.c_str(), errno, cmd.c_str())
            int ret = std::system(cmd.c_str());
            if(ret)
            {
                LOGWARN("执行命令 '%s' 报错：%d", cmd.c_str(), ret);
                // throw PORT_EXCEPTION("add read & write permissions failed! port name: " + port_name);
                return false;
            }
            if(access(port_name.c_str(), R_OK | W_OK) < 0){
                LOGERROR("串口 %s 无法通过chmod赋予读写权限, access error code: %d", port_name.c_str(), errno)
                return false;
            }
        }
        return true;
    }

};

} // namespace transport

#endif //__SERIAL_PORT_HPP__