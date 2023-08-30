#ifndef __WMJ_PACKAGE_HPP__
#define __WMJ_PACKAGE_HPP__

#include <iostream>
#include <queue>
#include <mutex>
#include <thread>
#include <functional>
#include <cmath>
#include <sstream>

#include "Utility.hpp"
#include "Protocal.hpp"

class BasePackage
{
public:
    using SharedPtr = std::shared_ptr<BasePackage>;

private:
    std::queue<BufferWithTime> m_buffer_queue; //收包的buffer队列
    timeval last_tv;
    std::mutex m_buffer_mutex;

protected:
    size_t m_max_queue_length;

public:
    CAN_ID m_can_id; // can_id

    BasePackage() = default;
    ~BasePackage() = default;

    BasePackage(CAN_ID can_id)
    {
        m_can_id = can_id;
        m_max_queue_length = 1;
        last_tv.tv_sec = 0;
        last_tv.tv_usec = 0;
    }

    BasePackage(CAN_ID can_id, size_t max_queue_length)
    {
        m_can_id = can_id;
        m_max_queue_length = max_queue_length;
        last_tv.tv_sec = 0;
        last_tv.tv_usec = 0;
    }
    /**
     * @brief 接收Port发来的数据
     *
     * @param buffer
     */
    void recvBuffer(BufferWithTime buffer)
    {
        timeval new_buffer_tv = buffer.tv;
        if (new_buffer_tv > last_tv)
        {
            last_tv = new_buffer_tv;

            std::lock_guard lock(m_buffer_mutex);
            m_buffer_queue.push(buffer);

            while (m_buffer_queue.size() > m_max_queue_length)
                m_buffer_queue.pop();
        }
    }

    /**
     * @brief 从队列取出一个buffer
     *
     * @return BufferWithTime
     */
    BufferWithTime readBuffer()
    {
        std::lock_guard lock(m_buffer_mutex);
        auto buffer = m_buffer_queue.front();
        return buffer;
    }

    /**
     * @brief 向Port发送数据
     * 
     * @param buffer 数据包
     * @param id 包id
     */
    int sendBuffer(Buffer &buffer, CAN_ID id)
    {
        if(sendBufferFunc) {
            sendBufferFunc(buffer, id);
            return 0;
        }
        else
        {
            std::cout << "send buffer function is nullptr! send falied" << std::endl;
            return -1;
        }
        return 0;
    }

    /**
     * @brief 向Port发送数据
     *
     */
    std::function<void(Buffer, int)> sendBufferFunc;
};





#endif //__WMJ_PACKAGE_HPP__