#ifndef __WMJ_PACKAGE_HPP__
#define __WMJ_PACKAGE_HPP__

#include <iostream>
#include <queue>
#include <mutex>
#include <thread>
#include <functional>
#include <cmath>
#include <sstream>

#include "utils/Utility.hpp"
#include "protocal/Protocal.hpp"
#include "protocal/IDs.hpp"
#include "impls/logger.hpp"

namespace transport{

class BasePackage
{
public:
    using SharedPtr = std::shared_ptr<BasePackage>;

private:
    std::queue<BufferWithTime> m_buffer_queue; //收包的buffer队列
    timeval m_last_tv;
    std::mutex m_buffer_mutex;

protected:
    size_t m_max_queue_length;

public:
    ID m_id; // id
    int m_debug_flag;

    BasePackage() = default;
    ~BasePackage() = default;

    BasePackage(ID id, int debug_flag = 0, size_t max_queue_length = 1):
    m_id(id),
    m_debug_flag(debug_flag),
    m_max_queue_length(max_queue_length),
    m_last_tv{0, 0}
    {}

    /**
     * @brief 接收Port发来的数据
     *
     * @param buffer
     */
    void recvBuffer(BufferWithTime &buffer)
    {
        timeval new_buffer_tv = buffer.tv;
        if (new_buffer_tv > m_last_tv)
        {
            m_last_tv = new_buffer_tv;

            std::lock_guard lock(m_buffer_mutex);
            m_buffer_queue.push(buffer);

            while (m_buffer_queue.size() > m_max_queue_length)
                m_buffer_queue.pop();
        }
        else
        {
            LOGWARN("package id 0x%x, in function %s, received an expired package.", (int)m_id, __PRETTY_FUNCTION__);
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
        if(m_buffer_queue.size() == 0){
            LOGWARN("package id 0x%x, BasePackage::m_buffer_queue is empty, it may have never received any package! 电控是不是没发包啊?过滤器设了吗?看看是谁包头设错了?你是不是没在config里添加这个包?", (int)m_id);
            return BufferWithTime{};
        }
        auto buffer = m_buffer_queue.front();
        if(gettimeval().tv_sec - buffer.tv.tv_sec > 0){ // 1秒超时
            LOGWARN("package id 0x%x, It has been more than 1 second since the last package was received! 超时了啊,是不是电控发包逻辑写的有问题了?", (int)m_id);
        }
        return buffer;
    }

    /**
     * @brief 向Port发送数据
     * 
     * @param buffer 数据包
     * @param id 包id
     */
    int sendBuffer(Buffer &buffer, ID id)
    {
        if(sendBufferFunc) {
            sendBufferFunc(buffer, id);
            return 0;
        }
        else
        {
            LOGWARN("package id 0x%x, BasePackage::sendBufferFunc is nullptr, it may not be registered in class transport::Port, send falied!", (int)m_id);
            return -1;
        }
        return 0;
    }

    /**
     * @brief 向Port发送数据
     *
     */
    std::function<void(Buffer&, ID)> sendBufferFunc;
};


} // namespace transport


#endif //__WMJ_PACKAGE_HPP__